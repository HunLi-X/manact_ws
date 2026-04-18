"""
Loco 前进控制节点 — 检测到目标对齐后，通过 LocoClient 控制机器人前进
...
"""

# ==================================================================
# 1. 标准库导入
# ==================================================================
import os   # sys.path 修改
import sys  # sys.path 修改
import time  # 计时与延时
from typing import Optional  # 类型注解

# ROS2 colcon 隔离 PYTHONPATH，必须在所有 import 之前追加路径
for _p in [
    "/usr/lib/python3/dist-packages",
    os.path.expanduser("~/.local/lib/python3.8/site-packages"),
    "/usr/local/lib/python3.8/dist-packages",
]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

# ==================================================================
# 2. 第三方库与 ROS2 导入
# ==================================================================
import rclpy  # ROS2 Python 客户端库
from rclpy.node import Node  # ROS2 节点基类
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # QoS 配置
from vision_msgs.msg import Detection2DArray  # 2D 检测结果消息

# unitree_sdk2py: 宇树机器人底层 SDK（可选依赖）
try:
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient  # G1 运动控制客户端
    LOCO_AVAILABLE = True
except ImportError:
    LOCO_AVAILABLE = False


class LocoForwardNode(Node):
    """
    Loco 前进节点 — 目标对齐后前进到目标。

    状态机：
        IDLE ──(目标居中且稳定)──→ MOVING ──(到达/丢失)──→ IDLE
    """

    def __init__(self) -> None:
        super().__init__("g1_loco_forward_node")

        # ---- 参数 ----
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("target_class_id", "chair")
        self.declare_parameter("forward_speed", 0.3)
        self.declare_parameter("center_tolerance", 0.08)
        self.declare_parameter("align_stable_time", 0.8)
        self.declare_parameter("arrive_bbox_ratio", 0.45)
        self.declare_parameter("lost_timeout", 1.0)
        self.declare_parameter("check_rate", 10.0)
        self.declare_parameter("network_interface", "")

        p = lambda n: self.get_parameter(n).value
        self._det_topic = p("detection_topic")
        self._target_class = p("target_class_id")
        self._speed = float(p("forward_speed"))
        self._center_tol = float(p("center_tolerance"))
        self._stable_time = float(p("align_stable_time"))
        self._arrive_ratio = float(p("arrive_bbox_ratio"))
        self._lost_timeout = float(p("lost_timeout"))
        self._rate = float(p("check_rate"))
        self._net_iface = p("network_interface")

        # ---- 内部状态 ----
        self._target_u: Optional[float] = None
        self._bbox_size_x: float = 0.0
        self._bbox_size_y: float = 0.0
        self._last_detect_time: float = 0.0
        self._moving: bool = False
        self._align_start: Optional[float] = None

        self._loco: Optional[LocoClient] = None

        # ---- ROS2 订阅 ----
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Detection2DArray, self._det_topic,
                                 self._on_detection, qos)

        # ---- 定时器（替代线程）----
        self._timer = self.create_timer(1.0 / self._rate, self._tick)

        # ---- 初始化 LocoClient ----
        if not LOCO_AVAILABLE:
            self.get_logger().error("LocoClient 不可用！pip install unitree_sdk2py")
            return

        self._init_loco()

        self.get_logger().info(
            f"Loco 前进节点就绪: 速度={self._speed}m/s, "
            f"到达条件={self._arrive_ratio}"
        )

    def _init_loco(self) -> None:
        try:
            # ChannelFactoryInitialize 已在 main() 中提前调用（先于 rclpy.init）
            self._loco = LocoClient()
            self._loco.SetTimeout(5.0)
            self._loco.Init()
            self.get_logger().info("LocoClient 初始化成功")
        except Exception as e:
            self.get_logger().error(f"LocoClient 失败: {e}")

    def _on_detection(self, msg: Detection2DArray) -> None:
        best_det = None
        best_score = 0.0
        for det in msg.detections:
            if det.results and det.results[0].id == self._target_class:
                if det.results[0].score > best_score:
                    best_score = det.results[0].score
                    best_det = det
        if best_det is not None:
            bbox = best_det.bbox
            self._target_u = bbox.center.x
            self._bbox_size_x = bbox.size_x
            self._bbox_size_y = bbox.size_y
            self._last_detect_time = time.time()
        else:
            self._target_u = None

    def _start_move(self) -> None:
        """开始前进。"""
        if self._loco is None or self._moving:
            return
        try:
            self._loco.Move(vx=self._speed, vy=0.0, vyaw=0.0, continuous=True)
            self._moving = True
            self.get_logger().info(f"开始前进: vx={self._speed} m/s")
        except Exception as e:
            self.get_logger().warn(f"Move 失败: {e}")

    def _stop_move(self) -> None:
        """停止前进。"""
        if not self._moving:
            return
        try:
            self._loco.StopMove()
            self._moving = False
            self.get_logger().info("停止前进")
        except Exception as e:
            self.get_logger().warn(f"StopMove 失败: {e}")

    def _tick(self) -> None:
        """
        定时回调 — 判断是否需要前进/停止。

        判断逻辑：
            1. 无目标 / 丢失 → 停止
            2. 目标偏离中心 > 容差 → 停止，重置计时
            3. 目标居中但未够久 → 继续等待
            4. 目标居中且足够久 → 开始/继续前进
            5. 检测框足够大（够近）→ 停止（到达）
        """
        now = time.time()

        # ---- 1. 目标丢失 ----
        if self._target_u is None or (now - self._last_detect_time > self._lost_timeout):
            self._stop_move()
            self._align_start = None
            return

        error = abs(self._target_u - 0.5)

        # ---- 2. 目标偏离中心 ----
        if error > self._center_tol:
            self._stop_move()
            self._align_start = None
            return

        # ---- 3. 开始/更新居中计时 ----
        if self._align_start is None:
            self._align_start = now

        aligned_dur = now - self._align_start

        # ---- 5. 到达判断（检测框够大）----
        bbox_max = max(self._bbox_size_x, self._bbox_size_y)
        if bbox_max >= self._arrive_ratio:
            self._stop_move()
            self.get_logger().info(
                f"到达目标! bbox={bbox_max:.2f} >= {self._arrive_ratio}"
            )
            return

        # ---- 4. 居中稳定 → 前进 ----
        if aligned_dur >= self._stable_time:
            self._start_move()

    def destroy_node(self) -> None:
        self._stop_move()
        super().destroy_node()


def main(args=None):
    _iface = ""
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        _iface = sys.argv[1]

    # 在 rclpy.init() 之前初始化 unitree DDS（CycloneDDS 兼容层）
    from g1_yolo_nav_py._dds_compat import init_unitree_dds_before_ros2
    init_unitree_dds_before_ros2(_iface)

    rclpy.init(args=args)
    node = LocoForwardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
