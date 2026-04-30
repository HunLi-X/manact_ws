"""
偏航对齐节点 — 步进式旋转对齐（适配慢速相机更新）。

运动控制通过 SportClient 统一封装（Loco API 方式）。
启动时自动执行 FSM 初始化（DAMP → STAND_UP → START → WALK_RUN → CONTINUOUS_GAIT）。

控制逻辑（步进式）：
    1. 检测目标位置 u（归一化 0~1，0.5 = 画面中央）
    2. 若目标偏离中心，发送一次短时间小幅度旋转（SET_VELOCITY, duration=step_duration）
    3. 等待 camera_settle_time（默认 5 秒）让相机更新
    4. 重新检测目标位置，重复步骤 2~3
    5. 目标居中后停止

运行：
    ros2 run g1_yolo_nav_py yaw_align
    ros2 run g1_yolo_nav_py yaw_align --ros-args -p auto_stand:=false
"""

# ==================================================================
# 1. 标准库导入
# ==================================================================
import math
import time
from typing import Optional

# ==================================================================
# 2. 第三方库与 ROS2 导入
# ==================================================================
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from g1_yolo_nav_py._detection_utils import find_best_detection

# ==================================================================
# 3. 本项目导入
# ==================================================================
from g1_yolo_nav_py.sport_client import SportClient


class YawAlignNode(Node):
    """偏航对齐节点 — 步进式旋转，适配慢速相机更新。

    每次只移动一小步，等相机更新后再决定下一步，
    避免连续旋转时相机延迟导致过冲。
    """

    def __init__(self) -> None:
        super().__init__("g1_yaw_align_node")

        # ---- 参数 ----
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("target_class_id", "chair")
        self.declare_parameter("camera_fov_deg", 87.0)
        self.declare_parameter("center_tolerance", 0.08)
        self.declare_parameter("step_yaw_speed", 0.3)       # 每步旋转速度 (rad/s)
        self.declare_parameter("step_duration", 0.3)         # 每步旋转持续时间 (秒)
        self.declare_parameter("camera_settle_time", 2.0)    # 旋转后等待相机更新时间 (秒)
        self.declare_parameter("max_consecutive_steps", 10)   # 单次最大连续步数
        self.declare_parameter("lost_timeout", 10.0)
        self.declare_parameter("check_rate", 2.0)            # tick 频率（低频即可）
        self.declare_parameter("auto_stand", True)

        p = lambda n: self.get_parameter(n).value
        self._det_topic = p("detection_topic")
        self._target_class = p("target_class_id")
        self._fov_rad = math.radians(float(p("camera_fov_deg")))
        self._center_tol = float(p("center_tolerance"))
        self._step_speed = float(p("step_yaw_speed"))
        self._step_dur = float(p("step_duration"))
        self._settle_time = float(p("camera_settle_time"))
        self._max_steps = int(p("max_consecutive_steps"))
        self._lost_timeout = float(p("lost_timeout"))
        self._check_rate = float(p("check_rate"))
        self._auto_stand = bool(p("auto_stand"))

        # ---- 内部状态 ----
        self._target_u: Optional[float] = None
        self._last_detect_time: float = 0.0
        self._step_count: int = 0           # 本轮已走步数
        self._settling: bool = False        # 正在等待相机更新
        self._settle_start: float = 0.0     # 开始等待的时间
        self._aligned_logged: bool = False  # 已居中是否已打印日志

        # ---- ROS2 订阅 ----
        self.create_subscription(Detection2DArray, self._det_topic,
                                 self._on_detection, 10)

        # ---- 运动控制客户端 ----
        self._sport = SportClient(self)

        # ---- FSM 初始化 ----
        if self._auto_stand:
            self._sport.auto_init_if_needed()
        else:
            self._sport.skip_init()
            self.get_logger().info("跳过自动状态初始化，请确保机器人已处于走跑模式")

        # ---- 定时器 ----
        self._timer = self.create_timer(1.0 / self._check_rate, self._tick)

        # ---- 延迟诊断 ----
        self._diag_done = False
        self._diag_timer = self.create_timer(3.0, self._diag_check)

        self.get_logger().info(
            f"偏航对齐节点就绪（步进模式）: 目标={self._target_class}, "
            f"步速={self._step_speed}rad/s, 步时={self._step_dur}s, "
            f"等待={self._settle_time}s, 容差={self._center_tol}, "
            f"auto_stand={self._auto_stand}"
        )

    # ==================================================================
    #  检测回调
    # ==================================================================
    def _on_detection(self, msg: Detection2DArray) -> None:
        """从检测结果中提取最佳目标的 u 坐标。"""
        best_det, best_score = find_best_detection(msg.detections, self._target_class)
        if best_det is not None:
            prev_u = self._target_u
            self._target_u = best_det.bbox.center.x
            self._last_detect_time = time.time()
            if prev_u is None:
                self.get_logger().info(
                    f"[对齐] 检测到目标: u={self._target_u:.3f}"
                )
        else:
            self._target_u = None

    # ==================================================================
    #  诊断
    # ==================================================================
    def _diag_check(self):
        """启动后延迟检查关键话题状态。"""
        if self._diag_done:
            return
        self._diag_done = True
        if hasattr(self, '_diag_timer') and self._diag_timer is not None:
            self._diag_timer.cancel()

        det_pub_count = self.count_publishers(self._det_topic)
        self.get_logger().info(
            f"[诊断] 检测话题 '{self._det_topic}': 发布者数={det_pub_count}"
        )
        if det_pub_count == 0:
            self.get_logger().warn(
                f"[诊断] 检测话题无发布者! "
                f"请先启动 yolo_detector: "
                f"ros2 run g1_yolo_nav_py yolo_detector"
            )

        sport_sub_count = self.count_subscribers('/api/sport/request')
        self.get_logger().info(
            f"[诊断] 运动话题 '/api/sport/request': 订阅者数={sport_sub_count}"
        )
        if sport_sub_count == 0:
            self.get_logger().error(
                "[诊断] 运动话题无订阅者! "
                "请确认 unitree SDK bridge 已启动。"
            )

    # ==================================================================
    #  步进式对齐逻辑
    # ==================================================================
    def _tick(self) -> None:
        """步进式对齐：移动一小步 → 等待相机更新 → 再检测 → 再移动。"""
        # FSM 未就绪时不处理
        if not self._sport.ready:
            return

        now = time.time()

        # ---- 1. 目标丢失 ----
        if self._target_u is None or (now - self._last_detect_time > self._lost_timeout):
            if self._settling:
                # 等待中丢失目标，停止
                self._sport.stop()
                self._settling = False
                self._step_count = 0
                self.get_logger().info("[对齐] 等待中目标丢失，停止旋转")
            self._aligned_logged = False
            return

        error = self._target_u - 0.5

        # ---- 2. 已居中 ----
        if abs(error) < self._center_tol:
            if self._settling:
                # 等待后检测到已居中
                self._settling = False
                self._step_count = 0
            if not self._aligned_logged:
                self._aligned_logged = True
                self._sport.stop()
                self.get_logger().info(
                    f"[对齐] 目标已居中: u={self._target_u:.3f}, "
                    f"误差={error:.3f} < 容差={self._center_tol}"
                )
            return

        # ---- 3. 正在等待相机更新 ----
        if self._settling:
            elapsed = now - self._settle_start
            if elapsed < self._settle_time:
                # 还在等待中，不动作
                return
            # 等待结束，用最新的 target_u 重新判断（已在上面处理了居中/丢失）
            self._settling = False
            self.get_logger().info(
                f"[对齐] 等待结束，重新检测: u={self._target_u:.3f}, 误差={error:.3f}"
            )

        # ---- 4. 发送一步旋转 ----
        # 旋转方向：目标在右 (u > 0.5) → 机器人右转 (vyaw < 0)
        vyaw = -self._step_speed if error > 0 else self._step_speed

        self._sport.move(vyaw=vyaw, duration=self._step_dur)
        self._step_count += 1

        # 计算旋转角度供日志参考
        turn_deg = math.degrees(vyaw * self._step_dur)

        self.get_logger().info(
            f"[对齐] 第{self._step_count}步: u={self._target_u:.3f}, "
            f"误差={error:+.3f}, vyaw={vyaw:+.2f}rad/s × {self._step_dur}s "
            f"(≈{turn_deg:+.1f}°), 等待{self._settle_time}s..."
        )

        # ---- 5. 进入等待状态 ----
        self._settling = True
        self._settle_start = now
        self._aligned_logged = False

        # 超过最大步数保护
        if self._step_count >= self._max_steps:
            self.get_logger().warn(
                f"[对齐] 已连续旋转 {self._step_count} 步仍未居中，"
                f"重置步数计数器继续尝试"
            )
            self._step_count = 0

    def destroy_node(self) -> None:
        self._sport.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YawAlignNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._sport.stop()
    finally:
        node._sport.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
