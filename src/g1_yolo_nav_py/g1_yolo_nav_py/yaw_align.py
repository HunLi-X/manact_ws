"""
偏航对齐节点 — 通过 Loco API 控制机器人旋转使目标居中。

参考 ctr_keyboard 的工作方式：
    1. 启动时通过 SET_FSM_ID 做状态机切换（DAMP → STAND_UP）
    2. 运动控制通过 SET_VELOCITY (7105)，参数格式 {"velocity": [vx,vy,vyaw], "duration": ...}

控制逻辑：
    1. 从检测结果提取目标 u 坐标（归一化 0~1，0.5 = 画面中央）
    2. 计算误差 error = u - 0.5
    3. P 控制输出 vyaw = kp * error * FOV
    4. 限速后通过 SET_VELOCITY API 发布
"""

# ==================================================================
# 1. 标准库导入
# ==================================================================
import json  # Sport API 参数序列化
import os   # sys.path 修改
import sys  # sys.path 修改
import math  # 角度弧度转换
import time  # 计时
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
from vision_msgs.msg import Detection2DArray  # 2D 检测结果消息
from unitree_api.msg import Request  # Sport API 请求消息


# ==================================================================
# 3. Loco API 常量（参考 ctr_keyboard.py）
# ==================================================================
API_SET_FSM_ID = 7101
API_SET_VELOCITY = 7105

# FSM 状态 ID
FSM_DAMP = 1
FSM_STAND_UP = 4
FSM_SIT = 3


class YawAlignNode(Node):
    """偏航对齐节点 — 通过 Loco API SET_VELOCITY 让目标保持在画面中央。"""

    def __init__(self) -> None:
        super().__init__("g1_yaw_align_node")

        # ---- 参数 ----
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("target_class_id", "chair")
        self.declare_parameter("camera_fov_deg", 87.0)
        self.declare_parameter("center_tolerance", 0.05)
        self.declare_parameter("yaw_kp", 2.0)
        self.declare_parameter("max_yaw_speed", 0.6)
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("lost_timeout", 5.0)
        self.declare_parameter("auto_stand", True)
        self.declare_parameter("velocity_duration", 0.5)

        p = lambda n: self.get_parameter(n).value
        self._det_topic = p("detection_topic")
        self._target_class = p("target_class_id")
        self._fov_rad = math.radians(p("camera_fov_deg"))
        self._center_tol = float(p("center_tolerance"))
        self._kp = float(p("yaw_kp"))
        self._max_speed = float(p("max_yaw_speed"))
        self._ctrl_rate = float(p("control_rate"))
        self._lost_timeout = float(p("lost_timeout"))
        self._auto_stand = bool(p("auto_stand"))
        self._vel_duration = float(p("velocity_duration"))

        # ---- 内部状态 ----
        self._target_u: Optional[float] = None
        self._last_detect_time: float = 0.0
        self._tick_count: int = 0
        self._log_interval: int = 50  # 每 50 个 tick（~5s）打印一次状态
        self._is_moving: bool = False  # 追踪运动状态，避免重复发送零速
        self._ready: bool = False  # 状态机就绪标志

        # ---- ROS2 订阅 ----
        self.create_subscription(Detection2DArray, self._det_topic,
                                 self._on_detection, 10)

        # ---- Loco API 发布 ----
        self._sport_pub = self.create_publisher(Request, '/api/sport/request', 10)

        # ---- 启动状态机切换 ----
        if self._auto_stand:
            self._do_stand_up()
        else:
            self._ready = True
            self.get_logger().info("跳过自动站立，请确保机器人已处于站立状态")

        # ---- 定时器 ----
        self._timer = self.create_timer(1.0 / self._ctrl_rate, self._tick)

        self._first_move_logged = False

        self.get_logger().info(
            f"偏航对齐节点就绪（Loco API）: 目标={self._target_class}, "
            f"kp={self._kp}, 容差={self._center_tol}, "
            f"lost_timeout={self._lost_timeout}, auto_stand={self._auto_stand}"
        )

    def _publish_request(self, api_id: int, parameter: str = '') -> None:
        """创建新 Request 并发布（每次 publish 必须创建新对象，避免 DDS 缓冲区复用问题）。"""
        req = Request()
        req.header.identity.api_id = api_id
        req.parameter = parameter
        self._sport_pub.publish(req)

    def _do_stand_up(self) -> None:
        """执行状态机切换：DAMP → STAND_UP（参考 ctr_keyboard.py）。

        在单独线程中执行，避免阻塞 ROS2 spin。
        """
        import threading

        def _stand_up_thread():
            self.get_logger().info("[状态机] 切换到 DAMP 模式...")
            self._publish_request(API_SET_FSM_ID, json.dumps({"data": FSM_DAMP}))
            time.sleep(2)

            self.get_logger().info("[状态机] 切换到 STAND_UP 模式...")
            self._publish_request(API_SET_FSM_ID, json.dumps({"data": FSM_STAND_UP}))
            time.sleep(3)

            self._ready = True
            self.get_logger().info("[状态机] 站立完成，就绪")

        t = threading.Thread(target=_stand_up_thread, daemon=True)
        t.start()

    def _on_detection(self, msg: Detection2DArray) -> None:
        """从检测结果中提取最佳目标的 u 坐标。"""
        best_det = None
        best_score = 0.0
        for det in msg.detections:
            if det.results and det.results[0].id == self._target_class:
                if det.results[0].score > best_score:
                    best_score = det.results[0].score
                    best_det = det
        if best_det is not None:
            prev_u = self._target_u
            self._target_u = best_det.bbox.center.x
            self._last_detect_time = time.time()
            # 首次检测到目标时打印日志
            if prev_u is None:
                self.get_logger().info(
                    f"[对齐] 检测到目标: u={self._target_u:.3f}, vyaw={self._compute_vyaw():.3f}"
                )
        else:
            self._target_u = None
            self._first_move_logged = False  # 目标丢失时重置，方便下次调试

    def _compute_vyaw(self) -> float:
        """P 控制计算偏航角速度。"""
        # 目标丢失 → 不旋转
        if self._target_u is None or (time.time() - self._last_detect_time > self._lost_timeout):
            return 0.0

        error = self._target_u - 0.5

        # 在容差范围内不调整
        if abs(error) < self._center_tol:
            return 0.0

        # 偏移 → 角度误差 → P 控制
        error_angle = error * self._fov_rad
        vyaw = -self._kp * error_angle

        # 限速
        vyaw = max(-self._max_speed, min(self._max_speed, vyaw))

        return vyaw

    def _tick(self) -> None:
        """定时回调 — 计算并通过 Loco API SET_VELOCITY 发布旋转指令。"""
        # 状态机未就绪时不发送指令
        if not self._ready:
            return

        vyaw = self._compute_vyaw()
        if abs(vyaw) > 1e-6:
            self._publish_request(API_SET_VELOCITY, json.dumps({
                "velocity": [0.0, 0.0, vyaw],
                "duration": self._vel_duration,
            }))
            if not self._first_move_logged:
                self.get_logger().info(
                    f"[对齐] 发送 SET_VELOCITY: vyaw={vyaw:.3f} rad/s"
                )
                self._first_move_logged = True
            self._is_moving = True
        elif self._is_moving:
            # 仅在运动→停止状态切换时发送零速（避免 10Hz 刷屏重置 API 状态）
            self._publish_request(API_SET_VELOCITY, json.dumps({
                "velocity": [0.0, 0.0, 0.0],
                "duration": 0.5,
            }))
            self._is_moving = False

        # 周期性日志：每 _log_interval 个 tick 打印一次状态
        self._tick_count += 1
        if self._tick_count % self._log_interval == 0:
            target_lost = (self._target_u is None
                           or (time.time() - self._last_detect_time > self._lost_timeout))
            self.get_logger().info(
                f"[对齐] 状态: u={self._target_u if self._target_u is not None else 'N/A'}, "
                f"vyaw={vyaw:.3f}, lost={target_lost}"
            )

    def destroy_node(self) -> None:
        # 发送零速 + 坐下指令
        self._publish_request(API_SET_VELOCITY, json.dumps({
            "velocity": [0.0, 0.0, 0.0], "duration": 1.0
        }))
        time.sleep(0.2)
        self._publish_request(API_SET_FSM_ID, json.dumps({"data": FSM_SIT}))
        self.get_logger().info("偏航对齐节点停止，已发送停止+坐下指令")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YawAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
