"""
偏航对齐节点 — 通过 Sport API MOVE 控制机器人旋转使目标居中。

运动控制通过 SportClient 统一封装，使用 MOVE(1008)。
启动时自动执行 FSM 初始化（DAMP → STANDUP → BALANCESTAND → CONTINUOUSGAIT）。

控制逻辑：
    1. 从检测结果提取目标 u 坐标（归一化 0~1，0.5 = 画面中央）
    2. 计算误差 error = u - 0.5
    3. P 控制输出 vyaw = -kp * error * FOV（负号：目标在右→机器人右转→vyaw 为负）
    4. 限速后通过 MOVE API 发布

运行：
    ros2 run g1_yolo_nav_py yaw_align
    ros2 run g1_yolo_nav_py yaw_align --ros-args -p auto_stand:=false
"""

# ==================================================================
# 1. 标准库导入
# ==================================================================
import math  # 角度弧度转换
import time  # 计时
from typing import Optional  # 类型注解

# ==================================================================
# 2. 第三方库与 ROS2 导入
# ==================================================================
import rclpy  # ROS2 Python 客户端库
from rclpy.node import Node  # ROS2 节点基类
from vision_msgs.msg import Detection2DArray  # 2D 检测结果消息

# ==================================================================
# 3. 本项目导入
# ==================================================================
from g1_yolo_nav_py.sport_client import SportClient  # 统一运动控制客户端


class YawAlignNode(Node):
    """偏航对齐节点 — 通过 Sport API MOVE 让目标保持在画面中央。"""

    def __init__(self) -> None:
        super().__init__("g1_yaw_align_node")

        # ---- 参数 ----
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("target_class_id", "chair")
        self.declare_parameter("camera_fov_deg", 87.0)
        self.declare_parameter("center_tolerance", 0.05)
        self.declare_parameter("yaw_kp", 2.0)
        self.declare_parameter("max_yaw_speed", 1.2)
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("lost_timeout", 5.0)
        self.declare_parameter("auto_stand", True)

        p = lambda n: self.get_parameter(n).value
        self._det_topic = p("detection_topic")
        self._target_class = p("target_class_id")
        self._fov_rad = math.radians(float(p("camera_fov_deg")))
        self._center_tol = float(p("center_tolerance"))
        self._kp = float(p("yaw_kp"))
        self._max_speed = float(p("max_yaw_speed"))
        self._ctrl_rate = float(p("control_rate"))
        self._lost_timeout = float(p("lost_timeout"))
        self._auto_stand = bool(p("auto_stand"))

        # ---- 内部状态 ----
        self._target_u: Optional[float] = None
        self._last_detect_time: float = 0.0
        self._tick_count: int = 0
        self._log_interval: int = 50  # 每 50 个 tick（~5s）打印一次状态
        self._is_moving: bool = False  # 追踪运动状态，避免重复发送停止

        # ---- ROS2 订阅 ----
        self.create_subscription(Detection2DArray, self._det_topic,
                                 self._on_detection, 10)

        # ---- 运动控制客户端 ----
        self._sport = SportClient(self)

        # ---- FSM 初始化 ----
        if self._auto_stand:
            self._sport.init_fsm()
        else:
            self._sport.skip_init()
            self.get_logger().info("跳过自动状态初始化，请确保机器人已处于走跑模式")

        # ---- 定时器 ----
        self._timer = self.create_timer(1.0 / self._ctrl_rate, self._tick)

        # ---- 延迟诊断（3秒后检查关键话题的订阅者/发布者，只执行一次）----
        self._diag_done = False
        self._diag_timer = self.create_timer(3.0, self._diag_check)

        self._first_move_logged = False

        self.get_logger().info(
            f"偏航对齐节点就绪: 目标={self._target_class}, "
            f"kp={self._kp}, 容差={self._center_tol}, "
            f"lost_timeout={self._lost_timeout}, auto_stand={self._auto_stand}"
        )

    # ==================================================================
    #  检测回调
    # ==================================================================
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
            # ---- 延迟诊断（3秒后检查关键话题的订阅者/发布者，只执行一次）----
        self._diag_done = False
        self._diag_timer = self.create_timer(3.0, self._diag_check)

        self._first_move_logged = False  # 目标丢失时重置，方便下次调试

    # ==================================================================
    #  P 控制器
    # ==================================================================
    def _compute_vyaw(self) -> float:
        """P 控制计算偏航角速度。

        目标在画面右侧 (u > 0.5) → 机器人右转 (vyaw < 0)
        目标在画面左侧 (u < 0.5) → 机器人左转 (vyaw > 0)
        """
        # 目标丢失 → 不旋转
        if self._target_u is None or (time.time() - self._last_detect_time > self._lost_timeout):
            return 0.0

        error = self._target_u - 0.5

        # 在容差范围内不调整
        if abs(error) < self._center_tol:
            return 0.0

        # 偏移 → 角度误差 → P 控制（负号确保方向正确）
        error_angle = error * self._fov_rad
        vyaw = -self._kp * error_angle

        # 限速
        vyaw = max(-self._max_speed, min(self._max_speed, vyaw))

        return vyaw

    # ==================================================================
    #  定时回调
    # ==================================================================
    def _tick(self) -> None:
        """定时回调 — 计算并通过 Sport API MOVE 发布旋转指令。"""
        # FSM 未就绪时不发送运动指令
        if not self._sport.ready:
            return

        vyaw = self._compute_vyaw()
        if abs(vyaw) > 1e-6:
            self._sport.move(vyaw=vyaw)
            if not self._first_move_logged:
                self.get_logger().info(
                    f"[对齐] 发送 MOVE: vyaw={vyaw:.3f} rad/s"
                )
                self._first_move_logged = True
            self._is_moving = True
        elif self._is_moving:
            # 仅在运动→停止状态切换时发送 STOPMOVE
            self._sport.stop()
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
