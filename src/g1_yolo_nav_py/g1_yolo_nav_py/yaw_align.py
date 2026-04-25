"""
偏航对齐节点 — 高层运动控制，通过 Sport API MOVE 指令旋转机器人让目标居中。

直接发布 unitree_api/msg/Request 到 /api/sport/request，
使用 MOVE (1008) API 控制旋转，与 ctrl_keyboard 保持一致。

控制逻辑：
    1. 从检测结果提取目标 u 坐标（归一化 0~1，0.5 = 画面中央）
    2. 计算误差 error = u - 0.5
    3. P 控制输出 vyaw = kp * error * FOV
    4. 限速后通过 MOVE API 发布
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
# 3. Sport API 常量
# ==================================================================
API_MOVE = 1008
API_STOPMOVE = 1003


class YawAlignNode(Node):
    """偏航对齐节点 — 通过 Sport API MOVE 指令让目标保持在画面中央。"""

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
        self.declare_parameter("lost_timeout", 1.0)

        p = lambda n: self.get_parameter(n).value
        self._det_topic = p("detection_topic")
        self._target_class = p("target_class_id")
        self._fov_rad = math.radians(p("camera_fov_deg"))
        self._center_tol = float(p("center_tolerance"))
        self._kp = float(p("yaw_kp"))
        self._max_speed = float(p("max_yaw_speed"))
        self._ctrl_rate = float(p("control_rate"))
        self._lost_timeout = float(p("lost_timeout"))

        # ---- 内部状态 ----
        self._target_u: Optional[float] = None
        self._last_detect_time: float = 0.0
        self._first_target_logged = False

        # ---- ROS2 订阅 ----
        # YOLO 检测器用默认 RELIABLE 发布，这里也用 RELIABLE 确保兼容
        self.create_subscription(Detection2DArray, self._det_topic,
                                 self._on_detection, 10)

        # ---- Sport API 发布 ----
        self._sport_pub = self.create_publisher(Request, '/api/sport/request', 10)
        self._sport_req = Request()

        # ---- 定时器 ----
        self._timer = self.create_timer(1.0 / self._ctrl_rate, self._tick)

        # ---- 停转发布器（确保启动时为零速）----
        self._publish_stop()

        self.get_logger().info(
            f"偏航对齐节点就绪（Sport API）: 目标={self._target_class}, "
            f"kp={self._kp}, 容差={self._center_tol}"
        )

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
                self._first_target_logged = True
        else:
            self._target_u = None

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

    def _publish_stop(self) -> None:
        """发送 STOPMOVE 指令停止旋转。"""
        self._sport_req.header.identity.api_id = API_STOPMOVE
        self._sport_req.parameter = ''
        self._sport_pub.publish(self._sport_req)

    def _tick(self) -> None:
        """定时回调 — 计算并通过 Sport API 发布旋转指令。"""
        vyaw = self._compute_vyaw()
        if abs(vyaw) > 1e-6:
            self._sport_req.header.identity.api_id = API_MOVE
            self._sport_req.parameter = json.dumps({"x": 0.0, "y": 0.0, "z": vyaw})
        else:
            self._sport_req.header.identity.api_id = API_STOPMOVE
            self._sport_req.parameter = ''
        self._sport_pub.publish(self._sport_req)
        # 首次 tick 且有目标时，打印一次状态确认
        if self._first_target_logged and hasattr(self, '_tick_logged') is False:
            self._tick_logged = True
            self.get_logger().info(
                f"[对齐] tick 正常运行: target_u={self._target_u}, vyaw={vyaw:.3f}"
            )

    def destroy_node(self) -> None:
        self._publish_stop()
        self.get_logger().info("偏航对齐节点停止，已发送零速指令")
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
