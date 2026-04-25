#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
前进控制节点 — 检测到目标对齐后，通过 Sport API 控制机器人前进

控制方式：
    通过 unitree_api/msg/Request 发布到 /api/sport/request，
    使用 MOVE API 控制前进，与 ctrl_keyboard 保持一致。

状态机：
    IDLE ──(目标居中且稳定)──→ MOVING ──(到达/丢失)──→ IDLE

运行：
    ros2 run g1_yolo_nav_py loco_forward
"""

# ==================================================================
# 1. 标准库导入
# ==================================================================
import json        # Sport API 参数序列化
import time        # 计时与延时
from typing import Optional  # 类型注解

# ==================================================================
# 2. 第三方库与 ROS2 导入
# ==================================================================
import rclpy  # ROS2 Python 客户端库
from rclpy.node import Node  # ROS2 节点基类
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # QoS 配置
from vision_msgs.msg import Detection2DArray  # 2D 检测结果消息
from unitree_api.msg import Request  # Sport API 请求消息

# ==================================================================
# 3. Sport API 常量
# ==================================================================
API_MOVE = 1008
API_STOPMOVE = 1003


class LocoForwardNode(Node):
    """
    前进节点 — 目标对齐后通过 Sport API 前进到目标。

    状态机：
        IDLE ──(目标居中且稳定)──→ MOVING ──(到达/丢失)──→ IDLE
    """

    def __init__(self) -> None:
        super().__init__("g1_loco_forward_node")

        # ---- 参数 ----
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("target_class_id", "chair")
        self.declare_parameter("forward_speed", 0.3)
        self.declare_parameter("forward_duration", 0.5)
        self.declare_parameter("center_tolerance", 0.08)
        self.declare_parameter("align_stable_time", 0.8)
        self.declare_parameter("arrive_bbox_ratio", 0.45)
        self.declare_parameter("lost_timeout", 1.0)
        self.declare_parameter("check_rate", 10.0)

        p = lambda n: self.get_parameter(n).value
        self._det_topic = p("detection_topic")
        self._target_class = p("target_class_id")
        self._speed = float(p("forward_speed"))
        self._duration = float(p("forward_duration"))
        self._center_tol = float(p("center_tolerance"))
        self._stable_time = float(p("align_stable_time"))
        self._arrive_ratio = float(p("arrive_bbox_ratio"))
        self._lost_timeout = float(p("lost_timeout"))
        self._rate = float(p("check_rate"))

        # ---- 内部状态 ----
        self._target_u: Optional[float] = None
        self._bbox_size_x: float = 0.0
        self._bbox_size_y: float = 0.0
        self._last_detect_time: float = 0.0
        self._moving: bool = False
        self._align_start: Optional[float] = None
        self._last_forward_time: float = 0.0

        # ---- ROS2 订阅 ----
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Detection2DArray, self._det_topic,
                                 self._on_detection, qos)

        # ---- Sport API 发布 ----
        self._sport_pub = self.create_publisher(Request, '/api/sport/request', 10)
        self._sport_req = Request()

        # ---- 定时器 ----
        self._timer = self.create_timer(1.0 / self._rate, self._tick)

        self.get_logger().info(
            f"Loco 前进节点就绪（Sport API）: 速度={self._speed}m/s, "
            f"到达条件={self._arrive_ratio}"
        )

    # ==================================================================
    #  Sport API
    # ==================================================================
    def _publish_sport(self, api_id: int, params: dict) -> None:
        """发布 Sport API 请求。"""
        self._sport_req.header.identity.api_id = api_id
        self._sport_req.parameter = json.dumps(params)
        self._sport_pub.publish(self._sport_req)

    def _start_move(self) -> None:
        """开始前进。"""
        if self._moving:
            return
        self._publish_sport(API_MOVE, {
            "x": self._speed,
            "y": 0.0,
            "z": 0.0,
        })
        self._moving = True
        self._last_forward_time = time.time()
        self.get_logger().info(f"开始前进: vx={self._speed} m/s")

    def _continue_move(self) -> None:
        """持续前进（每秒发送一次 MOVE）。"""
        now = time.time()
        if now - self._last_forward_time >= 1.0:
            self._publish_sport(API_MOVE, {
                "x": self._speed,
                "y": 0.0,
                "z": 0.0,
            })
            self._last_forward_time = now

    def _stop_move(self) -> None:
        """停止前进。"""
        if not self._moving:
            return
        self._publish_sport(API_STOPMOVE, {})
        self._moving = False
        self.get_logger().info("停止前进")

    # ==================================================================
    #  检测回调
    # ==================================================================
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

    # ==================================================================
    #  状态机
    # ==================================================================
    def _tick(self) -> None:
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

        # ---- 5. 到达判断 ----
        bbox_max = max(self._bbox_size_x, self._bbox_size_y)
        if bbox_max >= self._arrive_ratio:
            self._stop_move()
            self.get_logger().info(
                f"到达目标! bbox={bbox_max:.2f} >= {self._arrive_ratio}"
            )
            return

        # ---- 4. 居中稳定 → 前进 ----
        if aligned_dur >= self._stable_time:
            if not self._moving:
                self._start_move()
            else:
                self._continue_move()

    def destroy_node(self) -> None:
        self._stop_move()
        super().destroy_node()


def main(args=None):
    # 纯 Sport API 通信，无需 DDS 兼容层
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
