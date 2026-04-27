#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
前进控制节点 — 检测到目标对齐后，通过 Sport API MOVE 控制机器人前进。

运动控制通过 SportClient 统一封装。
启动时自动执行 FSM 初始化（DAMP → STANDUP → BALANCESTAND → CONTINUOUSGAIT）。

状态机：
    IDLE ──(目标居中且稳定)──→ MOVING ──(到达/丢失)──→ IDLE

运行：
    ros2 run g1_yolo_nav_py loco_forward
    ros2 run g1_yolo_nav_py loco_forward --ros-args -p forward_speed:=0.15
"""

# ==================================================================
# 1. 标准库导入
# ==================================================================
import time  # 计时
from typing import Optional  # 类型注解

# ==================================================================
# 2. 第三方库与 ROS2 导入
# ==================================================================
import rclpy  # ROS2 Python 客户端库
from rclpy.node import Node  # ROS2 节点基类
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # QoS 配置
from vision_msgs.msg import Detection2DArray  # 2D 检测结果消息

# ==================================================================
# 3. 本项目导入
# ==================================================================
from g1_yolo_nav_py.sport_client import SportClient  # 统一运动控制客户端


class LocoForwardNode(Node):
    """
    前进节点 — 目标对齐后通过 Sport API MOVE 前进到目标。

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
        self.declare_parameter("auto_stand", True)

        p = lambda n: self.get_parameter(n).value
        self._det_topic = p("detection_topic")
        self._target_class = p("target_class_id")
        self._speed = float(p("forward_speed"))
        self._center_tol = float(p("center_tolerance"))
        self._stable_time = float(p("align_stable_time"))
        self._arrive_ratio = float(p("arrive_bbox_ratio"))
        self._lost_timeout = float(p("lost_timeout"))
        self._rate = float(p("check_rate"))
        self._auto_stand = bool(p("auto_stand"))

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

        # ---- 运动控制客户端 ----
        self._sport = SportClient(self)

        # ---- FSM 初始化 ----
        if self._auto_stand:
            self._sport.init_fsm()
        else:
            self._sport.skip_init()
            self.get_logger().info("跳过自动状态初始化，请确保机器人已处于走跑模式")

        # ---- 定时器 ----
        self._timer = self.create_timer(1.0 / self._rate, self._tick)

        self.get_logger().info(
            f"前进节点就绪（Sport API）: 速度={self._speed}m/s, "
            f"到达条件={self._arrive_ratio}, auto_stand={self._auto_stand}"
        )

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
    def _start_move(self) -> None:
        """开始前进。"""
        if self._moving:
            return
        self._sport.move(vx=self._speed)
        self._moving = True
        self._last_forward_time = time.time()
        self.get_logger().info(f"开始前进: vx={self._speed} m/s")

    def _continue_move(self) -> None:
        """持续前进（每秒发送一次 MOVE）。"""
        now = time.time()
        if now - self._last_forward_time >= 1.0:
            self._sport.move(vx=self._speed)
            self._last_forward_time = now

    def _stop_move(self) -> None:
        """停止前进。"""
        if not self._moving:
            return
        self._sport.stop()
        self._moving = False
        self.get_logger().info("停止前进")

    def _tick(self) -> None:
        now = time.time()

        # FSM 未就绪时不处理
        if not self._sport.ready:
            return

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

        # ---- 4. 到达判断 ----
        bbox_max = max(self._bbox_size_x, self._bbox_size_y)
        if bbox_max >= self._arrive_ratio:
            self._stop_move()
            self.get_logger().info(
                f"到达目标! bbox={bbox_max:.2f} >= {self._arrive_ratio}"
            )
            return

        # ---- 5. 居中稳定 → 前进 ----
        if aligned_dur >= self._stable_time:
            if not self._moving:
                self._start_move()
            else:
                self._continue_move()

    def destroy_node(self) -> None:
        self._sport.stop()
        time.sleep(0.2)
        self._sport.sit()
        super().destroy_node()


def main(args=None):
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
