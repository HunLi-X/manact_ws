#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
前进控制节点 — 检测到目标对齐后，通过 Loco API SET_VELOCITY 控制机器人前进。

运动控制通过 SportClient 统一封装（Loco API 方式）。
不自动执行 FSM 初始化，需手动进入走跑模式。

状态机：
    IDLE ──(目标居中且稳定)──→ MOVING ──(到达/丢失)──→ IDLE

运行：
    ros2 run g1_yolo_nav_py loco_forward
    ros2 run g1_yolo_nav_py loco_forward --ros-args -p forward_speed:=0.15
"""

# ==================================================================
# 1. 标准库导入
# ==================================================================
import math
import time  # 计时
from typing import Optional  # 类型注解

# ==================================================================
# 2. 第三方库与 ROS2 导入
# ==================================================================
import numpy as np
import rclpy  # ROS2 Python 客户端库
from rclpy.node import Node  # ROS2 节点基类
from g1_yolo_nav_py._detection_utils import find_best_detection, sample_depth_at_pixel, depth_to_meters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray  # 2D 检测结果消息
from cv_bridge import CvBridge

# ==================================================================
# 3. 本项目导入
# ==================================================================
from g1_yolo_nav_py.sport_client import SportClient  # 统一运动控制客户端


class LocoForwardNode(Node):
    """
    前进节点 — 目标对齐后通过 Loco API SET_VELOCITY 前进到目标。

    状态机：
        IDLE ──(目标居中且稳定)──→ MOVING ──(到达/丢失)──→ IDLE
    """

    def __init__(self) -> None:
        super().__init__("g1_loco_forward_node")

        # ---- 参数 ----
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("depth_topic", "/D455_1/depth/image_rect_raw")
        self.declare_parameter("target_class_id", "chair")
        self.declare_parameter("use_depth_distance", True)
        self.declare_parameter("stop_distance", 0.5)
        self.declare_parameter("depth_sample_radius", 5)
        self.declare_parameter("forward_speed", 0.3)
        self.declare_parameter("center_tolerance", 0.08)
        self.declare_parameter("align_stable_time", 0.8)
        self.declare_parameter("arrive_bbox_ratio", 0.45)
        self.declare_parameter("lost_timeout", 1.0)
        self.declare_parameter("check_rate", 10.0)
        self.declare_parameter("sit_on_exit", True)  # 退出时是否自动坐下

        p = lambda n: self.get_parameter(n).value
        self._det_topic = p("detection_topic")
        self._depth_topic = p("depth_topic")
        self._target_class = p("target_class_id")
        self._use_depth = bool(p("use_depth_distance"))
        self._stop_distance = float(p("stop_distance"))
        self._depth_radius = max(1, int(p("depth_sample_radius")))
        self._speed = float(p("forward_speed"))
        self._center_tol = float(p("center_tolerance"))
        self._stable_time = float(p("align_stable_time"))
        self._arrive_ratio = float(p("arrive_bbox_ratio"))
        self._lost_timeout = float(p("lost_timeout"))
        self._rate = float(p("check_rate"))
        self._sit_on_exit = bool(p("sit_on_exit"))

        # ---- 内部状态 ----
        self._target_u: Optional[float] = None
        self._target_v: Optional[float] = None
        self._target_distance: Optional[float] = None
        self._depth_image: Optional[np.ndarray] = None
        self._depth_encoding: str = ""
        self._bbox_size_x: float = 0.0
        self._bbox_size_y: float = 0.0
        self._last_detect_time: float = 0.0
        self._moving: bool = False
        self._align_start: Optional[float] = None
        self._last_forward_time: float = 0.0

        # ---- CV Bridge ----
        self._bridge = CvBridge()

        # ---- ROS2 订阅 ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Detection2DArray, self._det_topic,
                                 self._on_detection, 10)
        if self._use_depth:
            self.create_subscription(
                Image, self._depth_topic, self._on_depth, sensor_qos
            )

        # ---- 运动控制客户端 ----
        self._sport = SportClient(self)

        # ---- 跳过自动 FSM 初始化，由用户手动进入走跑模式 ----
        self._sport.skip_init()

        # ---- 定时器 ----
        self._timer = self.create_timer(1.0 / self._rate, self._tick)

        # ---- 延迟诊断（3秒后检查关键话题，只执行一次）----
        self._diag_done = False
        self._diag_timer = self.create_timer(3.0, self._diag_check)

        self.get_logger().info(
            f"前进节点就绪（Loco API）: 速度={self._speed}m/s, "
            f"深度距离={self._use_depth}(停止≤{self._stop_distance}m), "
            f"bbox到达={self._arrive_ratio}"
        )

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
                "[诊断] ⚠ 运动话题无订阅者! "
                "MOVE 命令不会被机器人执行! "
                "请确认 unitree SDK bridge 已启动。"
            )

    # ==================================================================
    #  检测回调
    # ==================================================================
    def _on_detection(self, msg: Detection2DArray) -> None:
        best_det, best_score = find_best_detection(msg.detections, self._target_class)
        if best_det is not None:
            bbox = best_det.bbox
            self._target_u = bbox.center.x
            self._target_v = bbox.center.y
            self._bbox_size_x = bbox.size_x
            self._bbox_size_y = bbox.size_y
            self._last_detect_time = time.time()
            self._update_target_distance()
        else:
            self._target_u = None
            self._target_v = None
            self._target_distance = None

    # ==================================================================
    #  深度图回调
    # ==================================================================
    def _on_depth(self, msg: Image) -> None:
        """缓存最新深度图，支持 16UC1(mm) 和 32FC1(m)。"""
        try:
            self._depth_image = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            self._depth_encoding = msg.encoding
        except Exception as e:
            self.get_logger().warn(f"深度图转换失败: {e}")

    def _update_target_distance(self) -> None:
        """按检测框中心区域计算目标距离。"""
        self._target_distance = None
        if not self._use_depth or self._depth_image is None:
            return
        if self._target_u is None or self._target_v is None:
            return

        raw = sample_depth_at_pixel(self._depth_image, self._target_u, self._target_v, self._depth_radius)
        if raw is None:
            return
        distance = depth_to_meters(raw, self._depth_encoding)
        if math.isfinite(distance) and distance > 0.0:
            self._target_distance = distance

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

        # ---- 4. 到达判断（深度距离优先，bbox 作为 fallback） ----
        if self._use_depth and self._target_distance is not None:
            if self._target_distance <= self._stop_distance:
                self._stop_move()
                self.get_logger().info(
                    f"到达目标! 深度距离={self._target_distance:.2f}m "
                    f"<= {self._stop_distance:.2f}m"
                )
                return

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
        if self._sit_on_exit:
            self.get_logger().info("退出: 自动坐下 (sit_on_exit=true)")
            time.sleep(0.2)
            self._sport.sit()
        else:
            self.get_logger().info("退出: 仅停止运动 (sit_on_exit=false)")
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
