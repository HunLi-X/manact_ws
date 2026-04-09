"""检测框可视化节点 — 订阅相机图像 + 检测结果，实时绘制检测框并显示。"""

import os
import sys

# ROS2 colcon 隔离 PYTHONPATH，追加系统和用户包路径
for _p in [
    "/usr/lib/python3/dist-packages",
    os.path.expanduser("~/.local/lib/python3.8/site-packages"),
]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge


# 预定义颜色表（BGR）
_COLORS = [
    (0, 255, 0), (255, 0, 0), (0, 0, 255),
    (255, 255, 0), (0, 255, 255), (255, 0, 255),
    (128, 255, 0), (0, 128, 255), (255, 128, 0),
]


def _get_color(class_id: str) -> tuple:
    """根据类别 ID 返回固定颜色。"""
    idx = hash(class_id) % len(_COLORS)
    return _COLORS[idx]


class DetectionVisualizerNode(Node):
    """订阅图像和检测结果，叠加检测框后通过 cv2 显示。"""

    def __init__(self) -> None:
        super().__init__("g1_detection_visualizer_node")

        # ---- 参数 ----
        self.declare_parameter("image_topic", "/robot1/D455_1/color/image_raw")
        self.declare_parameter("detection_topic", "/g1/vision/detections")

        # ---- CV Bridge ----
        self._bridge = CvBridge()

        # ---- QoS（与相机发布端一致） ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        image_topic = self.get_parameter("image_topic").value
        det_topic = self.get_parameter("detection_topic").value

        self._image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, sensor_qos
        )
        self._det_sub = self.create_subscription(
            Detection2DArray, det_topic, self._detection_callback, 10
        )

        # ---- 缓存 ----
        self._cv_image = None
        self._detections: Detection2DArray | None = None

        self.get_logger().info(
            f"可视化节点启动: 图像={image_topic}, 检测={det_topic}, 按 q 退出"
        )

    def _image_callback(self, msg: Image) -> None:
        """缓存最新图像并触发绘制。"""
        try:
            self._cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        self._draw()

    def _detection_callback(self, msg: Detection2DArray) -> None:
        """缓存最新检测结果。"""
        self._detections = msg

    def _draw(self) -> None:
        """在图像上绘制检测框并显示。"""
        if self._cv_image is None:
            return

        frame = self._cv_image.copy()
        h, w = frame.shape[:2]

        if self._detections is not None:
            for det in self._detections.detections:
                if not det.results:
                    continue

                class_id = det.results[0].id
                score = det.results[0].score
                color = _get_color(class_id)

                # 归一化坐标 → 像素坐标
                cx = det.bbox.center.position.x * w
                cy = det.bbox.center.position.y * h
                bw = det.bbox.size_x * w
                bh = det.bbox.size_y * h
                x1 = int(cx - bw / 2)
                y1 = int(cy - bh / 2)
                x2 = int(cx + bw / 2)
                y2 = int(cy + bh / 2)

                # 绘制边框
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                # 绘制标签
                label = f"{class_id} {score:.0%}"
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.6
                thickness = 1
                (tw, th), baseline = cv2.getTextSize(label, font, font_scale, thickness)

                # 标签背景
                cv2.rectangle(
                    frame, (x1, y1 - th - 6), (x1 + tw, y1), color, -1
                )
                cv2.putText(
                    frame, label, (x1, y1 - 4),
                    font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA
                )

        cv2.imshow("G1 YOLO Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("用户退出可视化")
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
