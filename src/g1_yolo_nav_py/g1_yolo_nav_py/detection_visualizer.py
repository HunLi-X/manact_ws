"""检测框可视化节点 — 订阅相机图像 + 检测结果，绘制检测框并发布为 ROS 图像话题。

同时尝试弹出 cv2 窗口（SSH 远程需 export DISPLAY=:0），
如无显示环境则自动降级为纯话题发布模式。
"""

import os
import sys

# ROS2 colcon 隔离 PYTHONPATH，追加系统和用户包路径
for _p in [
    "/usr/lib/python3/dist-packages",
    os.path.expanduser("~/.local/lib/python3.8/site-packages"),
    "/usr/local/lib/python3.8/dist-packages",
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
    """订阅图像和检测结果，叠加检测框后发布话题 + 可选窗口显示。"""

    def __init__(self) -> None:
        super().__init__("g1_detection_visualizer_node")

        # ---- 参数 ----
        self.declare_parameter("image_topic", "/robot1/D455_1/color/image_raw")
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("annotated_topic", "/g1/vision/annotated_image")
        self.declare_parameter("display", True)

        # ---- CV Bridge ----
        self._bridge = CvBridge()

        # ---- 显示环境检测 ----
        want_display = self.get_parameter("display").value
        if want_display and os.environ.get("DISPLAY"):
            self._display = True
        else:
            self._display = False
            if want_display:
                self.get_logger().warn(
                    "DISPLAY 未设置，降级为纯话题发布。"
                    "SSH 远程请: export DISPLAY=:0 && sudo xhost +local:"
                )

        # ---- QoS ----
        # 订阅相机用 BEST_EFFORT（与相机发布端一致）
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # 发布标注图像用 RELIABLE（兼容 rviz2/rqt 默认订阅）
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        image_topic = self.get_parameter("image_topic").value
        det_topic = self.get_parameter("detection_topic").value
        annotated_topic = self.get_parameter("annotated_topic").value

        self._image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, sensor_qos
        )
        self._det_sub = self.create_subscription(
            Detection2DArray, det_topic, self._detection_callback, 10
        )
        self._pub = self.create_publisher(Image, annotated_topic, pub_qos)

        # ---- 缓存 ----
        self._cv_image = None
        self._detections = None
        self._pub_count = 0

        # ---- 窗口刷新定时器（独立于图像回调，确保窗口不卡死）----
        if self._display:
            cv2.namedWindow("G1 YOLO Detection", cv2.WINDOW_NORMAL)
            self._window_timer = self.create_timer(0.033, self._window_tick)

        view_hint = (
            "窗口 + 话题" if self._display
            else f"话题 {annotated_topic} (rviz2/rqt_image_view 查看)"
        )
        self.get_logger().info(
            f"可视化节点启动: 图像={image_topic}, 检测={det_topic}, "
            f"输出={view_hint}, DISPLAY={os.environ.get('DISPLAY', '未设置')}"
        )

    def _image_callback(self, msg: Image) -> None:
        """缓存最新图像并发布标注结果。"""
        try:
            self._cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"图像转换失败: {e}")
            return
        self._publish_annotated(msg.header)

    def _detection_callback(self, msg: Detection2DArray) -> None:
        """缓存最新检测结果。"""
        self._detections = msg

    def _draw_on_frame(self, frame):
        """在图像上绘制检测框，返回标注后的图像。"""
        if self._detections is None:
            return frame
        h, w = frame.shape[:2]
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
        return frame

    def _publish_annotated(self, header) -> None:
        """绘制检测框并发布标注图像到话题。"""
        if self._cv_image is None:
            return

        frame = self._cv_image.copy()
        frame = self._draw_on_frame(frame)

        # 发布到话题
        annotated_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        annotated_msg.header = header
        self._pub.publish(annotated_msg)
        self._pub_count += 1
        if self._pub_count <= 3:
            self.get_logger().info(
                f"发布标注图像 #{self._pub_count}: "
                f"size={frame.shape[1]}x{frame.shape[0]}, "
                f"订阅者={self._pub.get_subscription_count()}"
            )

    def _window_tick(self) -> None:
        """定时刷新 cv2 窗口（30Hz，独立于图像回调）。"""
        if self._cv_image is None:
            return
        frame = self._cv_image.copy()
        frame = self._draw_on_frame(frame)
        cv2.imshow("G1 YOLO Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("用户退出可视化")
            rclpy.shutdown()

    def destroy_node(self) -> None:
        if self._display:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
