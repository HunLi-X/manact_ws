#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
检测框可视化节点 (tkinter)
==========================
订阅相机图像 + 检测结果，绘制检测框：
  - 左侧：原始相机图像
  - 右侧：YOLO 检测标注图像
  - 同时发布标注图像到 ROS 话题（兼容 rviz2/rqt）

相比 cv2.imshow 方案，tkinter 无需 DISPLAY 环境变量，SSH 下也能正常显示。

运行：
    ros2 run g1_yolo_nav_py detection_visualizer

依赖：
    pip install pillow
"""

# ==================================================================
# 1. 标准库导入
# ==================================================================
import threading
import time
from typing import Optional

# ==================================================================
# 2. 第三方库与 ROS2 导入
# ==================================================================
import tkinter as tk
import numpy as np
import cv2
from PIL import Image as PILImage, ImageTk

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
    idx = hash(class_id) % len(_COLORS)
    return _COLORS[idx]


class DetectionVisualizerNode(Node):
    """订阅图像和检测结果，叠加检测框后 tkinter 显示 + 话题发布。"""

    def __init__(self) -> None:
        super().__init__("g1_detection_visualizer_node")

        # ---- 参数 ----
        self.declare_parameter("image_topic", "/D455_1/color/image_raw")
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("annotated_topic", "/g1/vision/annotated_image")
        self.declare_parameter("display_width", 400)
        self.declare_parameter("display_height", 300)

        image_topic = self.get_parameter("image_topic").value
        det_topic = self.get_parameter("detection_topic").value
        annotated_topic = self.get_parameter("annotated_topic").value
        self._disp_w = int(self.get_parameter("display_width").value)
        self._disp_h = int(self.get_parameter("display_height").value)

        # ---- CV Bridge ----
        self._bridge = CvBridge()

        # ---- QoS ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )

        # ---- ROS2 订阅 ----
        self.create_subscription(Image, image_topic, self._image_cb, sensor_qos)
        self.create_subscription(Detection2DArray, det_topic, self._detection_cb, 10)

        # ---- ROS2 发布 ----
        self._pub = self.create_publisher(Image, annotated_topic, pub_qos)

        # ---- 缓存 ----
        self._cv_image: Optional[np.ndarray] = None
        self._detections: Optional[Detection2DArray] = None
        self._pub_count = 0
        self._running = True
        self._fps = 0.0
        self._frame_count = 0
        self._fps_time = time.time()

        # ---- tkinter GUI ----
        self._build_gui()
        self._update_loop()

        self.get_logger().info(
            f"可视化节点启动: 图像={image_topic}, 检测={det_topic}, "
            f"输出={annotated_topic}"
        )

    # ==================================================================
    # GUI 构建
    # ==================================================================
    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("G1 YOLO Detection Visualizer")
        self.root.configure(bg="#1e1e1e")
        self.root.geometry("860x440")
        self.root.minsize(640, 360)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # ---- 顶部标题栏 ----
        title_frame = tk.Frame(self.root, bg="#2d2d2d", height=32)
        title_frame.pack(fill=tk.X)
        title_frame.pack_propagate(False)
        tk.Label(
            title_frame, text="G1 YOLO 检测可视化",
            font=("Arial", 12, "bold"), fg="#00d4ff", bg="#2d2d2d"
        ).pack(side=tk.LEFT, padx=10, pady=4)

        self._fps_label = tk.Label(
            title_frame, text="FPS: --", font=("Consolas", 10),
            fg="#aaaaaa", bg="#2d2d2d"
        )
        self._fps_label.pack(side=tk.RIGHT, padx=10, pady=4)

        self._det_label = tk.Label(
            title_frame, text="检测: --", font=("Consolas", 10),
            fg="#ffaa00", bg="#2d2d2d"
        )
        self._det_label.pack(side=tk.RIGHT, padx=10, pady=4)

        # ---- 图像区域 ----
        img_frame = tk.Frame(self.root, bg="#1e1e1e")
        img_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 左侧：原始图像
        left_frame = tk.LabelFrame(
            img_frame, text=" 原始图像 ", font=("Arial", 10),
            fg="#cccccc", bg="#1e1e1e", labelanchor="n"
        )
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 2))
        self._raw_canvas = tk.Canvas(
            left_frame, width=self._disp_w, height=self._disp_h,
            bg="#000000", highlightthickness=0
        )
        self._raw_canvas.pack(fill=tk.BOTH, expand=True, padx=3, pady=3)

        # 右侧：检测图像
        right_frame = tk.LabelFrame(
            img_frame, text=" 检测结果 ", font=("Arial", 10),
            fg="#cccccc", bg="#1e1e1e", labelanchor="n"
        )
        right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(2, 0))
        self._det_canvas = tk.Canvas(
            right_frame, width=self._disp_w, height=self._disp_h,
            bg="#000000", highlightthickness=0
        )
        self._det_canvas.pack(fill=tk.BOTH, expand=True, padx=3, pady=3)

        # ---- 底部状态栏 ----
        status_frame = tk.Frame(self.root, bg="#2d2d2d", height=24)
        status_frame.pack(fill=tk.X, padx=5, pady=(0, 5))
        status_frame.pack_propagate(False)
        self._status_label = tk.Label(
            status_frame, text="等待图像...",
            font=("Consolas", 9), fg="#00ff88", bg="#2d2d2d", anchor="w"
        )
        self._status_label.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)

    # ==================================================================
    # ROS2 回调
    # ==================================================================
    def _image_cb(self, msg: Image):
        try:
            self._cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._frame_count += 1
        except Exception as e:
            self.get_logger().warn(f"图像转换失败: {e}")
            return
        self._publish_annotated(msg.header)

    def _detection_cb(self, msg: Detection2DArray):
        self._detections = msg

    # ==================================================================
    # 图像绘制
    # ==================================================================
    def _draw_on_frame(self, frame: np.ndarray) -> np.ndarray:
        """在图像上绘制检测框。"""
        if self._detections is None:
            return frame
        h, w = frame.shape[:2]
        det_count = 0
        for det in self._detections.detections:
            if not det.results:
                continue
            class_id = det.results[0].id
            score = det.results[0].score
            color = _get_color(class_id)

            cx = det.bbox.center.position.x * w
            cy = det.bbox.center.position.y * h
            bw = det.bbox.size_x * w
            bh = det.bbox.size_y * h
            x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
            x2, y2 = int(cx + bw / 2), int(cy + bh / 2)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            label = f"{class_id} {score:.0%}"
            font = cv2.FONT_HERSHEY_SIMPLEX
            (tw, th), _ = cv2.getTextSize(label, font, 0.6, 1)
            cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw, y1), color, -1)
            cv2.putText(frame, label, (x1, y1 - 4),
                        font, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            det_count += 1
        return frame

    def _cv2_to_tk(self, frame: np.ndarray, canvas: tk.Canvas) -> Optional[ImageTk.PhotoImage]:
        """将 OpenCV 图像转为 tkinter 可显示格式并缩放适配画布。"""
        cw = canvas.winfo_width() or self._disp_w
        ch = canvas.winfo_height() or self._disp_h
        if cw < 2 or ch < 2:
            cw, ch = self._disp_w, self._disp_h

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(rgb)
        pil_img = pil_img.resize((cw, ch), PILImage.LANCZOS)
        return ImageTk.PhotoImage(image=pil_img)

    # ==================================================================
    # 标注图像发布
    # ==================================================================
    def _publish_annotated(self, header) -> None:
        """绘制检测框并发布标注图像到话题。"""
        if self._cv_image is None:
            return
        frame = self._cv_image.copy()
        frame = self._draw_on_frame(frame)
        annotated_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        annotated_msg.header = header
        self._pub.publish(annotated_msg)
        self._pub_count += 1

    # ==================================================================
    # GUI 刷新
    # ==================================================================
    def _update_loop(self):
        """定时刷新 GUI（约 20Hz）。"""
        if not self._running:
            return

        try:
            self._do_update()
        except tk.TclError:
            self._running = False
            return

        if self._running:
            self.root.after(50, self._update_loop)

    def _do_update(self):
        # FPS 计算
        now = time.time()
        elapsed = now - self._fps_time
        if elapsed >= 1.0:
            self._fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_time = now
            self._fps_label.config(text=f"FPS: {self._fps:.0f}")

        if self._cv_image is not None:
            # 原始图像
            self._raw_photo = self._cv2_to_tk(self._cv_image.copy(), self._raw_canvas)
            if self._raw_photo:
                self._raw_canvas.delete("all")
                self._raw_canvas.create_image(0, 0, anchor=tk.NW, image=self._raw_photo)

            # 检测标注图像
            det_frame = self._draw_on_frame(self._cv_image.copy())
            self._det_photo = self._cv2_to_tk(det_frame, self._det_canvas)
            if self._det_photo:
                self._det_canvas.delete("all")
                self._det_canvas.create_image(0, 0, anchor=tk.NW, image=self._det_photo)

            # 检测信息
            det_count = len(self._detections.detections) if self._detections else 0
            if det_count > 0:
                best = max(
                    self._detections.detections,
                    key=lambda d: d.results[0].score if d.results else 0
                )
                if best.results:
                    self._det_label.config(
                        text=f"检测: {best.results[0].id} ({best.results[0].score:.0%}) x{det_count}"
                    )
            else:
                self._det_label.config(text="检测: 无目标")

            self._status_label.config(
                text=f"已发布 {self._pub_count} 帧 | {self._cv_image.shape[1]}x{self._cv_image.shape[0]}",
                fg="#00ff88"
            )
        else:
            self._status_label.config(text="等待图像...", fg="#aaaaaa")

    def _on_close(self):
        self._running = False
        try:
            self.root.destroy()
        except Exception:
            pass

    def run(self):
        self.root.mainloop()

    def destroy_node(self) -> None:
        self.get_logger().info("[清理] 可视化节点已停止")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectionVisualizerNode()

    # ROS2 spin 在后台线程
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()  # tkinter 主循环（阻塞）
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
