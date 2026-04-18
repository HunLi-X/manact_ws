#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 NavGrasp 控制面板 (tkinter)
================================
基于 tkinter 的前端控制界面：
  - 左侧：原始相机图像
  - 右侧：YOLO 检测标注图像
  - 下方：状态信息 + 操作按钮

运行：
    ros2 run g1_yolo_nav_py control_panel
    python3 -m g1_yolo_nav_py.control_panel

依赖：
    pip install pillow  (PIL 用于图像缩放显示)
"""

# ==================================================================
# 1. 标准库导入
# ==================================================================
import os
import sys
import threading
import time
from typing import Optional

# ROS2 colcon 隔离 PYTHONPATH
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
import tkinter as tk
from tkinter import ttk
import numpy as np
import cv2  # OpenCV 图像处理
from PIL import Image as PILImage, ImageTk  # PIL 用于 tkinter 图像显示

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


# 预定义颜色表 (BGR)
_COLORS = [
    (0, 255, 0), (255, 0, 0), (0, 0, 255),
    (255, 255, 0), (0, 255, 255), (255, 0, 255),
]


def _get_color(class_id: str) -> tuple:
    idx = hash(class_id) % len(_COLORS)
    return _COLORS[idx]


class ControlPanelNode(Node):
    """G1 控制面板节点 — tkinter GUI + ROS2 通信。"""

    def __init__(self) -> None:
        super().__init__("g1_control_panel_node")

        # ---- 参数 ----
        self.declare_parameter("image_topic", "/robot1/D455_1/color/image_raw")
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("display_width", 400)
        self.declare_parameter("display_height", 300)

        p = lambda n: self.get_parameter(n).value
        self._img_topic = p("image_topic")
        self._det_topic = p("detection_topic")
        self._cmd_topic = p("cmd_vel_topic")
        self._disp_w = int(p("display_width"))
        self._disp_h = int(p("display_height"))

        # ---- CV Bridge ----
        self._bridge = CvBridge()

        # ---- QoS ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )

        # ---- ROS2 订阅 ----
        self.create_subscription(Image, self._img_topic, self._image_cb, sensor_qos)
        self.create_subscription(Detection2DArray, self._det_topic, self._detection_cb, 10)

        # ---- ROS2 发布 ----
        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, 10)

        # ---- 缓存 ----
        self._raw_image: Optional[np.ndarray] = None
        self._detections: Optional[Detection2DArray] = None
        self._det_count = 0
        self._fps = 0.0
        self._frame_count = 0
        self._fps_time = time.time()
        self._running = True

        # ---- tkinter GUI ----
        self._build_gui()

        # ---- 定时刷新 ----
        self._update_loop()

        self.get_logger().info(
            f"控制面板启动: 图像={self._img_topic}, 检测={self._det_topic}"
        )

    # ==================================================================
    # GUI 构建
    # ==================================================================
    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("G1 NavGrasp 控制面板")
        self.root.configure(bg="#1e1e1e")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # ---- 顶部标题栏 ----
        title_frame = tk.Frame(self.root, bg="#2d2d2d", height=36)
        title_frame.pack(fill=tk.X, padx=0, pady=0)
        title_frame.pack_propagate(False)
        tk.Label(
            title_frame, text="G1 NavGrasp 控制面板",
            font=("Arial", 13, "bold"), fg="#00d4ff", bg="#2d2d2d"
        ).pack(side=tk.LEFT, padx=10, pady=5)

        self._fps_label = tk.Label(
            title_frame, text="FPS: --", font=("Consolas", 10),
            fg="#aaaaaa", bg="#2d2d2d"
        )
        self._fps_label.pack(side=tk.RIGHT, padx=10, pady=5)

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

        # ---- 状态信息栏 ----
        info_frame = tk.Frame(self.root, bg="#2d2d2d", height=28)
        info_frame.pack(fill=tk.X, padx=5, pady=(0, 3))
        info_frame.pack_propagate(False)
        self._status_label = tk.Label(
            info_frame, text="状态: 等待图像...",
            font=("Consolas", 10), fg="#00ff88", bg="#2d2d2d", anchor="w"
        )
        self._status_label.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)

        self._det_info_label = tk.Label(
            info_frame, text="检测: --",
            font=("Consolas", 10), fg="#ffaa00", bg="#2d2d2d"
        )
        self._det_info_label.pack(side=tk.RIGHT, padx=10)

        # ---- 控制按钮栏 ----
        btn_frame = tk.Frame(self.root, bg="#1e1e1e")
        btn_frame.pack(fill=tk.X, padx=5, pady=(0, 5))

        btn_style = {"font": ("Arial", 10), "width": 10, "height": 1, "bd": 0, "relief": "flat"}

        self._btn_forward = tk.Button(
            btn_frame, text="前进", bg="#3a7bd5", fg="white",
            activebackground="#2a5ba5", command=lambda: self._send_cmd(0.2, 0, 0),
            **btn_style
        )
        self._btn_forward.pack(side=tk.LEFT, padx=3)

        self._btn_backward = tk.Button(
            btn_frame, text="后退", bg="#3a7bd5", fg="white",
            activebackground="#2a5ba5", command=lambda: self._send_cmd(-0.2, 0, 0),
            **btn_style
        )
        self._btn_backward.pack(side=tk.LEFT, padx=3)

        self._btn_left = tk.Button(
            btn_frame, text="左转", bg="#3a7bd5", fg="white",
            activebackground="#2a5ba5", command=lambda: self._send_cmd(0, 0, 0.4),
            **btn_style
        )
        self._btn_left.pack(side=tk.LEFT, padx=3)

        self._btn_right = tk.Button(
            btn_frame, text="右转", bg="#3a7bd5", fg="white",
            activebackground="#2a5ba5", command=lambda: self._send_cmd(0, 0, -0.4),
            **btn_style
        )
        self._btn_right.pack(side=tk.LEFT, padx=3)

        self._btn_stop = tk.Button(
            btn_frame, text="停止", bg="#d9534f", fg="white",
            activebackground="#b5352f", command=lambda: self._send_cmd(0, 0, 0),
            **btn_style
        )
        self._btn_stop.pack(side=tk.LEFT, padx=3)

        # ---- 速度控制 ----
        speed_frame = tk.Frame(btn_frame, bg="#1e1e1e")
        speed_frame.pack(side=tk.RIGHT, padx=5)
        tk.Label(speed_frame, text="速度:", font=("Arial", 9),
                 fg="#aaaaaa", bg="#1e1e1e").pack(side=tk.LEFT)
        self._speed_var = tk.DoubleVar(value=0.2)
        speed_scale = tk.Scale(
            speed_frame, from_=0.05, to=0.5, resolution=0.05,
            orient=tk.HORIZONTAL, variable=self._speed_var,
            length=100, bg="#1e1e1e", fg="#cccccc",
            highlightthickness=0, troughcolor="#3a3a3a"
        )
        speed_scale.pack(side=tk.LEFT)

    # ==================================================================
    # ROS2 回调
    # ==================================================================
    def _image_cb(self, msg: Image):
        try:
            self._raw_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._frame_count += 1
        except Exception:
            pass

    def _detection_cb(self, msg: Detection2DArray):
        self._detections = msg
        self._det_count = len(msg.detections)

    # ==================================================================
    # 图像绘制
    # ==================================================================
    def _draw_detections(self, frame: np.ndarray) -> np.ndarray:
        """在图像上绘制检测框。"""
        if self._detections is None:
            return frame
        h, w = frame.shape[:2]
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
            cv2.rectangle(frame, (x1, y1 - 22), (x1 + len(label) * 10, y1), color, -1)
            cv2.putText(frame, label, (x1 + 3, y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
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
    # 按钮操作
    # ==================================================================
    def _send_cmd(self, vx: float, vy: float, vz: float):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = vz
        self._cmd_pub.publish(cmd)
        if vx == 0 and vy == 0 and vz == 0:
            self._status_label.config(text="状态: 已停止", fg="#ff6666")
        else:
            self._status_label.config(
                text=f"状态: vx={vx:.1f} vy={vy:.1f} vz={vz:.1f}", fg="#00ff88"
            )

    # ==================================================================
    # 主循环
    # ==================================================================
    def _update_loop(self):
        """定时刷新 GUI（约 20Hz）。"""
        if not self._running:
            return

        # FPS 计算
        now = time.time()
        elapsed = now - self._fps_time
        if elapsed >= 1.0:
            self._fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_time = now
            self._fps_label.config(text=f"FPS: {self._fps:.0f}")

        # 更新图像
        if self._raw_image is not None:
            # 原始图像
            raw_frame = self._raw_image.copy()
            self._raw_photo = self._cv2_to_tk(raw_frame, self._raw_canvas)
            if self._raw_photo:
                self._raw_canvas.delete("all")
                self._raw_canvas.create_image(0, 0, anchor=tk.NW, image=self._raw_photo)

            # 检测标注图像
            det_frame = self._draw_detections(self._raw_image.copy())
            self._det_photo = self._cv2_to_tk(det_frame, self._det_canvas)
            if self._det_photo:
                self._det_canvas.delete("all")
                self._det_canvas.create_image(0, 0, anchor=tk.NW, image=self._det_photo)

            # 状态信息
            if self._detections and self._det_count > 0:
                best = max(
                    self._detections.detections,
                    key=lambda d: d.results[0].score if d.results else 0
                )
                if best.results:
                    name = best.results[0].id
                    score = best.results[0].score
                    self._det_info_label.config(
                        text=f"检测: {name} ({score:.0%}) x{self._det_count}"
                    )
            else:
                self._det_info_label.config(text="检测: 无目标")
        else:
            self._status_label.config(text="状态: 等待图像...", fg="#aaaaaa")

        # 下一帧
        self.root.after(50, self._update_loop)

    def _on_close(self):
        self._running = False
        self._send_cmd(0, 0, 0)  # 停止运动
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = ControlPanelNode()

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
