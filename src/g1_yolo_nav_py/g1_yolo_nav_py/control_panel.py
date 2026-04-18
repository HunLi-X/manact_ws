#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 NavGrasp 控制面板 (tkinter)
================================
基于 tkinter 的前端控制界面，集成 grasp_task 全部能力：
  - 左侧：原始相机图像
  - 右侧：YOLO 检测标注图像
  - 下方：状态信息 + 操作按钮 + 一键抓取任务

状态机（参考 grasp_task.py）：
    SEARCHING   → 旋转搜索目标
    ALIGNING    → 偏航对齐让目标居中
    APPROACHING → 前进到目标附近
    GRABBING    → 执行 armup.py 抓取
    MENU        → 交互菜单（放下/右转放下）

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
import subprocess
import threading
import time
import math
from pathlib import Path
from typing import Optional
from enum import Enum, auto

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
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# 注意：control_panel 不导入 unitree_sdk2py！
# unitree_sdk2py 的模块级导入会加载 CycloneDDS 绑定，干扰 ROS2 的 CycloneDDS domain，
# 导致 ROS2 订阅收不到任何数据（原始图像加载不出来）。
# 前进/右转通过 cmd_vel → twist_bridge → Sport API 完成，无需 LocoClient。

# ==================================================================
# 3. 常量
# ==================================================================

# 预定义颜色表 (BGR)
_COLORS = [
    (0, 255, 0), (255, 0, 0), (0, 0, 255),
    (255, 255, 0), (0, 255, 255), (255, 0, 255),
]

# arm 脚本默认目录
_DEFAULT_ARM_DIR = os.path.expanduser("~/g1act_ws/manact_ws/src/g1_yolo_nav_py/arm")

# 状态枚举
class State(Enum):
    IDLE = auto()
    SEARCHING = auto()
    ALIGNING = auto()
    APPROACHING = auto()
    GRABBING = auto()
    MENU = auto()

# 状态中文显示
_STATE_LABELS = {
    State.IDLE: "空闲",
    State.SEARCHING: "搜索中",
    State.ALIGNING: "对齐中",
    State.APPROACHING: "接近中",
    State.GRABBING: "抓取中",
    State.MENU: "操作菜单",
}

# 状态对应颜色
_STATE_COLORS = {
    State.IDLE: "#aaaaaa",
    State.SEARCHING: "#ffaa00",
    State.ALIGNING: "#00d4ff",
    State.APPROACHING: "#3a7bd5",
    State.GRABBING: "#ff6666",
    State.MENU: "#00ff88",
}


def _get_color(class_id: str) -> tuple:
    idx = hash(class_id) % len(_COLORS)
    return _COLORS[idx]


class ControlPanelNode(Node):
    """G1 控制面板节点 — tkinter GUI + ROS2 通信 + 抓取任务状态机。"""

    def __init__(self) -> None:
        super().__init__("g1_control_panel_node")

        # ---- 参数（参考 grasp_task.py）----
        self.declare_parameter("image_topic", "/D455_1/color/image_raw")
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("target_class_id", "chair")
        self.declare_parameter("camera_fov_deg", 87.0)
        self.declare_parameter("yaw_kp", 2.0)
        self.declare_parameter("max_yaw_speed", 0.6)
        self.declare_parameter("center_tolerance", 0.08)
        self.declare_parameter("forward_speed", 0.2)
        self.declare_parameter("arrive_bbox_ratio", 0.45)
        self.declare_parameter("align_stable_time", 1.0)
        self.declare_parameter("lost_timeout", 2.0)
        self.declare_parameter("search_yaw_speed", 0.3)
        self.declare_parameter("network_interface", "")
        self.declare_parameter("arm_script_dir", _DEFAULT_ARM_DIR)
        self.declare_parameter("display_width", 400)
        self.declare_parameter("display_height", 300)

        p = lambda n: self.get_parameter(n).value
        self._img_topic = p("image_topic")
        self._det_topic = p("detection_topic")
        self._cmd_topic = p("cmd_vel_topic")
        self._target_class = p("target_class_id")
        self._fov_rad = math.radians(float(p("camera_fov_deg")))
        self._kp = float(p("yaw_kp"))
        self._max_yaw = float(p("max_yaw_speed"))
        self._center_tol = float(p("center_tolerance"))
        self._fwd_speed = float(p("forward_speed"))
        self._arrive_ratio = float(p("arrive_bbox_ratio"))
        self._stable_time = float(p("align_stable_time"))
        self._lost_timeout = float(p("lost_timeout"))
        self._search_speed = float(p("search_yaw_speed"))
        self._net_iface = p("network_interface")
        self._disp_w = int(p("display_width"))
        self._disp_h = int(p("display_height"))

        # arm 脚本路径
        arm_dir = p("arm_script_dir")
        self._armup_script = Path(arm_dir) / "armup.py"
        self._armdown_script = Path(arm_dir) / "armdown.py"

        # ---- CV Bridge ----
        self._bridge = CvBridge()

        # ---- QoS ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )

        # ---- ROS2 订阅 ----
        self._img_sub = self.create_subscription(
            Image, self._img_topic, self._image_cb, sensor_qos
        )
        self._det_sub = self.create_subscription(
            Detection2DArray, self._det_topic, self._detection_cb, 10
        )

        # ---- ROS2 发布 ----
        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, 10)

        # ---- 延迟诊断（2秒后检查订阅状态）----
        self._diag_timer = self.create_timer(2.0, self._diag_check)

        # ---- 缓存 ----
        self._raw_image: Optional[np.ndarray] = None
        self._detections: Optional[Detection2DArray] = None
        self._det_count = 0
        self._fps = 0.0
        self._frame_count = 0
        self._fps_time = time.time()
        self._running = True
        self._first_image_received = False
        self._new_frame = False  # ROS2 回调标记有新帧

        # ---- 状态机（参考 grasp_task.py）----
        self._target_u = None
        self._bbox_size_x = 0.0
        self._bbox_size_y = 0.0
        self._last_detect_time = 0.0
        self._align_start = None
        self._state = State.IDLE
        self._log_text = ""  # GUI 日志

        # ---- tkinter GUI ----
        self._build_gui()

        # ---- 状态机定时器（10Hz）----
        self._timer = self.create_timer(0.1, self._tick)

        # ---- GUI 刷新 ----
        self._update_loop()

        self.get_logger().info("=" * 50)
        self.get_logger().info("G1 NavGrasp 控制面板启动")
        self.get_logger().info(f"目标类别: {self._target_class}")
        self.get_logger().info(f"图像: {self._img_topic}, 检测: {self._det_topic}")
        self.get_logger().info(f"armup: {self._armup_script}")
        self.get_logger().info(f"armdown: {self._armdown_script}")
        self.get_logger().info("=" * 50)

    # ==================================================================
    # cmd_vel 发布
    # ==================================================================
    def _publish_cmd(self, vx=0.0, vy=0.0, vz=0.0) -> None:
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = vz
        self._cmd_pub.publish(cmd)

    def _publish_stop(self) -> None:
        self._publish_cmd(0.0, 0.0, 0.0)

    # ==================================================================
    # 诊断
    # ==================================================================
    def _diag_check(self):
        """启动后延迟检查订阅状态，帮助定位无法接收图像的问题。"""
        # 只执行一次
        if hasattr(self, '_diag_timer') and self._diag_timer is not None:
            self._diag_timer.cancel()
            self._diag_timer = None
        img_pub_count = self.count_publishers(self._img_topic)
        det_pub_count = self.count_publishers(self._det_topic)
        self.get_logger().info(
            f"[诊断] 图像话题 '{self._img_topic}': "
            f"发布者数={img_pub_count}, "
            f"QoS=BEST_EFFORT/depth=5"
        )
        self.get_logger().info(
            f"[诊断] 检测话题 '{self._det_topic}': "
            f"发布者数={det_pub_count}"
        )
        if img_pub_count == 0:
            self.get_logger().warn(
                f"[诊断] 图像话题无发布者! "
                f"请检查: 1) 相机驱动是否启动 2) 话题名是否正确 "
                f"(用 ros2 topic list 查看)"
            )
            # 尝试列出可用图像话题
            try:
                import subprocess
                result = subprocess.run(
                    ["ros2", "topic", "list"],
                    capture_output=True, text=True, timeout=3
                )
                img_topics = [
                    l for l in result.stdout.splitlines()
                    if "image" in l.lower() or "color" in l.lower()
                ]
                if img_topics:
                    self.get_logger().warn(
                        f"[诊断] 发现可能的图像话题: {img_topics}"
                    )
            except Exception:
                pass

    # ==================================================================
    # GUI 构建
    # ==================================================================
    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("G1 NavGrasp 控制面板")
        self.root.configure(bg="#1e1e1e")
        self.root.geometry("880x620")
        self.root.minsize(640, 480)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # ---- 顶部标题栏 ----
        title_frame = tk.Frame(self.root, bg="#2d2d2d", height=36)
        title_frame.pack(fill=tk.X, padx=0, pady=0)
        title_frame.pack_propagate(False)
        tk.Label(
            title_frame, text="G1 NavGrasp 控制面板",
            font=("Arial", 13, "bold"), fg="#00d4ff", bg="#2d2d2d"
        ).pack(side=tk.LEFT, padx=10, pady=5)

        self._state_label = tk.Label(
            title_frame, text="[ 空闲 ]",
            font=("Arial", 12, "bold"), fg="#aaaaaa", bg="#2d2d2d"
        )
        self._state_label.pack(side=tk.LEFT, padx=10, pady=5)

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
        btn_frame.pack(fill=tk.X, padx=5, pady=(0, 3))

        btn_style = {"font": ("Arial", 10), "width": 8, "height": 1, "bd": 0, "relief": "flat"}

        # 方向控制
        self._btn_forward = tk.Button(
            btn_frame, text="前进", bg="#3a7bd5", fg="white",
            activebackground="#2a5ba5", command=lambda: self._manual_cmd(0.2, 0.0, 0.0),
            **btn_style
        )
        self._btn_forward.pack(side=tk.LEFT, padx=2)

        self._btn_backward = tk.Button(
            btn_frame, text="后退", bg="#3a7bd5", fg="white",
            activebackground="#2a5ba5", command=lambda: self._manual_cmd(-0.2, 0.0, 0.0),
            **btn_style
        )
        self._btn_backward.pack(side=tk.LEFT, padx=2)

        self._btn_left = tk.Button(
            btn_frame, text="左转", bg="#3a7bd5", fg="white",
            activebackground="#2a5ba5", command=lambda: self._manual_cmd(0.0, 0.0, 0.4),
            **btn_style
        )
        self._btn_left.pack(side=tk.LEFT, padx=2)

        self._btn_right = tk.Button(
            btn_frame, text="右转", bg="#3a7bd5", fg="white",
            activebackground="#2a5ba5", command=lambda: self._manual_cmd(0.0, 0.0, -0.4),
            **btn_style
        )
        self._btn_right.pack(side=tk.LEFT, padx=2)

        self._btn_stop = tk.Button(
            btn_frame, text="停止", bg="#d9534f", fg="white",
            activebackground="#b5352f", command=self._do_stop,
            **btn_style
        )
        self._btn_stop.pack(side=tk.LEFT, padx=2)

        # 分隔线
        tk.Frame(btn_frame, width=2, bg="#555555").pack(
            side=tk.LEFT, fill=tk.Y, padx=6, pady=2
        )

        # 任务按钮
        self._btn_search = tk.Button(
            btn_frame, text="搜索", bg="#e67e22", fg="white",
            activebackground="#c0691e", command=self._do_start_search,
            **btn_style
        )
        self._btn_search.pack(side=tk.LEFT, padx=2)

        self._btn_grab = tk.Button(
            btn_frame, text="抓取", bg="#e74c3c", fg="white",
            activebackground="#c0392b", command=self._do_grab,
            **btn_style
        )
        self._btn_grab.pack(side=tk.LEFT, padx=2)

        self._btn_putdown = tk.Button(
            btn_frame, text="放下", bg="#27ae60", fg="white",
            activebackground="#1e8449", command=self._do_put_down,
            **btn_style
        )
        self._btn_putdown.pack(side=tk.LEFT, padx=2)

        self._btn_turn_putdown = tk.Button(
            btn_frame, text="右转放下", bg="#27ae60", fg="white",
            activebackground="#1e8449", command=self._do_turn_and_put_down,
            **btn_style
        )
        self._btn_turn_putdown.pack(side=tk.LEFT, padx=2)

        # 速度控制
        speed_frame = tk.Frame(btn_frame, bg="#1e1e1e")
        speed_frame.pack(side=tk.RIGHT, padx=5)
        tk.Label(speed_frame, text="速度:", font=("Arial", 9),
                 fg="#aaaaaa", bg="#1e1e1e").pack(side=tk.LEFT)
        self._speed_var = tk.DoubleVar(value=0.2)
        speed_scale = tk.Scale(
            speed_frame, from_=0.05, to=0.5, resolution=0.05,
            orient=tk.HORIZONTAL, variable=self._speed_var,
            length=80, bg="#1e1e1e", fg="#cccccc",
            highlightthickness=0, troughcolor="#3a3a3a"
        )
        speed_scale.pack(side=tk.LEFT)

        # ---- 日志栏 ----
        log_frame = tk.LabelFrame(
            self.root, text=" 日志 ", font=("Arial", 9),
            fg="#888888", bg="#1e1e1e", labelanchor="nw", height=80
        )
        log_frame.pack(fill=tk.X, padx=5, pady=(0, 5))
        log_frame.pack_propagate(False)

        self._log_textbox = tk.Text(
            log_frame, height=4, bg="#111111", fg="#00ff88",
            font=("Consolas", 9), bd=0, wrap=tk.WORD,
            state=tk.DISABLED, insertbackground="#00ff88"
        )
        scrollbar = tk.Scrollbar(log_frame, command=self._log_textbox.yview)
        self._log_textbox.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self._log_textbox.pack(fill=tk.BOTH, expand=True, padx=3, pady=3)

    # ==================================================================
    # GUI 日志
    # ==================================================================
    def _append_log(self, text: str):
        """向日志栏追加一行文本。"""
        def _update():
            self._log_textbox.config(state=tk.NORMAL)
            self._log_textbox.insert(tk.END, text + "\n")
            self._log_textbox.see(tk.END)
            # 保留最近 200 行
            lines = int(self._log_textbox.index("end-1c").split(".")[0])
            if lines > 200:
                self._log_textbox.delete("1.0", f"{lines - 200}.0")
            self._log_textbox.config(state=tk.DISABLED)

        if threading.current_thread() is threading.main_thread():
            _update()
        else:
            self.root.after(0, _update)

    def _update_state_display(self):
        """更新顶部状态显示。"""
        label = _STATE_LABELS.get(self._state, "未知")
        color = _STATE_COLORS.get(self._state, "#aaaaaa")
        self._state_label.config(text=f"[ {label} ]", fg=color)

    # ==================================================================
    # ROS2 回调
    # ==================================================================
    def _image_cb(self, msg: Image):
        try:
            self._raw_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._frame_count += 1
            self._new_frame = True
            if not self._first_image_received:
                self._first_image_received = True
                self.get_logger().info(
                    f"[图像] 首次收到图像: {msg.width}x{msg.height}, "
                    f"encoding={msg.encoding}"
                )
        except Exception as e:
            self.get_logger().warn(f"[图像] 转换失败: {e}")

    def _detection_cb(self, msg: Detection2DArray):
        self._detections = msg
        self._det_count = len(msg.detections)

        # 过滤目标类别（参考 grasp_task.py _on_detection）
        best_det = None
        best_score = 0.0
        for det in msg.detections:
            if det.results and det.results[0].id == self._target_class:
                if det.results[0].score > best_score:
                    best_score = det.results[0].score
                    best_det = det

        now = time.time()
        if best_det is not None:
            bbox = best_det.bbox
            self._target_u = bbox.center.x
            self._bbox_size_x = bbox.size_x
            self._bbox_size_y = bbox.size_y
            self._last_detect_time = now
        else:
            self._target_u = None

    # ==================================================================
    # 图像绘制
    # ==================================================================
    def _draw_detections(self, frame: np.ndarray) -> np.ndarray:
        """在图像上绘制检测框（返回新图像，不修改传入的 frame）。"""
        out = frame.copy() if frame is self._raw_image else frame
        if self._detections is None:
            return out
        h, w = out.shape[:2]
        for det in self._detections.detections:
            if not det.results:
                continue
            class_id = det.results[0].id
            score = det.results[0].score
            color = _get_color(class_id)

            cx = det.bbox.center.x * w
            cy = det.bbox.center.y * h
            bw = det.bbox.size_x * w
            bh = det.bbox.size_y * h
            x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
            x2, y2 = int(cx + bw / 2), int(cy + bh / 2)

            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            label = f"{class_id} {score:.0%}"
            cv2.rectangle(out, (x1, y1 - 22), (x1 + len(label) * 10, y1), color, -1)
            cv2.putText(out, label, (x1 + 3, y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # 绘制十字准心
        if self._target_u is not None and self._state in (State.ALIGNING, State.APPROACHING):
            cx = int(self._target_u * w)
            cy = h // 2
            cv2.line(out, (cx - 15, cy), (cx + 15, cy), (0, 255, 255), 1)
            cv2.line(out, (cx, cy - 15), (cx, cy + 15), (0, 255, 255), 1)

        return out

    def _cv2_to_tk(self, frame: np.ndarray, canvas: tk.Canvas) -> Optional[ImageTk.PhotoImage]:
        """将 OpenCV 图像转为 tkinter 可显示格式并缩放适配画布。"""
        cw = canvas.winfo_width() or self._disp_w
        ch = canvas.winfo_height() or self._disp_h
        if cw < 2 or ch < 2:
            cw, ch = self._disp_w, self._disp_h

        # 优先使用 INTER_LINEAR（比 INTER_AREA 快），小尺寸差异时效果一致
        resized = cv2.resize(frame, (cw, ch), interpolation=cv2.INTER_LINEAR)
        # BGR→RGB + 确保内存连续
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        rgb = np.ascontiguousarray(rgb)
        pil_img = PILImage.fromarray(rgb)
        return ImageTk.PhotoImage(image=pil_img)

    # ==================================================================
    # 手动控制
    # ==================================================================
    def _manual_cmd(self, vx: float, vy: float, vz: float):
        """手动方向控制（仅 IDLE 状态下生效）。"""
        if self._state not in (State.IDLE, State.MENU):
            self._append_log("[提示] 任务执行中，请先停止")
            return
        self._publish_cmd(vx=vx, vy=vy, vz=vz)

    def _do_stop(self):
        """停止所有运动并回到 IDLE。"""
        prev = self._state
        self._state = State.IDLE
        self._publish_stop()
        self._align_start = None
        self._append_log(f"[状态] {prev.name} → IDLE: 手动停止")
        self._update_state_display()

    # ==================================================================
    # 任务操作（参考 grasp_task.py）
    # ==================================================================
    def _do_start_search(self):
        """开始搜索目标。"""
        if self._state not in (State.IDLE, State.MENU):
            self._append_log("[提示] 请先停止当前任务")
            return
        self._state = State.SEARCHING
        self._align_start = None
        self._append_log(f"[状态] → SEARCHING: 开始搜索 '{self._target_class}'")
        self._update_state_display()

    def _do_grab(self):
        """手动触发抓取（执行 armup.py）。"""
        if self._state not in (State.IDLE, State.MENU):
            self._append_log("[提示] 请先停止当前任务")
            return
        self._state = State.GRABBING
        self._publish_stop()
        self._run_grab()
        self._update_state_display()

    def _do_put_down(self):
        """放下目标物（执行 armdown.py）。"""
        if self._state not in (State.IDLE, State.MENU):
            self._append_log("[提示] 请先停止当前任务")
            return
        self._run_armdown()

    def _do_turn_and_put_down(self):
        """右转 90° 后放下目标物。"""
        if self._state not in (State.IDLE, State.MENU):
            self._append_log("[提示] 请先停止当前任务")
            return
        self._append_log("[右转] 开始右转 90° ...")

        def _worker():
            self._publish_cmd(vz=-0.6)
            time.sleep(2.6)  # ≈ 90° at 0.6 rad/s
            self._publish_stop()
            self._append_log("[右转] 右转完成")
            self._run_armdown()

        t = threading.Thread(target=_worker, daemon=True)
        t.start()

    # ==================================================================
    # armup / armdown 子进程（参考 grasp_task.py）
    # ==================================================================
    def _run_grab(self) -> None:
        """执行 armup.py 抓取目标物。"""
        script = str(self._armup_script)
        if not Path(script).exists():
            self._append_log(f"[错误] armup.py 不存在: {script}")
            self._state = State.IDLE
            self._update_state_display()
            return

        self._append_log("[抓取] 执行 armup.py ...")

        def _worker():
            try:
                args = [sys.executable, script]
                if self._net_iface:
                    args.append(self._net_iface)
                proc = subprocess.run(
                    args, check=True, input=b"\n",
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                )
                if proc.stdout:
                    for line in proc.stdout.decode(errors="replace").splitlines():
                        self._append_log(f"[armup] {line}")
                self._append_log("[抓取] armup.py 完成")
            except subprocess.CalledProcessError as e:
                self._append_log(f"[错误] armup.py 失败: 返回码={e.returncode}")
            except Exception as e:
                self._append_log(f"[错误] armup.py 异常: {e}")
            finally:
                self._state = State.MENU
                self._append_log("[状态] → MENU: 抓取完成，可选择放下")
                self.root.after(0, self._update_state_display)

        t = threading.Thread(target=_worker, daemon=True)
        t.start()

    def _run_armdown(self) -> None:
        """执行 armdown.py 放下目标物。"""
        script = str(self._armdown_script)
        if not Path(script).exists():
            self._append_log(f"[错误] armdown.py 不存在: {script}")
            return
        self._append_log("[放下] 执行 armdown.py ...")

        def _worker():
            try:
                args = [sys.executable, script]
                if self._net_iface:
                    args.append(self._net_iface)
                proc = subprocess.run(
                    args, check=True, input=b"\n",
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                )
                if proc.stdout:
                    for line in proc.stdout.decode(errors="replace").splitlines():
                        self._append_log(f"[armdown] {line}")
                self._append_log("[放下] armdown.py 完成")
            except subprocess.CalledProcessError as e:
                self._append_log(f"[错误] armdown.py 失败: 返回码={e.returncode}")
            except Exception as e:
                self._append_log(f"[错误] armdown.py 异常: {e}")
            finally:
                self._state = State.IDLE
                self._append_log("[状态] → IDLE: 放下完成")
                self.root.after(0, self._update_state_display)

        t = threading.Thread(target=_worker, daemon=True)
        t.start()

    # ==================================================================
    # 状态机 — 主循环（参考 grasp_task.py）
    # ==================================================================
    def _tick(self) -> None:
        if self._state == State.SEARCHING:
            self._tick_searching()
        elif self._state == State.ALIGNING:
            self._tick_aligning()
        elif self._state == State.APPROACHING:
            self._tick_approaching()
        # IDLE / GRABBING / MENU 不在 tick 中驱动

    def _tick_searching(self) -> None:
        """旋转搜索目标。"""
        if self._target_u is not None:
            self._state = State.ALIGNING
            self._align_start = None
            self._publish_stop()
            self._append_log("[状态] SEARCHING → ALIGNING: 目标已找到")
            self.root.after(0, self._update_state_display)
            return
        self._publish_cmd(vz=self._search_speed)

    def _tick_aligning(self) -> None:
        """偏航对齐让目标居中。"""
        now = time.time()

        # 目标丢失 → 回搜索
        if self._target_u is None or (now - self._last_detect_time > self._lost_timeout):
            self._state = State.SEARCHING
            self._publish_stop()
            self._align_start = None
            self._append_log("[状态] ALIGNING → SEARCHING: 目标丢失")
            self.root.after(0, self._update_state_display)
            return

        error = self._target_u - 0.5

        # 居中 → 切换到前进
        if abs(error) < self._center_tol:
            if self._align_start is None:
                self._align_start = now
            if now - self._align_start >= self._stable_time:
                self._state = State.APPROACHING
                self._publish_stop()
                self._append_log("[状态] ALIGNING → APPROACHING: 目标已居中")
                self.root.after(0, self._update_state_display)
                return
        else:
            self._align_start = None

        # P 控制偏航
        vyaw = self._kp * error * self._fov_rad
        vyaw = max(-self._max_yaw, min(self._max_yaw, vyaw))
        self._publish_cmd(vz=vyaw)

    def _tick_approaching(self) -> None:
        """前进到目标附近。"""
        now = time.time()

        # 目标丢失 → 回搜索
        if self._target_u is None or (now - self._last_detect_time > self._lost_timeout):
            self._state = State.SEARCHING
            self._publish_stop()
            self._append_log("[状态] APPROACHING → SEARCHING: 目标丢失")
            self.root.after(0, self._update_state_display)
            return

        # 偏离中心 → 回对齐
        if abs(self._target_u - 0.5) > self._center_tol * 2:
            self._state = State.ALIGNING
            self._publish_stop()
            self._align_start = None
            self._append_log("[状态] APPROACHING → ALIGNING: 目标偏离中心")
            self.root.after(0, self._update_state_display)
            return

        # 到达判断
        bbox_max = max(self._bbox_size_x, self._bbox_size_y)
        if bbox_max >= self._arrive_ratio:
            self._publish_stop()
            self._state = State.GRABBING
            self._append_log(
                f"[状态] APPROACHING → GRABBING: 到达目标! bbox={bbox_max:.2f}"
            )
            self._run_grab()
            self.root.after(0, self._update_state_display)
            return

        # 前进（通过 cmd_vel → twist_bridge → Sport API）
        self._publish_cmd(vx=self._fwd_speed)

    # ==================================================================
    # GUI 主循环
    # ==================================================================
    def _update_loop(self):
        """定时刷新 GUI（约 60Hz+）。"""
        if not self._running:
            return

        try:
            self._do_update()
        except tk.TclError:
            # 窗口已销毁，停止刷新
            self._running = False
            return
        except Exception as e:
            # 其他异常不要打断 after 链，否则 GUI 卡死
            self.get_logger().warn(f"[GUI] 刷新异常: {e}")

        # 下一帧（~16ms ≈ 60Hz）
        if self._running:
            self.root.after(16, self._update_loop)

    def _do_update(self):
        """实际刷新逻辑（可被 _update_loop 安全捕获异常）。"""
        # FPS 计算
        now = time.time()
        elapsed = now - self._fps_time
        if elapsed >= 1.0:
            self._fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_time = now
            self._fps_label.config(text=f"FPS: {self._fps:.0f}")

        # 状态文字始终更新
        self._update_status_text()

        # 只在有新帧时刷新图像（ROS2 回调驱动，不人为限速）
        if not self._new_frame:
            return
        self._new_frame = False

        # 更新图像
        if self._raw_image is not None:
            frame_copy = self._raw_image.copy()

            # 原始图像
            self._raw_photo = self._cv2_to_tk(frame_copy, self._raw_canvas)
            if self._raw_photo:
                self._raw_canvas.delete("all")
                self._raw_canvas.create_image(0, 0, anchor=tk.NW, image=self._raw_photo)

            # 检测标注图像
            det_frame = self._draw_detections(frame_copy)
            self._det_photo = self._cv2_to_tk(det_frame, self._det_canvas)
            if self._det_photo:
                self._det_canvas.delete("all")
                self._det_canvas.create_image(0, 0, anchor=tk.NW, image=self._det_photo)

            # 检测信息
            if self._detections and self._det_count > 0:
                best = max(
                    self._detections.detections,
                    key=lambda d: d.results[0].score if d.results else 0
                )
                if best.results:
                    name = best.results[0].id
                    score = best.results[0].score
                    bbox_info = f"bbox={max(self._bbox_size_x, self._bbox_size_y):.2f}" if self._target_u is not None else ""
                    self._det_info_label.config(
                        text=f"检测: {name} ({score:.0%}) x{self._det_count} {bbox_info}"
                    )
            else:
                self._det_info_label.config(text="检测: 无目标")
        else:
            self._status_label.config(text="状态: 等待图像...", fg="#aaaaaa")

    def _update_status_text(self):
        """更新底部状态栏文字（轻量操作，可高频调用）。"""
        if self._raw_image is None:
            self._status_label.config(text="状态: 等待图像...", fg="#aaaaaa")
            return
        if self._state == State.IDLE:
            self._status_label.config(text="状态: 空闲", fg="#aaaaaa")
        elif self._state == State.SEARCHING:
            self._status_label.config(text="状态: 搜索目标中...", fg="#ffaa00")
        elif self._state == State.ALIGNING:
            err = abs(self._target_u - 0.5) if self._target_u else 0
            self._status_label.config(
                text=f"状态: 对齐中 err={err:.3f}", fg="#00d4ff"
            )
        elif self._state == State.APPROACHING:
            bbox_max = max(self._bbox_size_x, self._bbox_size_y)
            self._status_label.config(
                text=f"状态: 前进中 bbox={bbox_max:.2f}/{self._arrive_ratio:.2f}",
                fg="#3a7bd5"
            )
        elif self._state == State.GRABBING:
            self._status_label.config(text="状态: 抓取中...", fg="#ff6666")
        elif self._state == State.MENU:
            self._status_label.config(text="状态: 抓取完成，可放下", fg="#00ff88")

    def _on_close(self):
        self._running = False
        self._state = State.IDLE
        try:
            self._publish_stop()
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass

    def run(self):
        self.root.mainloop()

    def destroy_node(self) -> None:
        self._publish_stop()
        self.get_logger().info("[清理] 控制面板节点已停止")
        super().destroy_node()


def main(args=None):
    # control_panel 不使用 unitree SDK 的 ChannelFactoryInitialize，
    # 因此不需要 DDS 兼容层（不调用 _dds_compat.py）。
    # _dds_compat 会设置 ROS_DOMAIN_ID=1，导致 ROS2 收不到 Domain 0 的相机数据。

    """
    控制面板主入口函数
    
    初始化 ROS2 节点并在后台线程中启动 ROS2 spin，然后运行 tkinter 主循环。
    control_panel 不使用 unitree SDK 的 ChannelFactoryInitialize，
    因此不需要 DDS 兼容层（不调用 _dds_compat.py）。
    _dds_compat 会设置 ROS_DOMAIN_ID=1，导致 ROS2 收不到 Domain 0 的相机数据。
    
    Args:
        args (list, optional): ROS2 命令行参数，默认为 None
    """
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
