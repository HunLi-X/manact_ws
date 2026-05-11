#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 NavGrasp Web 控制面板 (Flask + MJPEG)
==========================================
基于 Flask 的 Web 前端，替代 tkinter 版 control_panel.py。

架构：
    ROS2 Node (rclpy.spin in thread)
        ↓ 订阅相机 / 检测
        ↓ 缓存最新帧 + 状态
    Flask App (main thread)
        ├── GET  /                       → 返回 HTML 页面
        ├── GET  /stream/raw             → MJPEG 原始图像流
        ├── GET  /stream/detection       → MJPEG 检测标注流
        ├── GET  /api/state              → JSON 当前状态（轮询）
        ├── POST /api/cmd/manual         → 手动运动（vx, vy, vyaw）
        ├── POST /api/cmd/stop           → 停止
        ├── POST /api/cmd/search         → 搜索目标
        ├── POST /api/cmd/grab           → 抓取
        ├── POST /api/cmd/putdown        → 放下
        └── POST /api/cmd/turn_putdown   → 右转放下

运行：
    ros2 run g1_yolo_nav_py web_panel
    # 浏览器访问 http://<机器人IP>:8080

依赖：
    pip install flask
"""

import os
import sys
import time
import json
import threading
import subprocess
from pathlib import Path
from collections import deque
from typing import Optional

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge

try:
    from flask import Flask, Response, request, jsonify, render_template_string
except ImportError:
    print("ERROR: Flask 未安装。请运行: pip install flask", file=sys.stderr)
    sys.exit(1)

# 不导入 unitree_sdk2py（避免 DDS 冲突），所有运动通过 SportClient
from g1_yolo_nav_py._grasp_state import GraspStateMachineMixin, GraspState
from g1_yolo_nav_py._vis_utils import draw_detections_on_frame


# ======================================================================
# 前端 HTML（Tailwind CDN + 原生 JS，单文件零构建）
# ======================================================================
INDEX_HTML = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8" />
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no" />
<title>G1 NavGrasp · Web 控制面板</title>
<script src="https://cdn.tailwindcss.com"></script>
<script>
  tailwind.config = {
    theme: {
      extend: {
        colors: {
          teal: { 50:'#F0FDFA', 100:'#CCFBF1', 500:'#14B8A6', 600:'#0D9488', 700:'#0F766E', 900:'#134E4A' },
          brand: '#0891B2',
        },
        fontFamily: {
          sans: ['"Inter"','"PingFang SC"','"Microsoft YaHei UI"','sans-serif'],
          mono: ['"JetBrains Mono"','"Consolas"','monospace'],
        },
        boxShadow: {
          soft: '0 2px 8px rgba(15, 118, 110, 0.08)',
          card: '0 4px 16px rgba(15, 118, 110, 0.10)',
        }
      }
    }
  }
</script>
<style>
  html, body { background: linear-gradient(135deg, #F0FDFA 0%, #E0F2FE 100%); min-height: 100vh; }
  .stream-img { background: #0F172A; border-radius: 8px; object-fit: contain; width: 100%; height: 100%; }
  .btn { transition: all 0.15s; user-select: none; -webkit-tap-highlight-color: transparent; }
  .btn:hover { transform: translateY(-1px); }
  .btn:active { transform: translateY(0); }
  .btn-primary { @apply bg-brand text-white hover:bg-teal-700 shadow-soft; }
  .btn-success { @apply bg-emerald-500 text-white hover:bg-emerald-600 shadow-soft; }
  .btn-warning { @apply bg-amber-500 text-white hover:bg-amber-600 shadow-soft; }
  .btn-danger  { @apply bg-red-500 text-white hover:bg-red-600 shadow-soft; }
  .btn-neutral { @apply bg-blue-500 text-white hover:bg-blue-600 shadow-soft; }
  .state-dot { width: 10px; height: 10px; border-radius: 50%; display: inline-block; }
  .log-line { font-family: 'JetBrains Mono', Consolas, monospace; font-size: 12px; line-height: 1.5; }
  .log-info  { color: #059669; }
  .log-warn  { color: #D97706; }
  .log-error { color: #DC2626; }
  .card { background: white; border-radius: 12px; padding: 16px; box-shadow: 0 2px 8px rgba(15,118,110,0.08); }
</style>
</head>
<body class="font-sans text-teal-900 antialiased">

<header class="bg-gradient-to-r from-brand to-teal-700 text-white shadow-card">
  <div class="max-w-7xl mx-auto px-6 py-4 flex items-center justify-between">
    <div class="flex items-center gap-3">
      <div class="w-9 h-9 rounded-lg bg-white/20 flex items-center justify-center text-lg font-bold">G1</div>
      <div>
        <h1 class="text-xl font-bold tracking-tight">G1 NavGrasp</h1>
        <p class="text-xs text-teal-100">Web Dashboard · 远程控制</p>
      </div>
    </div>
    <div class="flex items-center gap-4">
      <div class="flex items-center gap-2 bg-white/10 rounded-full px-4 py-1.5">
        <span id="state-dot" class="state-dot bg-slate-400"></span>
        <span id="state-label" class="font-mono text-sm font-medium">连接中</span>
      </div>
      <div class="text-xs text-teal-100 font-mono">
        <span id="fps-label">FPS —</span> · <span id="det-label">检测 —</span>
      </div>
    </div>
  </div>
</header>

<main class="max-w-7xl mx-auto px-6 py-6 grid grid-cols-1 lg:grid-cols-3 gap-5">

  <section class="lg:col-span-2 space-y-5">
    <div class="grid grid-cols-1 md:grid-cols-2 gap-4">
      <div class="card">
        <div class="flex items-center justify-between mb-2">
          <h3 class="text-sm font-semibold text-teal-900">原始图像</h3>
          <span class="text-xs text-slate-400 font-mono">raw</span>
        </div>
        <div class="aspect-[4/3]">
          <img src="/stream/raw" class="stream-img" alt="raw" />
        </div>
      </div>
      <div class="card">
        <div class="flex items-center justify-between mb-2">
          <h3 class="text-sm font-semibold text-teal-900">检测结果</h3>
          <span class="text-xs text-slate-400 font-mono">annotated</span>
        </div>
        <div class="aspect-[4/3]">
          <img src="/stream/detection" class="stream-img" alt="detection" />
        </div>
      </div>
    </div>

    <div class="card">
      <div class="flex items-center justify-between mb-3">
        <h3 class="text-sm font-semibold text-teal-900">手动控制</h3>
        <div class="flex items-center gap-2 text-xs text-slate-500">
          <label>速度</label>
          <input id="speed-slider" type="range" min="0.05" max="0.5" step="0.05" value="0.2"
                 class="w-32 accent-brand" />
          <span id="speed-value" class="font-mono text-brand font-semibold w-10">0.20</span>
        </div>
      </div>
      <div class="grid grid-cols-5 gap-2">
        <button class="btn btn-neutral py-3 rounded-lg" data-cmd="forward">↑ 前进</button>
        <button class="btn btn-neutral py-3 rounded-lg" data-cmd="backward">↓ 后退</button>
        <button class="btn btn-neutral py-3 rounded-lg" data-cmd="left">↺ 左转</button>
        <button class="btn btn-neutral py-3 rounded-lg" data-cmd="right">↻ 右转</button>
        <button class="btn btn-danger py-3 rounded-lg" data-cmd="stop">■ 停止</button>
      </div>
    </div>

    <div class="card">
      <h3 class="text-sm font-semibold text-teal-900 mb-3">任务控制</h3>
      <div class="grid grid-cols-2 md:grid-cols-4 gap-2">
        <button class="btn btn-warning py-3 rounded-lg" data-cmd="search">🔍 搜索目标</button>
        <button class="btn btn-danger py-3 rounded-lg" data-cmd="grab">🤖 抓取</button>
        <button class="btn btn-success py-3 rounded-lg" data-cmd="putdown">📥 放下</button>
        <button class="btn btn-success py-3 rounded-lg" data-cmd="turn_putdown">↪ 右转放下</button>
      </div>
    </div>
  </section>

  <aside class="space-y-5">
    <div class="card">
      <h3 class="text-sm font-semibold text-teal-900 mb-3">状态信息</h3>
      <dl class="space-y-2 text-sm">
        <div class="flex justify-between"><dt class="text-slate-500">任务状态</dt><dd id="info-state" class="font-mono font-semibold">—</dd></div>
        <div class="flex justify-between"><dt class="text-slate-500">目标类别</dt><dd id="info-target" class="font-mono">—</dd></div>
        <div class="flex justify-between"><dt class="text-slate-500">目标位置 u</dt><dd id="info-u" class="font-mono">—</dd></div>
        <div class="flex justify-between"><dt class="text-slate-500">BBox 大小</dt><dd id="info-bbox" class="font-mono">—</dd></div>
        <div class="flex justify-between"><dt class="text-slate-500">深度距离</dt><dd id="info-dist" class="font-mono">—</dd></div>
        <div class="flex justify-between"><dt class="text-slate-500">检测数量</dt><dd id="info-count" class="font-mono">—</dd></div>
      </dl>
    </div>

    <div class="card">
      <div class="flex items-center justify-between mb-3">
        <h3 class="text-sm font-semibold text-teal-900">日志</h3>
        <button onclick="clearLog()" class="text-xs text-slate-400 hover:text-slate-600">清空</button>
      </div>
      <div id="log-box" class="bg-slate-50 rounded-lg p-3 h-80 overflow-y-auto border border-slate-200"></div>
    </div>
  </aside>
</main>

<script>
const STATE_COLORS = {
  IDLE:     { dot:'bg-slate-400',   text:'空闲' },
  WORKING:  { dot:'bg-cyan-500',    text:'执行中' },
  GRABBING: { dot:'bg-red-500',     text:'抓取中' },
  MENU:     { dot:'bg-emerald-500', text:'可放下' },
};

const speedSlider = document.getElementById('speed-slider');
const speedValue = document.getElementById('speed-value');
speedSlider.addEventListener('input', () => speedValue.textContent = (+speedSlider.value).toFixed(2));

function getSpeed() { return parseFloat(speedSlider.value); }

async function postCmd(path, body={}) {
  try {
    const r = await fetch(path, {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify(body)
    });
    const data = await r.json();
    if (!r.ok) appendLog(data.error || 'error', 'error');
  } catch (e) { appendLog('网络错误: ' + e.message, 'error'); }
}

document.querySelectorAll('button[data-cmd]').forEach(btn => {
  btn.addEventListener('click', () => {
    const cmd = btn.dataset.cmd;
    const v = getSpeed();
    const vyaw = v * 2.0;   // 转向速度按线速度的 2 倍
    switch (cmd) {
      case 'forward':  postCmd('/api/cmd/manual', {vx:  v,   vy:0, vyaw:0}); break;
      case 'backward': postCmd('/api/cmd/manual', {vx: -v,   vy:0, vyaw:0}); break;
      case 'left':     postCmd('/api/cmd/manual', {vx:0, vy:0, vyaw:  vyaw}); break;
      case 'right':    postCmd('/api/cmd/manual', {vx:0, vy:0, vyaw: -vyaw}); break;
      case 'stop':     postCmd('/api/cmd/stop'); break;
      case 'search':   postCmd('/api/cmd/search'); break;
      case 'grab':     postCmd('/api/cmd/grab'); break;
      case 'putdown':  postCmd('/api/cmd/putdown'); break;
      case 'turn_putdown': postCmd('/api/cmd/turn_putdown'); break;
    }
  });
});

function appendLog(msg, level='info') {
  const box = document.getElementById('log-box');
  const line = document.createElement('div');
  const ts = new Date().toLocaleTimeString('zh-CN', {hour12:false});
  line.className = `log-line log-${level}`;
  line.textContent = `[${ts}] ${msg}`;
  box.appendChild(line);
  if (box.children.length > 300) box.removeChild(box.firstChild);
  box.scrollTop = box.scrollHeight;
}

function clearLog() { document.getElementById('log-box').innerHTML = ''; }

let lastLogIdx = 0;
async function pollState() {
  try {
    const r = await fetch('/api/state?since=' + lastLogIdx);
    if (!r.ok) return;
    const s = await r.json();

    const st = STATE_COLORS[s.state] || { dot:'bg-slate-400', text:s.state };
    document.getElementById('state-dot').className = 'state-dot ' + st.dot;
    document.getElementById('state-label').textContent = st.text;

    document.getElementById('fps-label').textContent = 'FPS ' + (s.fps || 0).toFixed(0);
    document.getElementById('det-label').textContent = '检测 ' + (s.det_count || 0);

    document.getElementById('info-state').textContent = s.state;
    document.getElementById('info-target').textContent = s.target_class || '—';
    document.getElementById('info-u').textContent = s.target_u != null ? s.target_u.toFixed(3) : '—';
    document.getElementById('info-bbox').textContent = s.bbox_max != null ? s.bbox_max.toFixed(2) : '—';
    document.getElementById('info-dist').textContent = s.distance != null ? s.distance.toFixed(2) + ' m' : '—';
    document.getElementById('info-count').textContent = s.det_count != null ? s.det_count : '—';

    if (s.logs && s.logs.length > 0) {
      for (const entry of s.logs) appendLog(entry.msg, entry.level);
      lastLogIdx = s.log_idx;
    }
  } catch (e) { /* 暂时忽略网络异常 */ }
}
setInterval(pollState, 500);
pollState();
appendLog('Web 面板已连接', 'info');
</script>
</body>
</html>"""


# ======================================================================
# ROS2 节点 + Flask 后端
# ======================================================================
class WebPanelNode(Node, GraspStateMachineMixin):
    """G1 Web 控制面板节点。"""

    def __init__(self) -> None:
        Node.__init__(self, "g1_web_panel_node")

        self.declare_parameter("image_topic", "/D455_1/color/image_raw")
        self.declare_parameter("http_host", "0.0.0.0")
        self.declare_parameter("http_port", 8080)
        self.declare_parameter("stream_width", 640)
        self.declare_parameter("stream_height", 480)
        self.declare_parameter("stream_quality", 70)  # JPEG 质量 1-100
        self.declare_parameter("stream_fps", 15.0)

        p = lambda n: self.get_parameter(n).value
        self._img_topic = p("image_topic")
        self._http_host = p("http_host")
        self._http_port = int(p("http_port"))
        self._sw = int(p("stream_width"))
        self._sh = int(p("stream_height"))
        self._sq = max(1, min(100, int(p("stream_quality"))))
        self._stream_interval = 1.0 / max(1.0, float(p("stream_fps")))

        # network_interface 从 ROS 参数传递给 arm 脚本
        net_iface = self.get_parameter("network_interface").value if self.has_parameter("network_interface") else ""

        self._init_grasp_state(
            self,
            include_idle=True,
            start_state=GraspState.IDLE,
            network_interface=net_iface,
        )

        self._bridge = CvBridge()
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        self.create_subscription(Image, self._img_topic, self._image_cb, sensor_qos)

        self._raw_image: Optional[np.ndarray] = None
        self._detections: Optional[Detection2DArray] = None
        self._det_count = 0
        self._fps = 0.0
        self._frame_count = 0
        self._fps_time = time.time()
        self._image_lock = threading.Lock()

        # 手动控制持续发布
        self._manual_vx = 0.0
        self._manual_vy = 0.0
        self._manual_vyaw = 0.0
        self._manual_active = False

        # 日志环形缓冲（前端轮询拉取）
        self._log_lock = threading.Lock()
        self._logs = deque(maxlen=500)
        self._log_idx = 0

        # 状态机 tick 定时器（10Hz）
        self._tick_timer = self.create_timer(0.1, self._tick)

        self._log_info(f"Web 面板启动: http://{self._http_host}:{self._http_port}")
        self._log_info(f"目标类别: {self._gs_target_class}")
        self._log_info(f"图像话题: {self._img_topic}")

    # --- GraspStateMachineMixin 抽象方法实现 ---
    def _log_info(self, msg: str) -> None:
        self.get_logger().info(msg)
        self._append_log(msg, "info")

    def _log_error(self, msg: str) -> None:
        self.get_logger().error(msg)
        self._append_log(msg, "error")

    def _on_state_changed(self, old_state: GraspState, new_state: GraspState) -> None:
        self._append_log(f"[状态] {old_state.name} → {new_state.name}", "info")

    def _append_log(self, msg: str, level: str = "info") -> None:
        with self._log_lock:
            self._log_idx += 1
            self._logs.append({"idx": self._log_idx, "msg": msg, "level": level})

    # --- 覆盖基类的 grab/putdown 以适配 Web（不需要 GUI 状态刷新）---
    def _gs_show_menu(self) -> None:
        """Web 版不弹终端菜单，通过 HTTP 请求驱动。"""
        self._append_log("[状态] 抓取完成，可通过 Web 面板选择放下", "info")

    # --- ROS2 回调 ---
    def _image_cb(self, msg: Image) -> None:
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._image_lock:
                self._raw_image = img
                self._frame_count += 1
            now = time.time()
            elapsed = now - self._fps_time
            if elapsed >= 1.0:
                self._fps = self._frame_count / elapsed
                self._frame_count = 0
                self._fps_time = now
        except Exception as e:
            self._log_error(f"[图像] 转换失败: {e}")

    def _gs_on_detection(self, msg: Detection2DArray) -> None:
        self._detections = msg
        self._det_count = len(msg.detections)
        super()._gs_on_detection(msg)

    # --- 状态机 tick（含手动控制持续发布）---
    def _tick(self) -> None:
        if self._manual_active and self._gs_state in (GraspState.IDLE, GraspState.MENU):
            self._sport.move(vx=self._manual_vx, vy=self._manual_vy, vyaw=self._manual_vyaw)
        self._gs_tick()

    # --- MJPEG 编码帧（原始 / 标注）---
    def _encode_jpeg(self, frame: np.ndarray) -> Optional[bytes]:
        try:
            resized = cv2.resize(frame, (self._sw, self._sh), interpolation=cv2.INTER_LINEAR)
            ok, buf = cv2.imencode(".jpg", resized, [cv2.IMWRITE_JPEG_QUALITY, self._sq])
            if not ok:
                return None
            return buf.tobytes()
        except Exception:
            return None

    def get_raw_jpeg(self) -> Optional[bytes]:
        with self._image_lock:
            img = None if self._raw_image is None else self._raw_image.copy()
        if img is None:
            return None
        return self._encode_jpeg(img)

    def get_annotated_jpeg(self) -> Optional[bytes]:
        with self._image_lock:
            img = None if self._raw_image is None else self._raw_image.copy()
        if img is None:
            return None
        if self._detections is not None:
            img = draw_detections_on_frame(img, self._detections)
            # 叠加中心十字（目标 u 坐标）
            if self._gs_target_u is not None and self._gs_state == GraspState.WORKING:
                h, w = img.shape[:2]
                cx = int(self._gs_target_u * w)
                cy = h // 2
                cv2.line(img, (cx - 15, cy), (cx + 15, cy), (0, 255, 255), 2)
                cv2.line(img, (cx, cy - 15), (cx, cy + 15), (0, 255, 255), 2)
        return self._encode_jpeg(img)

    # --- 前端状态快照 ---
    def get_state_snapshot(self, since_idx: int = 0) -> dict:
        with self._log_lock:
            new_logs = [l for l in self._logs if l["idx"] > since_idx]
            cur_idx = self._log_idx

        bbox_max = max(self._gs_bbox_size_x, self._gs_bbox_size_y) if self._gs_target_u is not None else None
        return {
            "state": self._gs_state.name,
            "target_class": self._gs_target_class,
            "target_u": float(self._gs_target_u) if self._gs_target_u is not None else None,
            "bbox_max": float(bbox_max) if bbox_max is not None else None,
            "distance": float(self._gs_target_distance) if self._gs_target_distance is not None else None,
            "det_count": self._det_count,
            "fps": float(self._fps),
            "logs": new_logs,
            "log_idx": cur_idx,
        }

    # --- 命令接口（由 Flask 路由调用）---
    def cmd_manual(self, vx: float, vy: float, vyaw: float) -> None:
        if self._gs_state not in (GraspState.IDLE, GraspState.MENU):
            raise RuntimeError("任务执行中，请先停止")
        self._manual_vx = vx
        self._manual_vy = vy
        self._manual_vyaw = vyaw
        self._manual_active = True
        self._log_info(f"[手动] 持续运动: vx={vx:.2f} vy={vy:.2f} vyaw={vyaw:.2f}")

    def cmd_stop(self) -> None:
        self._manual_active = False
        self._manual_vx = self._manual_vy = self._manual_vyaw = 0.0
        prev = self._gs_state
        self.gs_state = GraspState.IDLE
        self._sport.stop()
        self._gs_aligner.reset()
        self._gs_approach.reset()
        self._log_info(f"[停止] {prev.name} → IDLE")

    def cmd_search(self) -> None:
        if self._gs_state not in (GraspState.IDLE, GraspState.MENU):
            raise RuntimeError("请先停止当前任务")
        self.gs_state = GraspState.WORKING
        self._gs_aligned = False
        self._gs_aligner.reset()
        self._gs_approach.reset()
        self._log_info(f"[搜索] 开始搜索 '{self._gs_target_class}'")

    def cmd_grab(self) -> None:
        if self._gs_state not in (GraspState.IDLE, GraspState.MENU):
            raise RuntimeError("请先停止当前任务")
        self.gs_state = GraspState.GRABBING
        self._sport.stop()
        self._gs_run_grab()

    def cmd_put_down(self) -> None:
        if self._gs_state not in (GraspState.IDLE, GraspState.MENU):
            raise RuntimeError("请先停止当前任务")
        threading.Thread(target=self._gs_run_armdown, daemon=True).start()

    def cmd_turn_put_down(self) -> None:
        if self._gs_state not in (GraspState.IDLE, GraspState.MENU):
            raise RuntimeError("请先停止当前任务")
        threading.Thread(target=self._gs_do_turn_and_put_down, daemon=True).start()

    # --- 生成 MJPEG 流（用于 Flask Response）---
    def mjpeg_generator(self, mode: str):
        """mode: 'raw' | 'annotated'"""
        getter = self.get_raw_jpeg if mode == "raw" else self.get_annotated_jpeg
        boundary = b"--frame"
        while True:
            jpg = getter()
            if jpg is not None:
                yield (
                    boundary + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n"
                    + jpg + b"\r\n"
                )
            time.sleep(self._stream_interval)

    def destroy_node(self) -> None:
        self._gs_destroy()
        super().destroy_node()


# ======================================================================
# Flask App 工厂
# ======================================================================
def create_app(node: WebPanelNode) -> Flask:
    app = Flask(__name__)
    # 关闭 Flask 自身 log 的噪声
    import logging
    logging.getLogger("werkzeug").setLevel(logging.WARNING)

    @app.route("/")
    def index():
        return render_template_string(INDEX_HTML)

    @app.route("/stream/raw")
    def stream_raw():
        return Response(node.mjpeg_generator("raw"),
                        mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/stream/detection")
    def stream_det():
        return Response(node.mjpeg_generator("annotated"),
                        mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/api/state")
    def api_state():
        since = int(request.args.get("since", 0))
        return jsonify(node.get_state_snapshot(since))

    def _cmd_route(fn):
        try:
            fn()
            return jsonify({"ok": True})
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400

    @app.route("/api/cmd/manual", methods=["POST"])
    def cmd_manual():
        data = request.get_json(force=True, silent=True) or {}
        vx = float(data.get("vx", 0.0))
        vy = float(data.get("vy", 0.0))
        vyaw = float(data.get("vyaw", 0.0))
        # 速度钳位
        vx = max(-1.0, min(1.0, vx))
        vy = max(-0.5, min(0.5, vy))
        vyaw = max(-1.0, min(1.0, vyaw))
        return _cmd_route(lambda: node.cmd_manual(vx, vy, vyaw))

    @app.route("/api/cmd/stop", methods=["POST"])
    def cmd_stop():
        return _cmd_route(node.cmd_stop)

    @app.route("/api/cmd/search", methods=["POST"])
    def cmd_search():
        return _cmd_route(node.cmd_search)

    @app.route("/api/cmd/grab", methods=["POST"])
    def cmd_grab():
        return _cmd_route(node.cmd_grab)

    @app.route("/api/cmd/putdown", methods=["POST"])
    def cmd_putdown():
        return _cmd_route(node.cmd_put_down)

    @app.route("/api/cmd/turn_putdown", methods=["POST"])
    def cmd_turn_putdown():
        return _cmd_route(node.cmd_turn_put_down)

    return app


# ======================================================================
# 主入口
# ======================================================================
def main(args=None):
    """启动 ROS2 节点 + Flask 服务器。

    ROS2 spin 在后台线程，Flask 在主线程（threaded=True 以支持 MJPEG 并发）。
    """
    rclpy.init(args=args)
    node = WebPanelNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = create_app(node)
    try:
        # threaded=True 让 MJPEG 流和 API 请求并发处理
        app.run(
            host=node._http_host,
            port=node._http_port,
            threaded=True,
            debug=False,
            use_reloader=False,
        )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
