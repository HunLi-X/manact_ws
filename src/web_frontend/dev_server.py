#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 NavGrasp Web Frontend — 本地开发预览服务器
================================================
零 ROS2 依赖，纯 Flask mock 后端，用于在本地（Windows/Mac/Linux）预览前端效果。

功能：
    ✓ 提供与 web_panel.py 完全兼容的所有 API（/api/state, /api/config, /api/cmd/*）
    ✓ 生成动画测试图（彩条滚动 + 时间戳）作为 /stream/raw 和 /stream/detection
    ✓ 模拟状态机切换（IDLE → WORKING → GRABBING → MENU → IDLE 循环演示）
    ✓ 配置数据保存在内存中，支持热更新

依赖：
    pip install flask pillow

运行：
    cd src/web_frontend
    python dev_server.py
    # 浏览器访问 http://localhost:8080

修改前端文件后：
    直接刷新浏览器即可（静态文件由 Flask 实时读取）
"""

import io
import json
import math
import os
import sys
import time
import threading
from collections import deque
from pathlib import Path

try:
    from flask import Flask, Response, request, jsonify, send_from_directory
except ImportError:
    print("ERROR: Flask 未安装，请运行: pip install flask", file=sys.stderr)
    sys.exit(1)

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    print("ERROR: Pillow 未安装，请运行: pip install pillow", file=sys.stderr)
    sys.exit(1)


# ======================================================================
# Mock 状态
# ======================================================================
class MockState:
    """模拟 WebPanelNode 的状态机 + 配置存储。"""

    # 默认配置（与后端 _CONFIG_SCHEMA 对应）
    DEFAULT_CONFIG = {
        "target_class_id": "chair",
        "use_depth_distance": True,
        "stop_distance": 0.5,
        "depth_sample_radius": 5,
        "lost_timeout": 2.0,
        "search_yaw_speed": 0.3,
        "turn_yaw_speed": 0.6,
        "turn_duration": 2.6,
        "side_step_speed": 0.2,
        "side_step_duration": 2.0,
        "step_yaw_speed": 0.3,
        "step_duration": 0.3,
        "camera_settle_time": 2.0,
        "max_consecutive_steps": 10,
        "align_center_tolerance": 0.08,
        "forward_speed": 0.2,
        "arrive_bbox_ratio": 0.45,
        "align_stable_time": 1.0,
        "stream_quality": 70,
    }

    CONFIG_SCHEMA = [
        {"key": k, "type": type(v).__name__, "desc": k.replace("_", " ")}
        for k, v in DEFAULT_CONFIG.items()
    ]

    def __init__(self) -> None:
        self.config = dict(self.DEFAULT_CONFIG)

        # 状态机状态
        self.state = "IDLE"
        self.target_u = None
        self.bbox_max = None
        self.distance = None
        self.det_count = 0
        self.fps = 25.0
        self.target_class = self.config["target_class_id"]

        # 日志环形缓冲
        self._log_lock = threading.Lock()
        self._logs = deque(maxlen=500)
        self._log_idx = 0

        # 手动控制
        self._manual_active = False
        self._manual_vx = 0.0
        self._manual_vyaw = 0.0

        # 相机驱动 mock 状态
        self._camera_running = False
        self._camera_pid = None
        self._camera_start_time = 0.0
        self._camera_params = {
            "camera_namespace": "robot1",
            "camera_name": "D455_1",
            "align_depth.enable": "true",
        }
        self._camera_logs = deque(maxlen=200)

        self.append_log("[Mock] 本地预览服务器启动", "info")
        self.append_log(f"[Mock] 目标类别: {self.target_class}", "info")

    def append_log(self, msg: str, level: str = "info") -> None:
        with self._log_lock:
            self._log_idx += 1
            self._logs.append({"idx": self._log_idx, "msg": msg, "level": level})

    def snapshot(self, since_idx: int = 0) -> dict:
        with self._log_lock:
            new_logs = [l for l in self._logs if l["idx"] > since_idx]
            cur_idx = self._log_idx
        return {
            "state": self.state,
            "target_class": self.target_class,
            "target_u": self.target_u,
            "bbox_max": self.bbox_max,
            "distance": self.distance,
            "det_count": self.det_count,
            "fps": self.fps,
            "camera": self.camera_status(),
            "logs": new_logs,
            "log_idx": cur_idx,
            "mock": True,
        }

    # ---- 相机驱动 mock ----
    def camera_status(self) -> dict:
        uptime = (time.time() - self._camera_start_time) if self._camera_running else 0.0
        return {
            "running": self._camera_running,
            "pid": self._camera_pid,
            "uptime": round(uptime, 1),
            "launch_pkg": "realsense2_camera",
            "launch_file": "rs_launch.py",
            "params": dict(self._camera_params),
        }

    def camera_recent_logs(self, n: int = 50) -> list:
        return list(self._camera_logs)[-n:]

    def camera_start(self) -> dict:
        if self._camera_running:
            return {"ok": True, "msg": f"已在运行 (pid={self._camera_pid})", "running": True}
        self._camera_running = True
        self._camera_pid = 99999
        self._camera_start_time = time.time()
        self._camera_logs.clear()
        cmd = "ros2 launch realsense2_camera rs_launch.py " + " ".join(
            f"{k}:={v}" for k, v in self._camera_params.items()
        )
        self._camera_logs.append(f"[launch] {cmd}")
        self._camera_logs.append("[INFO] [realsense2_camera_node]: RealSense ROS v4.55.1")
        self._camera_logs.append("[INFO] [realsense2_camera_node]: Device with serial number 123456789 was found.")
        self.append_log(f"[Mock][相机] 启动: {cmd}", "info")
        return {"ok": True, "msg": f"已启动 (pid={self._camera_pid})", "running": True}

    def camera_stop(self) -> dict:
        if not self._camera_running:
            return {"ok": True, "msg": "未在运行", "running": False}
        self._camera_running = False
        self._camera_pid = None
        self._camera_start_time = 0.0
        self.append_log("[Mock][相机] 已停止", "info")
        return {"ok": True, "msg": "已停止", "running": False}

    def camera_set_params(self, params: dict) -> dict:
        if not isinstance(params, dict):
            return {"ok": False, "error": "params must be dict"}
        for k, v in params.items():
            self._camera_params[k] = str(v)
        self.append_log(f"[Mock][相机] 参数已更新: {params}", "info")
        return {"ok": True, "params": dict(self._camera_params)}

    # ---- 配置 ----
    def get_config(self) -> dict:
        return dict(self.config)

    def set_config(self, updates: dict) -> dict:
        updated, skipped = [], []
        for k, v in updates.items():
            if k not in self.config:
                skipped.append({"key": k, "reason": "unknown key"})
                continue
            self.config[k] = v
            if k == "target_class_id":
                self.target_class = v
            updated.append({"key": k, "value": v})
        if updated:
            kv = ", ".join(f"{u['key']}={u['value']}" for u in updated)
            self.append_log(f"[配置] 已更新: {kv}", "info")
        return {"updated": updated, "skipped": skipped}

    # ---- 命令（仅记录日志）----
    def cmd_manual(self, vx, vy, vyaw):
        self._manual_vx = vx
        self._manual_vyaw = vyaw
        self._manual_active = abs(vx) > 1e-3 or abs(vyaw) > 1e-3
        self.append_log(f"[Mock] 手动: vx={vx:.2f} vyaw={vyaw:.2f}", "info")

    def cmd_stop(self):
        self._manual_active = False
        prev = self.state
        self.state = "IDLE"
        self.append_log(f"[Mock] {prev} → IDLE 停止", "info")

    def cmd_search(self):
        self.state = "WORKING"
        self.append_log("[Mock] 开始搜索目标", "info")

    def cmd_grab(self):
        self.state = "GRABBING"
        self.append_log("[Mock] 执行 armup.py (模拟)", "info")
        # 2 秒后自动切 MENU
        threading.Timer(2.0, self._finish_grab).start()

    def _finish_grab(self):
        self.state = "MENU"
        self.append_log("[Mock] 抓取完成，可放下", "info")

    def cmd_put_down(self):
        self.append_log("[Mock] 执行 armdown.py (模拟)", "info")
        threading.Timer(1.5, self._finish_putdown).start()

    def _finish_putdown(self):
        self.state = "IDLE"
        self.append_log("[Mock] 放下完成，回到 IDLE", "info")

    def cmd_turn_put_down(self):
        self.append_log("[Mock] 右转 90° + 放下 (模拟)", "info")
        threading.Timer(3.0, self._finish_putdown).start()

    def cmd_left_put_down(self):
        v = self.config.get("side_step_speed", 0.2)
        t = self.config.get("side_step_duration", 2.0)
        self.append_log(f"[Mock] 向左侧移 {t:.1f}s @ {v:.2f}m/s + 放下 (模拟)", "info")
        threading.Timer(t + 1.5, self._finish_putdown).start()


# ======================================================================
# 后台状态模拟线程（每 0.2s 推进一次状态）
# ======================================================================
def state_simulator(mock: MockState) -> None:
    """模拟 WORKING 状态下检测变化。"""
    t = 0.0
    while True:
        time.sleep(0.2)
        t += 0.2

        if mock.state == "WORKING":
            # 模拟目标 u 从 0.2 → 0.5（对齐中），距离从 1.5m → 0.4m（接近中）
            phase = (t % 12.0) / 12.0
            if phase < 0.3:
                # 搜索阶段 — 无目标
                mock.target_u = None
                mock.bbox_max = None
                mock.distance = None
                mock.det_count = 0
            elif phase < 0.6:
                # 对齐阶段
                mock.target_u = 0.2 + (phase - 0.3) / 0.3 * 0.3
                mock.bbox_max = 0.15 + (phase - 0.3) / 0.3 * 0.08
                mock.distance = 1.5
                mock.det_count = 1
            else:
                # 接近阶段
                mock.target_u = 0.5 + math.sin(t * 2) * 0.02
                mock.bbox_max = 0.25 + (phase - 0.6) / 0.4 * 0.20
                mock.distance = max(0.35, 1.5 - (phase - 0.6) / 0.4 * 1.2)
                mock.det_count = 1
        elif mock.state == "IDLE":
            # 即使 IDLE 也显示一些偶发检测（模拟 YOLO 在跑）
            if int(t) % 3 == 0:
                mock.target_u = 0.4 + math.sin(t) * 0.1
                mock.bbox_max = 0.2
                mock.distance = 1.2
                mock.det_count = 1
            else:
                mock.target_u = None
                mock.bbox_max = None
                mock.distance = None
                mock.det_count = 0


# ======================================================================
# 动画测试图生成
# ======================================================================
def generate_test_frame(mock: MockState, mode: str, frame_idx: int) -> bytes:
    """生成测试图（彩条滚动 + 时间戳 + 状态文字）。"""
    w, h = 640, 480
    img = Image.new("RGB", (w, h), "#0F172A")
    draw = ImageDraw.Draw(img)

    # 滚动彩条
    colors = ["#0891B2", "#14B8A6", "#10B981", "#F59E0B", "#EF4444", "#3B82F6"]
    band_h = h // len(colors)
    offset = (frame_idx * 3) % band_h
    for i, c in enumerate(colors):
        y0 = i * band_h - offset
        draw.rectangle([0, y0, w, y0 + band_h + 2], fill=c)

    # 半透明覆盖黑层使文字清晰
    overlay = Image.new("RGBA", (w, h), (15, 23, 42, 140))
    img = Image.alpha_composite(img.convert("RGBA"), overlay).convert("RGB")
    draw = ImageDraw.Draw(img)

    # 文字：标题 + 状态
    try:
        # 尝试使用系统字体
        font_large = ImageFont.truetype("arial.ttf", 38)
        font_small = ImageFont.truetype("arial.ttf", 18)
        font_mono = ImageFont.truetype("consola.ttf", 14)
    except Exception:
        font_large = ImageFont.load_default()
        font_small = ImageFont.load_default()
        font_mono = ImageFont.load_default()

    tag = "DETECTION" if mode == "annotated" else "RAW STREAM"
    draw.text((20, 16), tag, font=font_small, fill="#5EEAD4")
    draw.text((20, 42), "G1 NavGrasp · Dev Preview", font=font_large, fill="white")

    # 状态信息
    info = [
        f"state       : {mock.state}",
        f"target      : {mock.target_class}",
        f"target_u    : {mock.target_u:.3f}" if mock.target_u is not None else "target_u    : —",
        f"distance    : {mock.distance:.2f} m" if mock.distance is not None else "distance    : —",
        f"det_count   : {mock.det_count}",
        f"fps (mock)  : {mock.fps:.1f}",
        f"frame       : #{frame_idx}",
        f"timestamp   : {time.strftime('%H:%M:%S')}",
    ]
    y = 160
    for line in info:
        draw.text((28, y), line, font=font_mono, fill="#CCFBF1")
        y += 22

    # 如果是 annotated 模式，画一个模拟的检测框
    if mode == "annotated" and mock.target_u is not None:
        cx = int(mock.target_u * w)
        cy = int(h * 0.6)
        box_w = int((mock.bbox_max or 0.2) * w)
        box_h = int((mock.bbox_max or 0.2) * h * 0.7)
        x1, y1 = cx - box_w // 2, cy - box_h // 2
        x2, y2 = cx + box_w // 2, cy + box_h // 2
        # 绿色检测框
        for i in range(3):
            draw.rectangle([x1 - i, y1 - i, x2 + i, y2 + i], outline="#10B981")
        # 中心十字（WORKING 状态）
        if mock.state == "WORKING":
            draw.line([(cx - 15, h // 2), (cx + 15, h // 2)], fill="#FBBF24", width=2)
            draw.line([(cx, h // 2 - 15), (cx, h // 2 + 15)], fill="#FBBF24", width=2)
        # 标签
        label = f"{mock.target_class} 95%"
        tw = 80
        draw.rectangle([x1, y1 - 22, x1 + tw, y1], fill="#10B981")
        draw.text((x1 + 4, y1 - 20), label, font=font_small, fill="white")

    # 底部提示条
    draw.rectangle([0, h - 30, w, h], fill="#0891B2")
    draw.text((20, h - 25), "Mock preview · No ROS2 required · python dev_server.py",
              font=font_small, fill="white")

    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=75)
    return buf.getvalue()


def mjpeg_generator(mock: MockState, mode: str):
    """MJPEG 流生成器。"""
    boundary = b"--frame"
    frame_idx = 0
    interval = 1.0 / 15  # 15 FPS
    while True:
        frame_idx += 1
        jpg = generate_test_frame(mock, mode, frame_idx)
        yield (
            boundary + b"\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n"
            + jpg + b"\r\n"
        )
        time.sleep(interval)


# ======================================================================
# Flask App
# ======================================================================
def create_app() -> Flask:
    # 前端目录 = 本文件所在目录
    frontend_dir = Path(__file__).resolve().parent
    if not (frontend_dir / "index.html").is_file():
        print(f"ERROR: 未找到 index.html，前端目录: {frontend_dir}", file=sys.stderr)
        sys.exit(1)

    mock = MockState()

    # 启动模拟后台
    sim_thread = threading.Thread(target=state_simulator, args=(mock,), daemon=True)
    sim_thread.start()

    app = Flask(__name__, static_folder=str(frontend_dir), static_url_path="/static")

    import logging
    logging.getLogger("werkzeug").setLevel(logging.WARNING)

    @app.route("/")
    def index():
        return send_from_directory(str(frontend_dir), "index.html")

    @app.route("/stream/raw")
    def stream_raw():
        return Response(mjpeg_generator(mock, "raw"),
                        mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/stream/detection")
    def stream_detection():
        return Response(mjpeg_generator(mock, "annotated"),
                        mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/api/state")
    def api_state():
        since = int(request.args.get("since", 0))
        return jsonify(mock.snapshot(since))

    @app.route("/api/config", methods=["GET"])
    def api_config_get():
        return jsonify({
            "values": mock.get_config(),
            "schema": MockState.CONFIG_SCHEMA,
        })

    @app.route("/api/config", methods=["POST"])
    def api_config_set():
        data = request.get_json(force=True, silent=True) or {}
        result = mock.set_config(data)
        return jsonify({"ok": True, **result})

    def _cmd_wrap(fn):
        try:
            fn()
            return jsonify({"ok": True})
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400

    @app.route("/api/cmd/manual", methods=["POST"])
    def cmd_manual():
        data = request.get_json(force=True, silent=True) or {}
        return _cmd_wrap(lambda: mock.cmd_manual(
            float(data.get("vx", 0)),
            float(data.get("vy", 0)),
            float(data.get("vyaw", 0)),
        ))

    @app.route("/api/cmd/stop", methods=["POST"])
    def cmd_stop():
        return _cmd_wrap(mock.cmd_stop)

    @app.route("/api/cmd/search", methods=["POST"])
    def cmd_search():
        return _cmd_wrap(mock.cmd_search)

    @app.route("/api/cmd/grab", methods=["POST"])
    def cmd_grab():
        return _cmd_wrap(mock.cmd_grab)

    @app.route("/api/cmd/putdown", methods=["POST"])
    def cmd_putdown():
        return _cmd_wrap(mock.cmd_put_down)

    @app.route("/api/cmd/turn_putdown", methods=["POST"])
    def cmd_turn_putdown():
        return _cmd_wrap(mock.cmd_turn_put_down)

    @app.route("/api/cmd/left_putdown", methods=["POST"])
    def cmd_left_putdown():
        return _cmd_wrap(mock.cmd_left_put_down)

    # ---- 相机驱动 mock 路由 ----
    @app.route("/api/camera/start", methods=["POST"])
    def api_camera_start():
        res = mock.camera_start()
        return jsonify(res), 200 if res.get("ok") else 400

    @app.route("/api/camera/stop", methods=["POST"])
    def api_camera_stop():
        res = mock.camera_stop()
        return jsonify(res), 200 if res.get("ok") else 400

    @app.route("/api/camera/status", methods=["GET"])
    def api_camera_status():
        return jsonify({**mock.camera_status(), "logs": mock.camera_recent_logs(80)})

    @app.route("/api/camera/params", methods=["POST"])
    def api_camera_params():
        data = request.get_json(force=True, silent=True) or {}
        return jsonify(mock.camera_set_params(data))

    return app


# ======================================================================
# 主入口
# ======================================================================
def main():
    host = os.environ.get("DEV_HOST", "0.0.0.0")
    port = int(os.environ.get("DEV_PORT", "8080"))

    print("=" * 60)
    print("  G1 NavGrasp · 本地开发预览服务器")
    print("=" * 60)
    print(f"  前端目录: {Path(__file__).resolve().parent}")
    print(f"  访问地址: http://localhost:{port}")
    print(f"  网络地址: http://<your-ip>:{port}")
    print("  后端模式: Mock（无需 ROS2）")
    print("")
    print("  修改前端文件后刷新浏览器即可看到变化")
    print("  按 Ctrl+C 退出")
    print("=" * 60)

    app = create_app()
    try:
        app.run(host=host, port=port, threaded=True, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        print("\n  已退出")


if __name__ == "__main__":
    main()
