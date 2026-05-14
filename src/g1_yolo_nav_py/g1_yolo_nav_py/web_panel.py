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
        ├── POST /api/cmd/turn_putdown   → 右转放下
        └── POST /api/cmd/left_putdown   → 左移放下

运行：
    ros2 run g1_yolo_nav_py web_panel
    # 浏览器访问 http://<机器人IP>:8080

依赖：
    pip install flask
"""

import os
import sys
import time
import signal
import shutil
import threading
import subprocess
from pathlib import Path
from collections import deque
from typing import Optional, List

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge

try:
    from flask import Flask, Response, request, jsonify, send_from_directory
except ImportError:
    print("ERROR: Flask 未安装。请运行: pip install flask", file=sys.stderr)
    sys.exit(1)

# 不导入 unitree_sdk2py（避免 DDS 冲突），所有运动通过 SportClient
from g1_yolo_nav_py._grasp_state import GraspStateMachineMixin, GraspState
from g1_yolo_nav_py._vis_utils import draw_detections_on_frame


# ======================================================================
# 前端目录解析
# ======================================================================
def _resolve_frontend_dir() -> Path:
    """按优先级查找 web_frontend/ 目录：

    1. 环境变量 G1_WEB_FRONTEND_DIR（开发/手动覆盖）
    2. 安装目录：share/g1_yolo_nav_py/web_frontend/（colcon install 后）
    3. 开发源码树：src/web_frontend/（相对当前文件向上查找）

    Returns:
        web_frontend 目录的绝对路径。

    Raises:
        FileNotFoundError: 所有候选位置都没找到。
    """
    candidates = []

    env_dir = os.environ.get("G1_WEB_FRONTEND_DIR")
    if env_dir:
        candidates.append(Path(env_dir))

    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("g1_yolo_nav_py"))
        candidates.append(share / "web_frontend")
    except Exception:
        pass

    # 开发模式：从 src/g1_yolo_nav_py/g1_yolo_nav_py/web_panel.py 向上找 src/web_frontend
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "web_frontend"
        if candidate.is_dir():
            candidates.append(candidate)
            break
        if parent.name == "src":
            candidates.append(parent / "web_frontend")
            break

    for c in candidates:
        if c and c.is_dir() and (c / "index.html").is_file():
            return c

    raise FileNotFoundError(
        f"未找到 web_frontend 目录。已查找: {[str(c) for c in candidates]}\n"
        f"请检查：1) colcon build 已完成  2) src/web_frontend/ 存在"
    )




# ======================================================================
# 相机驱动启动器
# ======================================================================
class CameraLauncher:
    """通过 subprocess 管理 RealSense ROS2 驱动进程。

    默认命令：
        ros2 launch realsense2_camera rs_launch.py \
            camera_namespace:=robot1 camera_name:=D455_1 align_depth.enable:=true

    特性：
        - 子进程在独立 process group（preexec_fn=os.setsid），便于完整 kill
        - 输出实时收集到环形缓冲（最近 200 行），供 /api/state 暴露
        - 重复 start 是幂等的（已运行则直接返回）
        - stop 先 SIGINT，超时后 SIGKILL
    """

    DEFAULT_PARAMS = {
        "camera_namespace": "robot1",
        "camera_name": "D455_1",
        "align_depth.enable": "true",
    }

    def __init__(self, log_callback=None):
        self._proc: Optional[subprocess.Popen] = None
        self._start_time: float = 0.0
        self._lock = threading.Lock()
        self._log_lines = deque(maxlen=200)
        self._log_thread: Optional[threading.Thread] = None
        self._launch_pkg = "realsense2_camera"
        self._launch_file = "rs_launch.py"
        self._params = dict(self.DEFAULT_PARAMS)
        # 外部日志回调（注入到主面板日志流）
        self._on_log = log_callback or (lambda msg, level="info": None)

    # ---- 配置参数访问 ----
    def get_params(self) -> dict:
        return dict(self._params)

    def set_param(self, key: str, value) -> None:
        if not key:
            return
        self._params[key] = str(value)

    def set_launch(self, pkg: str, launch_file: str) -> None:
        if pkg:
            self._launch_pkg = pkg
        if launch_file:
            self._launch_file = launch_file

    # ---- 状态 ----
    def is_running(self) -> bool:
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    def status(self) -> dict:
        with self._lock:
            running = self._proc is not None and self._proc.poll() is None
            pid = self._proc.pid if (self._proc and running) else None
            uptime = (time.time() - self._start_time) if running else 0.0
            return {
                "running": running,
                "pid": pid,
                "uptime": round(uptime, 1),
                "launch_pkg": self._launch_pkg,
                "launch_file": self._launch_file,
                "params": dict(self._params),
            }

    def recent_logs(self, n: int = 50) -> List[str]:
        with self._lock:
            return list(self._log_lines)[-n:]

    # ---- 启动 / 停止 ----
    def build_command(self) -> List[str]:
        cmd = ["ros2", "launch", self._launch_pkg, self._launch_file]
        for k, v in self._params.items():
            cmd.append(f"{k}:={v}")
        return cmd

    def start(self) -> dict:
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return {"ok": True, "msg": f"已在运行 (pid={self._proc.pid})", "running": True}

            if shutil.which("ros2") is None:
                msg = "找不到 ros2 命令，请确认已 source ROS2 setup.bash"
                self._on_log(f"[相机] {msg}", "error")
                return {"ok": False, "error": msg}

            cmd = self.build_command()
            self._on_log(f"[相机] 启动: {' '.join(cmd)}", "info")
            try:
                # preexec_fn=os.setsid 让子进程在独立 process group，便于完整 kill 整个 launch 树
                preexec = os.setsid if hasattr(os, "setsid") else None
                self._proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    bufsize=1,
                    universal_newlines=True,
                    preexec_fn=preexec,
                )
                self._start_time = time.time()
                self._log_lines.clear()
            except Exception as e:
                self._proc = None
                self._on_log(f"[相机] 启动失败: {e}", "error")
                return {"ok": False, "error": str(e)}

            # 启动后台日志收集线程
            self._log_thread = threading.Thread(
                target=self._read_output, args=(self._proc,), daemon=True,
            )
            self._log_thread.start()
            return {"ok": True, "msg": f"已启动 (pid={self._proc.pid})", "running": True}

    def stop(self, timeout: float = 5.0) -> dict:
        with self._lock:
            if self._proc is None or self._proc.poll() is not None:
                self._proc = None
                return {"ok": True, "msg": "未在运行", "running": False}
            proc = self._proc

        self._on_log(f"[相机] 正在停止 (pid={proc.pid}) ...", "info")
        try:
            if hasattr(os, "killpg"):
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            else:
                proc.terminate()
        except (ProcessLookupError, OSError) as e:
            self._on_log(f"[相机] 终止信号失败: {e}", "warn")

        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self._on_log("[相机] SIGINT 超时，发送 SIGKILL", "warn")
            try:
                if hasattr(os, "killpg"):
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                else:
                    proc.kill()
                proc.wait(timeout=2.0)
            except Exception as e:
                self._on_log(f"[相机] 强制终止失败: {e}", "error")

        with self._lock:
            self._proc = None
        self._on_log("[相机] 已停止", "info")
        return {"ok": True, "msg": "已停止", "running": False}

    def _read_output(self, proc: subprocess.Popen) -> None:
        """后台线程：读取子进程输出到环形缓冲。"""
        try:
            for line in iter(proc.stdout.readline, ""):
                if not line:
                    break
                line = line.rstrip()
                with self._lock:
                    self._log_lines.append(line)
        except Exception:
            pass
        finally:
            try:
                proc.stdout.close()
            except Exception:
                pass




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

        # 相机驱动启动器（subprocess 管理 ros2 launch realsense2_camera）
        self.declare_parameter("camera_launch_pkg", "realsense2_camera")
        self.declare_parameter("camera_launch_file", "rs_launch.py")
        self.declare_parameter("camera_namespace", "robot1")
        self.declare_parameter("camera_name", "D455_1")
        self.declare_parameter("camera_align_depth", True)
        self._camera = CameraLauncher(log_callback=self._append_log)
        self._camera.set_launch(
            self.get_parameter("camera_launch_pkg").value,
            self.get_parameter("camera_launch_file").value,
        )
        self._camera.set_param("camera_namespace", self.get_parameter("camera_namespace").value)
        self._camera.set_param("camera_name", self.get_parameter("camera_name").value)
        self._camera.set_param(
            "align_depth.enable",
            "true" if bool(self.get_parameter("camera_align_depth").value) else "false",
        )

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

    # --- 配置管理（热更新）---
    # 可热更新的参数映射：前端字段名 → (GraspStateMachineMixin 属性 / aligner / approach 的属性, 类型, 描述)
    _CONFIG_SCHEMA = {
        "target_class_id": ("_gs_target_class", str, "目标类别"),
        "use_depth_distance": ("_gs_use_depth", bool, "启用深度距离判断"),
        "stop_distance": ("_gs_stop_distance", float, "停止距离 (m)"),
        "depth_sample_radius": ("_gs_depth_radius", int, "深度采样半径"),
        "lost_timeout": ("_gs_lost_timeout", float, "目标丢失超时 (s)"),
        "search_yaw_speed": ("_gs_search_speed", float, "搜索旋转速度 (rad/s)"),
        "turn_yaw_speed": ("_gs_turn_speed", float, "转身速度 (rad/s)"),
        "turn_duration": ("_gs_turn_duration", float, "转身时长 (s)"),
        "side_step_speed": ("_gs_side_step_speed", float, "左移放下速度 (m/s)"),
        "side_step_duration": ("_gs_side_step_duration", float, "左移放下时长 (s)"),
        # StepAligner 属性
        "step_yaw_speed":       ("aligner.step_yaw_speed",      float, "每步旋转速度 (rad/s)"),
        "step_duration":        ("aligner.step_duration",       float, "每步持续时间 (s)"),
        "camera_settle_time":   ("aligner.camera_settle_time",  float, "相机等待时间 (s)"),
        "max_consecutive_steps":("aligner.max_consecutive_steps", int, "单次最大连续步数"),
        "align_center_tolerance": ("aligner.center_tolerance",  float, "对齐居中容差"),
        # ForwardApproach 属性
        "forward_speed":      ("approach.forward_speed",       float, "前进速度 (m/s)"),
        "arrive_bbox_ratio":  ("approach.arrive_bbox_ratio",   float, "到达 BBox 阈值（深度 fallback）"),
        "align_stable_time":  ("approach.align_stable_time",   float, "居中稳定时长 (s)"),
        # 流编码参数
        "stream_quality": ("_sq", int, "JPEG 质量 (1-100)"),
    }

    def get_config(self) -> dict:
        """读取当前配置值。"""
        out = {}
        for key, (attr_path, typ, _desc) in self._CONFIG_SCHEMA.items():
            try:
                val = self._resolve_attr(attr_path)
                out[key] = val
            except Exception:
                out[key] = None
        return out

    def get_config_schema(self) -> list:
        """返回字段元数据，供前端渲染表单使用。"""
        return [
            {"key": k, "type": t.__name__, "desc": d}
            for k, (_, t, d) in self._CONFIG_SCHEMA.items()
        ]

    def set_config(self, updates: dict) -> dict:
        """批量更新配置。返回 {updated: [...], skipped: [...]}。"""
        updated, skipped = [], []
        for key, val in updates.items():
            if key not in self._CONFIG_SCHEMA:
                skipped.append({"key": key, "reason": "unknown key"})
                continue
            attr_path, typ, _desc = self._CONFIG_SCHEMA[key]
            try:
                cast = self._cast_value(val, typ)
                self._assign_attr(attr_path, cast)
                updated.append({"key": key, "value": cast})
            except Exception as e:
                skipped.append({"key": key, "reason": str(e)})

        if updated:
            kv = ", ".join(f"{u['key']}={u['value']}" for u in updated)
            self._log_info(f"[配置] 已更新: {kv}")
        return {"updated": updated, "skipped": skipped}

    def _resolve_attr(self, path: str):
        """按 'root.sub' 解析属性路径，root 是 aligner/approach 或 self 的属性。"""
        if "." in path:
            root_name, attr = path.split(".", 1)
            root_obj = {"aligner": self._gs_aligner, "approach": self._gs_approach}[root_name]
            return getattr(root_obj, attr)
        return getattr(self, path)

    def _assign_attr(self, path: str, value) -> None:
        if "." in path:
            root_name, attr = path.split(".", 1)
            root_obj = {"aligner": self._gs_aligner, "approach": self._gs_approach}[root_name]
            setattr(root_obj, attr, value)
        else:
            setattr(self, path, value)

    @staticmethod
    def _cast_value(val, typ):
        if typ is bool:
            if isinstance(val, bool):
                return val
            if isinstance(val, str):
                return val.strip().lower() in ("true", "1", "yes", "on")
            return bool(val)
        return typ(val)

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
            "camera": self._camera.status(),
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

    def cmd_left_put_down(self) -> None:
        if self._gs_state not in (GraspState.IDLE, GraspState.MENU):
            raise RuntimeError("请先停止当前任务")
        threading.Thread(target=self._gs_do_left_put_down, daemon=True).start()

    # ---- 相机驱动控制 ----
    def cmd_camera_start(self) -> dict:
        return self._camera.start()

    def cmd_camera_stop(self) -> dict:
        return self._camera.stop()

    def cmd_camera_set(self, params: dict) -> dict:
        """更新相机参数（仅在下次启动生效）。"""
        if not isinstance(params, dict):
            return {"ok": False, "error": "params must be dict"}
        for k, v in params.items():
            self._camera.set_param(k, v)
        self._log_info(f"[相机] 参数已更新: {params}")
        return {"ok": True, "params": self._camera.get_params()}

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
        try:
            if self._camera.is_running():
                self._camera.stop(timeout=3.0)
        except Exception:
            pass
        self._gs_destroy()
        super().destroy_node()


# ======================================================================
# Flask App 工厂
# ======================================================================
def create_app(node: WebPanelNode) -> Flask:
    frontend_dir = _resolve_frontend_dir()
    node.get_logger().info(f"[Web] 前端目录: {frontend_dir}")

    app = Flask(
        __name__,
        static_folder=str(frontend_dir),
        static_url_path="/static",
    )
    # 关闭 Flask 自身 log 的噪声
    import logging
    logging.getLogger("werkzeug").setLevel(logging.WARNING)

    @app.route("/")
    def index():
        return send_from_directory(str(frontend_dir), "index.html")

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

    @app.route("/api/config", methods=["GET"])
    def api_config_get():
        return jsonify({
            "values": node.get_config(),
            "schema": node.get_config_schema(),
        })

    @app.route("/api/config", methods=["POST"])
    def api_config_set():
        data = request.get_json(force=True, silent=True) or {}
        result = node.set_config(data)
        return jsonify({"ok": True, **result})

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

    @app.route("/api/cmd/left_putdown", methods=["POST"])
    def cmd_left_putdown():
        return _cmd_route(node.cmd_left_put_down)

    # ---- 相机驱动 ----
    @app.route("/api/camera/start", methods=["POST"])
    def api_camera_start():
        try:
            res = node.cmd_camera_start()
            code = 200 if res.get("ok") else 400
            return jsonify(res), code
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400

    @app.route("/api/camera/stop", methods=["POST"])
    def api_camera_stop():
        try:
            res = node.cmd_camera_stop()
            code = 200 if res.get("ok") else 400
            return jsonify(res), code
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 400

    @app.route("/api/camera/status", methods=["GET"])
    def api_camera_status():
        return jsonify({
            **node._camera.status(),
            "logs": node._camera.recent_logs(80),
        })

    @app.route("/api/camera/params", methods=["POST"])
    def api_camera_params():
        data = request.get_json(force=True, silent=True) or {}
        return jsonify(node.cmd_camera_set(data))

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
