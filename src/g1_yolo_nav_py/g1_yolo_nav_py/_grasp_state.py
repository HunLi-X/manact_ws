"""抓取任务状态机基类 — grasp_task.py 与 control_panel.py 共享的逻辑。

提取以下共享内容：
    - State 枚举（WORKING = 搜索 + 对齐 + 接近，无状态切换）
    - 公共参数声明
    - 检测回调 _on_detection
    - 统一工作循环 _gs_tick_working（搜索 + StepAligner 对齐 + 接近）
    - arm 脚本执行（_run_grab / _run_armdown）
    - 右转放下 _do_turn_and_put_down

对齐逻辑直接使用 StepAligner（_step_aligner.py），与 yaw_align.py 共用同一份代码。

子类需要实现：
    - _log_info(msg) — 信息日志（grasp_task 用 logger，control_panel 用 _append_log）
    - _log_error(msg) — 错误日志
    - _on_state_changed(old_state, new_state) — 状态变化回调（更新 GUI/终端显示）
"""

import os
import sys
import time
import math
import subprocess
import threading
from pathlib import Path
from enum import Enum, auto
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge

from g1_yolo_nav_py.sport_client import SportClient
from g1_yolo_nav_py._detection_utils import find_best_detection, sample_depth_at_pixel, depth_to_meters
from g1_yolo_nav_py._step_aligner import StepAligner, AlignAction
from g1_yolo_nav_py._forward_approach import ForwardApproach, ApproachAction


# ==================================================================
# 状态枚举
# ==================================================================
class GraspState(Enum):
    """抓取任务状态枚举。"""
    IDLE = auto()        # 控制面板专用：空闲等待
    WORKING = auto()     # 搜索 + 对齐 + 接近（连续行为，无状态切换）
    GRABBING = auto()    # 执行 armup.py 抓取
    MENU = auto()        # 交互菜单
    DONE = auto()        # 任务完成（grasp_task 专用）


# arm 脚本默认目录
_DEFAULT_ARM_DIR = os.path.expanduser("~/g1act_ws/manact_ws/src/g1_yolo_nav_py/arm")


# ==================================================================
# 基类
# ==================================================================
class GraspStateMachineMixin:
    """抓取任务状态机混入类 — 提供共享的状态机逻辑。

    用法：
        class GraspTaskNode(Node, GraspStateMachineMixin):
            def __init__(self):
                super().__init__("g1_grasp_task_node")
                self._init_grasp_state(...)
    """

    # ------------------------------------------------------------------
    #  初始化
    # ------------------------------------------------------------------
    def _init_grasp_state(
        self,
        node: Node,
        *,
        include_idle: bool = False,
        start_state: GraspState = GraspState.WORKING,
        arm_script_dir: Optional[str] = None,
        network_interface: str = "",
    ) -> None:
        """初始化抓取状态机。

        Args:
            node: ROS2 节点实例
            include_idle: 是否包含 IDLE 状态（control_panel 需要）
            start_state: 初始状态
            arm_script_dir: arm 脚本目录路径
            network_interface: 传递给 arm 脚本的网络接口
        """
        self._gs_node = node
        self._gs_include_idle = include_idle

        # ---- 声明公共参数 ----
        node.declare_parameter("network_interface", "")
        node.declare_parameter("detection_topic", "/g1/vision/detections")
        node.declare_parameter("depth_topic", "/D455_1/depth/image_rect_raw")
        node.declare_parameter("target_class_id", "chair")
        node.declare_parameter("use_depth_distance", True)
        node.declare_parameter("stop_distance", 0.5)
        node.declare_parameter("depth_sample_radius", 5)
        node.declare_parameter("camera_fov_deg", 87.0)
        node.declare_parameter("center_tolerance", 0.08)
        node.declare_parameter("step_yaw_speed", 0.3)         # 步进式对齐：每步旋转速度
        node.declare_parameter("step_duration", 0.3)           # 步进式对齐：每步持续时间
        node.declare_parameter("camera_settle_time", 2.0)      # 步进式对齐：等待相机更新
        node.declare_parameter("max_consecutive_steps", 10)   # 步进式对齐：单次最大连续步数
        node.declare_parameter("forward_speed", 0.2)
        node.declare_parameter("arrive_bbox_ratio", 0.45)
        node.declare_parameter("align_stable_time", 1.0)
        node.declare_parameter("lost_timeout", 2.0)
        node.declare_parameter("search_yaw_speed", 0.3)
        node.declare_parameter("turn_yaw_speed", 0.6)
        node.declare_parameter("turn_duration", 2.6)
        node.declare_parameter("arm_script_dir", arm_script_dir or _DEFAULT_ARM_DIR)

        p = lambda n: node.get_parameter(n).value
        self._gs_det_topic = p("detection_topic")
        self._gs_depth_topic = p("depth_topic")
        self._gs_target_class = p("target_class_id")
        self._gs_use_depth = bool(p("use_depth_distance"))
        self._gs_stop_distance = float(p("stop_distance"))
        self._gs_depth_radius = max(1, int(p("depth_sample_radius")))
        self._gs_fov_rad = math.radians(float(p("camera_fov_deg")))
        self._gs_center_tol = float(p("center_tolerance"))
        self._gs_step_speed = float(p("step_yaw_speed"))
        self._gs_step_dur = float(p("step_duration"))
        self._gs_settle_time = float(p("camera_settle_time"))
        self._gs_max_steps = int(p("max_consecutive_steps"))
        self._gs_fwd_speed = float(p("forward_speed"))
        self._gs_arrive_ratio = float(p("arrive_bbox_ratio"))
        self._gs_stable_time = float(p("align_stable_time"))
        self._gs_lost_timeout = float(p("lost_timeout"))
        self._gs_search_speed = float(p("search_yaw_speed"))
        self._gs_turn_speed = float(p("turn_yaw_speed"))
        self._gs_turn_duration = float(p("turn_duration"))

        # network_interface: 优先使用函数参数，否则从 ROS 参数读取
        self._gs_net_iface = network_interface or p("network_interface")

        # arm 脚本路径
        arm_dir = p("arm_script_dir")
        self._gs_armup_script = Path(arm_dir) / "armup.py"
        self._gs_armdown_script = Path(arm_dir) / "armdown.py"

        # ---- 运动控制客户端 ----
        self._sport = SportClient(node)

        # ---- 步进式对齐器（与 yaw_align.py 共用同一份代码） ----
        self._gs_aligner = StepAligner(
            move_fn=self._sport.move,
            logger=node.get_logger(),
            center_tolerance=float(p("center_tolerance")),
            step_yaw_speed=float(p("step_yaw_speed")),
            step_duration=float(p("step_duration")),
            camera_settle_time=float(p("camera_settle_time")),
            max_consecutive_steps=int(p("max_consecutive_steps")),
        )

        # ---- 前进接近器（与 loco_forward.py 共用同一份代码） ----
        self._gs_approach = ForwardApproach(
            move_fn=self._sport.move,
            stop_fn=self._sport.stop,
            logger=node.get_logger(),
            forward_speed=float(p("forward_speed")),
            center_tolerance=float(p("center_tolerance")),
            align_stable_time=float(p("align_stable_time")),
            use_depth=bool(p("use_depth_distance")),
            stop_distance=float(p("stop_distance")),
            arrive_bbox_ratio=float(p("arrive_bbox_ratio")),
        )

        # ---- 内部状态 ----
        self._gs_target_u: Optional[float] = None
        self._gs_target_v: Optional[float] = None
        self._gs_target_distance: Optional[float] = None
        self._gs_depth_image: Optional[np.ndarray] = None
        self._gs_depth_encoding: str = ""
        self._gs_bbox_size_x: float = 0.0
        self._gs_bbox_size_y: float = 0.0
        self._gs_last_detect_time: float = 0.0
        self._gs_state: GraspState = start_state
        self._gs_aligned: bool = False        # 目标已居中（用于对齐完成日志）

        # ---- ROS2 订阅 ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._gs_bridge = CvBridge()
        node.create_subscription(
            Detection2DArray, self._gs_det_topic, self._gs_on_detection, 10
        )
        if self._gs_use_depth:
            node.create_subscription(
                Image, self._gs_depth_topic, self._gs_on_depth, sensor_qos
            )

        # ---- 跳过自动 FSM 初始化，由用户手动进入走跑模式 ----
        self._sport.skip_init()

    # ------------------------------------------------------------------
    #  属性
    # ------------------------------------------------------------------
    @property
    def gs_state(self) -> GraspState:
        """当前状态机状态。"""
        return self._gs_state

    @gs_state.setter
    def gs_state(self, new_state: GraspState) -> None:
        """设置新状态并触发回调。"""
        old_state = self._gs_state
        self._gs_state = new_state
        self._on_state_changed(old_state, new_state)

    # ------------------------------------------------------------------
    #  子类需实现
    # ------------------------------------------------------------------
    def _log_info(self, msg: str) -> None:
        """信息日志 — 子类必须实现。"""
        raise NotImplementedError

    def _log_error(self, msg: str) -> None:
        """错误日志 — 子类必须实现。"""
        raise NotImplementedError

    def _on_state_changed(self, old_state: GraspState, new_state: GraspState) -> None:
        """状态变化回调 — 子类可覆盖以更新 GUI/终端显示。"""
        pass

    # ------------------------------------------------------------------
    #  检测回调
    # ------------------------------------------------------------------
    def _gs_on_detection(self, msg: Detection2DArray) -> None:
        """从检测结果中提取最佳目标的 u 坐标和 bbox 尺寸。"""
        best_det, best_score = find_best_detection(msg.detections, self._gs_target_class)
        if best_det is not None:
            bbox = best_det.bbox
            self._gs_target_u = bbox.center.x
            self._gs_target_v = bbox.center.y
            self._gs_bbox_size_x = bbox.size_x
            self._gs_bbox_size_y = bbox.size_y
            self._gs_last_detect_time = time.time()
            self._gs_update_target_distance()
        else:
            self._gs_target_u = None
            self._gs_target_v = None
            self._gs_target_distance = None

    def _gs_on_depth(self, msg: Image) -> None:
        """缓存最新深度图，支持 16UC1(mm) 和 32FC1(m)。"""
        try:
            self._gs_depth_image = self._gs_bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            self._gs_depth_encoding = msg.encoding
        except Exception as e:
            self._log_error(f"[深度] 深度图转换失败: {e}")

    def _gs_update_target_distance(self) -> None:
        """按检测框中心区域计算目标距离。"""
        self._gs_target_distance = None
        if not self._gs_use_depth or self._gs_depth_image is None:
            return
        if self._gs_target_u is None or self._gs_target_v is None:
            return

        raw = sample_depth_at_pixel(self._gs_depth_image, self._gs_target_u, self._gs_target_v, self._gs_depth_radius)
        if raw is None:
            return
        distance = depth_to_meters(raw, self._gs_depth_encoding)
        if math.isfinite(distance) and distance > 0.0:
            self._gs_target_distance = distance

    # ------------------------------------------------------------------
    #  状态机 tick
    # ------------------------------------------------------------------
    def _gs_tick(self) -> None:
        """状态机主 tick。"""
        if not self._sport.ready:
            return

        if self._gs_state == GraspState.WORKING:
            self._gs_tick_working()
        # IDLE / GRABBING / MENU / DONE 不在 tick 中驱动

    # ------------------------------------------------------------------
    #  统一工作循环（搜索 + 对齐 + 接近，无状态切换）
    # ------------------------------------------------------------------
    def _gs_tick_working(self) -> None:
        """WORKING：搜索 → StepAligner 对齐 → ForwardApproach 接近。

        不做状态切换，一个连续函数处理所有运动阶段。
        对齐逻辑委托给 StepAligner（与 yaw_align.py 共用）。
        接近逻辑委托给 ForwardApproach（与 loco_forward.py 共用）。
        """
        now = time.time()

        # ---- 1. 未检测到目标：原地旋转搜索 ----
        if self._gs_target_u is None or (now - self._gs_last_detect_time > self._gs_lost_timeout):
            if self._gs_aligner.settling:
                self._gs_aligner.stop()
                self._log_info("[工作] 等待中目标丢失，停止旋转")
            self._gs_aligned = False
            self._gs_aligner.reset()
            self._gs_approach.reset()
            self._sport.move(vyaw=self._gs_search_speed)
            return

        # ---- 2. 检测到目标，步进式对齐（委托给 StepAligner） ----
        action, extra = self._gs_aligner.tick(self._gs_target_u)

        if action == AlignAction.ROTATING:
            if extra:
                self._log_info(f"[工作] {extra}")
            return

        if action == AlignAction.WAIT:
            return

        if action == AlignAction.LOST:
            if extra:
                self._log_info(f"[工作] {extra}")
            return

        # ---- 3. 已居中，前进接近（委托给 ForwardApproach） ----
        if not self._gs_aligned:
            self._gs_aligned = True
            self._log_info(f"[工作] {extra}")

        app_action, app_msg = self._gs_approach.tick(
            target_u=self._gs_target_u,
            target_distance=self._gs_target_distance,
            bbox_size_x=self._gs_bbox_size_x,
            bbox_size_y=self._gs_bbox_size_y,
        )

        if app_action == ApproachAction.ARRIVED:
            self.gs_state = GraspState.GRABBING
            self._log_info(f"[工作] {app_msg}，开始抓取")
            self._gs_run_grab()
            return

        if app_action == ApproachAction.DRIFTED:
            self._gs_aligned = False
            self._gs_aligner.reset()
            self._log_info(f"[工作] {app_msg}")
            return

        # APPROACHING / WAIT → 继续等下一 tick

    # ------------------------------------------------------------------
    #  arm 脚本执行
    # ------------------------------------------------------------------
    def _gs_run_grab(self) -> None:
        """执行 armup.py 抓取目标物。"""
        script = str(self._gs_armup_script)
        if not Path(script).exists():
            self._log_error(f"[抓取] armup.py 不存在: {script}")
            self.gs_state = GraspState.MENU
            self._gs_show_menu()
            return

        self._log_info("[抓取] 执行 armup.py ...")

        def _worker():
            try:
                args = [sys.executable, script]
                if self._gs_net_iface:
                    args.append(self._gs_net_iface)
                proc = subprocess.run(
                    args, check=True, input=b"\n",
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    timeout=120,
                )
                if proc.stdout:
                    for line in proc.stdout.decode(errors="replace").splitlines():
                        self._log_info(f"[armup] {line}")
                self._log_info("[抓取] armup.py 执行完成")
            except subprocess.TimeoutExpired:
                self._log_error("[抓取] armup.py 超时（120秒）")
            except subprocess.CalledProcessError as e:
                self._log_error(f"[抓取] armup.py 执行失败: 返回码={e.returncode}")
            except Exception as e:
                self._log_error(f"[抓取] armup.py 异常: {e}")
            finally:
                self.gs_state = GraspState.MENU
                self._gs_show_menu()

        t = threading.Thread(target=_worker, daemon=True)
        t.start()

    def _gs_run_armdown(self) -> None:
        """执行 armdown.py 放下目标物。"""
        script = str(self._gs_armdown_script)
        if not Path(script).exists():
            self._log_error(f"[放下] armdown.py 不存在: {script}")
            return

        self._log_info("[放下] 执行 armdown.py ...")

        try:
            args = [sys.executable, script]
            if self._gs_net_iface:
                args.append(self._gs_net_iface)
            proc = subprocess.run(
                args, check=True, input=b"\n",
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                timeout=120,
            )
            if proc.stdout:
                for line in proc.stdout.decode(errors="replace").splitlines():
                    self._log_info(f"[armdown] {line}")
            self._log_info("[放下] armdown.py 执行完成")
        except subprocess.TimeoutExpired:
            self._log_error("[放下] armdown.py 超时（120秒）")
        except subprocess.CalledProcessError as e:
            self._log_error(f"[放下] armdown.py 失败: 返回码={e.returncode}")
        except Exception as e:
            self._log_error(f"[放下] armdown.py 异常: {e}")

    # ------------------------------------------------------------------
    #  交互菜单（grasp_task 专用，control_panel 通过 GUI 交互）
    # ------------------------------------------------------------------
    def _gs_show_menu(self) -> None:
        """显示交互菜单（在后台线程中运行，不阻塞 ROS2 spin）。

        子类可覆盖此方法以提供不同的交互方式（如 GUI 菜单）。
        """
        def _menu_loop():
            while self._gs_state == GraspState.MENU:
                print("\n" + "=" * 40)
                print("  G1 抓取任务 — 操作菜单")
                print("=" * 40)
                print("  1. 放下目标物")
                print("  2. 右转放下目标物")
                print("  3. 自定义控制（输入 xyz）")
                print("  4. 退出")
                print("=" * 40)

                try:
                    choice = input("请选择 [1-4]: ").strip()
                except EOFError:
                    choice = "4"

                if choice == "1":
                    self._gs_do_put_down()
                elif choice == "2":
                    self._gs_do_turn_and_put_down()
                elif choice == "3":
                    self._gs_do_custom_control()
                elif choice == "4":
                    self._log_info("[退出] 任务结束")
                    self.gs_state = GraspState.DONE
                    rclpy.shutdown()
                    return
                else:
                    print("无效选择，请输入 1-4")

        t = threading.Thread(target=_menu_loop, daemon=True)
        t.start()

    # ------------------------------------------------------------------
    #  菜单操作
    # ------------------------------------------------------------------
    def _gs_do_put_down(self) -> None:
        """放下目标物。"""
        self._gs_run_armdown()

    def _gs_do_turn_and_put_down(self) -> None:
        """右转 90° 后放下目标物。"""
        self._log_info("[右转] 开始右转 90° ...")
        self._sport.move(vyaw=-self._gs_turn_speed)
        time.sleep(self._gs_turn_duration)
        self._sport.stop()
        self._log_info("[右转] 右转完成")
        self._gs_do_put_down()

    def _gs_do_custom_control(self) -> None:
        """自定义控制：输入 x y vyaw 控制机器人行动。"""
        print("\n  自定义控制模式")
        print("  输入 x y vyaw（m/s, m/s, rad/s），例如: 0.2 0.0 0.3")
        print("  输入 q 返回菜单")
        print("-" * 40)

        while self._gs_state == GraspState.MENU:
            try:
                line = input("xyz> ").strip()
            except EOFError:
                break

            if line.lower() == "q":
                self._sport.stop()
                print("  已停止运动，返回菜单")
                return

            parts = line.split()
            if len(parts) != 3:
                print("  格式错误，请输入: x y vyaw（三个数值）")
                continue

            try:
                vx = float(parts[0])
                vy = float(parts[1])
                vyaw = float(parts[2])
            except ValueError:
                print("  格式错误，请输入数字")
                continue

            # 安全限幅
            vx = max(-1.0, min(1.0, vx))
            vy = max(-0.5, min(0.5, vy))
            vyaw = max(-1.0, min(1.0, vyaw))

            self._log_info(f"[自定义] cmd: vx={vx}, vy={vy}, vyaw={vyaw}")
            self._sport.move(vx=vx, vy=vy, vyaw=vyaw)

    # ------------------------------------------------------------------
    #  清理
    # ------------------------------------------------------------------
    def _gs_destroy(self) -> None:
        """停止运动并清理。"""
        self._sport.stop()
        self._log_info("[清理] 抓取状态机已停止")
