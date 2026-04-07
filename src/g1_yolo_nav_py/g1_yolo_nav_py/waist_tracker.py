"""
视觉伺服追踪与趋近控制节点

功能：
    1. 检测目标在图像中的位置
    2. 旋转腰部（Arm SDK）让目标居中对齐
    3. 目标对齐后，通过 LocoClient 控制机器人前进到目标位置

控制策略：
    ┌─────────────┐     ┌──────────────┐     ┌─────────────┐
    │ YOLO 检测   │ ──→ │ 腰部旋转对齐  │ ──→ │ Loco前进    │
    │ (图像坐标)   │     │ (Arm SDK)     │     │ (RPC)       │
    └─────────────┘     └──────────────┘     └─────────────┘

状态机：
    IDLE → TRACKING(腰部对齐) → APPROACHING(前进) → ARRIVED(到达)

运行方式：
    ros2 run g1_yolo_nav_py waist_tracker
    ros2 launch g1_yolo_nav_py yolo_nav.launch.py enable_waist_tracking:=true
"""

import math
import threading
import time
from typing import Optional
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from vision_msgs.msg import Detection2DArray


# ======================================================================
# unitree_sdk2py 导入（可选依赖）
# ======================================================================
try:
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC
    UNITREE_SDK_AVAILABLE = True
except ImportError:
    UNITREE_SDK_AVAILABLE = False

try:
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    LOCO_AVAILABLE = True
except ImportError:
    LOCO_AVAILABLE = False


# ======================================================================
# G1 关节索引常量
# ======================================================================
class G1JointIndex:
    """G1 机器人关节索引（LowCmd/LowState 数组下标）。"""
    WaistYaw = 12           # 腰部偏航
    kNotUsedJoint = 29      # Arm SDK 使能位（q=1 使能，q=0 失能）


# ======================================================================
# 状态机枚举
# ======================================================================
class TrackState(Enum):
    """视觉伺服状态机。"""
    IDLE = auto()            # 空闲（未检测到目标）
    ALIGNING = auto()        # 正在对齐（腰部旋转让目标居中）
    APPROACHING = auto()     # 对齐完成，正在前进
    ARRIVED = auto()         # 到达目标附近
    LOST = auto()            # 目标丢失


class VisualServoNode(Node):
    """
    视觉伺服追踪节点 — 腰部对齐 + LocoClient 前进。

    核心逻辑：
        1. 从 /g1/vision/detections 获取目标检测框中心 u 坐标
        2. 若 |u - 0.5| > 容差 → 状态=ALIGNING，旋转腰部
        3. 若 |u - 0.5| <= 容差 且 持续稳定 → 切换到 APPROACHING
        4. APPROACHING 时调用 LocoClient.Move(vx) 前进
        5. 检测框面积/高度足够大时 → ARRIVED，停止前进
    """

    def __init__(self) -> None:
        super().__init__("g1_visual_servo_node")

        # ------------------------------------------------------------------
        # ROS2 参数
        # ------------------------------------------------------------------
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("target_class_id", "chair")

        # 相机参数
        self.declare_parameter("camera_fov_deg", 87.0)
        self.declare_parameter("center_tolerance", 0.08)         # 居中容差（归一化）

        # 腰部控制参数（Arm SDK DDS）
        self.declare_parameter("waist_kp", 1.5)                 # 腰部 P 增益
        self.declare_parameter("max_waist_speed", 0.5)          # 最大腰部速度 rad/s
        self.declare_parameter("max_waist_angle", 0.8)          # 最大腰部角度 ~45°
        self.declare_parameter("control_rate", 50.0)             # DDS 频率 Hz

        # 前进控制参数（LocoClient RPC）
        self.declare_parameter("forward_speed", 0.3)            # 前进速度 m/s
        self.declare_parameter("align_stable_time", 0.8)        # 对齐稳定时间 s（持续居中多久才算对齐成功）
        self.declare_parameter("approach_bbox_ratio", 0.45)     # 到达条件：检测框占画面比例

        # 安全参数
        self.declare_parameter("lost_timeout", 1.0)             # 目标丢失超时 s
        self.declare_parameter("network_interface", "")           # 网络接口

        # ---- 读取参数 ----
        p = lambda name: self.get_parameter(name).value
        self._det_topic = p("detection_topic")
        self._target_class = p("target_class_id")
        self._fov_rad = math.radians(p("camera_fov_deg"))
        self._center_tol = float(p("center_tolerance"))

        self._waist_kp = float(p("waist_kp"))
        self._waist_max_speed = float(p("max_waist_speed"))
        self._waist_max_angle = float(p("max_waist_angle"))
        self._ctrl_rate = float(p("control_rate"))

        self._forward_speed = float(p("forward_speed"))
        self._align_stable_time = float(p("align_stable_time"))
        self._arrive_bbox_ratio = float(p("approach_bbox_ratio"))
        self._lost_timeout = float(p("lost_timeout"))
        self._net_iface = p("network_interface")

        # ------------------------------------------------------------------
        # 内部状态
        # ------------------------------------------------------------------
        self._state = TrackState.IDLE
        self._target_u: Optional[float] = None      # 目标中心 x（归一化）
        self._target_v: Optional[float] = None      # 目标中心 y（归一化）
        self._bbox_size_x: float = 0.0              # 框宽度占比
        self._bbox_size_y: float = 0.0              # 框高度占比
        self._last_detect_time: float = 0.0

        # 对齐计时器：记录目标进入容差范围的起始时刻
        self._align_start_time: Optional[float] = None

        # DDS 控制（腰部 Arm SDK）
        self._low_cmd: Optional[LowCmd_] = None
        self._low_state: Optional[LowState_] = None
        self._waist_target: float = 0.0
        self._waist_current: float = 0.0
        self._dds_ok: bool = False
        self._running: bool = True
        self._crc = CRC()

        # LocoClient（前进运动 RPC）
        self._loco: Optional[LocoClient] = None
        self._loco_moving: bool = False               # 当前是否在移动中

        # ------------------------------------------------------------------
        # ROS2 订阅
        # ------------------------------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Detection2DArray, self._det_topic, self._on_detection, qos)

        # ------------------------------------------------------------------
        # 初始化
        # ------------------------------------------------------------------
        if not UNITREE_SDK_AVAILABLE:
            self.get_logger().error("unitree_sdk2py 未安装！请执行: pip install unitree_sdk2py")
            return

        if not LOCO_AVAILABLE:
            self.get_logger().error("LocoClient 不可用！检查 unitree_sdk2py 版本")
            return

        self._init_dds()
        self._init_loco()
        self._start_control_thread()

        self.get_logger().info(
            f"视觉伺服节点就绪: 目标={self._target_class}, "
            f"FOV={math.degrees(self._fov_rad):.0f}°, "
            f"前速={self._forward_speed}m/s"
        )

    # ------------------------------------------------------------------
    # 初始化方法
    # ------------------------------------------------------------------

    def _init_dds(self) -> None:
        """初始化 Arm SDK DDS 通信（用于腰部旋转）。"""
        try:
            ChannelFactoryInitialize(0, self._net_iface or "")

            pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
            pub.Init()

            sub = ChannelSubscriber("rt/lowstate", LowState_)
            sub.Init(self._on_lowstate, 10)

            self._low_cmd = unitree_hg_msg_dds__LowCmd_()
            self._arm_pub = pub
            self._dds_ok = True
            self.get_logger().info("DDS(Arm SDK) 初始化成功")
        except Exception as e:
            self.get_logger().error(f"DDS 初始化失败: {e}")
            self._dds_ok = False

    def _init_loco(self) -> None:
        """初始化 LocoClient RPC（用于前进运动）。"""
        try:
            self._loco = LocoClient()
            self._loco.SetTimeout(5.0)
            self._loco.Init()
            self.get_logger().info("LocoClient(RPC) 初始化成功")
        except Exception as e:
            self.get_logger().error(f"LocoClient 初始化失败: {e}")

    def _start_control_thread(self) -> None:
        """启动 50Hz 控制线程。"""
        t = threading.Thread(target=self._control_loop, daemon=True, name="vis_servo")
        t.start()

    # ------------------------------------------------------------------
    # 回调函数
    # ------------------------------------------------------------------

    def _on_detection(self, msg: Detection2DArray) -> None:
        """
        检测结果回调 — 提取目标位置和框大小。

        提取信息：
            - target_u: 归一化的水平中心位置 [0, 1]，0.5 = 画面正中央
            - target_v: 归一化的垂直中心位置 [0, 1]
            - bbox_size_x/y: 归一化的框宽高（用于判断距离远近）
        """
        best_det = None
        best_score = 0.0

        for det in msg.detections:
            if det.results and det.results[0].id == self._target_class:
                if det.results[0].score > best_score:
                    best_score = det.results[0].score
                    best_det = det

        if best_det is not None:
            bbox = best_det.bbox
            self._target_u = bbox.center.position.x
            self._target_v = bbox.center.position.y
            self._bbox_size_x = bbox.size_x
            self._bbox_size_y = bbox.size_y
            self._last_detect_time = time.time()
        else:
            self._target_u = None
            self._target_v = None

    def _on_lowstate(self, msg: LowState_) -> None:
        """低层关节状态回调。"""
        self._low_state = msg
        if msg is not None:
            self._waist_current = msg.motor_state[G1JointIndex.WaistYaw].q

    # ------------------------------------------------------------------
    # 状态机更新
    # ------------------------------------------------------------------

    def _update_state(self) -> None:
        """
        根据当前检测结果更新状态机。

        状态转换规则：

        ┌────────┐ 有目标且偏离大  ┌──────────┐  居中稳定  ┌────────────┐
        │  IDLE  │ ─────────────→ │ ALIGNING │ ────────→ │APPROACHING│
        └────────┘                └──────────┘           └────────────┘
           ↑                          ↑                        │
           │  目标丢失                 │  目标偏离              │  到达/丢失
           └──────────────────────────┴────────────────────────┘
        """
        now = time.time()

        # ---- 检查目标是否超时丢失 ----
        if self._target_u is None or (now - self._last_detect_time > self._lost_timeout):
            if self._state != TrackState.LOST:
                self._on_state_change(TrackState.LOST)
            self._align_start_time = None
            return

        # 计算偏离中心的误差
        error = abs(self._target_u - 0.5)

        # ---- 状态判断 ----
        if error > self._center_tol:
            # 目标偏离较大 → 对齐阶段
            if self._state != TrackState.ALIGNING:
                self._on_state_change(TrackState.ALIGNING)
            self._align_start_time = None
            self._stop_loco_move()

        else:
            # 目标接近中心 → 开始/继续计时
            if self._align_start_time is None:
                self._align_start_time = now

            aligned_duration = now - self._align_start_time

            # ---- 检查是否到达（检测框足够大表示够近）----
            if max(self._bbox_size_x, self._bbox_size_y) >= self._arrive_bbox_ratio:
                if self._state != TrackState.ARRIVED:
                    self._on_state_change(TrackState.ARRIVED)
                self._stop_loco_move()
            elif aligned_duration >= self._align_stable_time:
                # 对齐稳定 → 进入前进阶段
                if self._state != TrackState.APPROACHING:
                    self._on_state_change(TrackState.APPROACHING)
                self._start_loco_move()
            else:
                # 还在对齐中
                if self._state != TrackState.ALIGNING:
                    self._on_state_change(TrackState.ALIGNING)
                self._stop_loco_move()

    def _on_state_change(self, new_state: TrackState) -> None:
        """状态切换回调（日志输出）。"""
        old = self._state.name
        self._state = new_state
        self.get_logger().info(f"状态切换: {old} → {new_state.name}")

    # ------------------------------------------------------------------
    # 腰部控制计算（Arm SDK DDS）
    # ------------------------------------------------------------------

    def _compute_waist(self) -> float:
        """
        计算 WaistYaw 的 PD 控制目标角度。

        仅在 ALIGNING 和 APPROACHING 状态下进行微调。
        返回目标弧度值。
        """
        if self._target_u is None:
            return self._waist_current

        # 归一化偏移 → 角度误差
        error_u = self._target_u - 0.5
        error_angle = error_u * self._fov_rad

        # P 控制增量
        delta = self._waist_kp * error_angle

        # 限速
        dt = 1.0 / self._ctrl_rate
        max_delta = self._waist_max_speed * dt
        delta = np.clip(delta, -max_delta, max_delta)

        # 累加并限幅
        target = np.clip(self._waist_current + delta, -self._waist_max_angle, self._waist_max_angle)
        return target

    # ------------------------------------------------------------------
    # LocoClient 运动控制（RPC）
    # ------------------------------------------------------------------

    def _start_loco_move(self) -> None:
        """开始前进。"""
        if self._loco is None or self._loco_moving:
            return
        try:
            self._loco.Move(
                vx=self._forward_speed,
                vy=0.0,
                vyaw=0.0,
                continuous=True,
            )
            self._loco_moving = True
        except Exception as e:
            self.get_logger().warn(f"Loco Move 失败: {e}")

    def _stop_loco_move(self) -> None:
        """停止前进。"""
        if not self._loco_moving:
            return
        try:
            self._loco.StopMove()
            self._loco_moving = False
        except Exception as e:
            self.get_logger().warn(f"Loco StopMove 失败: {e}")

    # ------------------------------------------------------------------
    # 主控制循环 (50Hz)
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        """
        50Hz 控制循环。

        每帧执行：
            1. 更新状态机
            2. 计算腰部目标角度
            3. 发送 Arm SDK DDS 指令（腰部 PD 控制）
            4. LocoClient 前进由状态机触发（非每帧调用）
        """
        if not self._dds_ok:
            self.get_logger().error("DDS 未初始化，控制退出")
            return

        dt = 1.0 / self._ctrl_rate
        kp_pd = 100.0      # 电机级位置增益
        kd_pd = 2.0        # 电机级阻尼增益

        # 等待关节状态
        deadline = time.time() + 10.0
        while self._low_state is None and self._running and time.time() < deadline:
            time.sleep(0.05)

        if self._low_state is None:
            self.get_logger().error("等待关节状态超时")
            return

        self.get_logger().info("控制循环启动 (50Hz)")

        while self._running:
            t0 = time.time()

            # ---- 1. 状态机更新 ----
            self._update_state()

            # ---- 2. 计算腰部目标 ----
            self._waist_target = self._compute_waist()

            # ---- 3. 发送 Arm SDK DDS 指令 ----
            cmd = self._low_cmd

            # 使能 arm_sdk
            cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0

            # WaistYaw PD 控制
            wc = cmd.motor_cmd[G1JointIndex.WaistYaw]
            wc.mode = 1
            wc.q = self._waist_target
            wc.dq = 0.0
            wc.tau = 0.0
            wc.kp = kp_pd
            wc.kd = kd_pd

            # CRC 并发送
            cmd.crc = self._crc.Crc(cmd)
            self._arm_pub.Write(cmd)

            # ---- 4. 定时 ----
            elapsed = time.time() - t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ------------------------------------------------------------------
    # 清理
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self._running = False
        self._stop_loco_move()
        super().destroy_node()


# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
