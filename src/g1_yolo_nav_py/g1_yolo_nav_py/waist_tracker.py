"""
腰部追踪节点 — 通过旋转腰部让目标保持在画面中心

视觉伺服原理：
    1. 检测目标在图像中的位置（归一化坐标 u ∈ [0,1]）
    2. 计算相对于画面中心的偏移：error = u - 0.5
    3. 将偏移转换为角度误差（基于相机 FOV）
    4. 使用 P 控制计算腰部旋转目标角度
    5. 通过 unitree_sdk2py DDS 控制 WaistYaw 关节

运行前提：
    - G1 机器人已连接且处于站立状态
    - YOLO 检测节点已启动并发布检测结果
    - 相机已启动

启动方式：
    ros2 run g1_yolo_nav_py waist_tracker
"""

import math
import sys
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from vision_msgs.msg import Detection2DArray

# unitree_sdk2py DDS 通信
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC
    UNITREE_SDK_AVAILABLE = True
except ImportError:
    UNITREE_SDK_AVAILABLE = False


# ======================================================================
# G1 关节索引常量
# ======================================================================
class G1JointIndex:
    """G1 机器人关节索引（与 LowCmd/LowState 数组下标对应）。"""
    WaistYaw = 12           # 腰部偏航（旋转）
    WaistRoll = 13          # 腰部横滚
    WaistPitch = 14         # 腰部俯仰
    kNotUsedJoint = 29      # arm_sdk 使能标志（q=1 使能，q=0 失能）


class WaistTrackerNode(Node):
    """
    腰部追踪节点 — 视觉伺服控制腰部旋转。

    控制策略：
        - 目标偏离画面中心时，旋转腰部让目标居中
        - 使用 P 控制平滑追踪，避免剧烈抖动
        - 限制最大旋转速度和角度范围，确保安全
    """

    def __init__(self) -> None:
        super().__init__("g1_waist_tracker_node")

        # ------------------------------------------------------------------
        # ROS2 参数
        # ------------------------------------------------------------------
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("target_class_id", "chair")
        self.declare_parameter("camera_fov_deg", 87.0)        # D455 水平 FOV
        self.declare_parameter("kp", 1.5)                     # P 控制增益
        self.declare_parameter("max_waist_speed", 0.5)        # 最大旋转速度 (rad/s)
        self.declare_parameter("max_waist_angle", 0.8)        # 最大旋转角度 (rad, ~45°)
        self.declare_parameter("center_tolerance", 0.05)      # 中心容差（归一化坐标）
        self.declare_parameter("control_rate", 50.0)          # DDS 控制频率 (Hz)
        self.declare_parameter("network_interface", "")       # 网络接口（空则自动检测）

        self._detection_topic = self.get_parameter("detection_topic").value
        self._target_class = self.get_parameter("target_class_id").value
        self._camera_fov = math.radians(self.get_parameter("camera_fov_deg").value)
        self._kp = float(self.get_parameter("kp").value)
        self._max_speed = float(self.get_parameter("max_waist_speed").value)
        self._max_angle = float(self.get_parameter("max_waist_angle").value)
        self._center_tol = float(self.get_parameter("center_tolerance").value)
        self._control_rate = float(self.get_parameter("control_rate").value)
        self._net_iface = self.get_parameter("network_interface").value

        # ------------------------------------------------------------------
        # 内部状态
        # ------------------------------------------------------------------
        self._target_u: Optional[float] = None      # 目标中心 u 坐标（归一化）
        self._target_score: float = 0.0             # 目标置信度
        self._last_detection_time: float = 0.0      # 最后检测时间戳

        # DDS 控制相关（由控制线程使用）
        self._low_cmd: Optional[LowCmd_] = None
        self._low_state: Optional[LowState_] = None
        self._waist_target: float = 0.0             # 腰部目标角度
        self._waist_current: float = 0.0            # 腰部当前角度
        self._dds_initialized: bool = False
        self._running: bool = True
        self._crc = CRC()

        # ------------------------------------------------------------------
        # ROS2 订阅
        # ------------------------------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._det_sub = self.create_subscription(
            Detection2DArray, self._detection_topic, self._detection_callback, qos
        )

        # ------------------------------------------------------------------
        # 初始化 DDS
        # ------------------------------------------------------------------
        if not UNITREE_SDK_AVAILABLE:
            self.get_logger().error("unitree_sdk2py 未安装，无法控制腰部")
            self.get_logger().error("请执行: pip install unitree_sdk2py")
            return

        self._init_dds()

        # ------------------------------------------------------------------
        # 启动 DDS 控制线程
        # ------------------------------------------------------------------
        self._dds_thread = threading.Thread(target=self._dds_control_loop, daemon=True)
        self._dds_thread.start()

        self.get_logger().info(
            f"腰部追踪节点启动: 目标={self._target_class}, "
            f"FOV={math.degrees(self._camera_fov):.1f}°, KP={self._kp}"
        )

    # ----------------------------------------------------------------------
    def _init_dds(self) -> None:
        """初始化 unitree_sdk2py DDS 通信。"""
        try:
            # 初始化 DDS 工厂
            if self._net_iface:
                ChannelFactoryInitialize(0, self._net_iface)
            else:
                ChannelFactoryInitialize(0)

            # 创建指令发布者
            self._arm_sdk_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
            self._arm_sdk_pub.Init()

            # 创建状态订阅者
            self._lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
            self._lowstate_sub.Init(self._lowstate_callback, 10)

            # 初始化指令消息
            self._low_cmd = unitree_hg_msg_dds__LowCmd_()

            self._dds_initialized = True
            self.get_logger().info("DDS 初始化成功")

        except Exception as e:
            self.get_logger().error(f"DDS 初始化失败: {e}")
            self._dds_initialized = False

    # ----------------------------------------------------------------------
    def _lowstate_callback(self, msg: LowState_) -> None:
        """低层状态回调 — 获取腰部当前角度。"""
        self._low_state = msg
        if msg is not None:
            self._waist_current = msg.motor_state[G1JointIndex.WaistYaw].q

    # ----------------------------------------------------------------------
    def _detection_callback(self, msg: Detection2DArray) -> None:
        """检测结果回调 — 提取目标位置。"""
        best_det = None
        best_score = 0.0

        for det in msg.detections:
            if det.results and det.results[0].id == self._target_class:
                if det.results[0].score > best_score:
                    best_score = det.results[0].score
                    best_det = det

        if best_det is not None:
            # 目标中心 u 坐标（归一化，0=左边缘，0.5=中心，1=右边缘）
            self._target_u = best_det.bbox.center.position.x
            self._target_score = best_score
            self._last_detection_time = time.time()
        else:
            # 没有检测到目标
            self._target_u = None

    # ----------------------------------------------------------------------
    def _compute_waist_target(self) -> None:
        """
        计算腰部目标角度。

        视觉伺服原理：
            - 目标偏离中心 -> 计算角度误差 -> 累加到腰部目标角度
            - u = 0.5 时目标居中，不调整
            - u > 0.5 时目标在右侧，需要右转（waist_yaw 正方向）
            - u < 0.5 时目标在左侧，需要左转（waist_yaw 负方向）

        角度误差计算：
            error_u = u - 0.5              # 归一化偏移 [-0.5, 0.5]
            error_angle = error_u × FOV    # 角度误差 (rad)
            delta = kp × error_angle       # P 控制增量
        """
        if self._target_u is None:
            return

        # 检查检测是否超时（超过 0.5 秒未更新）
        if time.time() - self._last_detection_time > 0.5:
            self._target_u = None
            return

        # 检查是否在中心容差范围内
        error_u = self._target_u - 0.5
        if abs(error_u) < self._center_tol:
            return

        # 计算角度误差
        error_angle = error_u * self._camera_fov

        # P 控制增量
        delta = self._kp * error_angle

        # 限制最大增量（避免剧烈抖动）
        max_delta_per_frame = self._max_speed / self._control_rate
        delta = np.clip(delta, -max_delta_per_frame, max_delta_per_frame)

        # 累加到目标角度
        self._waist_target = np.clip(
            self._waist_current + delta,
            -self._max_angle,
            self._max_angle
        )

    # ----------------------------------------------------------------------
    def _dds_control_loop(self) -> None:
        """
        DDS 控制循环 — 50Hz 发送腰部控制指令。

        控制阶段：
            1. 等待状态消息到达
            2. 使能 arm_sdk（kNotUsedJoint.q = 1）
            3. 根据检测结果计算目标角度
            4. PD 位置控制发送指令
        """
        if not self._dds_initialized:
            self.get_logger().error("DDS 未初始化，控制线程退出")
            return

        control_dt = 1.0 / self._control_rate
        kp = 100.0      # 位置控制增益
        kd = 2.0        # 阻尼增益

        # 等待状态消息
        timeout = 10.0
        start_time = time.time()
        while self._low_state is None and self._running:
            time.sleep(0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error("等待关节状态超时")
                return

        self.get_logger().info("开始腰部追踪控制")

        while self._running:
            loop_start = time.time()

            # 计算目标角度
            self._compute_waist_target()

            # 使能 arm_sdk
            self._low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0

            # 设置腰部控制指令
            waist_cmd = self._low_cmd.motor_cmd[G1JointIndex.WaistYaw]
            waist_cmd.q = self._waist_target
            waist_cmd.dq = 0.0
            waist_cmd.kp = kp
            waist_cmd.kd = kd
            waist_cmd.tau = 0.0

            # 保持其他关节不变（读取当前状态）
            # 这里只控制 WaistYaw，其他关节保持当前位置

            # CRC 校验
            self._low_cmd.crc = self._crc.Crc(self._low_cmd)

            # 发送指令
            self._arm_sdk_pub.Write(self._low_cmd)

            # 控制周期
            elapsed = time.time() - loop_start
            sleep_time = control_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ----------------------------------------------------------------------
    def destroy_node(self) -> None:
        """清理资源。"""
        self._running = False
        if hasattr(self, '_dds_thread') and self._dds_thread.is_alive():
            self._dds_thread.join(timeout=1.0)
        super().destroy_node()


# ======================================================================
def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaistTrackerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
