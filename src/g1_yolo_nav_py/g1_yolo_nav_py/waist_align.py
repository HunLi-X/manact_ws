"""
腰部对齐节点 — 纯视觉伺服，通过旋转腰部让目标保持在画面中心
...
"""

# ==================================================================
# 1. 标准库导入
# ==================================================================
import os   # sys.path 修改
import sys  # sys.path 修改
import math  # 角度弧度转换
import threading  # DDS 控制线程
import time  # 计时与延时
from typing import Optional  # 类型注解

# ROS2 colcon 隔离 PYTHONPATH，必须在所有 import 之前追加路径
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
import numpy as np  # 数值计算，用于 np.clip 限幅
import rclpy  # ROS2 Python 客户端库
from rclpy.node import Node  # ROS2 节点基类
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # QoS 配置
from vision_msgs.msg import Detection2DArray  # 2D 检测结果消息

# unitree_sdk2py: 宇树机器人底层 SDK（可选依赖）
try:
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,  # DDS 通信工厂初始化
        ChannelPublisher,  # DDS 发布者，发送 Arm SDK 指令
        ChannelSubscriber,  # DDS 订阅者，接收关节状态
    )
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_  # 低层指令消息默认构造
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_  # 低层指令/状态 IDL 消息类型
    from unitree_sdk2py.utils.crc import CRC  # CRC 校验，G1 固件要求每帧指令附带
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False


class G1JointIndex:
    WaistYaw = 12
    kNotUsedJoint = 29


class WaistAlignNode(Node):
    """
    腰部对齐节点 — 视觉伺服让目标居中。

    每帧（50Hz）：
        1. 从检测结果提取目标 u 坐标
        2. 计算偏离中心的误差 angle_error = (u - 0.5) * FOV
        3. P 控制增量 delta = kp * angle_error
        4. 限速后累加到目标角度，发送 Arm SDK DDS 指令
    """

    def __init__(self) -> None:
        super().__init__("g1_waist_align_node")

        # ---- 参数 ----
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("target_class_id", "chair")
        self.declare_parameter("camera_fov_deg", 87.0)
        self.declare_parameter("center_tolerance", 0.08)
        self.declare_parameter("waist_kp", 1.5)
        self.declare_parameter("max_waist_speed", 0.5)
        self.declare_parameter("max_waist_angle", 0.8)
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("lost_timeout", 1.0)
        self.declare_parameter("network_interface", "")

        p = lambda n: self.get_parameter(n).value
        self._det_topic = p("detection_topic")
        self._target_class = p("target_class_id")
        self._fov_rad = math.radians(p("camera_fov_deg"))
        self._center_tol = float(p("center_tolerance"))
        self._kp = float(p("waist_kp"))
        self._max_speed = float(p("max_waist_speed"))
        self._max_angle = float(p("max_waist_angle"))
        self._ctrl_rate = float(p("control_rate"))
        self._lost_timeout = float(p("lost_timeout"))
        self._net_iface = p("network_interface")

        # ---- 内部状态 ----
        self._target_u: Optional[float] = None
        self._last_detect_time: float = 0.0

        self._low_cmd: Optional[LowCmd_] = None
        self._low_state: Optional[LowState_] = None
        self._waist_target: float = 0.0
        self._waist_current: float = 0.0
        self._dds_ok: bool = False
        self._running: bool = True
        self._crc = CRC()

        # ---- ROS2 订阅 ----
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Detection2DArray, self._det_topic,
                                 self._on_detection, qos)

        # ---- 初始化 DDS ----
        if not SDK_AVAILABLE:
            self.get_logger().error("unitree_sdk2py 未安装！pip install unitree_sdk2py")
            return

        self._init_dds()
        self._start_thread()

        self.get_logger().info(
            f"腰部对齐节点就绪: 目标={self._target_class}, "
            f"kp={self._kp}, 容差={self._center_tol}"
        )

    def _init_dds(self) -> None:
        try:
            # ChannelFactoryInitialize 已在 main() 中提前调用（先于 rclpy.init），
            # 此处只需创建 Publisher / Subscriber
            pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
            pub.Init()
            sub = ChannelSubscriber("rt/lowstate", LowState_)
            sub.Init(self._on_lowstate, 10)
            self._low_cmd = unitree_hg_msg_dds__LowCmd_()
            self._arm_pub = pub
            self._dds_ok = True
            self.get_logger().info("DDS 初始化成功")
        except Exception as e:
            self.get_logger().error(f"DDS 失败: {e}")

    def _start_thread(self) -> None:
        t = threading.Thread(target=self._control_loop, daemon=True)
        t.start()

    def _on_detection(self, msg: Detection2DArray) -> None:
        best_det = None
        best_score = 0.0
        for det in msg.detections:
            if det.results and det.results[0].id == self._target_class:
                if det.results[0].score > best_score:
                    best_score = det.results[0].score
                    best_det = det
        if best_det is not None:
            self._target_u = best_det.bbox.center.position.x
            self._last_detect_time = time.time()
        else:
            self._target_u = None

    def _on_lowstate(self, msg: LowState_) -> None:
        self._low_state = msg
        if msg is not None:
            self._waist_current = msg.motor_state[G1JointIndex.WaistYaw].q

    def _compute_target(self) -> float:
        """P 控制计算腰部目标角度。"""
        if self._target_u is None or time.time() - self._last_detect_time > self._lost_timeout:
            return self._waist_current

        # 在容差范围内不调整
        error = self._target_u - 0.5
        if abs(error) < self._center_tol:
            return self._waist_current

        # 偏移 → 角度误差 → P 增量
        error_angle = error * self._fov_rad
        delta = self._kp * error_angle

        # 限速
        dt = 1.0 / self._ctrl_rate
        max_delta = self._max_speed * dt
        delta = np.clip(delta, -max_delta, max_delta)

        return np.clip(
            self._waist_current + delta,
            -self._max_angle, self._max_angle
        )

    def _control_loop(self) -> None:
        if not self._dds_ok:
            self.get_logger().error("DDS 未初始化")
            return

        dt = 1.0 / self._ctrl_rate
        kp_pd, kd_pd = 100.0, 2.0

        # 等待关节状态
        deadline = time.time() + 10.0
        while self._low_state is None and self._running and time.time() < deadline:
            time.sleep(0.05)
        if self._low_state is None:
            self.get_logger().error("等待状态超时")
            return

        self.get_logger().info("腰部对齐控制启动")

        while self._running:
            t0 = time.time()

            self._waist_target = self._compute_target()

            cmd = self._low_cmd
            cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0   # 使能 arm_sdk
            wc = cmd.motor_cmd[G1JointIndex.WaistYaw]
            wc.mode = 1
            wc.q = self._waist_target
            wc.dq = wc.tau = 0.0
            wc.kp = kp_pd
            wc.kd = kd_pd
            cmd.crc = self._crc.Crc(cmd)
            self._arm_pub.Write(cmd)

            sleep_t = dt - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    def destroy_node(self) -> None:
        self._running = False
        super().destroy_node()


def main(args=None):
    # 从命令行参数提取网卡名（在 --ros-args 之前的第一个非选项参数）
    _iface = ""
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        _iface = sys.argv[1]

    # 在 rclpy.init() 之前初始化 unitree DDS（CycloneDDS 兼容层）
    from g1_yolo_nav_py._dds_compat import init_unitree_dds_before_ros2
    init_unitree_dds_before_ros2(_iface)

    rclpy.init(args=args)
    node = WaistAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
