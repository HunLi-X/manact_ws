"""G1 运动控制客户端 — Loco API 封装（参考 ctrl_keyboard 已验证可用方案）。

提供统一的运动控制接口，所有需要控制 G1 机器人运动的节点
都应使用本模块，避免重复定义 API 常量和 Request 发布逻辑。

全部使用 Loco API（ctrl_keyboard 已验证可用）：
    - 状态机控制: SET_FSM_ID(7101) — 参数 {"data": fsm_id}
    - 速度控制:   SET_VELOCITY(7105) — 参数 {"velocity": [vx, vy, vyaw], "duration": t}
    - 平衡模式:   SET_BALANCE_MODE(7102) — 参数 {"data": mode}
    - 速度模式:   SET_SPEED_MODE(7107) — 参数 {"data": mode}

使用方式：
    from g1_yolo_nav_py.sport_client import SportClient

    class MyNode(Node):
        def __init__(self):
            ...
            self._sport = SportClient(self)
            self._sport.init_fsm()

        def move_forward(self):
            if self._sport.ready:
                self._sport.move(vx=0.3)

        def stop(self):
            self._sport.stop()
"""

# ==================================================================
# Loco API 常量（ctrl_keyboard 已验证可用）
# ==================================================================
class LocoAPI:
    """Loco Mode API ID 常量（7xxx 系列，ctrl_keyboard 已验证可用）。"""
    GET_FSM_ID = 7001
    GET_FSM_MODE = 7002
    GET_BALANCE_MODE = 7003
    GET_SWING_HEIGHT = 7004
    GET_STAND_HEIGHT = 7005
    GET_PHASE = 7006

    SET_FSM_ID = 7101         # 状态机控制
    SET_BALANCE_MODE = 7102   # 平衡模式
    SET_SWING_HEIGHT = 7103   # 摆动高度
    SET_STAND_HEIGHT = 7104   # 站立高度
    SET_VELOCITY = 7105       # 速度控制
    SET_SPEED_MODE = 7107     # 速度模式


# ==================================================================
# FSM 状态机 ID（ctrl_keyboard 已验证可用）
# ==================================================================
class FSM_ID:
    """状态机 ID 映射（ctrl_keyboard 已验证可用）。"""
    ZERO_TORQUE = 0    # 零力矩模式（初始坐姿）
    DAMP = 1           # 阻尼控制模式（从坐姿准备站立的过渡）
    SQUAT = 2          # 蹲下
    SIT = 3            # 坐下（稳定坐姿）
    STAND_UP = 4       # 锁定站立
    START = 500        # 常规运控
    WALK_RUN = 801     # 走跑运控


# ==================================================================
# 平衡模式（ctrl_keyboard 已验证可用）
# ==================================================================
class BalanceMode:
    """平衡模式常量。"""
    BALANCE_STAND = 0     # 平衡站立（速度为0时停止踏步）
    CONTINUOUS_GAIT = 1   # 连续步态（持续踏步）


# ==================================================================
# SportClient — Loco API 运动控制客户端
# ==================================================================
import json
import time
import threading
from typing import Optional, Callable

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request


class SportClient:
    """G1 运动控制客户端 — Loco API 封装（参考 ctrl_keyboard 已验证方案）。

    功能：
        - 统一 Request 发布（每次创建新对象，避免 DDS 缓冲区复用问题）
        - FSM 状态机管理（DAMP → STAND_UP → WALK_RUN → CONTINUOUS_GAIT）
        - 速度控制（SET_VELOCITY，带 duration 参数）
        - 停止 / 坐下 / 站立

    使用：
        sport = SportClient(node)
        sport.init_fsm()       # 后台线程初始化 FSM
        sport.move(vx=0.3)     # 前进
        sport.stop()           # 停止
    """

    def __init__(self, node: Node) -> None:
        """初始化运动控制客户端。

        Args:
            node: ROS2 节点实例（用于创建 publisher 和日志）
        """
        self._node = node
        self._pub = node.create_publisher(
            Request, '/api/sport/request', 10
        )
        self._ready: bool = False
        self._sub_warned: bool = False  # 是否已警告过无订阅者
        # 默认 MOVE 持续时间（秒），参考 ctrl_keyboard: 连续=1.0，单次=0.2
        self._move_duration: float = 1.0
        # 停止时的 duration
        self._stop_duration: float = 0.5

    # ------------------------------------------------------------------
    #  属性
    # ------------------------------------------------------------------
    @property
    def ready(self) -> bool:
        """FSM 是否已初始化完成，可以接受运动指令。"""
        return self._ready

    # ------------------------------------------------------------------
    #  诊断
    # ------------------------------------------------------------------
    def check_subscribers(self) -> int:
        """返回 /api/sport/request 话题的订阅者数量。"""
        return self._pub.get_subscription_count() if hasattr(self._pub, 'get_subscription_count') else 0

    def has_subscribers(self) -> bool:
        """检查 /api/sport/request 是否有订阅者。"""
        try:
            count = self._node.count_subscribers('/api/sport/request')
            return count > 0
        except Exception:
            return True  # 无法判断时假设正常

    # ------------------------------------------------------------------
    #  核心：发布 Request
    # ------------------------------------------------------------------
    def publish(self, api_id: int, params: Optional[dict] = None) -> None:
        """发布运动控制请求。

        每次调用都创建新的 Request 对象，避免 DDS 缓冲区复用导致的数据损坏。

        Args:
            api_id: API ID（如 LocoAPI.SET_VELOCITY, LocoAPI.SET_FSM_ID）
            params: 参数字典，会被 JSON 序列化
        """
        req = Request()
        req.header.identity.api_id = api_id
        if params is not None:
            req.parameter = json.dumps(params)
        self._pub.publish(req)

    # ------------------------------------------------------------------
    #  FSM 状态机初始化（Loco API，参考 ctrl_keyboard 已验证方案）
    # ------------------------------------------------------------------
    def init_fsm(self, auto_balance_stand: bool = True,
                 callback: Optional[Callable[[], None]] = None) -> None:
        """在后台线程执行 FSM 初始化（Loco API 方式，参考 ctrl_keyboard）。

        流程（与 ctrl_keyboard.auto_ctrl 一致）：
            SET_FSM_ID(DAMP) → SET_FSM_ID(STAND_UP) → SET_FSM_ID(WALK_RUN) → SET_BALANCE_MODE(CONTINUOUS_GAIT)

        Args:
            auto_balance_stand: 是否自动切换到走跑模式并开启连续步态
            callback: FSM 初始化完成后的回调（在后台线程中调用）
        """
        def _init_thread():
            logger = self._node.get_logger

            logger().info("[FSM] 切换到 DAMP 模式 (SET_FSM_ID=7101)...")
            self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.DAMP})
            time.sleep(2)

            logger().info("[FSM] 站立 (SET_FSM_ID STAND_UP=4)...")
            self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.STAND_UP})
            time.sleep(3)

            if auto_balance_stand:
                logger().info("[FSM] 进入走跑模式 (SET_FSM_ID WALK_RUN=801)...")
                self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.WALK_RUN})
                time.sleep(1)

                logger().info("[FSM] 开启连续步态 (SET_BALANCE_MODE CONTINUOUS_GAIT=1)...")
                self.publish(LocoAPI.SET_BALANCE_MODE, {"data": BalanceMode.CONTINUOUS_GAIT})
                time.sleep(1)

            self._ready = True
            logger().info("[FSM] 初始化完成，就绪")

            # 诊断：检查订阅者
            if not self.has_subscribers():
                logger().warn(
                    "[FSM] /api/sport/request 无订阅者! "
                    "FSM 指令可能未生效，运动指令也不会被执行。"
                    "请确认 unitree SDK bridge 已启动。"
                )

            if callback is not None:
                callback()

        t = threading.Thread(target=_init_thread, daemon=True)
        t.start()

    # ------------------------------------------------------------------
    #  Loco API 速度控制（参考 ctrl_keyboard 已验证方案）
    # ------------------------------------------------------------------
    def move(self, vx: float = 0.0, vy: float = 0.0,
             vyaw: float = 0.0, duration: Optional[float] = None) -> None:
        """通过 Loco API SET_VELOCITY 控制机器人运动。

        需要机器人已在 WALK_RUN 模式下。

        Args:
            vx: 前进速度 (m/s)，正=前进，负=后退
            vy: 侧移速度 (m/s)，正=左，负=右
            vyaw: 偏航角速度 (rad/s)，正=左转，负=右转
            duration: 运动持续时间（秒），None 则使用默认值
        """
        # 诊断：检查是否有订阅者接收运动指令
        if not self._sub_warned and not self.has_subscribers():
            self._sub_warned = True
            self._node.get_logger().warn(
                "[SportClient] /api/sport/request 无订阅者! "
                "运动指令不会被机器人执行。"
                "请确认 unitree SDK bridge 已启动 "
                "(检查: ros2 topic info /api/sport/request)"
            )
        dur = duration if duration is not None else self._move_duration
        self.publish(LocoAPI.SET_VELOCITY,
                     {"velocity": [vx, vy, vyaw], "duration": dur})

    def stop(self, duration: Optional[float] = None) -> None:
        """发送零速度指令停止运动（SET_VELOCITY velocity=[0,0,0]）。

        Args:
            duration: 停止持续时间，None 则使用默认值
        """
        dur = duration if duration is not None else self._stop_duration
        self.publish(LocoAPI.SET_VELOCITY,
                     {"velocity": [0.0, 0.0, 0.0], "duration": dur})

    def skip_init(self) -> None:
        """跳过 FSM 初始化，手动标记为就绪状态。

        用于机器人已处于 WALK_RUN + CONTINUOUS_GAIT 模式时，
        避免重复执行 DAMP → STAND_UP → WALK_RUN 流程。
        """
        self._ready = True

    # ------------------------------------------------------------------
    #  FSM 状态切换（全部通过 SET_FSM_ID）
    # ------------------------------------------------------------------
    def sit(self) -> None:
        """发送 SIT 指令（SET_FSM_ID SIT=3）。"""
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.SIT})

    def stand_up(self) -> None:
        """发送 STAND_UP 指令（SET_FSM_ID STAND_UP=4）。"""
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.STAND_UP})

    def stand_down(self) -> None:
        """发送 SQUAT 指令（SET_FSM_ID SQUAT=2）。"""
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.SQUAT})

    def damp(self) -> None:
        """发送 DAMP 指令（SET_FSM_ID DAMP=1）。"""
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.DAMP})

    def walk_run(self) -> None:
        """发送 WALK_RUN 指令（SET_FSM_ID WALK_RUN=801）。"""
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.WALK_RUN})

    def start_mode(self) -> None:
        """发送 START 指令（SET_FSM_ID START=500，常规运控）。"""
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSM_ID.START})

    # ------------------------------------------------------------------
    #  平衡模式切换（SET_BALANCE_MODE）
    # ------------------------------------------------------------------
    def continuous_gait(self) -> None:
        """开启连续步态（SET_BALANCE_MODE CONTINUOUS_GAIT=1）。"""
        self.publish(LocoAPI.SET_BALANCE_MODE, {"data": BalanceMode.CONTINUOUS_GAIT})

    def balance_stand(self) -> None:
        """切换到平衡站立（SET_BALANCE_MODE BALANCE_STAND=0）。"""
        self.publish(LocoAPI.SET_BALANCE_MODE, {"data": BalanceMode.BALANCE_STAND})

    # ------------------------------------------------------------------
    #  其他参数设置
    # ------------------------------------------------------------------
    def set_speed_mode(self, mode: int) -> None:
        """设置速度模式（SET_SPEED_MODE）。

        Args:
            mode: 速度模式 0=1.0m/s, 1=2.0m/s, 2=2.7m/s, 3=3.0m/s
        """
        self.publish(LocoAPI.SET_SPEED_MODE, {"data": mode})

    def set_stand_height(self, height: float) -> None:
        """设置站立高度（SET_STAND_HEIGHT）。

        Args:
            height: 站立高度（0.4~0.7m）
        """
        self.publish(LocoAPI.SET_STAND_HEIGHT, {"data": height})
