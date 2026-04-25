"""G1 运动控制客户端 — 统一封装 Sport API 和 Loco API。

提供统一的运动控制接口，所有需要控制 G1 机器人运动的节点
都应使用本模块，避免重复定义 API 常量和 Request 发布逻辑。

两种控制接口：
    1. Sport API (unitree_api): MOVE(1008), BALANCESTAND(1002), STOPMOVE(1003) 等
       — 适用于已在 BALANCESTAND/WALK_RUN 模式下的简单运动控制
       — 被 g1_teleop_ctrl_keyboard 和 twist_bridge 使用
    2. Loco API (unitree_api): SET_FSM_ID(7101), SET_VELOCITY(7105), SET_BALANCE_MODE(7102) 等
       — 适用于需要 FSM 状态管理的完整运动控制流程
       — 被 ctrl_keyboard/auto_ctrl.py 使用

使用方式：
    from g1_yolo_nav_py.sport_client import SportClient, LocoAPI, FSM_ID

    class MyNode(Node):
        def __init__(self):
            ...
            self._sport = SportClient(self)
            self._sport.init_fsm(auto_walk_run=True)

        def move_forward(self):
            if self._sport.ready:
                self._sport.set_velocity(vx=0.3)

        def stop(self):
            self._sport.stop()
"""

# ==================================================================
# Sport API 常量 (参考 g1_teleop_ctrl_keyboard/g1_teleop_ctrl_keyboard.py)
# ==================================================================
class SportAPI:
    """Sport Mode API ID 常量。"""
    DAMP = 101
    BALANCESTAND = 1002
    STOPMOVE = 1003
    STANDUP = 1004
    STANDDOWN = 1005
    RECOVERYSTAND = 1006
    EULER = 1007
    MOVE = 1008
    SIT = 1009
    RISESIT = 1010
    SWITCHGAIT = 1011
    TRIGGER = 1012
    BODYHEIGHT = 1013
    FOOTRAISEHEIGHT = 1014
    SPEEDLEVEL = 1015
    HELLO = 1016
    STRETCH = 1017
    TRAJECTORYFOLLOW = 1018
    CONTINUOUSGAIT = 1019
    CONTENT = 1020
    WALLOW = 1021
    DANCE1 = 1022
    DANCE2 = 1023
    GETBODYHEIGHT = 1024
    GETFOOTRAISEHEIGHT = 1025
    GETSPEEDLEVEL = 1026
    SWITCHJOYSTICK = 1027
    POSE = 1028
    SCRAPE = 1029
    FRONTFLIP = 1030
    FRONTJUMP = 1031
    FRONTPOUNCE = 1032


# ==================================================================
# Loco API 常量 (参考 ctrl_keyboard/ctrl_keyboard/auto_ctrl.py)
# ==================================================================
class LocoAPI:
    """Loco (走跑模式) API ID 常量。"""
    GET_FSM_ID = 7001
    GET_FSM_MODE = 7002
    GET_BALANCE_MODE = 7003
    GET_SWING_HEIGHT = 7004
    GET_STAND_HEIGHT = 7005
    GET_PHASE = 7006
    SET_FSM_ID = 7101
    SET_BALANCE_MODE = 7102
    SET_SWING_HEIGHT = 7103
    SET_STAND_HEIGHT = 7104
    SET_VELOCITY = 7105
    SET_SPEED_MODE = 7107


# ==================================================================
# FSM 状态 ID (参考 ctrl_keyboard/ctrl_keyboard/auto_ctrl.py)
# ==================================================================
class FSMID:
    """G1 运动状态机状态 ID。"""
    ZERO_TORQUE = 0
    DAMP = 1
    SQUAT = 2
    SIT = 3
    STAND_UP = 4
    START = 500
    WALK_RUN = 801


# ==================================================================
# 平衡模式
# ==================================================================
class BalanceMode:
    """平衡模式常量。"""
    BALANCE_STAND = 0
    CONTINUOUS_GAIT = 1


# ==================================================================
# SportClient — 统一运动控制客户端
# ==================================================================
import json
import time
import threading
from typing import Optional, Callable

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request


class SportClient:
    """G1 运动控制客户端 — 封装 Sport API / Loco API 调用。

    功能：
        - 统一 Request 发布（每次创建新对象，避免 DDS 缓冲区复用问题）
        - FSM 状态机管理（DAMP → STAND_UP → WALK_RUN → CONTINUOUS_GAIT）
        - 速度控制（SET_VELOCITY）
        - 停止 / 坐下

    使用：
        sport = SportClient(node)
        sport.init_fsm()        # 后台线程初始化 FSM
        sport.set_velocity(0.3) # 前进
        sport.stop()            # 停止
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

    # ------------------------------------------------------------------
    #  属性
    # ------------------------------------------------------------------
    @property
    def ready(self) -> bool:
        """FSM 是否已初始化完成，可以接受运动指令。"""
        return self._ready

    # ------------------------------------------------------------------
    #  核心：发布 Request
    # ------------------------------------------------------------------
    def publish(self, api_id: int, params: Optional[dict] = None) -> None:
        """发布 Sport API / Loco API 请求。

        每次调用都创建新的 Request 对象，避免 DDS 缓冲区复用导致的数据损坏。

        Args:
            api_id: API ID（如 LocoAPI.SET_VELOCITY, SportAPI.MOVE）
            params: 参数字典，会被 JSON 序列化（如 {"velocity": [0.3, 0, 0], "duration": 0.5}）
        """
        req = Request()
        req.header.identity.api_id = api_id
        if params is not None:
            req.parameter = json.dumps(params)
        self._pub.publish(req)

    # ------------------------------------------------------------------
    #  FSM 状态机初始化
    # ------------------------------------------------------------------
    def init_fsm(self, auto_walk_run: bool = True,
                 callback: Optional[Callable[[], None]] = None) -> None:
        """在后台线程执行 FSM 初始化。

        流程：DAMP → STAND_UP → [WALK_RUN → CONTINUOUS_GAIT]

        Args:
            auto_walk_run: 是否自动切换到走跑模式并开启连续步态
            callback: FSM 初始化完成后的回调（在后台线程中调用）
        """
        def _init_thread():
            logger = self._node.get_logger

            logger().info("[FSM] 切换到 DAMP 模式...")
            self.publish(LocoAPI.SET_FSM_ID, {"data": FSMID.DAMP})
            time.sleep(2)

            logger().info("[FSM] 切换到 STAND_UP 模式...")
            self.publish(LocoAPI.SET_FSM_ID, {"data": FSMID.STAND_UP})
            time.sleep(3)

            if auto_walk_run:
                logger().info("[FSM] 切换到 WALK_RUN 走跑模式...")
                self.publish(LocoAPI.SET_FSM_ID, {"data": FSMID.WALK_RUN})
                time.sleep(1)

                logger().info("[FSM] 开启连续步态 CONTINUOUS_GAIT...")
                self.publish(LocoAPI.SET_BALANCE_MODE, {"data": BalanceMode.CONTINUOUS_GAIT})
                time.sleep(1)

            self._ready = True
            logger().info("[FSM] 初始化完成，就绪")

            if callback is not None:
                callback()

        t = threading.Thread(target=_init_thread, daemon=True)
        t.start()

    # ------------------------------------------------------------------
    #  Loco API 速度控制
    # ------------------------------------------------------------------
    def set_velocity(self, vx: float = 0.0, vy: float = 0.0,
                     vyaw: float = 0.0, duration: float = 0.5) -> None:
        """通过 Loco API SET_VELOCITY 控制机器人运动。

        需要机器人已在 WALK_RUN 模式 + CONTINUOUS_GAIT 下。
        使用 init_fsm() 自动完成初始化。

        Args:
            vx: 前进速度 (m/s)，正=前进，负=后退
            vy: 侧移速度 (m/s)，正=左，负=右
            vyaw: 偏航角速度 (rad/s)，正=左转，负=右转
            duration: 速度指令持续时间 (s)
        """
        self.publish(LocoAPI.SET_VELOCITY, {
            "velocity": [vx, vy, vyaw],
            "duration": duration,
        })

    def stop(self) -> None:
        """发送零速停止指令。"""
        self.set_velocity(0.0, 0.0, 0.0, 0.5)

    def balance_stand(self) -> None:
        """发送 BALANCESTAND 指令（Sport API）。"""
        self.publish(SportAPI.BALANCESTAND)

    def stop_move(self) -> None:
        """发送 STOPMOVE 指令（Sport API）。"""
        self.publish(SportAPI.STOPMOVE)

    def sit(self) -> None:
        """发送 SIT 指令（Loco API）。"""
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSMID.SIT})

    def stand_up(self) -> None:
        """发送 STAND_UP 指令（Loco API）。"""
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSMID.STAND_UP})

    def walk_run(self) -> None:
        """发送 WALK_RUN 指令（Loco API）。"""
        self.publish(LocoAPI.SET_FSM_ID, {"data": FSMID.WALK_RUN})
