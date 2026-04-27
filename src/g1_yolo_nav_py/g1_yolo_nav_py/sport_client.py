"""G1 运动控制客户端 — 纯 Sport API 封装。

提供统一的运动控制接口，所有需要控制 G1 机器人运动的节点
都应使用本模块，避免重复定义 API 常量和 Request 发布逻辑。

全部使用 Sport API (unitree_api)：
    - 运动控制: MOVE(1008) — 参数 {x, y, z} 表示 (vx, vy, vyaw)
    - 停止运动: STOPMOVE(1003)
    - 姿态切换: DAMP(101), STANDUP(1004), STANDDOWN(1005),
                BALANCESTAND(1002), SIT(1009), RISESIT(1010)
    - 步态控制: CONTINUOUSGAIT(1019), SWITCHGAIT(1011)
    - 速度等级: SPEEDLEVEL(1015)

使用方式：
    from g1_yolo_nav_py.sport_client import SportClient, SportAPI

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
# Sport API 常量
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
# SportClient — 纯 Sport API 运动控制客户端
# ==================================================================
import json
import time
import threading
from typing import Optional, Callable

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request


class SportClient:
    """G1 运动控制客户端 — 纯 Sport API 封装。

    功能：
        - 统一 Request 发布（每次创建新对象，避免 DDS 缓冲区复用问题）
        - FSM 状态机管理（DAMP → STANDUP → BALANCESTAND → CONTINUOUSGAIT）
        - 速度控制（MOVE）
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
        # ROS2 Foxy: publisher 没有 get_subscription_count，用 node.count_subscribers
        try:
            count = self._node.count_subscribers('/api/sport/request')
            return count > 0
        except Exception:
            return True  # 无法判断时假设正常

    # ------------------------------------------------------------------
    #  核心：发布 Request
    # ------------------------------------------------------------------
    def publish(self, api_id: int, params: Optional[dict] = None) -> None:
        """发布 Sport API 请求。

        每次调用都创建新的 Request 对象，避免 DDS 缓冲区复用导致的数据损坏。

        Args:
            api_id: API ID（如 SportAPI.MOVE, SportAPI.BALANCESTAND）
            params: 参数字典，会被 JSON 序列化（如 {"x": 0.3, "y": 0, "z": 0}）
        """
        req = Request()
        req.header.identity.api_id = api_id
        if params is not None:
            req.parameter = json.dumps(params)
        self._pub.publish(req)

    # ------------------------------------------------------------------
    #  FSM 状态机初始化（纯 Sport API）
    # ------------------------------------------------------------------
    def init_fsm(self, auto_balance_stand: bool = True,
                 callback: Optional[Callable[[], None]] = None) -> None:
        """在后台线程执行 FSM 初始化（纯 Sport API）。

        流程：DAMP → STANDUP → BALANCESTAND → CONTINUOUSGAIT

        Args:
            auto_balance_stand: 是否自动切换到平衡站立并开启连续步态
            callback: FSM 初始化完成后的回调（在后台线程中调用）
        """
        def _init_thread():
            logger = self._node.get_logger

            logger().info("[FSM] 切换到 DAMP 模式...")
            self.publish(SportAPI.DAMP)
            time.sleep(2)

            logger().info("[FSM] 站立 (STANDUP)...")
            self.publish(SportAPI.STANDUP)
            time.sleep(3)

            if auto_balance_stand:
                logger().info("[FSM] 切换到平衡站立 (BALANCESTAND)...")
                self.publish(SportAPI.BALANCESTAND)
                time.sleep(1)

                logger().info("[FSM] 开启连续步态 (CONTINUOUSGAIT)...")
                self.publish(SportAPI.CONTINUOUSGAIT)
                time.sleep(1)

            self._ready = True
            logger().info("[FSM] 初始化完成，就绪")

            if callback is not None:
                callback()

        t = threading.Thread(target=_init_thread, daemon=True)
        t.start()

    # ------------------------------------------------------------------
    #  Sport API 速度控制
    # ------------------------------------------------------------------
    def move(self, vx: float = 0.0, vy: float = 0.0,
             vyaw: float = 0.0) -> None:
        """通过 Sport API MOVE 控制机器人运动。

        需要机器人已在 BALANCESTAND 模式下。

        Args:
            vx: 前进速度 (m/s)，正=前进，负=后退
            vy: 侧移速度 (m/s)，正=左，负=右
            vyaw: 偏航角速度 (rad/s)，正=左转，负=右转
        """
        self.publish(SportAPI.MOVE, {"x": vx, "y": vy, "z": vyaw})

    def stop(self) -> None:
        """发送 STOPMOVE 指令停止运动。"""
        self.publish(SportAPI.STOPMOVE)

    def skip_init(self) -> None:
        """跳过 FSM 初始化，手动标记为就绪状态。

        用于机器人已处于 BALANCESTAND + CONTINUOUSGAIT 模式时，
        避免重复执行 DAMP → STANDUP → BALANCESTAND 流程。
        """
        self._ready = True

    def sit(self) -> None:
        """发送 SIT 指令。"""
        self.publish(SportAPI.SIT)

    def stand_up(self) -> None:
        """发送 STANDUP 指令。"""
        self.publish(SportAPI.STANDUP)

    def stand_down(self) -> None:
        """发送 STANDDOWN 指令。"""
        self.publish(SportAPI.STANDDOWN)

    def rise_sit(self) -> None:
        """发送 RISESIT 指令（从坐姿恢复站立）。"""
        self.publish(SportAPI.RISESIT)

    def continuous_gait(self) -> None:
        """发送 CONTINUOUSGAIT 指令。"""
        self.publish(SportAPI.CONTINUOUSGAIT)

    def damp(self) -> None:
        """发送 DAMP 指令。"""
        self.publish(SportAPI.DAMP)
