"""
需求：编写键盘控制节点，控制 G1 人形机器人运动

    启动时自动初始化 FSM：DAMP → STANDUP → BALANCESTAND → CONTINUOUSGAIT
    初始化完成后 MOVE(1008) 指令才会生效。

    移动按键
    w:向前移动。
    e: 向后并向左转移动(在普通模式下)
    a:向左转。
    d:向右转。
    q:向前并向右转移动(在普通模式下)
    s:向后移动。
    c:向后并向左转移动(在全向模式下,与e相对)
    z:向前并向右转移动(在全向模式下,与q相对)
    当按下shift键时,机器人进入全向模式(Holonomic mode) ，允许它进行侧向移动(strafing):

    W:向前移动(全向模式与w相同)
    E:向左移动。
    A:向后并向左移动。
    D:向后并向右移动。
    Q:向右移动。
    S:停止移动。
    C:向前并向左移动。
    Z:向前并向右移动。

    速度调整按键
    r:增加最大速度和转向速度10%。
    t:减少最大速度和转向速度10%。
    f:仅增加线性速度10%(不影响转向速度)。
    g:仅减少线性速度10%
    v:仅增加转向速度10%(不影响线性速度)。
    b:仅减少转向速度

    状态控制按键
    u: 站立 (STANDUP)
    i: 平衡站立 (BALANCESTAND)
    o: 连续步态 (CONTINUOUSGAIT)

    运动模式切换
    h:打招呼
    j:前跳
    k:伸懒腰
    n:坐下
    m:从坐下恢复。
    1:跳舞1。
    2:跳舞2。
"""

# ==================================================================
# 1. 标准库导入
# ==================================================================
import termios  # 终端属性控制，用于键盘原始模式
import sys      # 标准输入读取
import tty      # 终端模式设置
import threading  # ROS2 spin 后台线程
import json     # JSON 序列化，构造 Request 参数
import time     # FSM 初始化延时

# ==================================================================
# 2. 第三方库与 ROS2 导入
# ==================================================================
import rclpy  # ROS2 Python 客户端库
from rclpy.node import Node  # ROS2 节点基类
from unitree_api.msg import Request  # 宇树 API 请求消息

msg = """
G1 Teleop Keyboard Controller
This node takes keypresses from the keyboard and publishes them
as unitree api/msg/Request messages. It works best with a us keyboard layout.
----------------------
Moving around:
    q   w   e
    a   x   d
    z   s   c

For Holonomic mode (strafing), hold down the shift key:
----------------------
    Q   W   E
    A   X   D
    Z   S   C

anything else : stop

r/t :increase/decrease max speeds by 10%
f/g :increase/decrease only linear speed by 10%
v/b :increase/decrease only angular speed by 10%

u/i/o : STANDUP / BALANCESTAND / CONTINUOUSGAIT

h: Greet
j: Front Jump
k: Stretch
n: Sit Down
m: Stand Up from Sitting
1: Dance 1
2: Dance 2

CTRL-C to quit"""

ROBOT_SPORT_API_IDS = {
    "DAMP": 101,
    "BALANCESTAND": 1002,
    "STOPMOVE": 1003,
    "STANDUP": 1004,
    "STANDDOWN": 1005,
    "RECOVERYSTAND": 1006,
    "EULER": 1007,
    "MOVE": 1008,
    "SIT": 1009,
    "RISESIT": 1010,
    "SWITCHGAIT": 1011,
    "TRIGGER": 1012,
    "BODYHEIGHT": 1013,
    "FOOTRAISEHEIGHT": 1014,
    "SPEEDLEVEL": 1015,
    "HELLO": 1016,
    "STRETCH": 1017,
    "TRAJECTORYFOLLOW": 1018,
    "CONTINUOUSGAIT": 1019,
    "CONTENT": 1020,
    "WALLOW": 1021,
    "DANCE1": 1022,
    "DANCE2": 1023,
    "GETBODYHEIGHT": 1024,
    "GETFOOTRAISEHEIGHT": 1025,
    "GETSPEEDLEVEL": 1026,
    "SWITCHJOYSTICK": 1027,
    "POSE": 1028,
    "SCRAPE": 1029,
    "FRONTFLIP": 1030,
    "FRONTJUMP": 1031,
    "FRONTPOUNCE": 1032,
}

sportModel = {
    "h": ROBOT_SPORT_API_IDS["HELLO"],
    "j": ROBOT_SPORT_API_IDS["FRONTJUMP"],
    "k": ROBOT_SPORT_API_IDS["STRETCH"],
    "n": ROBOT_SPORT_API_IDS["SIT"],
    "m": ROBOT_SPORT_API_IDS["RISESIT"],
    "1": ROBOT_SPORT_API_IDS["DANCE1"],
    "2": ROBOT_SPORT_API_IDS["DANCE2"],
}

stateControlKeys = {
    "u": (ROBOT_SPORT_API_IDS["STANDUP"], "STANDUP"),
    "i": (ROBOT_SPORT_API_IDS["BALANCESTAND"], "BALANCESTAND"),
    "o": (ROBOT_SPORT_API_IDS["CONTINUOUSGAIT"], "CONTINUOUSGAIT"),
}

moveBindings = {
    "w": (1, 0, 0, 0),
    "e": (1, 0, 0, -1),
    "a": (0, 0, 0, 1),
    "d": (0, 0, 0, -1),
    "q": (1, 0, 0, 1),
    "s": (-1, 0, 0, 0),
    "c": (-1, 0, 0, 1),
    "z": (-1, 0, 0, -1),
    "E": (1, -1, 0, 0),
    "W": (1, 0, 0, 0),
    "A": (0, 1, 0, 0),
    "D": (0, -1, 0, 0),
    "Q": (1, 1, 0, 0),
    "S": (-1, 0, 0, 0),
    "C": (-1, -1, 0, 0),
    "Z": (-1, 1, 0, 0),
}

speedBindings = {
    "r": (1.1, 1.1),
    "t": (0.9, 0.9),
    "f": (1.1, 1.0),
    "g": (0.9, 1.0),
    "v": (1.0, 1.1),
    "b": (1.0, 0.9),
}


class TeleopNode(Node):
    def __init__(self):
        super().__init__("g1_teleop_keyboard_node")
        self.pub = self.create_publisher(Request, "/api/sport/request", 10)
        self.declare_parameter("speed", 0.2)
        self.declare_parameter("angular", 0.5)
        self.declare_parameter("auto_init", True)
        self.speed = self.get_parameter("speed").value
        self.angular = self.get_parameter("angular").value
        self.auto_init = self.get_parameter("auto_init").value
        self.fsm_ready = False

    def publish(self, api_id, params=None):
        """发布 Sport API 请求，每次创建新 Request 对象。"""
        req = Request()
        req.header.identity.api_id = api_id
        if params is not None:
            req.parameter = json.dumps(params)
        self.pub.publish(req)

    def init_fsm(self):
        """自动初始化 FSM：DAMP → STANDUP → BALANCESTAND → CONTINUOUSGAIT。

        返回 True 表示初始化成功完成。
        """
        print("\n[FSM] 初始化中...")
        try:
            print("[FSM] 1/4 DAMP 模式...")
            self.publish(ROBOT_SPORT_API_IDS["DAMP"])
            time.sleep(2)

            print("[FSM] 2/4 STANDUP...")
            self.publish(ROBOT_SPORT_API_IDS["STANDUP"])
            time.sleep(3)

            print("[FSM] 3/4 BALANCESTAND...")
            self.publish(ROBOT_SPORT_API_IDS["BALANCESTAND"])
            time.sleep(1)

            print("[FSM] 4/4 CONTINUOUSGAIT...")
            self.publish(ROBOT_SPORT_API_IDS["CONTINUOUSGAIT"])
            time.sleep(1)

            self.fsm_ready = True
            print("[FSM] 初始化完成，可以使用 w/a/s/d 控制移动\n")
            return True
        except Exception as e:
            print(f"[FSM] 初始化失败: {e}")
            return False


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    print(msg)
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    teleopNode = TeleopNode()
    spinner = threading.Thread(target=rclpy.spin, args=(teleopNode,), daemon=True)
    spinner.start()

    # 自动初始化 FSM
    if teleopNode.auto_init:
        teleopNode.init_fsm()
    else:
        print("\n[提示] 自动初始化已关闭，请手动按 u/i/o 初始化状态\n")

    try:
        while True:
            key = getKey(settings)
            if key == "\x03":  # ctrl + c
                break
            elif key in stateControlKeys:
                api_id, name = stateControlKeys[key]
                teleopNode.publish(api_id)
                print(f"[状态] 已发送 {name}")
            elif key in sportModel.keys():
                teleopNode.publish(sportModel[key])
            elif key in moveBindings.keys():
                x_bind = moveBindings[key][0]
                y_bind = moveBindings[key][1]
                z_bind = moveBindings[key][3]
                teleopNode.publish(
                    ROBOT_SPORT_API_IDS["MOVE"],
                    params={"x": x_bind * teleopNode.speed,
                            "y": y_bind * teleopNode.speed,
                            "z": z_bind * teleopNode.angular},
                )
            elif key in speedBindings.keys():
                speed_bind = speedBindings[key][0]
                angular_bind = speedBindings[key][1]
                teleopNode.speed *= speed_bind
                teleopNode.angular *= angular_bind
                print(
                    "current speed:%.5f, angular:%.5f"
                    % (teleopNode.speed, teleopNode.angular)
                )
            else:
                teleopNode.publish(ROBOT_SPORT_API_IDS["STOPMOVE"])

    finally:
        teleopNode.publish(ROBOT_SPORT_API_IDS["STOPMOVE"])
        rclpy.shutdown()


if __name__ == "__main__":
    main()
