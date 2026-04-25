import sys
import threading
import json

from unitree_api.msg import Request
import rclpy
from rclpy.node import Node

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes movement commands
for G1 robot (initial state: sitting with zero torque).
---------------------------
Basic movement:
   q    w    e
   a    x    d
   z    s    c

Holonomic mode (hold Shift):
---------------------------
   Q    W    E
   A    X    D
   Z    S    C

Mode control (initial state: sitting zero torque):
z : Zero torque mode (relax)
x : Damping mode (prepare to stand)
c : Stand up (from sitting)
v : Sit down
b : Balance stand
r : Start mode (常规运控)
t : Walk/Run mode (走跑运控) 【新增】

Height adjustment:
h : Increase stand height
l : Decrease stand height
H : Maximum stand height
L : Minimum stand height

Speed control:
+ : Increase linear speed
- : Decrease linear speed
* : Increase angular speed
/ : Decrease angular speed
p : Toggle continuous movement
1-4: Set speed mode (0:1.0m/s,1:2.0m/s,2:2.7m/s,3:3.0m/s) 【新增】

Arm Actions (only in STAND_UP/START/BALANCE_STAND mode):
5 : 恢复初始手臂位姿
6 : 双手飞吻
7 : 单手飞吻
8 : 平举
9 : 鼓掌
f : 击掌
g : 拥抱
n : 双手比心
m : 右手比心
j : 双手打X
- : 右手平举
= : 动感光波
[ : 胸前挥手
] : 高举挥手
; : 握手
ESC: Stop arm action
F1 : Get arm action list
---------------------------
CTRL-C to quit
"""

# G1运动控制API ID（保持原有）
ROBOT_LOCO_API_IDS = {
    "GET_FSM_ID": 7001,
    "GET_FSM_MODE": 7002,
    "GET_BALANCE_MODE": 7003,
    "GET_SWING_HEIGHT": 7004,
    "GET_STAND_HEIGHT": 7005,
    "GET_PHASE": 7006,  # deprecated
    
    "SET_FSM_ID": 7101,       # 状态机控制
    "SET_BALANCE_MODE": 7102,
    "SET_SWING_HEIGHT": 7103,
    "SET_STAND_HEIGHT": 7104, # 站立高度调整
    "SET_VELOCITY": 7105,     # 速度控制
    "SET_SPEED_MODE": 7107,   # 速度模式
}

# 手臂控制API ID（独立定义）
ARM_API_IDS = {
    "EXECUTE_ARM_ACTION": 7400, # 手臂动作执行API ID
    "STOP_ARM_ACTION": 7401,    # 停止手臂动作API ID
    "GET_ARM_ACTION_LIST": 7402 # 获取手臂动作列表API ID
}

# 状态机ID映射（补充文档走跑运控ID=801）
FSM_ID = {
    "ZERO_TORQUE": 0,    # 零力矩模式（初始坐姿）
    "DAMP": 1,           # 阻尼控制模式（从坐姿准备站立的过渡）
    "SQUAT": 2,          # 蹲下
    "SIT": 3,            # 坐下（稳定坐姿）
    "STAND_UP": 4,       # 锁定站立
    "START": 500,        # 常规运控
    "WALK_RUN": 801      # 走跑运控（文档新增）
}

# 平衡模式（文档明确说明含义）
BALANCE_MODE = {
    "BALANCE_STAND": 0,  # 平衡站立（速度为0时停止踏步）
    "CONTINUOUS_GAIT": 1 # 连续步态（持续踏步）
}

# 速度模式参数（文档明确：0-3对应不同最高速度）
SPEED_MODE = {
    '1': 0,  # 1.0m/s
    '2': 1,  # 2.0m/s
    '3': 2,  # 2.7m/s
    '4': 3   # 3.0m/s
}

# 手臂动作ID映射（来自文档）
ARM_ACTION_ID = {
    '5': 99,  # 恢复初始手臂位姿
    '6': 11,  # 双手飞吻
    '7': 12,  # 单手飞吻
    '8': 15,  # 平举
    '9': 17,  # 鼓掌
    'f': 18,  # 击掌
    'g': 19,  # 拥抱
    'n': 20,  # 双手比心
    'm': 21,  # 右手比心
    'j': 22,  # 双手打X
    '-': 23,  # 右手平举
    '=': 24,  # 动感光波
    '[': 25,  # 胸前挥手
    ']': 26,  # 高举挥手
    ';': 27   # 握手
}

# 手臂动作名称映射（用于打印日志）
ARM_ACTION_NAME = {
    99: "恢复初始手臂位姿",
    11: "双手飞吻",
    12: "单手飞吻",
    15: "平举",
    17: "鼓掌",
    18: "击掌",
    19: "拥抱",
    20: "双手比心",
    21: "右手比心",
    22: "双手打X",
    23: "右手平举",
    24: "动感光波",
    25: "胸前挥手",
    26: "高举挥手",
    27: "握手"
}

# 手臂动作允许的状态机模式（文档说明：部分动作在走跑运控下不可触发）
VALID_ARM_ACTION_MODES = ["STAND_UP", "BALANCE_STAND", "START"]

# 移动控制绑定（调整速度适配走跑模式上限）
moveBindings = {
    'w': (0.5, 0, 0),     # 前进（基础速度）
    's': (-0.5, 0, 0),    # 后退
    'a': (0, 0.3, 0),     # 左移
    'd': (0, -0.3, 0),    # 右移
    'q': (0, 0, 0.8),     # 左转
    'e': (0, 0, -0.8),    # 右转
    'W': (2.5, 0, 0),     # 快速前进（适配走跑模式最高3.0m/s）
    'S': (-2.5, 0, 0),    # 快速后退
    'A': (0, 1.5, 0),     # 快速左移
    'D': (0, -1.5, 0),    # 快速右移
    'Q': (0, 0, 1.5),     # 快速左转
    'E': (0, 0, -1.5),    # 快速右转
}

# 速度调整绑定
speedAdjustBindings = {
    '+': (1.1, 1),     # 增加线速度
    '-': (0.9, 1),     # 减小线速度
    '*': (1, 1.1),     # 增加角速度
    '/': (1, 0.9),     # 减小角速度
}


def getKey(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(linear, angular):
    return f'Current speeds: linear {linear:.2f} m/s, angular {angular:.2f} rad/s'


class G1TeleopNode(Node):
    def __init__(self):
        super().__init__('g1_teleop_ctrl_keyboard')
        
        # 创建运动控制发布者
        self.sport_pub = self.create_publisher(Request, '/api/sport/request', 10)
        
        # 创建手臂控制发布者（适配/api/arm/request话题）
        self.arm_pub = self.create_publisher(Request, '/api/arm/request', 10)
        
        # 订阅手臂响应话题（获取执行状态）
        self.arm_sub = self.create_subscription(
            Request,
            '/api/arm/response',
            self.arm_response_callback,
            10
        )
        
        # 初始化参数
        self.linear_scale = 1.0
        self.angular_scale = 1.0
        self.stand_height = 0.55
        self.height_step = 0.02
        self.continuous_move = False
        self.current_mode = "ZERO_TORQUE"
        
        # 请求消息初始化
        self.sport_req = Request()
        self.arm_req = Request()

    def arm_response_callback(self, msg):
        """处理手臂动作响应回调"""
        try:
            api_id = msg.header.identity.api_id
            response_data = json.loads(msg.parameter) if msg.parameter else {}
            
            if api_id == ARM_API_IDS["EXECUTE_ARM_ACTION"]:
                if "error_code" in response_data:
                    error_code = response_data["error_code"]
                    if error_code == 0:
                        self.get_logger().info("手臂动作执行成功")
                    else:
                        error_msg = self.get_arm_error_message(error_code)
                        self.get_logger().error(f"手臂动作执行失败: {error_msg} (错误码: {error_code})")
            
            elif api_id == ARM_API_IDS["GET_ARM_ACTION_LIST"]:
                self.get_logger().info(f"手臂动作列表: {response_data}")
                
        except json.JSONDecodeError:
            self.get_logger().warning("无法解析手臂响应数据")

    def get_arm_error_message(self, error_code):
        """获取手臂动作错误码对应的消息"""
        error_messages = {
            7400: "话题被占用（有动作正在执行）",
            7401: "手臂正举起，请使用动作ID99恢复初始状态",
            7402: "动作ID不存在",
            7404: "当前模式不可触发此动作（走跑运控下部分动作禁用）"
        }
        return error_messages.get(error_code, f"未知错误（{error_code}）")

    def publish_sport_request(self, api_id, params):
        """发布运动控制请求"""
        self.sport_req.header.identity.api_id = api_id
        self.sport_req.parameter = json.dumps(params)
        self.sport_pub.publish(self.sport_req)

    def publish_arm_request(self, api_id, params):
        """发布手臂控制请求"""
        self.arm_req.header.identity.api_id = api_id
        self.arm_req.parameter = json.dumps(params)
        self.arm_pub.publish(self.arm_req)


def main():
    settings = saveTerminalSettings()

    rclpy.init()
    node = G1TeleopNode()
    
    # 启动ROS2回调线程
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    try:
        print(msg)
        print(f"Initial state: Sitting in ZERO_TORQUE mode. Press 'x' then 'c' to stand up.")
        print(vels(node.linear_scale, node.angular_scale))
        
        while True:
            key = getKey(settings)
            param = {}

            # 处理移动按键（仅在站立/运控模式有效）
            if key in moveBindings:
                valid_modes = ["STAND_UP", "BALANCE_STAND", "START", "WALK_RUN"]
                if node.current_mode in valid_modes:
                    vx, vy, vw = moveBindings[key]
                    # 应用缩放系数
                    vx *= node.linear_scale
                    vy *= node.linear_scale
                    vw *= node.angular_scale
                    # 走跑模式速度上限限制（文档最高3.0m/s）
                    vx = max(-3.0, min(vx, 3.0))
                    vy = max(-1.5, min(vy, 1.5))
                    vw = max(-1.5, min(vw, 1.5))
                    # 连续运动设置较长持续时间，否则短时间
                    duration = 1.0 if node.continuous_move else 0.2
                    param = {"velocity": [vx, vy, vw], "duration": duration}
                    node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_VELOCITY"], param)
                else:
                    print("Cannot move - robot is not in standing/run mode. Press 'x' then 'c' to stand.")
                continue

            # 处理速度调整
            elif key in speedAdjustBindings:
                node.linear_scale *= speedAdjustBindings[key][0]
                node.angular_scale *= speedAdjustBindings[key][1]
                # 限制速度范围（适配文档走跑模式上限）
                node.linear_scale = max(0.1, min(node.linear_scale, 3.0))
                node.angular_scale = max(0.1, min(node.angular_scale, 1.5))
                print(vels(node.linear_scale, node.angular_scale))
                continue  # 不发送指令，只更新显示

            # 处理速度模式切换（文档新增1-4对应不同最高速度）
            elif key in SPEED_MODE:
                if node.current_mode == "WALK_RUN":
                    speed_mode = SPEED_MODE[key]
                    param = {"data": speed_mode}
                    node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_SPEED_MODE"], param)
                    speed_val = [1.0, 2.0, 2.7, 3.0][speed_mode]
                    print(f"Set walk/run speed mode to {speed_val}m/s (mode {speed_mode})")
                else:
                    print("Only available in WALK_RUN mode (press 't' to enter)")
                continue

            # 处理手臂动作按键（核心逻辑）
            elif key in ARM_ACTION_ID:
                # 检查当前模式是否允许执行手臂动作
                if node.current_mode not in VALID_ARM_ACTION_MODES:
                    print(f"Cannot execute arm action - current mode {node.current_mode} is invalid. Valid modes: {VALID_ARM_ACTION_MODES}")
                    continue
                
                # 获取动作ID和名称
                action_id = ARM_ACTION_ID[key]
                action_name = ARM_ACTION_NAME.get(action_id, f"Action_{action_id}")
                
                # 配置手臂动作执行参数并发布到/api/arm/request话题
                param = {"action_id": action_id}
                node.publish_arm_request(ARM_API_IDS["EXECUTE_ARM_ACTION"], param)
                print(f"Executing arm action: {action_name} (ID: {action_id})...")

            # 处理停止手臂动作（ESC键）
            elif key == '\x1b':  # ESC键
                node.publish_arm_request(ARM_API_IDS["STOP_ARM_ACTION"], {})
                print("Stopping arm action, restoring to initial arm pose...")
                continue

            # 处理获取手臂动作列表（F1键）
            elif key == '\x1bOP':  # F1键（不同终端可能有差异）
                node.publish_arm_request(ARM_API_IDS["GET_ARM_ACTION_LIST"], {})
                print("Requesting arm action list... (check response for details)")
                continue

            # 处理模式切换
            elif key == 'z':  # 零力矩模式
                param = {"data": FSM_ID["ZERO_TORQUE"]}
                node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_FSM_ID"], param)
                node.current_mode = "ZERO_TORQUE"
                print("Switched to ZERO_TORQUE mode (sitting)")
            elif key == 'x':  # 阻尼模式
                if node.current_mode == "ZERO_TORQUE":
                    param = {"data": FSM_ID["DAMP"]}
                    node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_FSM_ID"], param)
                    node.current_mode = "DAMP"
                    print("Switched to DAMP mode (preparing to stand)")
                else:
                    print("Only switch to DAMP mode from ZERO_TORQUE")
            elif key == 'c':  # 站立
                if node.current_mode == "DAMP":
                    param = {"data": FSM_ID["STAND_UP"]}
                    node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_FSM_ID"], param)
                    node.current_mode = "STAND_UP"
                    print("Standing up... (locked stand mode)")
                else:
                    print("Please switch to DAMP mode first (press 'x') before standing")
            elif key == 'v':  # 坐下
                param = {"data": FSM_ID["SIT"]}
                node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_FSM_ID"], param)
                node.current_mode = "SIT"
                print("Sitting down...")
            elif key == 'b':  # 平衡站立
                if node.current_mode in ["STAND_UP", "START", "WALK_RUN"]:
                    param = {"data": BALANCE_MODE["BALANCE_STAND"]}
                    node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_BALANCE_MODE"], param)
                    node.current_mode = "BALANCE_STAND"
                    print("Switched to BALANCE_STAND mode (stop when velocity=0)")
                else:
                    print("Cannot enter balance stand - robot is not standing")
            elif key == 'r':  # 启动模式
                if node.current_mode in ["STAND_UP", "BALANCE_STAND"]:
                    param = {"data": FSM_ID["START"]}
                    node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_FSM_ID"], param)
                    node.current_mode = "START"
                    print("Switched to START mode (normal locomotion)")
                else:
                    print("Please stand up first before entering start mode")
            elif key == 't':  # 走跑运控模式
                if node.current_mode in ["STAND_UP", "START", "BALANCE_STAND"]:
                    param = {"data": FSM_ID["WALK_RUN"]}
                    node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_FSM_ID"], param)
                    node.current_mode = "WALK_RUN"
                    print("Switched to WALK_RUN mode (use 1-4 to adjust max speed)")
                else:
                    print("Please stand up first (press 'x'->'c') before entering walk/run mode")

            # 处理高度调整
            elif key == 'h':  # 增加高度
                if node.current_mode in ["STAND_UP", "BALANCE_STAND", "START", "WALK_RUN"]:
                    node.stand_height = min(node.stand_height + node.height_step, 0.7)
                    param = {"data": node.stand_height}
                    node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_STAND_HEIGHT"], param)
                    print(f"Stand height: {node.stand_height:.2f}m")
                else:
                    print("Cannot adjust height - robot is not standing")
            elif key == 'l':  # 降低高度
                if node.current_mode in ["STAND_UP", "BALANCE_STAND", "START", "WALK_RUN"]:
                    node.stand_height = max(node.stand_height - node.height_step, 0.4)
                    param = {"data": node.stand_height}
                    node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_STAND_HEIGHT"], param)
                    print(f"Stand height: {node.stand_height:.2f}m")
                else:
                    print("Cannot adjust height - robot is not standing")
            elif key == 'H':  # 最高高度
                if node.current_mode in ["STAND_UP", "BALANCE_STAND", "START", "WALK_RUN"]:
                    node.stand_height = 0.7
                    param = {"data": node.stand_height}
                    node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_STAND_HEIGHT"], param)
                    print(f"Maximum stand height: {node.stand_height:.2f}m")
                else:
                    print("Cannot adjust height - robot is not standing")
            elif key == 'L':  # 最低高度
                if node.current_mode in ["STAND_UP", "BALANCE_STAND", "START", "WALK_RUN"]:
                    node.stand_height = 0.4
                    param = {"data": node.stand_height}
                    node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_STAND_HEIGHT"], param)
                    print(f"Minimum stand height: {node.stand_height:.2f}m")
                else:
                    print("Cannot adjust height - robot is not standing")

            # 连续运动模式切换
            elif key == 'p':
                node.continuous_move = not node.continuous_move
                status = "enabled" if node.continuous_move else "disabled"
                print(f"Continuous movement: {status} (caution: enabled for walk/run)")
                continue

            # 退出程序
            elif key == '\x03':  # CTRL+C
                break

            # 其他按键：停止移动
            else:
                if node.current_mode in ["STAND_UP", "BALANCE_STAND", "START", "WALK_RUN"]:
                    param = {"velocity": [0, 0, 0], "duration": 0.5}
                    node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_VELOCITY"], param)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # 停止时发送坐下指令
        param = {"data": FSM_ID["SIT"]}
        node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_FSM_ID"], param)
        
        # 停止手臂动作
        node.publish_arm_request(ARM_API_IDS["STOP_ARM_ACTION"], {})
        
        # 确保停止运动
        param = {"velocity": [0, 0, 0], "duration": 1.0}
        node.publish_sport_request(ROBOT_LOCO_API_IDS["SET_VELOCITY"], param)

        rclpy.shutdown()
        spinner.join()
        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()

