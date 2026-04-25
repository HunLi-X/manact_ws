import sys
import threading

from unitree_api.msg import Request
import rclpy
import json

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
d : Damping mode (prepare to stand)
u : Stand up (from sitting)
s : Sit down
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
c : Toggle continuous movement
1-4: Set speed mode (0:1.0m/s,1:2.0m/s,2:2.7m/s,3:3.0m/s) 【新增】

CTRL-C to quit
"""

# G1运动控制API ID（保持原有，文档确认无需修改）
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
    "SET_SPEED_MODE": 7107    # 速度模式（文档新增明确参数）
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

req = Request()
def main():
    settings = saveTerminalSettings()

    rclpy.init()
    node = rclpy.create_node('g1_teleop_ctrl_keyboard')
    pub = node.create_publisher(Request, '/api/sport/request', 10)

    # 启动ROS2回调线程
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    # 初始化参数（按文档调整默认值）
    linear_scale = 1.0       # 线速度缩放系数
    angular_scale = 1.0      # 角速度缩放系数
    stand_height = 0.55      # 初始站立高度(米)
    height_step = 0.02       # 高度调整步长
    continuous_move = False  # 连续运动模式（文档默认关闭，安全优先）
    current_mode = "ZERO_TORQUE"  # 记录当前模式（初始为零力矩坐姿）

    try:
        print(msg)
        print(f"Initial state: Sitting in ZERO_TORQUE mode. Press 'd' then 'u' to stand up.")
        print(vels(linear_scale, angular_scale))
        
        while True:
            key = getKey(settings)
            api_id = ROBOT_LOCO_API_IDS["SET_VELOCITY"]
            param = {}
            vx, vy, vw = 0.0, 0.0, 0.0

            # 处理移动按键（仅在站立/运控模式有效）
            if key in moveBindings:
                valid_modes = ["STAND_UP", "BALANCE_STAND", "START", "WALK_RUN"]
                if current_mode in valid_modes:
                    vx, vy, vw = moveBindings[key]
                    # 应用缩放系数
                    vx *= linear_scale
                    vy *= linear_scale
                    vw *= angular_scale
                    # 走跑模式速度上限限制（文档最高3.0m/s）
                    vx = max(-3.0, min(vx, 3.0))
                    vy = max(-1.5, min(vy, 1.5))
                    vw = max(-1.5, min(vw, 1.5))
                    # 连续运动设置较长持续时间，否则短时间
                    duration = 1.0 if continuous_move else 0.2
                    param = {"velocity": [vx, vy, vw], "duration": duration}
                else:
                    print("Cannot move - robot is not in standing/run mode. Press 'd' then 'u' to stand.")
                    continue

            # 处理速度调整
            elif key in speedAdjustBindings:
                linear_scale *= speedAdjustBindings[key][0]
                angular_scale *= speedAdjustBindings[key][1]
                # 限制速度范围（适配文档走跑模式上限）
                linear_scale = max(0.1, min(linear_scale, 3.0))
                angular_scale = max(0.1, min(angular_scale, 1.5))
                print(vels(linear_scale, angular_scale))
                continue  # 不发送指令，只更新显示

            # 处理速度模式切换（文档新增1-4对应不同最高速度）
            elif key in SPEED_MODE:
                if current_mode == "WALK_RUN":
                    api_id = ROBOT_LOCO_API_IDS["SET_SPEED_MODE"]
                    speed_mode = SPEED_MODE[key]
                    param = {"data": speed_mode}
                    speed_val = [1.0, 2.0, 2.7, 3.0][speed_mode]
                    print(f"Set walk/run speed mode to {speed_val}m/s (mode {speed_mode})")
                else:
                    print("Only available in WALK_RUN mode (press 't' to enter)")
                continue

            # 处理模式切换（补充走跑运控模式，按文档要求排序）
            elif key == 'z':  # 零力矩模式（放松，回到初始坐姿）
                api_id = ROBOT_LOCO_API_IDS["SET_FSM_ID"]
                param = {"data": FSM_ID["ZERO_TORQUE"]}
                current_mode = "ZERO_TORQUE"
                print("Switched to ZERO_TORQUE mode (sitting)")
            elif key == 'x':  # 阻尼模式（从坐姿准备站立）
                if current_mode == "ZERO_TORQUE":
                    api_id = ROBOT_LOCO_API_IDS["SET_FSM_ID"]
                    param = {"data": FSM_ID["DAMP"]}
                    current_mode = "DAMP"
                    print("Switched to DAMP mode (preparing to stand)")
                else:
                    print("Only switch to DAMP mode from ZERO_TORQUE")
            elif key == 'u':  # 站立（从坐姿站起来）
                if current_mode == "DAMP":  # 必须先进入阻尼模式才能站立
                    api_id = ROBOT_LOCO_API_IDS["SET_FSM_ID"]
                    param = {"data": FSM_ID["STAND_UP"]}
                    current_mode = "STAND_UP"
                    print("Standing up... (locked stand mode)")
                else:
                    print("Please switch to DAMP mode first (press 'd') before standing")
            elif key == 's':  # 坐下
                api_id = ROBOT_LOCO_API_IDS["SET_FSM_ID"]
                param = {"data": FSM_ID["SIT"]}
                current_mode = "SIT"
                print("Sitting down...")
            elif key == 'b':  # 平衡站立
                if current_mode in ["STAND_UP", "START", "WALK_RUN"]:
                    api_id = ROBOT_LOCO_API_IDS["SET_BALANCE_MODE"]
                    param = {"data": BALANCE_MODE["BALANCE_STAND"]}
                    current_mode = "BALANCE_STAND"
                    print("Switched to BALANCE_STAND mode (stop when velocity=0)")
                else:
                    print("Cannot enter balance stand - robot is not standing")
            elif key == 'r':  # 启动模式（常规运控）
                if current_mode in ["STAND_UP", "BALANCE_STAND"]:
                    api_id = ROBOT_LOCO_API_IDS["SET_FSM_ID"]
                    param = {"data": FSM_ID["START"]}
                    current_mode = "START"
                    print("Switched to START mode (normal locomotion)")
                else:
                    print("Please stand up first before entering start mode")
            elif key == 't':  # 走跑运控模式（新增，按文档要求从站立模式进入）
                if current_mode in ["STAND_UP", "START", "BALANCE_STAND"]:
                    api_id = ROBOT_LOCO_API_IDS["SET_FSM_ID"]
                    param = {"data": FSM_ID["WALK_RUN"]}
                    current_mode = "WALK_RUN"
                    print("Switched to WALK_RUN mode (use 1-4 to adjust max speed)")
                else:
                    print("Please stand up first (press 'd'->'u') before entering walk/run mode")

            # 处理高度调整（仅在站立模式有效）
            elif key == 'h':  # 增加高度
                if current_mode in ["STAND_UP", "BALANCE_STAND", "START", "WALK_RUN"]:
                    api_id = ROBOT_LOCO_API_IDS["SET_STAND_HEIGHT"]
                    stand_height = min(stand_height + height_step, 0.7)
                    param = {"data": stand_height}
                    print(f"Stand height: {stand_height:.2f}m")
                else:
                    print("Cannot adjust height - robot is not standing")
            elif key == 'l':  # 降低高度
                if current_mode in ["STAND_UP", "BALANCE_STAND", "START", "WALK_RUN"]:
                    api_id = ROBOT_LOCO_API_IDS["SET_STAND_HEIGHT"]
                    stand_height = max(stand_height - height_step, 0.4)
                    param = {"data": stand_height}
                    print(f"Stand height: {stand_height:.2f}m")
                else:
                    print("Cannot adjust height - robot is not standing")
            elif key == 'H':  # 最高高度
                if current_mode in ["STAND_UP", "BALANCE_STAND", "START", "WALK_RUN"]:
                    api_id = ROBOT_LOCO_API_IDS["SET_STAND_HEIGHT"]
                    stand_height = 0.7
                    param = {"data": stand_height}
                    print(f"Maximum stand height: {stand_height:.2f}m")
                else:
                    print("Cannot adjust height - robot is not standing")
            elif key == 'L':  # 最低高度
                if current_mode in ["STAND_UP", "BALANCE_STAND", "START", "WALK_RUN"]:
                    api_id = ROBOT_LOCO_API_IDS["SET_STAND_HEIGHT"]
                    stand_height = 0.4
                    param = {"data": stand_height}
                    print(f"Minimum stand height: {stand_height:.2f}m")
                else:
                    print("Cannot adjust height - robot is not standing")

            # 连续运动模式切换（文档提示谨慎开启）
            elif key == 'c':
                continuous_move = not continuous_move
                status = "enabled" if continuous_move else "disabled"
                print(f"Continuous movement: {status} (caution: enabled for walk/run)")
                continue

            # 退出程序
            elif key == '\x03':  # CTRL+C
                break

            # 其他按键：停止移动（仅在移动模式有效）
            else:
                if api_id == ROBOT_LOCO_API_IDS["SET_VELOCITY"] and current_mode in ["STAND_UP", "BALANCE_STAND", "START", "WALK_RUN"]:
                    param = {"velocity": [0, 0, 0], "duration": 0.5}

            # 发送请求（仅当参数非空时）
            if param:
                req.header.identity.api_id = api_id
                req.parameter = json.dumps(param)
                pub.publish(req)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # 停止时发送坐下指令（适应初始坐姿）
        req.header.identity.api_id = ROBOT_LOCO_API_IDS["SET_FSM_ID"]
        req.parameter = json.dumps({"data": FSM_ID["SIT"]})
        pub.publish(req)
        
        # 确保停止运动
        req.header.identity.api_id = ROBOT_LOCO_API_IDS["SET_VELOCITY"]
        req.parameter = json.dumps({"velocity": [0, 0, 0], "duration": 1.0})
        pub.publish(req)

        rclpy.shutdown()
        spinner.join()
        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()

