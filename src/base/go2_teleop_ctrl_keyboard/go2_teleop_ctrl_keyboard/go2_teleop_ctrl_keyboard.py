"""
需求：编写键盘控制节点，控制 Go2 机器人运动
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
    9:仅减少线性速度10%
    y:仅增加转向速度10%(不影响线性速度)。
    b:仅减少转向速度
    
    运动模式切换
    h:打招呼
    j:前跳
    k:伸懒腰
    n:坐下
    m:从坐下恢复。
    y:跳舞1。
    u:跳舞2。

步骤:
    1.引入第三方库
    2.编写ROS2节点
    3.编写按键映射逻辑

"""

# 1.导包
import rclpy
from rclpy.node import Node
import termios
import sys
import tty
import threading
from unitree_api.msg import Request
import json

msg = """
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

h: Greet
j: Front Jump
k: Stretch
n: Sit Down
m: Stand Up from Sitting
y: Dance 1
u: Dance 2

CTRL-C to quit"""

# 定义常量字典
ROBOT_SPORT_API_IDS = {
"DAMP": 101,                    # 阻尼控制
"BALANCESTAND": 1002,           # 平衡站立
"STOPMOVE": 1003,               # 停止运动
"STANDUP": 1004,                # 站立
"STANDDOWN": 1005,              # 站立下降
"RECOVERYSTAND": 1006,          # 恢复站立
"EULER": 1007,                  # 欧拉角控制
"MOVE": 1008,                   # 移动
"SIT": 1009,                    # 坐下
"RISESIT": 1010,                # 从坐下恢复站立
"SWITCHGAIT": 1011,             # 切换步态
"TRIGGER": 1012,                # 触发
"BODYHEIGHT": 1013,             # 身体高度调整
"FOOTRAISEHEIGHT": 1014,        # 脚部抬起高度调整
"SPEEDLEVEL": 1015,             # 速度级别调整
"HELLO": 1016,                  # 打招呼
"STRETCH": 1017,                # 伸展
"TRAJECTORYFOLLOW": 1018,       # 轨迹跟随
"CONTINUOUSGAIT": 1019,         # 连续步态
"CONTENT": 1020,                # 内容
"WALLOW": 1021,                 # 打滚
"DANCE1": 1022,                 # 舞蹈1
"DANCE2": 1023,                 # 舞蹈2
"GETBODYHEIGHT": 1024,          # 获取身体高度
"GETFOOTRAISEHEIGHT": 1025,     # 获取脚部抬起高度
"GETSPEEDLEVEL": 1026,          # 获取速度级别
"SWITCHJOYSTICK":1027,          # 切换操纵杆
"POSE": 1028,                   # 姿态
"SCRAPE": 1029,                 # 刮擦
"FRONTFLIP": 1030,              # 前空翻
"FRONTJUMP": 1031,              # 前跳
"FRONTPOUNCE": 1032             # 前扑
}

sportModel = {
    'h': ROBOT_SPORT_API_IDS["HELLO"],               # 打招呼
    'j': ROBOT_SPORT_API_IDS["FRONTJUMP"],           # 前跳
    'k': ROBOT_SPORT_API_IDS["STRETCH"],             # 伸懒腰
    'n': ROBOT_SPORT_API_IDS["SIT"],                 # 坐下
    'm': ROBOT_SPORT_API_IDS["RISESIT"],             # 从坐下恢复
    'y': ROBOT_SPORT_API_IDS["DANCE1"],              # 跳舞1
    'u': ROBOT_SPORT_API_IDS["DANCE2"]               # 跳舞2
}

moveBindings={
    'w':(1,0,0,0),  #x*1,y*0,z*0,th*0
    'e':(1,0,0,-1),
    'a':(0,0,0,1),
    'd':(0,0,0,-1),
    'q':(1,0,0,1),
    's':(-1,0,0,0),
    'c':(-1,0,0,1),
    'z':(-1,0,0,-1),
    'E':(1,-1,0,0),
    'W':(1,0,0,0),
    'A':(0,1,0,0),
    'D':(0,-1,0,0),
    'Q':(1,1,0,0),
    'S':(-1,0,0,0),
    'C':(-1,-1,0,0),
    'Z':(-1,1,0,0),
}

speedBindings ={
    'r':(1.1,1.1),  #线速度 *1.1, 角速度 *1.1
    't':(0.9,0.9),
    'f':(1.1,1.0),
    'g':(0.9,1.0),
    'v':(1.0,1.1),
    'b':(1.0,0.9),
    }
# 3.定义节点类
class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_ctrl_keyboard")
        self.pub = self.create_publisher(Request,"/api/sport/request",10)
        #以参数方式设置线速度角速度初始值
        self.declare_parameter("speed",0.2)         #假设x和y上的线速度一致
        self.declare_parameter("angular",0.5)

        self.speed = self.get_parameter("speed").value
        self.angular = self.get_parameter("angular").value

    #发布速度指令
    def publish(self,api_id,x = 0.0,y = 0.0,z = 0.0):
        req = Request()
        req.header.identity.api_id = api_id
        js = {"x": x, "y": y, "z": z}
        req.parameter = json.dumps(js)
        self.pub.publish(req)

def getKey(settings):
    #设置读取模式
    tty.setraw(sys.stdin.fileno())
            # 将终端设置为原始模式,这样可以逐个字符读取输入而不需要按下回车键
    #获取一个按键
    key = sys.stdin.read(1)
    #恢复终端原始设置
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
                            # 等待所有已排队等待输出的数据完成传输后，才应用新的终端属性设置
    #返回按键值
    return key

def main():
    print(msg)
    #读取按键录入实现
    #1.获取标准输入流终端属性并返回
    settings = termios.tcgetattr(sys.stdin)
                                # stdin: standard input 标准输入流

    #2.ros2 node实现，需要单独子线程处理
    rclpy.init()
    #子线程执行
    teleopNode = TeleopNode()
    spinner = threading.Thread(target=rclpy.spin, args=(teleopNode,))
    spinner.start()

    #3.循环读取按键,并映射
    #异常处理，主要利用finally保证资源释放
    try:
        while True:
            key = getKey(settings)
            # if key == 'h':
            #     teleopNode.publish(1016)  # 打招呼
            #1.结束终端判断 ctrl + c --->   站立平衡
            if key == '\x03':  # ctrl + c
                teleopNode.publish(ROBOT_SPORT_API_IDS["BALANCESTAND"])
                break
            #2.运动模式切换(设置 api_id)
            elif key in sportModel.keys():
                teleopNode.publish(sportModel[key])
            #3.运动控制(设置 api_id,和速度消息)     拆分:x 线速度 +y 线速度+z 线速度
            elif key in moveBindings.keys():
                x_bind = moveBindings[key][0]
                y_bind = moveBindings[key][1]
                z_bind = moveBindings[key][3]
                teleopNode.publish(ROBOT_SPORT_API_IDS["MOVE"],
                                   x = x_bind * teleopNode.speed,
                                   y = y_bind * teleopNode.speed,
                                   z = z_bind * teleopNode.angular)
            #4.速度调整(设置 api_id,和速度消息)
            elif key in speedBindings.keys():
                speed_bind = speedBindings[key][0]
                angular_bind = speedBindings[key][1]
                teleopNode.speed *= speed_bind
                teleopNode.angular *= angular_bind
                print("current speed:%.5f, angular:%.5f"%(teleopNode.speed,teleopNode.angular))
            else:
                #停止运动
                teleopNode.publish(ROBOT_SPORT_API_IDS["STOPMOVE"])

    finally:
        rclpy.shutdown()
        #go2 进入站立平衡状态
        teleopNode.publish(ROBOT_SPORT_API_IDS["BALANCESTAND"])


if __name__ == '__main__':
    main()