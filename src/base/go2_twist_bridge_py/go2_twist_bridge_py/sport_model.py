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