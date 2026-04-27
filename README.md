[English](README.en.md) | 简体中文

<div align="center">
<h1>G1 NavGrasp</h1>

[![ROS2 Foxy](https://img.shields.io/badge/ROS2-Foxy-blue)](https://docs.ros.org/en/foxy/)
[![YOLOv11](https://img.shields.io/badge/YOLOv11-目标检测-green)](https://docs.ultralytics.com/)
[![纯视觉](https://img.shields.io/badge/纯视觉-路径规划-orange)](https://navigation.ros.org/)
[![Python 3.8](https://img.shields.io/badge/Python-3.8+-yellow)](https://www.python.org/)

[![Auth](https://img.shields.io/badge/Auther--HunLi-ff69b4.svg?logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAADIAAAAyCAYAAAAeP4ixAAAACXBIWXMAAAsTAAALEwEAmpwYAAADZElEQVR4nO2ZX2iPURjHP/7/aZN/E21DaZvtwoVYyQUuGXLB/LtkLRcUhSJMSVwg3KCUJPJvLmRZtMQFLvwZhUJk/saGLWaYV6eet06n9/3tfd+9531/sW89td9z3vOc8z3nPOc8zzPoQQ+yAnlAA/AbcCzKR2ClTSL7LRNwNPkB5Noi0pggEQeYbovIh4SJLLBBojfwK2EiVTaIjEyYhANsskGkNAUi+2wQmZECkRM2iCxMgUi9DSKrxPhS7GOZjHXPhvGtYlwdMduYKWO9tmH8oBgvxj6KZawOoFfcxk+L8RzsI0fzkyFhO28EvqXg0FHlK1DtReRzFkzOCSmfvIhczIKJOSHlmheRgcAK4E4WTNDpQp4Ba4P4zxRgO3ArgSQqqDQDx4AKCWBDYwSwRDP4JWECG4CpQB9iwCTNsBsJl8srvxk4CtQBN4Enkre0GKQ7RaekCXgAXAcuAHuB1bLaZcAgrd+2uK78ai0PsfLSBshG70u9IDJmyUq2AYvlbD5N4Dg9l/EnArdFV9sdIlfEiLrNzFAlqPwE2kP2OaKNlyfH8Q9QFJVIixgepukqIuTdRRI3Be1TaczjnOgXRSXyTgwUarrBIVb4htbvcMA+nR7+UCdtc6MSOS8G9hj6+gjlnHEBd+WuMVaZ9FMyJiqRydrgB4B80VcFmFBtxALfFm3nlwPv48rfK7WoWDkc8jgpYq0eE/kOnAGG+4RBh3z6qTGOyzfIJeG2nQL6EQMmaEaTgtNdv+jKcFJwbI33XxEpAM76+IArrXIbFqVFpE0MuzeXF4nmEI9ecwZbhVo6GzsaxLhnniw74YQUVdDwwjppv2yzaPbIJ7HJdJz8RIX5JgYAL6V9vg0ifYEXMsAaj3Z3cpfkmPmhQAs5vHxgh+gbbdS0XMyTQdrkbdHhTiwTCdMHTCLl8hCqeGsalnFSO2JDNb3XxOZINqhkttFmfq8W4I1PbGcFuVr21qCloe7EarRvmzT9K01fYxAZJVmgA1yNKxwh4NF4q0Wr+UZs5JIxndskoVLoEskI1e/HRu6TCEpklR1ZebPcqk/YT9cu1UL190NgNCmhQGpfTjelLo2dMNEf2BkylXVF7eL6qAU3WyiVvCHIv7A7pBY2nizGWGC3FOr0XeoQZ96VIcbqAf8K/gLNGaTJ3vwbFgAAAABJRU5ErkJggg==)](https://github.com/HunLi-X)
[![cnb](https://img.shields.io/badge/CNB-xhunli-F76945?logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAAAXNSR0IArs4c6QAAAppJREFUOE9tk01rE1EUhs+5H5lJ0iQT3FRX2Yi4avoLkv4CWxBcNt11oZiuFKGmXQhFN6kuRESSgitXKf6A5g+I6cqFLiKCiptOkraZ5H4cuZOkH9oLw+UO8z7nvGfei3DFohoEQLAKCGWyGJCSbQZsD3ei7r+f45WATV6xmjdISyAlYLaDliveux+ti5ozwLAaFPhIlk6G6f18sxuO7l9bBiWbVoncDGCMWMl++PI/gJ7IohkmD0zkBxT5oRn5LTpN7ULqBByEtFywWvRyHz8Fx3dvN0UwuMPyR0uJFycddH7tKPHZRn7BjnwwkQcUud0HO/K7oOENcTYPRCEAFERmsCpyfZDZfpdnBosOUKGx13BC60QTIdjpmSzrEWDR+UagjsgMciLXAwfhc4MNpKfQtOPE6mVh0gEOaeTXtWXXEdADpJAR/GaZ/rrM9Us82wORPd5zHbRp7JVcyxR5PRP5LTPy6uo0HXIlG9awMgLuEUABkBYI7VLyxs+yyPW3+Nyg4zqoG+UV7TDRlMBbWA/D43u3qlbLGmgRWM2BjFxjQjWQGQBuu4bYYvrm14JMnQSXcjBcny9A5DXIiDJp9/8FWCXAAOaFUEfILTgIcttOvf+2NJnLdI0fBFUyskZKBGTEJEAmBuzr9LCSUDwGQAwwgMzu+m9/VWOAeZSsk+YP48C4li/sRok1nlBd5PogBjgxNw70PfHyqBADyEVXicasZXCVpxYiLfO+HxVZDHCVYwvu2ebPT7fOLNAm65AWLnEAzvsEcOi9/lPU1aACXDfOLdhD9kxNszGdweQGYstqXgItJwDFNxKvwrp57G8h0zVwldHuA0IFt8El83yIs2FSDcpgxDJpUUTwK7gTduN3AK5iG7ehc/E2/gUPD3q3eY4awwAAAABJRU5ErkJggq==)](https://cnb.cool/u/xhunli)
<p>让 G1 看见世界，走向目标，伸出双手。</p>

<img src="https://cnb.cool/66666/resource/-/git/raw/main/img/hengtiao.gif" width="100%" height="3">
</div><br>

## 项目介绍

宇树 G1 人形机器人 **YOLO 目标识别与路径规划导航** 工作空间。基于 ROS2 Foxy + Python，实现实时目标检测、3D 空间定位与自主导航接近目标。

**技术栈**：Ubuntu 20.04 · ROS2 Foxy · Python 3.8+ · YOLOv11 · Nav2 · colcon

**核心能力**：

- YOLO 实时目标检测（支持自定义训练模型）
- 2D 检测结果到 3D 空间坐标投影
- Nav2 路径规划与自主导航
- 紧急停止与安全限速保护

### 项目结构

```
g1act_ws/
├── src/
│   ├── g1_yolo_nav_py/                      # YOLO 目标识别与导航功能包
│   │   ├── g1_yolo_nav_py/
│   │   │   ├── sport_client.py              # 统一运动控制模块（Loco API）
│   │   │   ├── yolo_detector.py             # YOLO 检测节点
│   │   │   ├── spatial_target.py            # 3D 空间投影节点
│   │   │   ├── detection_visualizer.py      # 检测可视化节点（tkinter）
│   │   │   ├── control_panel.py             # 控制面板节点（tkinter GUI）
│   │   │   ├── grasp_task.py                # 抓取任务主控节点
│   │   │   ├── yaw_align.py                 # 偏航对齐节点（机器人旋转）
│   │   │   ├── loco_forward.py              # 前进控制节点（Loco API）
│   │   │   ├── rgbd_capture.py              # RGBD 数据采集节点
│   │   │   └── distence.py                  # 距离估算工具
│   │   ├── arm/                             # 手臂控制脚本（unitree_sdk2py，非 ROS2 节点）
│   │   │   ├── armup.py                     # 手臂抓取（完成后自动退出）
│   │   │   ├── armdown.py                   # 手臂放下（完成后自动退出）
│   │   │   └── arm.py                       # 手臂 SDK 基础控制
│   │   ├── launch/
│   │   │   └── yolo_nav.launch.py           # 导航管线启动文件
│   │   ├── config/
│   │   │   └── yolo_nav.yaml                # 参数配置
│   │   └── yolo_v11x_best.pt                # YOLOv11 自定义训练模型权重
│   └── base/                                # G1 机器人基础包
│       ├── g1_description/                  # URDF/MJCF 模型（12/23/29dof 变体）
│       ├── g1_driver_py/                    # ROS2 驱动：odom, TF, joint_states
│       ├── g1_teleop_ctrl_keyboard/         # 键盘遥控（Sport API）
│       ├── g1_twist_bridge_py/              # Twist → Sport API 桥接
│       ├── ctrl_keyboard/                   # 键盘+语音控制（Loco API + 手臂动作）
│       └── h1_description/                  # H1 机器人描述
├── requirements.txt
└── README.md
```
```更新代码命令
cd ~/g1act_ws/manact_ws
git pull
colcon build --packages-skip h1_description
```

```每次启动激活虚拟环境命令
cd ~/g1act_ws/manact_ws
source ~/g1act_venv/bin/activate
. install/setup.bash
```
### 模型说明

| 文件                  | 说明                                              |
| --------------------- | ------------------------------------------------- |
| `yolo_v11x_best.pt` | YOLOv11x 自定义训练模型权重文件，用于目标检测推理 |

## 快速开始

### 环境搭建（首次必做）

> **实验室要求**：每个实验者必须使用独立的 Python 虚拟环境，禁止在主系统 Python 中安装包。

```bash
# 1) 创建虚拟环境（允许访问系统已安装的 ROS2 / unitree_sdk2py）
python3 -m venv ~/g1act_venv --system-site-packages

# 2) 激活虚拟环境
source ~/g1act_venv/bin/activate

# 3) 安装项目依赖
cd ~/g1act_ws/manact_ws
pip install -r requirements.txt

# 4) 编译 ROS2 工作空间
colcon build
```

```bash
# --- 每次启动项目 ---
source ~/g1act_venv/bin/activate
cd ~/g1act_ws
. install/setup.bash
```

### 仅测试 YOLO 目标检测

```bash
# 1. 启动 D455 相机（终端 1）
ros2 launch realsense2_camera rs_launch.py camera_namespace:=robot1 camera_name:=D455_1

# 2. 编译并启动 YOLO 检测节点（终端 2）
source ~/g1act_venv/bin/activate
colcon build --packages-select g1_yolo_nav_py && . install/setup.bash
ros2 run g1_yolo_nav_py yolo_detector

# 3. 启动可视化节点（终端 3）
ros2 run g1_yolo_nav_py detection_visualizer

# 4. 查看标注图像（终端 4）
ros2 topic echo /g1/vision/detections
```

**自定义参数：**

```bash
# 指定检测目标为 person
ros2 run g1_yolo_nav_py yolo_detector --ros-args -p target_classes:='["person"]'

# 指定自定义模型路径
ros2 run g1_yolo_nav_py yolo_detector --ros-args -p model_path:=/path/to/model.pt
```

---

## 分开调试：目标对齐 vs 运动控制

对齐和前进是**独立节点**，可以单独启动调试，互不影响。

### 调试目标对齐（yaw_align）

**只验证**：机器人能否通过旋转让目标保持在画面中央。

```bash
# 终端 1：相机
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=robot1 camera_name:=D455_1

# 终端 2：YOLO 检测
ros2 run g1_yolo_nav_py yolo_detector

# 终端 3：偏航对齐（Loco API SET_VELOCITY）
ros2 run g1_yolo_nav_py yaw_align

# 终端 4（可选）：可视化
ros2 run g1_yolo_nav_py detection_visualizer
```

**观察要点：**

- 日志显示 `[对齐] 检测到目标: u=0.xxx, vyaw=x.xxx`
- 把目标放在机器人前方偏左/偏右 → 机器人应该自动旋转
- 目标在画面正中央时停止旋转

**对齐参数调优：**

```bash
# 追踪太慢 — 增大 kp
ros2 run g1_yolo_nav_py yaw_align --ros-args -p yaw_kp:=3.0

# 抖动太厉害 — 减小 kp + 放宽容差
ros2 run g1_yolo_nav_py yaw_align --ros-args \
  -p yaw_kp:=1.0 -p center_tolerance:=0.12

# 限制最大旋转速度
ros2 run g1_yolo_nav_py yaw_align --ros-args -p max_yaw_speed:=0.3
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `yaw_kp` | 2.0 | P 增益，太慢→增大，抖动→减小 |
| `center_tolerance` | 0.08 | 居中容差（归一化），越小越严格 |
| `max_yaw_speed` | 0.6 rad/s | 最大旋转速度 |
| `lost_timeout` | 1.0 s | 目标丢失超时 |

**调试通过标志**：目标移动时机器人能平滑旋转跟踪，目标居中时稳定不抖。

---

### 调试运动控制（loco_forward）

**只验证**：目标居中后机器人能否前进并自动停止。

```bash
# 终端 1~3：同上（相机 + YOLO + yaw_align）

# 终端 4：前进控制（Loco API SET_VELOCITY）
ros2 run g1_yolo_nav_py loco_forward
```

**观察要点：**

- 目标在画面中心且持续 >0.8s 后，日志显示 `开始前进`
- 机器人以设定的速度向前走
- 检测框变大到占画面 45% 以上时，日志显示 `到达目标!` 并停止
- 目标丢失或偏离中心时自动停止前进

**前进参数调优：**

```bash
# 首次调试用低速度确保安全
ros2 run g1_yolo_nav_py loco_forward --ros-args -p forward_speed:=0.15

# 想让机器人离目标更近才停下
ros2 run g1_yolo_nav_py loco_forward --ros-args -p arrive_bbox_ratio:=0.6
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `forward_speed` | 0.3 m/s | 前进速度（首次建议 0.15） |
| `align_stable_time` | 0.8 s | 居中多久才开始前进 |
| `arrive_bbox_ratio` | 0.45 | 检测框占比到达阈值 |
| `center_tolerance` | 0.08 | 居中容差 |
| `lost_timeout` | 1.0 s | 目标丢失超时 |

**调试通过标志**：机器人能正确前进、到达目标附近停止、目标丢失时立即停止。

---

### 联合运行

对齐和前进都调通后，一起使用：

```bash
# 方式 A：手动分别启动（推荐调试，方便看各自日志）
# 终端 1: 相机
# 终端 2: ros2 run g1_yolo_nav_py yolo_detector
# 终端 3: ros2 run g1_yolo_nav_py yaw_align
# 终端 4: ros2 run g1_yolo_nav_py loco_forward

# 方式 B：launch 一键启动
ros2 launch g1_yolo_nav_py yolo_nav.launch.py enable_approach:=true
```

**安全提示**：

- 首次联调时 `forward_speed` 建议 ≤ 0.2 m/s
- 随时准备按遥控器急停
- 确保前方无障碍物

---

## 控制面板（GUI 集成控制）

基于 tkinter 的图形化控制界面，集成检测可视化 + 手动遥控 + 一键抓取任务全流程。

**功能：**

- 左右双画布：原始相机图像 + YOLO 检测标注图像
- 方向按钮：前进/后退/左转/右转/停止（IDLE 状态下可用）
- 任务按钮：搜索/抓取/放下/右转放下
- 速度滑块：0.05~0.5 m/s 可调
- 实时 FPS、状态机状态、检测信息显示
- 底部日志栏：arm 脚本输出实时显示

**状态机流程：**

```
IDLE → [搜索] → SEARCHING（旋转找目标）
                  ↓ 目标出现
                ALIGNING（偏航对齐居中）
                  ↓ 居中稳定
                APPROACHING（Loco API 前进）
                  ↓ 到达目标
                GRABBING（执行 armup.py）
                  ↓ 抓取完成
                MENU（可放下/右转放下）
                  ↓ 放下完成
                IDLE
```

**运行：**

```bash
# 终端 1：YOLO 检测
ros2 run g1_yolo_nav_py yolo_detector

# 终端 2：控制面板
ros2 run g1_yolo_nav_py control_panel
```

> **注意**：控制面板不导入 `unitree_sdk2py`（避免 DDS 冲突），
> 运动控制通过 `SportClient` 封装，使用 Loco API 直接发布到 `/api/sport/request`。

**参数：**

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `image_topic` | `/D455_1/color/image_raw` | 相机图像话题 |
| `detection_topic` | `/g1/vision/detections` | 检测结果话题 |
| `target_class_id` | `chair` | 目标类别 |
| `forward_speed` | `0.2` | 前进速度 m/s |
| `search_yaw_speed` | `0.3` | 搜索旋转速度 rad/s |
| `yaw_kp` | `2.0` | 偏航对齐 P 增益 |
| `arrive_bbox_ratio` | `0.45` | 到达判断阈值 |
| `arm_script_dir` | `~/g1act_ws/manact_ws/src/g1_yolo_nav_py/arm` | arm 脚本目录 |

**依赖：** `pip install pillow`

---

## 抓取任务（一键全流程）

自动执行：目标检测 → 偏航对齐 → 前进接近 → 抓取 → 交互菜单。

**前置条件：**

- G1 机器人已连接并站立
- `armup.py` / `armdown.py` 已就位（`arm/` 目录下，完成后自动退出）

**手动分步启动（调试用）：**

```bash
# 终端 1：相机
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=robot1 camera_name:=D455_1 align_depth.enable:=true

# 终端 2：YOLO 检测
ros2 run g1_yolo_nav_py yolo_detector

# 终端 3：抓取任务（Loco API 模式，无需 DDS）
ros2 run g1_yolo_nav_py grasp_task
```

**执行流程与关键日志：**

```
==================================================
G1 抓取任务启动（Loco API 模式）
目标类别: chair
armup: ~/g1act_ws/manact_ws/src/g1_yolo_nav_py/arm/armup.py
armdown: ~/g1act_ws/manact_ws/src/g1_yolo_nav_py/arm/armdown.py
==================================================
[SEARCHING]  旋转搜索目标...
[检测] 识别到目标: chair, 置信度=95%, u=0.32
[状态] SEARCHING → ALIGNING: 目标已找到
[状态] ALIGNING → APPROACHING: 目标已居中
[状态] APPROACHING → GRABBING: 到达目标! bbox=0.46 >= 0.45
[抓取] 执行 armup.py ...
[抓取] armup.py 执行完成
```

**抓取完成后自动弹出交互菜单：**

```
========================================
  G1 抓取任务 — 操作菜单
========================================
  1. 放下目标物
  2. 右转放下目标物
  3. 自定义控制（输入 xyz）
  4. 退出
========================================
请选择 [1-4]:
```

| 选项 | 动作 | 说明 |
| --- | --- | --- |
| 1 | 执行 `armdown.py` | 原地放下目标物 |
| 2 | 右转 90° + `armdown.py` | 转身后放下 |
| 3 | 输入 `x y z` 控制移动 | 如 `0.2 0.0 0.3`，输入 `q` 停止 |
| 4 | 安全退出 | 停止运动并退出程序 |

**参数调优：**

```bash
# 检测瓶子、降低速度
ros2 run g1_yolo_nav_py grasp_task --ros-args \
  -p target_class_id:=bottle \
  -p forward_speed:=0.15

# 自定义 arm 脚本目录
ros2 run g1_yolo_nav_py grasp_task --ros-args \
  -p arm_script_dir:=/home/unitree/my_arm_scripts
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `target_class_id` | chair | YOLO 检测目标类别 |
| `forward_speed` | 0.2 m/s | 接近速度（首次建议 0.15） |
| `arrive_bbox_ratio` | 0.45 | 检测框占比到达阈值 |
| `search_yaw_speed` | 0.3 rad/s | 搜索旋转速度 |
| `yaw_kp` | 2.0 | 偏航对齐 P 增益 |
| `lost_timeout` | 2.0 s | 目标丢失超时 |
| `arm_script_dir` | `~/g1act_ws/manact_ws/src/g1_yolo_nav_py/arm` | arm 脚本目录 |

> **安全提示**：首次运行建议 `forward_speed:=0.15`，随时准备急停。

---

## RGBD 数据采集

```bash
# 相机启动时需开启深度对齐
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=robot1 camera_name:=D455_1 align_depth.enable:=true

# 运行采集（默认每 5s 一帧，持续 60s）
ros2 run g1_yolo_nav_py rgbd_capture

# 自定义参数
ros2 run g1_yolo_nav_py rgbd_capture --ros-args \
  -p interval_sec:=2.0 -p duration_sec:=30.0
```

## 运动控制方式

所有运动控制节点统一使用 **Loco API（7xxx 系列）**，通过 `SportClient` 封装发布到 `/api/sport/request`。

| API | ID | 用途 | 参数格式 |
| --- | --- | --- | --- |
| SET_FSM_ID | 7101 | 状态机控制 | `{"data": fsm_id}` |
| SET_VELOCITY | 7105 | 速度控制 | `{"velocity": [vx, vy, vyaw], "duration": t}` |
| SET_BALANCE_MODE | 7102 | 平衡模式 | `{"data": mode}` |
| SET_SPEED_MODE | 7107 | 速度模式 | `{"data": mode}` |
| SET_STAND_HEIGHT | 7104 | 站立高度 | `{"data": height}` |

**FSM 状态机 ID：**

| ID | 状态 | 说明 |
| --- | --- | --- |
| 0 | ZERO_TORQUE | 零力矩模式（初始坐姿） |
| 1 | DAMP | 阻尼模式（准备站立） |
| 3 | SIT | 坐下 |
| 4 | STAND_UP | 锁定站立 |
| 500 | START | 常规运控 |
| 801 | WALK_RUN | 走跑运控 |

**平衡模式：**

| ID | 模式 | 说明 |
| --- | --- | --- |
| 0 | BALANCE_STAND | 平衡站立（速度为0时停止踏步） |
| 1 | CONTINUOUS_GAIT | 连续步态（持续踏步） |

**FSM 初始化流程：**

```
SET_FSM_ID(DAMP=1) → SET_FSM_ID(STAND_UP=4) → SET_FSM_ID(WALK_RUN=801) → SET_BALANCE_MODE(CONTINUOUS_GAIT=1)
```

WALK_RUN + CONTINUOUS_GAIT 模式下 `SET_VELOCITY(7105)` 才生效。

> **注意**：运动控制方案参考 `ctrl_keyboard` 已验证可用的实现，与 `ctrl_keyboard` 使用完全相同的 API 体系。

---

![Fork 历史趋势图。](https://commit.cool/forks/1255027942/cloud/manact_ws?interval=day)
