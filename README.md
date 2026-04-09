[English](README.en.md) | 简体中文

<div align="center">
<h1>G1 YOLO Nav</h1>

[![ROS2 Foxy](https://img.shields.io/badge/ROS2-Foxy-blue)](https://docs.ros.org/en/foxy/)
[![YOLOv11](https://img.shields.io/badge/YOLOv11-目标检测-green)](https://docs.ultralytics.com/)
[![Nav2](https://img.shields.io/badge/Nav2-路径规划-orange)](https://navigation.ros.org/)
[![Python 3.8](https://img.shields.io/badge/Python-3.8+-yellow)](https://www.python.org/)

[![Auth](https://img.shields.io/badge/Auther--HunLi-ff69b4.svg?logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAADIAAAAyCAYAAAAeP4ixAAAACXBIWXMAAAsTAAALEwEAmpwYAAADZElEQVR4nO2ZX2iPURjHP/7/aZN/E21DaZvtwoVYyQUuGXLB/LtkLRcUhSJMSVwg3KCUJPJvLmRZtMQFLvwZhUJk/saGLWaYV6eet06n9/3tfd+9531/sW89td9z3vOc8z3nPOc8zzPoQQ+yAnlAA/AbcCzKR2ClTSL7LRNwNPkB5Noi0pggEQeYbovIh4SJLLBBojfwK2EiVTaIjEyYhANsskGkNAUi+2wQmZECkRM2iCxMgUi9DSKrxPhS7GOZjHXPhvGtYlwdMduYKWO9tmH8oBgvxj6KZawOoFfcxk+L8RzsI0fzkyFhO28EvqXg0FHlK1DtReRzFkzOCSmfvIhczIKJOSHlmheRgcAK4E4WTNDpQp4Ba4P4zxRgO3ArgSQqqDQDx4AKCWBDYwSwRDP4JWECG4CpQB9iwCTNsBsJl8srvxk4CtQBN4Enkre0GKQ7RaekCXgAXAcuAHuB1bLaZcAgrd+2uK78ai0PsfLSBshG70u9IDJmyUq2AYvlbD5N4Dg9l/EnArdFV9sdIlfEiLrNzFAlqPwE2kP2OaKNlyfH8Q9QFJVIixgepukqIuTdRRI3Be1TaczjnOgXRSXyTgwUarrBIVb4htbvcMA+nR7+UCdtc6MSOS8G9hj6+gjlnHEBd+WuMVaZ9FMyJiqRydrgB4B80VcFmFBtxALfFm3nlwPv48rfK7WoWDkc8jgpYq0eE/kOnAGG+4RBh3z6qTGOyzfIJeG2nQL6EQMmaEaTgtNdv+jKcFJwbI33XxEpAM76+IArrXIbFqVFpE0MuzeXF4nmEI9ecwZbhVo6GzsaxLhnniw74YQUVdDwwjppv2yzaPbIJ7HJdJz8RIX5JgYAL6V9vg0ifYEXMsAaj3Z3cpfkmPmhQAs5vHxgh+gbbdS0XMyTQdrkbdHhTiwTCdMHTCLl8hCqeGsalnFSO2JDNb3XxOZINqhkttFmfq8W4I1PbGcFuVr21qCloe7EarRvmzT9K01fYxAZJVmgA1yNKxwh4NF4q0Wr+UZs5JIxndskoVLoEskI1e/HRu6TCEpklR1ZebPcqk/YT9cu1UL190NgNCmhQGpfTjelLo2dMNEf2BkylXVF7eL6qAU3WyiVvCHIv7A7pBY2nizGWGC3FOr0XeoQZ96VIcbqAf8K/gLNGaTJ3vwbFgAAAABJRU5ErkJggg==)](https://github.com/HunLi-X)
[![cnb](https://img.shields.io/badge/CNB-xhunli-F76945?logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAAAXNSR0IArs4c6QAAAppJREFUOE9tk01rE1EUhs+5H5lJ0iQT3FRX2Yi4avoLkv4CWxBcNt11oZiuFKGmXQhFN6kuRESSgitXKf6A5g+I6cqFLiKCiptOkraZ5H4cuZOkH9oLw+UO8z7nvGfei3DFohoEQLAKCGWyGJCSbQZsD3ei7r+f45WATV6xmjdISyAlYLaDliveux+ti5ozwLAaFPhIlk6G6f18sxuO7l9bBiWbVoncDGCMWMl++PI/gJ7IohkmD0zkBxT5oRn5LTpN7ULqBByEtFywWvRyHz8Fx3dvN0UwuMPyR0uJFycddH7tKPHZRn7BjnwwkQcUud0HO/K7oOENcTYPRCEAFERmsCpyfZDZfpdnBosOUKGx13BC60QTIdjpmSzrEWDR+UagjsgMciLXAwfhc4MNpKfQtOPE6mVh0gEOaeTXtWXXEdADpJAR/GaZ/rrM9Us82wORPd5zHbRp7JVcyxR5PRP5LTPy6uo0HXIlG9awMgLuEUABkBYI7VLyxs+yyPW3+Nyg4zqoG+UV7TDRlMBbWA/D43u3qlbLGmgRWM2BjFxjQjWQGQBuu4bYYvrm14JMnQSXcjBcny9A5DXIiDJp9/8FWCXAAOaFUEfILTgIcttOvf+2NJnLdI0fBFUyskZKBGTEJEAmBuzr9LCSUDwGQAwwgMzu+m9/VWOAeZSsk+YP48C4li/sRok1nlBd5PogBjgxNw70PfHyqBADyEVXicasZXCVpxYiLfO+HxVZDHCVYwvu2ebPT7fOLNAm65AWLnEAzvsEcOi9/lPU1aACXDfOLdhD9kxNszGdweQGYstqXgItJwDFNxKvwrp57G8h0zVwldHuA0IFt8El83yIs2FSDcpgxDJpUUTwK7gTduN3AK5iG7ehc/E2/gUPD3q3eY4awwAAAABJRU5ErkJggq==)](https://cnb.cool/u/xhunli)

<p>导语</p>

<img src="https://cnb.cool/66666/resource/-/git/raw/main/img/hengtiao.gif" width="100%" height="3">
</div><br>


## ✨ 项目介绍

宇树 G1 人形机器人 **YOLO 目标识别与路径规划导航** 工作空间。基于 ROS2 Foxy + Python，实现实时目标检测、3D 空间定位与自主导航接近目标。

**技术栈**：Ubuntu 20.04 · ROS2 Foxy · Python 3.8+ · YOLOv11 · Nav2 · colcon

**核心能力**：

- YOLO 实时目标检测（支持自定义训练模型）
- 2D 检测结果到 3D 空间坐标投影
- Nav2 路径规划与自主导航
- 紧急停止与安全限速保护

### 📁 项目结构

```
g1act_ws/
├── src/
│   └── g1_yolo_nav_py/                  # YOLO 目标识别与导航功能包
│       ├── g1_yolo_nav_py/
│       │   ├── __init__.py
│       │   ├── yolo_detector.py          # YOLO 检测节点
│       │   ├── spatial_target.py         # 3D 空间投影节点
│       │   └── nav_planner.py            # 导航规划节点
│       ├── launch/
│       │   └── yolo_nav.launch.py        # 统一启动文件
│       ├── config/
│       │   └── yolo_nav.yaml             # 参数配置
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       └── yolo_v11x_best.pt             # YOLOv11 自定义训练模型权重
├── ros2doc_skill/                        # ROS2 开发规范技能包
└── README.md
```

### 📦 模型说明

| 文件                  | 说明                                              |
| --------------------- | ------------------------------------------------- |
| `yolo_v11x_best.pt` | YOLOv11x 自定义训练模型权重文件，用于目标检测推理 |

> 该模型为已训练完成的 YOLOv11 权重文件，部署时放置于 `src/g1_yolo_nav_py/` 根目录下，通过 launch 参数 `model_path` 指定加载路径。

### 🚀 快速开始

#### x86 开发机

```bash
# 安装依赖
pip3 install ultralytics

# 编译
colcon build --packages-select g1_yolo_nav_py
. install/setup.bash

# 运行完整导航管线（需先启动相机驱动和 TF 树）
ros2 launch g1_yolo_nav_py yolo_nav.launch.py

# 启用 Nav2 导航 + 深度传感器
ros2 launch g1_yolo_nav_py yolo_nav.launch.py use_nav2:=true use_depth_sensor:=true
```

#### aarch64 机器人端（G1 实机）

> PyTorch 的 `libgomp` 在 aarch64 上需要 `LD_PRELOAD`，而 `ros2 run` 会清除该环境变量，
> 因此需使用 wrapper 脚本启动。

```bash
# 安装依赖
pip3 install ultralytics

# 编译
colcon build --packages-select g1_yolo_nav_py

# 使用 wrapper 脚本启动 YOLO 检测（自动注入 LD_PRELOAD）
./run_yolo.sh

# 手动设置 LD_PRELOAD 后直接运行
. install/setup.bash
export LD_PRELOAD=~/.local/lib/python3.8/site-packages/torch.libs/libgomp-804f19d4.so.1.0.0
python3 -m g1_yolo_nav_py.yolo_detector

# 其他节点（不依赖 PyTorch）可直接 ros2 run
ros2 run g1_yolo_nav_py waist_align
ros2 run g1_yolo_nav_py loco_forward
```

**`run_yolo.sh` 内容参考：**
```bash
#!/bin/bash
cd $(dirname $0)
. install/setup.bash
export LD_PRELOAD=~/.local/lib/python3.8/site-packages/torch.libs/libgomp-804f19d4.so.1.0.0
python3 -m g1_yolo_nav_py.yolo_detector
```

### 🧪 仅测试 YOLO 目标检测

如果只需验证 YOLO 检测功能，无需启动完整导航管线：

```bash
# 1. 启动 D455 相机（终端 1）
ros2 launch realsense2_camera rs_launch.py camera_namespace:=robot1 camera_name:=D455_1

# 2. 编译并启动 YOLO 检测节点（终端 2）
colcon build --packages-select g1_yolo_nav_py
. install/setup.bash
ros2 run g1_yolo_nav_py yolo_detector          # x86
# 或 ./run_yolo.sh                               # aarch64 机器人

# 3. 启动可视化节点（终端 3）
ros2 run g1_yolo_nav_py detection_visualizer
# 标注图像发布到 /g1/vision/annotated_image 话题

# 4. 查看标注图像（终端 4，远程 SSH 也可用）
rqt_image_view  # 选择 /g1/vision/annotated_image

# 本地有显示器时可开启窗口显示
ros2 run g1_yolo_nav_py detection_visualizer --ros-args -p display:=true
# 按 q 键退出

# 或查看原始检测结果文本
ros2 topic echo /g1/vision/detections
```

**自定义参数示例：**

```bash
# 指定检测目标为 person
ros2 run g1_yolo_nav_py yolo_detector --ros-args -p target_classes:='["person"]'

# 指定自定义模型路径
ros2 run g1_yolo_nav_py yolo_detector --ros-args -p model_path:=/path/to/model.pt

# 查看模型所有可用类别（在 Python 中执行）
python3 -c "from ultralytics import YOLO; print(YOLO('src/g1_yolo_nav_py/yolo_v11x_best.pt').names)"
```

### 🎯 视觉伺服追踪与趋近

两个独立程序，可单独调试：

| 程序 | 功能 | 控制方式 | 入口命令 |
|------|------|---------|---------|
| `waist_align` | 腰部旋转让目标居中 | Arm SDK DDS | `ros2 run g1_yolo_nav_py waist_align` |
| `loco_forward` | 对齐后前进到目标 | LocoClient RPC | `ros2 run g1_yolo_nav_py loco_forward` |

**工作流程（两个节点协作）：**
```
  YOLO检测 ──→ waist_align(腰部对齐) ──→ loco_forward(Loco前进)
                目标居中后才允许前进         居中+稳定 → Move(vx)
```

**前置条件：**
- G1 机器人已连接并处于站立状态
- 已安装 `unitree_sdk2py`：`pip install unitree_sdk2py`
- 相机已启动并发布图像

---

#### 📌 步骤一：调试腰部对齐 (`waist_align`)

**目的**：确认腰部能正确追踪目标，让目标保持在画面中心。

```bash
# 终端 1：启动相机
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=robot1 camera_name:=D455_1

# 终端 2：编译 + 启动检测
colcon build --packages-select g1_yolo_nav_py && . install/setup.bash
ros2 run g1_yolo_nav_py yolo_detector

# 终端 3：启动腰部对齐（先只跑这个！）
ros2 run g1_yolo_nav_py waist_align

# 或一键启动（仅腰部对齐，不含前进）
ros2 launch g1_yolo_nav_py yolo_nav.launch.py enable_waist_tracking:=true
```

**观察要点：**
- 日志应显示 "腰部对齐控制启动"
- 把椅子放在机器人前方偏左/偏右 → 腰部应该自动转向
- 目标在画面正中央时腰部停止转动

**腰部对齐参数：**

| 参数 | 默认值 | 调试建议 |
|------|--------|---------|
| `waist_kp` | 1.5 | 太慢→增大(如2.0)，抖动→减小(如0.8) |
| `center_tolerance` | 0.08 | 容差范围，太小会不停微调 |
| `max_waist_speed` | 0.5 rad/s | 追踪最大转速 |
| `max_waist_angle` | 0.8 rad (~45°) | 最大转角限制 |

**参数调优：**
```bash
# 追踪太慢 — 增大 kp
ros2 run g1_yolo_nav_py waist_align --ros-args -p waist_kp:=2.5

# 抖动太厉害 — 减小 kp + 放宽容差
ros2 run g1_yolo_nav_py waist_align --ros-args \
  -p waist_kp:=0.8 -p center_tolerance:=0.12

# 多网卡指定接口
ros2 run g1_yolo_nav_py waist_align --ros-args -p network_interface:=eth0
```

**调试通过标志**：目标移动时腰部能平滑跟踪，目标停在画面中央时腰部稳定不抖。✅

---

#### 📌 步骤二：调试 Loco 前进 (`loco_forward`)

**前提**：步骤一的腰部对齐已经正常工作。

**目的**：确认目标居中后机器人能正确前进并自动停止。

```bash
# 在步骤一的基础上，额外开一个终端：
# 终端 4：启动 Loco 前进
ros2 run g1_yolo_nav_py loco_forward
```

**观察要点：**
- 当目标在画面中心且持续 >0.8s 后，日志显示 "开始前进"
- 机器人以设定的速度向前走
- 检测框变大到占画面 45% 以上时，日志显示 "到达目标!" 并停止
- 目标丢失或偏离中心时自动停止前进

**Loco 前进参数：**

| 参数 | 默认值 | 调试建议 |
|------|--------|---------|
| `forward_speed` | 0.3 m/s | 首次调试建议用 0.1~0.2 |
| `align_stable_time` | 0.8 s | 居中多久才开始前进 |
| `arrive_bbox_ratio` | 0.45 | 检测框占比，越大=停得越远 |
| `center_tolerance` | 0.08 | 与 waist_align 保持一致 |
| `check_rate` | 10 Hz | 判断频率 |

**参数调优：**
```bash
# 首次调试用低速度确保安全
ros2 run g1_yolo_nav_py loco_forward --ros-args -p forward_speed:=0.15

# 想让机器人离目标更近才停下（提高阈值）
ros2 run g1_yolo_nav_py loco_forward --ros-args -p arrive_bbox_ratio:=0.6

# 加速判断（减少延迟）
ros2 run g1_yolo_nav_py loco_forward --ros-args \
  -p align_stable_time:=0.3 -p check_rate:=20.0
```

**调试通过标志**：机器人能正确前进、到达目标附近停止、目标丢失时立即停止。✅

---

#### 📌 步骤三：联合运行

两个程序都调通后，可以一起使用：

```bash
# 方式 A：launch 一键启动（推荐正式使用）
ros2 launch g1_yolo_nav_py yolo_nav.launch.py enable_waist_tracking:=true

# 方式 B：手动分别启动（方便看各自日志）
# 终端 2: ros2 run g1_yolo_nav_py yolo_detector
# 终端 3: ros2 run g1_yolo_nav_py waist_align
# 终端 4: ros2 run g1_yolo_nav_py loco_forward
```

**安全提示：**
- 首次联调时 `forward_speed` 建议 ≤ 0.2 m/s
- 随时准备按遥控器急停
- 确保前方无障碍物

---

![Fork 历史趋势图。](https://commit.cool/forks/1255027942/cloud/manact_ws?interval=day)
