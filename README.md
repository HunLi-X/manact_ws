[English](README.en.md) | 简体中文

<div align="center">
<h1>G1 YOLO Nav</h1>

[![ROS2 Foxy](https://img.shields.io/badge/ROS2-Foxy-blue)](https://docs.ros.org/en/foxy/)
[![YOLOv11](https://img.shields.io/badge/YOLOv11-目标检测-green)](https://docs.ultralytics.com/)
[![纯视觉](https://img.shields.io/badge/纯视觉-路径规划-orange)](https://navigation.ros.org/)
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

```更新代码命令
cd ~/g1act_ws/manact_ws

git pull

colcon build
```
### 📦 模型说明

| 文件                  | 说明                                              |
| --------------------- | ------------------------------------------------- |
| `yolo_v11x_best.pt` | YOLOv11x 自定义训练模型权重文件，用于目标检测推理 |

> 该模型为已训练完成的 YOLOv11 权重文件，部署时放置于 `src/g1_yolo_nav_py/` 根目录下，通过 launch 参数 `model_path` 指定加载路径。

### 🚀 快速开始

#### 1. 环境搭建（首次必做）

> **实验室要求**：每个实验者必须使用独立的 Python 虚拟环境，禁止在主系统 Python 中安装包。
> 禁止混用 ROS1 与 ROS2，禁止修改 `~/.bashrc` 中的 ROS 配置。

```bash
# --- 首次搭建 ---

# 1) 创建虚拟环境（允许访问系统已安装的 ROS2 / unitree_sdk2py）
python3 -m venv ~/g1act_venv --system-site-packages

# 2) 激活虚拟环境
source ~/g1act_venv/bin/activate

# 3) 安装项目依赖
cd ~/g1act_ws
pip install -r requirements.txt

# 4) 编译 ROS2 工作空间
colcon build
```

```bash
# --- 每次启动项目 ---

# 激活虚拟环境（每次开新终端都需要执行）
source ~/g1act_venv/bin/activate

# 加载 ROS2 工作空间
cd ~/g1act_ws
. install/setup.bash

# 之后即可运行项目（见下方各功能说明）
```

> **说明**：
> - `--system-site-packages` 让虚拟环境可以访问系统级已安装的 `rclpy`、`unitree_sdk2py` 等，
>   避免在虚拟环境中重复安装 ROS2 等大型依赖。
> - `ultralytics`、`opencv-python`、`numpy` 等项目依赖安装在虚拟环境中，与系统环境隔离。
> - 如需确认是否在虚拟环境中：`which python3` 应输出 `~/g1act_venv/bin/python3`。
> - 退出虚拟环境：`deactivate`

#### 2. x86 开发机

```bash
source ~/g1act_venv/bin/activate   # 激活虚拟环境

# 编译
colcon build --packages-select g1_yolo_nav_py
. install/setup.bash

# 运行完整导航管线（需先启动相机驱动和 TF 树）
ros2 launch g1_yolo_nav_py yolo_nav.launch.py

# 启用 Nav2 导航 + 深度传感器
ros2 launch g1_yolo_nav_py yolo_nav.launch.py use_nav2:=true use_depth_sensor:=true
```

#### 3. aarch64 机器人端（G1 实机）

> PyTorch 的 `libgomp` 在 aarch64 上需要 `LD_PRELOAD`，而 `ros2 run` 会清除该环境变量，
> 因此需使用 wrapper 脚本启动。

```bash
source ~/g1act_venv/bin/activate   # 激活虚拟环境

# 编译
colcon build --packages-select g1_yolo_nav_py

# 使用 wrapper 脚本启动 YOLO 检测（自动注入 LD_PRELOAD）
./run_yolo.sh

# 手动设置 LD_PRELOAD 后直接运行
. install/setup.bash
export LD_PRELOAD=~/.local/lib/python3.8/site-packages/torch.libs/libgomp-804f19d4.so.1.0.0
python3 -m g1_yolo_nav_py.yolo_detector

# 其他节点（不依赖 PyTorch）可直接 ros2 run
ros2 run g1_yolo_nav_py yaw_align
ros2 run g1_yolo_nav_py waist_align
ros2 run g1_yolo_nav_py loco_forward
```

**`run_yolo.sh` 内容参考：**
```bash
#!/bin/bash
cd $(dirname $0)
. install/setup.bash
export LD_PRELOAD=~/.local/lib/python3.8/site-packages/torch.libs/libgomp-804f19d4.so.1.0.0
export DISPLAY=:0              # SSH 远程时显示窗口到机器人桌面
python3 -m g1_yolo_nav_py.yolo_detector
```

> **远程查看可视化窗口**：机器人有桌面环境时，SSH 登录后设置 `export DISPLAY=:0`，
> 即可将 `cv2.imshow` 窗口显示在机器人本机屏幕上。也可通过 VNC/RDP 远程桌面查看。

### 📷 RGBD 数据采集

一行命令采集带检测框的彩色图像 + 深度图，默认每 5s 采集一帧，持续 60s（共 12 帧）。

```bash
# 前提：相机 + YOLO 检测已启动（见上方「仅测试 YOLO 目标检测」）
# 相机启动时需开启深度对齐：
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=robot1 camera_name:=D455_1 \
    align_depth.enable:=true

# 编译 + 运行采集
source ~/g1act_venv/bin/activate
colcon build --packages-select g1_yolo_nav_py && . install/setup.bash
ros2 run g1_yolo_nav_py rgbd_capture
```

**输出目录结构**（默认保存在 `src/g1_yolo_nav_py/rgbd_data/`）：
```
rgbd_data/
├── img/            # 彩色图像（带检测框标注），.jpg
│   ├── 0000.jpg
│   ├── 0005.jpg
│   └── ...
└── d/              # 深度图（16位 PNG，单位 mm）
    ├── 0000.png
    ├── 0005.png
    └── ...
文件名一一对应：0000.jpg ↔ 0000.png
```

**自定义参数：**
```bash
# 每 2s 采集一次，持续 30s
ros2 run g1_yolo_nav_py rgbd_capture --ros-args \
  -p interval_sec:=2.0 -p duration_sec:=30.0

# 指定输出目录
ros2 run g1_yolo_nav_py rgbd_capture --ros-args \
  -p output_dir:=/tmp/my_capture

# 指定深度话题（默认对齐到彩色的深度图）
ros2 run g1_yolo_nav_py rgbd_capture --ros-args \
  -p depth_topic:=/robot1/D455_1/depth/image_rect_raw
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `interval_sec` | 5.0 | 采集间隔（秒） |
| `duration_sec` | 60.0 | 总采集时长（秒） |
| `output_dir` | `src/g1_yolo_nav_py/rgbd_data/` | 输出目录 |
| `color_topic` | `/robot1/D455_1/color/image_raw` | 彩色图像话题 |
| `depth_topic` | `/robot1/D455_1/aligned_depth_to_color/image_raw` | 深度图话题 |

> **注意**：深度图需要相机启动时加 `align_depth.enable:=true` 参数，否则深度话题无数据。
> 如无深度相机，节点仍会保存彩色图像，日志提示"无深度"。

如果只需验证 YOLO 检测功能，无需启动完整导航管线：

```bash
# 1. 启动 D455 相机（终端 1）
ros2 launch realsense2_camera rs_launch.py camera_namespace:=robot1 camera_name:=D455_1

# 2. 编译并启动 YOLO 检测节点（终端 2）
source ~/g1act_venv/bin/activate
colcon build --packages-select g1_yolo_nav_py
. install/setup.bash
ros2 run g1_yolo_nav_py yolo_detector          # x86
# 或 ./run_yolo.sh                               # aarch64 机器人

# 3. 启动可视化节点（终端 3）
ros2 run g1_yolo_nav_py detection_visualizer
# 自动将标注图像发布到 /g1/vision/annotated_image 话题
# 如 X11 可用，同时弹出窗口显示

# 4. 查看标注图像（终端 4）
# 方式 A：rviz2（推荐，ROS2 自带）
sudo xhost +local:                              # 首次需授权 X11
DISPLAY=:0 rviz2
# 在 rviz2 中：Add → By topic → /g1/vision/annotated_image → Image

# 方式 B：rqt_image_view（轻量）
DISPLAY=:0 rqt_image_view                       # 下拉选择 /g1/vision/annotated_image

# 方式 C：确认数据是否在发布
ros2 topic hz /g1/vision/annotated_image

# 或查看原始检测结果文本
ros2 topic echo /g1/vision/detections
```

> **SSH 远程窗口提示**：如需在机器人桌面弹出 cv2/rviz2 窗口，先执行 `sudo xhost +local:` 解除 X11 认证限制，
> 可视化节点会自动检测 X11 并尝试弹窗，失败时自动降级为纯话题发布模式。

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

三个独立程序，可单独调试：

| 程序 | 功能 | 控制方式 | 入口命令 |
|------|------|---------|---------|
| `yaw_align` | 整机旋转让目标居中 | cmd_vel → Sport API | `ros2 run g1_yolo_nav_py yaw_align` |
| `waist_align` | 腰部旋转让目标居中 | Arm SDK DDS | `ros2 run g1_yolo_nav_py waist_align` |
| `loco_forward` | 对齐后前进到目标 | LocoClient RPC | `ros2 run g1_yolo_nav_py loco_forward` |

> **`yaw_align` vs `waist_align`**：
> - `yaw_align` 使用高层 `/cmd_vel` 控制整机旋转（angular.z），无需 DDS 依赖，更简单安全
> - `waist_align` 使用低层 DDS 直接控制腰部关节，精度更高但需要 `unitree_sdk2py`
> - **两者不能同时运行**，选择一种即可

**工作流程（两个节点协作）：**
```
  YOLO检测 ──→ yaw_align 或 waist_align ──→ loco_forward(Loco前进)
                目标居中后才允许前进             居中+稳定 → Move(vx)
```

**前置条件：**
- G1 机器人已连接并处于站立状态
- 相机已启动并发布图像
- `yaw_align` 无额外依赖；`waist_align` / `loco_forward` 需 `unitree_sdk2py`（已安装）

---

#### 📌 两种对齐方案对比调试

> 两种方案**互斥**，同一时间只能运行一种。建议先分别调试，再选定最优方案。

**方案 A：偏航对齐 (`yaw_align`)**
```
终端 1: ros2 launch realsense2_camera rs_launch.py \     # 相机
            camera_namespace:=robot1 camera_name:=D455_1
终端 2: ./run_yolo.sh                                   # YOLO 检测
终端 3: ros2 run g1_yolo_nav_py yaw_align               # 整机旋转对齐
终端 4: ros2 run g1_twist_bridge_py twist_bridge         # cmd_vel → Sport API
终端 5: ros2 run g1_yolo_nav_py detection_visualizer    # 可视化（可选）
```

**方案 B：腰部对齐 (`waist_align`)**
```
终端 1: ros2 launch realsense2_camera rs_launch.py \     # 相机
            camera_namespace:=robot1 camera_name:=D455_1
终端 2: ./run_yolo.sh                                   # YOLO 检测
终端 3: ros2 run g1_yolo_nav_py waist_align              # 腰部旋转对齐
终端 4: ros2 run g1_yolo_nav_py detection_visualizer    # 可视化（可选）
# 不需要 twist_bridge，直接 DDS 控制腰部关节
```

**查看可视化画面：**
```bash
# 方式 1：rviz2（推荐）
sudo xhost +local: && DISPLAY=:0 rviz2
# Add → By topic → /g1/vision/annotated_image → Image

# 方式 2：rqt_image_view
DISPLAY=:0 rqt_image_view  # 下拉选择 /g1/vision/annotated_image
```

**切换方式：**
```bash
# 方案 A → 方案 B：在终端 3 按 Ctrl+C 停止 yaw_align，然后启动 waist_align
ros2 run g1_yolo_nav_py waist_align

# 方案 B → 方案 A：在终端 3 按 Ctrl+C 停止 waist_align，然后启动 yaw_align
ros2 run g1_yolo_nav_py yaw_align
# 注意：yaw_align 需要额外运行 twist_bridge（终端 4）
```

**对比评估要点：**

| 评估项 | yaw_align (整机旋转) | waist_align (腰部旋转) |
|--------|---------------------|----------------------|
| 响应速度 | □ 快 / □ 慢 | □ 快 / □ 慢 |
| 稳定性（静止时不抖） | □ 稳定 / □ 抖动 | □ 稳定 / □ 抖动 |
| 追踪精度 | □ 精准 / □ 偏移 | □ 精准 / □ 偏移 |
| 脚部稳定性 | □ 站稳 / □ 晃动 | □ 站稳 / □ 晃动 |
| 依赖复杂度 | 低（仅需 twist_bridge） | 高（需 unitree_sdk2py + DDS） |
| 最终选择 | □ | □ |

> **调试技巧**：同一位姿下，先跑方案 A 记录表现，`Ctrl+C` 切换到方案 B 再记录，直接对比最直观。

---

#### 📌 步骤一（备选）：调试腰部对齐 (`waist_align`)

**目的**：确认腰部能正确追踪目标，让目标保持在画面中心。

```bash
# 终端 1：启动相机
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=robot1 camera_name:=D455_1

# 终端 2：编译 + 启动检测
source ~/g1act_venv/bin/activate
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

**前提**：步骤一的对齐已经正常工作。

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
| `center_tolerance` | 0.08 | 与对齐节点保持一致 |
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

对齐和前进都调通后，可以一起使用：

```bash
# 方式 A：手动分别启动（推荐调试，方便看各自日志）
# 终端 2: ros2 run g1_yolo_nav_py yolo_detector
# 终端 3: ros2 run g1_yolo_nav_py yaw_align
# 终端 4: ros2 run g1_twist_bridge_py twist_bridge
# 终端 5: ros2 run g1_yolo_nav_py loco_forward

# 方式 B：launch 一键启动
ros2 launch g1_yolo_nav_py yolo_nav.launch.py enable_waist_tracking:=true
```

**安全提示：**
- 首次联调时 `forward_speed` 建议 ≤ 0.2 m/s
- 随时准备按遥控器急停
- 确保前方无障碍物

### 🤖 抓取任务（一键全流程）

自动执行：目标检测 → 偏航对齐 → 前进接近 → 抓取 → 交互菜单。

**前置条件：**
- G1 机器人已连接并站立
- `unitree_sdk2py` 已安装在系统环境中
- `armup.py` / `armdown.py` 已就位（`src/g1_yolo_nav_py/arm/` 目录下，由同事维护）

**实机运行（一键启动）：**
```bash
source ~/g1act_venv/bin/activate   # 激活虚拟环境
cd ~/g1act_ws/manact_ws
colcon build --packages-select g1_yolo_nav_py && . install/setup.bash

# 一键启动全流程（相机 + 检测 + 桥接 + 抓取任务）
ros2 launch g1_yolo_nav_py grasp_task.launch.py

# 检测瓶子、降低速度
ros2 launch g1_yolo_nav_py grasp_task.launch.py \
    target_class:=bottle forward_speed:=0.15

# 相机已在其他地方启动时
ros2 launch g1_yolo_nav_py grasp_task.launch.py start_camera:=false
```

> **aarch64 自动处理**：launch 文件在 ARM 平台自动注入 `LD_PRELOAD`（解决 libgomp TLS 问题），x86 上则正常启动。

**手动分步启动（调试用）：**
```bash
# 终端 1：相机
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=robot1 camera_name:=D455_1 align_depth.enable:=true

# 终端 2：YOLO 检测
./run_yolo.sh

# 终端 3：twist_bridge
ros2 run g1_twist_bridge_py twist_bridge

# 终端 4：抓取任务
ros2 run g1_yolo_nav_py grasp_task
```

**执行流程与关键日志：**
```
==================================================
G1 抓取任务启动
目标类别: chair
armup: ~/g1act_ws/manact_ws/src/g1_yolo_nav_py/arm/armup.py
armdown: ~/g1act_ws/manact_ws/src/g1_yolo_nav_py/arm/armdown.py
==================================================
[LocoClient] 初始化成功
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
|------|------|------|
| 1 | 执行 `armdown.py` | 原地放下目标物 |
| 2 | 右转 90° + `armdown.py` | 转身后放下 |
| 3 | 输入 `x y z` 控制移动 | 如 `0.2 0.0 0.3`，输入 `q` 停止返回 |
| 4 | 安全退出 | 停止运动并退出程序 |

**参数调优：**
```bash
# 检测瓶子、降低速度
ros2 run g1_yolo_nav_py grasp_task --ros-args \
  -p target_class_id:=bottle \
  -p forward_speed:=0.15 \
  -p arrive_bbox_ratio:=0.5

# 自定义 arm 脚本目录
ros2 run g1_yolo_nav_py grasp_task --ros-args \
  -p arm_script_dir:=/home/unitree/my_arm_scripts
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `target_class_id` | chair | YOLO 检测目标类别 |
| `forward_speed` | 0.2 m/s | 接近速度（首次建议 0.15） |
| `arrive_bbox_ratio` | 0.45 | 检测框占比到达阈值 |
| `search_yaw_speed` | 0.3 rad/s | 搜索旋转速度 |
| `yaw_kp` | 2.0 | 偏航对齐 P 增益 |
| `lost_timeout` | 2.0 s | 目标丢失超时 |
| `arm_script_dir` | `~/g1act_ws/manact_ws/src/g1_yolo_nav_py/arm` | arm 脚本目录 |

> **安全提示**：首次运行建议 `forward_speed:=0.15`，随时准备急停。

---

![Fork 历史趋势图。](https://commit.cool/forks/1255027942/cloud/manact_ws?interval=day)
