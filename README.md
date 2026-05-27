[English](README.en.md) | 简体中文

<div align="center">
<h1>G1 NavGrasp</h1>

[![ROS2 Foxy](https://img.shields.io/badge/ROS2-Foxy-blue)](https://docs.ros.org/en/foxy/)
[![YOLOv11](https://img.shields.io/badge/YOLOv11-目标检测-green)](https://docs.ultralytics.com/)
[![纯视觉](https://img.shields.io/badge/纯视觉-路径规划-orange)](https://navigation.ros.org/)
[![Python 3.8](https://img.shields.io/badge/Python-3.8+-yellow)](https://www.python.org/)

<p align="center">
  <a href="https://github.com/HunLi-X">
    <img src="https://img.shields.io/badge/Auther--HunLi-ff69b4?logo=github&logoColor=white" alt="Auth" />
  </a>
  <a href="https://cnb.cool/u/xhunli">
    <img src="https://img.shields.io/badge/CNB-xhunli-F76945?logo=data:image/svg+xml;base64,PHN2ZyB2aWV3Qm94PSIwIDAgMzIwIDMyMCIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4NCjxwYXRoIGQ9Ik0yMjguOTA2IDQwLjI0MTJDMjI5Ljg4MiAzNy41MTA4IDIyOC45MDYgMzQuMzkwMyAyMjYuNzU5IDMyLjQ0QzIxOS4zNDIgMjYuMDA0IDIwMC43OTkgMTIuMzUxOSAxNzMuMDgyIDEwLjQwMTZDMTQxLjg1MiA4LjA2MTIxIDEyMi41MjggMTYuNDQ3NSAxMTIuNzY5IDIyLjY4ODVDMTA4LjQ3NCAyNS40MTg5IDEwOC4yNzkgMzEuNDY0OSAxMTIuMTgzIDM0LjM5MDNMMTkxLjYyNSA5Ni4yMTQ5QzE5OC42NTIgMTAxLjY3NiAyMDguOTk3IDk4LjU1NTMgMjExLjcyOSA5MC4xNjlMMjI4LjcxMSA0MC4yNDEySDIyOC45MDZaIiBmaWxsPSIjRkY2MjAwIi8+DQo8cGF0aCBkPSJNMzIuOTM4MSAyMjMuNTY0QzI5LjYxOTkgMjI1LjcxIDI4LjI1MzYgMjI5LjgwNSAyOS4yMjk1IDIzMy41MTFDMzIuMTU3MyAyNDQuNDMyIDQxLjMzMTIgMjY2Ljg2MSA2Ni45MDA5IDI4Ny41MzRDOTIuNDcwNiAzMDguMDEyIDEyMi43MjUgMzEwLjM1MyAxMzUuNjA3IDMwOS45NjNDMTM5LjUxMSAzMDkuOTYzIDE0Mi44MjkgMzA3LjQyNyAxNDQgMzAzLjcyMkwxOTQuOTQ1IDE0Mi42MjdDMTk4LjY1MyAxMzAuOTI1IDE4NS41NzYgMTIxLjE3MyAxNzUuNDI2IDEyNy45OTlMMzIuOTM4MSAyMjMuNTY0WiIgZmlsbD0iI0ZGNjIwMCIvPg0KPHBhdGggZD0iTTcwLjIxNjkgNTMuNDk1NUM2Ny42Nzk0IDUyLjUyMDMgNjQuOTQ2OCA1Mi43MTUzIDYyLjYwNDUgNTMuODg1NUM1My4yMzU1IDU4Ljk1NjMgMjkuMDMyIDc0Ljc1MzggMTYuNTQgMTA3LjMyNEM2Ljc4MDU0IDEzMi4yODggMTAuMDk4NyAxNTkuOTgyIDEyLjgzMTQgMTczLjQzOUMxMy42MTIxIDE3Ny45MjUgMTguMjk2NyAxODAuNDYgMjIuNTkwOCAxNzguNzA1TDE3NS40MjQgMTE5LjAyNkMxODYuMzU0IDExNC43MzUgMTg2LjM1NCA5OS4zMjc2IDE3NS40MjQgOTUuMDM2OUw3MC4yMTY5IDUzLjQ5NTVaIiBmaWxsPSIjRkY2MjAwIi8+DQo8cGF0aCBkPSJNMjk3LjAzIDE2OC45NjhDMzAxLjUxOSAxNzEuODkzIDMwNy41NyAxNjkuMzU4IDMwOC4zNTEgMTY0LjA5MkMzMTAuMzAxIDE1MC4wNSAzMTIuMDYgMTI1Ljg2NiAzMDQuMDU3IDEwNy4zMzZDMjkzLjMyMSA4Mi45NTkxIDI3NC45NzQgNjcuNzQ2OCAyNjYuMTkgNjEuNzAwOEMyNjMuNDU4IDU5Ljc1MDUgMjU5Ljc0OSA1OS45NDU2IDI1Ny4yMTIgNjIuMjg1OUwyMTguNTY0IDk2LjQxNjJDMjEyLjMxOCAxMDIuMDcyIDIxMi45MDQgMTEyLjAxOSAyMTkuOTMxIDExNi42OTlMMjk3LjAzIDE2OC45NjhaIiBmaWxsPSIjRkY2MjAwIi8+DQo8cGF0aCBkPSJNMTg5LjA4OSAyOTkuNDI4QzE4OC42OTkgMzAzLjkxNCAxOTIuNjAzIDMwNy44MTQgMTk3LjA5MiAzMDcuMjI5QzIxMS43MzEgMzA1LjY2OSAyNDEuNzkgMjk5LjgxOCAyNjQuMjM3IDI3OC4zNjVDMjg2LjA5OCAyNTcuNDk2IDI5My4zMiAyMzIuNzI4IDI5NS4yNzIgMjIyLjc4MUMyOTUuODU4IDIyMC4wNTEgMjk1LjI3MiAyMTcuMzIgMjkzLjUxNSAyMTUuMTc1TDIyNS45OCAxMzEuODk3QzIxOC43NTggMTIyLjkyNSAyMDQuMTE5IDEyNy40MTEgMjAzLjE0MyAxMzguOTE4TDE4OS4wODkgMjk5LjIzM1YyOTkuNDI4WiIgZmlsbD0iI0ZGNjIwMCIvPg0KPC9zdmc+DQo=&logoColor=white" alt="CNB" />
  </a>
  <a href="https://hunli.100w.top/">
    <img src="https://img.shields.io/badge/Blog-昏黎站-008080?logo=googlechrome&logoColor=white" alt="Blog" />
  </a>
</p>
<p align="center">
  <img src="https://ziadoua.github.io/m3-Markdown-Badges/badges/Python/python1.svg" />&nbsp;
  <img src="https://ziadoua.github.io/m3-Markdown-Badges/badges/Flask/flask1.svg" />&nbsp;
  <img src="https://ziadoua.github.io/m3-Markdown-Badges/badges/JSON/json1.svg" />&nbsp;
  <img src="https://ziadoua.github.io/m3-Markdown-Badges/badges/Linux/linux1.svg" />&nbsp;
  <img src="https://ziadoua.github.io/m3-Markdown-Badges/badges/Ubuntu/ubuntu1.svg" />&nbsp;
  <img src="https://ziadoua.github.io/m3-Markdown-Badges/badges/VisualStudioCode/visualstudiocode1.svg" />
</p>
<p>让 G1 看见世界，走向目标，伸出双手。</p>
</div><br>

## 项目介绍

宇树 G1 人形机器人 **YOLO 目标识别与路径规划导航** 工作空间。基于 ROS2 Foxy + Python，实现实时目标检测、3D 空间定位与自主导航接近目标。

**技术栈**：Ubuntu 20.04 · ROS2 Foxy · Python 3.8+ · YOLOv11 · Nav2 · colcon

**核心能力**：

- YOLO 实时目标检测（支持自定义训练模型）
- 2D 检测结果到 3D 空间坐标投影
- Nav2 路径规划与自主导航
- 紧急停止与安全限速保护
- **Web 控制面板**：浏览器/手机/平板远程控制全套功能（推荐）

---

## ⚡ 快速启动（Web 控制面板，推荐）

**一条命令启动 Web 面板，所有功能在浏览器里点几下即可：**

```bash
# 一次性环境（首次）
python3 -m venv ~/g1act_venv --system-site-packages
source ~/g1act_venv/bin/activate
cd ~/g1act_ws
pip install -r requirements.txt
colcon build --packages-select g1_yolo_nav_py
. install/setup.bash

# 每次启动
source ~/g1act_venv/bin/activate
cd ~/g1act_ws && . install/setup.bash
ros2 run g1_yolo_nav_py web_panel
```

打开浏览器访问 `http://<机器人IP>:8080`（同局域网手机/平板/电脑均可）。

### Web 面板功能模块

| 页面 | 功能 |
|---|---|
| 🎯 **目标抓取** | 完整抓取流水线：搜索 → 对齐 → 接近 → 抓取 → 放下。支持「一键执行」全自动流水线（YOLO检测 → Yaw对齐 → 前进靠近 → armup抓取 → 等待放下）|
| 🔍 **目标识别** | 大屏沉浸式 YOLO 检测视频流 + 实时检测信息 |
| 🕹️ **运动控制** | 手动遥控 D-Pad（前进/后退/左转/右转/急停）+ 速度滑块 |
| 📊 **系统状态** | 仪表盘（FPS/检测数/距离/u 位置/健康度环）+ 实时日志 |
| 📦 **节点管理** | 一键启停 RealSense 相机 / YOLO 检测器 / RGBD 采集，内联参数编辑 |
| ⚙️ **系统设置** | 19 个运行时参数热更新（无需重启）+ 动态背景 + 界面偏好 |

**优势：**
- 🌐 跨设备 — 手机、平板、电脑浏览器都能访问，多人同时观察
- 🚫 零终端 — 不用 ssh 开 4 个终端，一个面板搞定所有节点启动
- 🎨 现代界面 — 液态玻璃风设计 + 实时状态可视化
- 🔌 SSH 友好 — 不依赖 X11，纯 Web，机器人无显示器也能用

### 本地开发预览（无需 ROS2）

想改前端 UI 但不想登机器人？

```bash
cd src/web_frontend
pip install flask pillow
python dev_server.py   # 浏览器 http://localhost:8080
```

`dev_server.py` 是纯 Flask mock 后端，模拟所有 API 和 MJPEG 视频流，改 HTML/CSS/JS 后刷新浏览器即时生效。

---

### 项目结构

```
g1act_ws/
├── README.md                  # 项目说明（中文）
├── README.en.md              # 项目说明（英文）
├── requirements.txt           # Python 依赖
├── src/
│   ├── web_frontend/         # Web 前端（HTML/CSS/JS + dev server）
│   │   ├── index.html            # 主页面（7 个视图）
│   │   ├── css/style.css         # 样式表
│   │   ├── js/app.js             # 前端逻辑
│   │   └── dev_server.py         # Mock 后端（无 ROS2 依赖）
│   ├── g1_yolo_nav_py/       # 主 ROS2 包
│   │   ├── setup.py              # 包设置
│   │   ├── config/
│   │   │   └── arm_poses.json    # 手臂姿态配置文件
│   │   ├── launch/               # ROS2 启动文件
│   │   │   ├── grasp_task.launch.py
│   │   │   └── yolo_nav.launch.py
│   │   ├── arm/                  # 手臂控制（独立子进程，DDS 隔离）
│   │   │   ├── arm_common.py       # 关节常量 + 基类
│   │   │   ├── arm_debug.py        # 上肢调试子进程（stdin JSON 协议）
│   │   │   ├── armup.py            # 抓取动作
│   │   │   └── armdown.py          # 放下动作
│   │   └── g1_yolo_nav_py/      # Python 节点模块
│   │       ├── web_panel.py        # Web 面板节点 ★ 推荐入口
│   │       ├── control_panel.py    # tkinter 控制面板（旧版）
│   │       ├── yolo_detector.py    # YOLO 检测
│   │       ├── yaw_align.py        # 步进式偏航对齐
│   │       ├── loco_forward.py     # 前进 + 深度停止
│   │       ├── grasp_task.py       # 终端抓取任务
│   │       ├── rgbd_capture.py     # RGBD 数据采集
│   │       ├── spatial_target.py   # 2D→3D 空间投影
│   │       ├── _grasp_state.py     # 抓取状态机 Mixin
│   │       └── _dds_compat.py      # DDS 兼容层
│   └── base/                  # 参考包
│       ├── ctrl_keyboard/       # Loco API 参考
│       └── g1_description/     # URDF 模型
```
#### 更新代码
```bash
cd ~/g1act_ws/manact_ws
git pull
colcon build
```
#### 每次启动
```bash
cd ~/g1act_ws/manact_ws
source ~/g1act_venv/bin/activate
. install/setup.bash
```

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
# 每步旋转更大 — 增大步进速度或持续时间
ros2 run g1_yolo_nav_py yaw_align --ros-args \
  -p step_yaw_speed:=0.5 -p step_duration:=0.4

# 相机更新太慢 — 增加等待时间
ros2 run g1_yolo_nav_py yaw_align --ros-args -p camera_settle_time:=8.0

# 对齐精度不够 — 缩小居中容差
ros2 run g1_yolo_nav_py yaw_align --ros-args -p center_tolerance:=0.05
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `step_yaw_speed` | 0.3 rad/s | 每步旋转速度 |
| `step_duration` | 0.3 s | 每步旋转持续时间 |
| `camera_settle_time` | 5.0 s | 旋转后等待相机更新 |
| `max_consecutive_steps` | 10 | 单次最大连续步数 |
| `center_tolerance` | 0.08 | 居中容差（归一化），越小越严格 |
| `lost_timeout` | 10.0 s | 目标丢失超时 |

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
- **深度距离 ≤ 0.5m 时**，日志显示 `到达目标! 深度距离=0.xxm <= 0.50m` 并停止
- 深度不可用时回退到检测框占比判断（bbox ≥ 45% 停止）
- 目标丢失或偏离中心时自动停止前进

**前进参数调优：**

```bash
# 首次调试用低速度确保安全
ros2 run g1_yolo_nav_py loco_forward --ros-args -p forward_speed:=0.15

# 调整停止距离（离目标更近才停下）
ros2 run g1_yolo_nav_py loco_forward --ros-args -p stop_distance:=0.3

# 关闭深度距离判断，只用 bbox 占比
ros2 run g1_yolo_nav_py loco_forward --ros-args -p use_depth_distance:=false
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `forward_speed` | 0.3 m/s | 前进速度（首次建议 0.15） |
| `use_depth_distance` | true | 是否使用深度距离判断停止 |
| `stop_distance` | 0.5 m | 深度距离停止阈值 |
| `depth_topic` | `/D455_1/depth/image_rect_raw` | 深度图话题 |
| `depth_sample_radius` | 5 px | 深度采样半径 |
| `align_stable_time` | 0.8 s | 居中多久才开始前进 |
| `arrive_bbox_ratio` | 0.45 | 检测框占比到达阈值（深度 fallback） |
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

## Web 控制面板（推荐）

基于 Flask 的浏览器控制面板，**单进程启动整套机器人能力**，无需多个终端开 ssh。

```bash
# 启动（首次需 pip install flask pillow）
ros2 run g1_yolo_nav_py web_panel
# 浏览器打开 http://<机器人IP>:8080
```

### 节点管理

进入「📦 节点管理」页面，**3 张大卡分别管理**：

| 进程 | 命令模式 | 用途 |
|---|---|---|
| 📷 RealSense 相机驱动 | `ros2 launch realsense2_camera rs_launch.py` | 视觉前置 |
| 🔍 YOLO 目标检测器 | `ros2 run g1_yolo_nav_py yolo_detector` | 目标识别 |
| 📦 RGBD 数据采集 | `ros2 run g1_yolo_nav_py rgbd_capture` | 定时存彩色+深度图 |

每张卡都有：
- **启动 / 停止按钮** — subprocess.Popen + 独立 process group，停止时 SIGINT → SIGKILL 整树清理
- **状态胶囊** — 运行中绿色脉冲 + PID + 运行时长
- **内联参数表单** — 直接改 `camera_namespace` / `image_topic` / `interval_sec` 等参数，点保存写入后端
- **子进程日志** — 折叠展开看最近 80 行 ros2 stdout

### 完整 API

```
GET  /                          → 主页
GET  /stream/raw                → MJPEG 原始视频流
GET  /stream/detection          → MJPEG 检测标注流
GET  /api/state                 → 全局状态轮询
GET  /api/config                → 读取热更新配置
POST /api/config                → 批量热更新
POST /api/cmd/manual            → 手动遥控
POST /api/cmd/stop              → 急停
POST /api/cmd/search            → 搜索
POST /api/cmd/grab              → 抓取
POST /api/cmd/putdown           → 放下
POST /api/cmd/turn_putdown      → 右转 90° 放下
POST /api/cmd/left_putdown      → 左移放下
POST /api/cmd/auto_execute      → 一键执行全自动流水线
POST /api/cmd/auto_stop         → 停止一键执行流水线
GET  /api/cmd/auto_status       → 流水线状态
POST /api/arm_debug/start       → 启动上肢调试子进程
POST /api/arm_debug/stop        → 停止上肢调试
POST /api/arm_debug/send        → 发送目标角度
GET  /api/arm_debug/status      → 调试进程状态
GET  /api/arm_debug/presets     → 获取预设姿态
GET  /api/arm_poses             → 读取姿态配置
POST /api/arm_poses             → 保存姿态配置
POST /api/arm_poses/run/<name>  → 执行 armup/armdown 序列
GET  /api/arm_poses/run/status  → 序列执行状态
POST /api/process/<n>/start     → 启动子进程
POST /api/process/<n>/stop      → 停止子进程
GET  /api/process/<n>/status    → 进程状态 + 日志
POST /api/process/<n>/params    → 更新进程参数
```

### 系统设置（4 个分组）

浏览器里直接调，不用改 yaml：

| 分组 | 参数 |
|---|---|
| 检测与对齐 | target_class_id / use_depth_distance / stop_distance / depth_sample_radius / lost_timeout / step_yaw_speed / step_duration / camera_settle_time / max_consecutive_steps / center_tolerance |
| 运动与相机 | forward_speed / arrive_bbox_ratio / align_stable_time / search_yaw_speed / turn_yaw_speed / turn_duration / side_step_speed / side_step_duration / camera_namespace / camera_name |
| 系统环境 | network_interface / cyclonedds_home / sdk_python_path / arm_script_dir |
| 偏好与背景 | stream_quality / default_view / log_toast / poll_interval + 背景类型/蒙版/模糊 |

### 自定义参数（启动时固定）

```bash
# 自定义端口
ros2 run g1_yolo_nav_py web_panel --ros-args -p http_port:=9090

# 自定义目标类别 + 视频流质量
ros2 run g1_yolo_nav_py web_panel --ros-args \
  -p target_class_id:=bottle \
  -p stream_quality:=50

# 改图像话题（如果命名空间不同）
ros2 run g1_yolo_nav_py web_panel --ros-args \
  -p image_topic:=/camera/color/image_raw
```

> **依赖：** `pip install flask pillow`（首次安装）

---

## tkinter 控制面板（旧版本，可选）

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
                ALIGNING（步进式偏航对齐）
                  ↓ 居中稳定
                APPROACHING（Loco API 前进）
                  ↓ 深度距离 ≤ 0.5m（或 bbox 占比 ≥ 0.45）
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
| `step_yaw_speed` | `0.3` | 步进式对齐旋转速度 rad/s |
| `step_duration` | `0.3` | 步进式对齐持续时间 s |
| `camera_settle_time` | `5.0` | 等待相机更新 s |
| `use_depth_distance` | `true` | 深度距离判断停止 |
| `stop_distance` | `0.5` | 停止距离 m |
| `arrive_bbox_ratio` | `0.45` | 到达判断阈值（深度 fallback） |
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
[对齐] 一步: u=0.320, 误差=-0.180, 旋转≈+5.2°, 等待5.0s...
[对齐] 等待结束，重新检测: u=0.480
[状态] ALIGNING → APPROACHING: 目标已居中
[状态] APPROACHING → GRABBING: 深度距离=0.48m <= 0.50m
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
| `use_depth_distance` | true | 使用深度距离判断停止并抓取 |
| `stop_distance` | 0.5 m | 深度距离停止阈值 |
| `depth_topic` | `/D455_1/depth/image_rect_raw` | 深度图话题 |
| `depth_sample_radius` | 5 px | 深度采样半径 |
| `arrive_bbox_ratio` | 0.45 | 检测框占比到达阈值（深度 fallback） |
| `search_yaw_speed` | 0.3 rad/s | 搜索旋转速度 |
| `step_yaw_speed` | 0.3 rad/s | 步进式对齐旋转速度 |
| `step_duration` | 0.3 s | 步进式对齐持续时间 |
| `camera_settle_time` | 5.0 s | 等待相机更新 |
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
SET_FSM_ID(DAMP=1) → SET_FSM_ID(STAND_UP=4) → SET_FSM_ID(START=500) → SET_FSM_ID(WALK_RUN=801) → SET_BALANCE_MODE(CONTINUOUS_GAIT=1)
```

WALK_RUN + CONTINUOUS_GAIT 模式下 `SET_VELOCITY(7105)` 才生效。

> **注意**：运动控制方案参考 `ctrl_keyboard` 已验证可用的实现，与 `ctrl_keyboard` 使用完全相同的 API 体系。

---

## 调试指南

### 1. 检查话题连通性

```bash
# 检查相机是否发布图像
ros2 topic hz /D455_1/color/image_raw

# 检查深度图是否发布
ros2 topic hz /D455_1/depth/image_rect_raw

# 检查 YOLO 检测结果
ros2 topic echo /g1/vision/detections --once

# 检查运动控制话题订阅者
ros2 topic info /api/sport/request
```

### 2. 相机启动（含深度）

```bash
# 必须启用深度对齐
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=robot1 camera_name:=D455_1 align_depth.enable:=true
```

> **关键**：深度距离判断依赖深度图话题，启动相机时必须加 `align_depth.enable:=true`。

### 3. 验证深度距离

```bash
# 单独测试深度图是否正常（应输出 16UC1 或 32FC1 编码）
ros2 topic echo /D455_1/depth/image_rect_raw --once | head -5

# 用 loco_forward 观察距离日志
ros2 run g1_yolo_nav_py loco_forward --ros-args -p forward_speed:=0.0
# forward_speed:=0.0 使机器人不移动，仅观察深度距离日志
```

### 4. 关闭深度距离（仅用 bbox）

```bash
# 如果深度相机不可用，可关闭深度距离判断
ros2 run g1_yolo_nav_py grasp_task --ros-args -p use_depth_distance:=false
```

### 5. 常见问题

| 现象 | 原因 | 解决 |
| --- | --- | --- |
| 深度距离始终为 None | 深度图话题无数据 | 检查 `align_depth.enable:=true` |
| 距离值异常大（>10m） | 16UC1 编码未正确转换 | 检查深度图 encoding 是否为 `16UC1` |
| 到达后不执行 arm | armup.py 路径错误 | 检查 `arm_script_dir` 参数 |
| 机器人不运动 | FSM 未进入 WALK_RUN | 检查日志 `[FSM]` 行，确认 START(500) 步骤 |
| 目标丢失频繁 | `lost_timeout` 太短 | 增大 `lost_timeout`，如 `2.0` |
| 对齐过冲 | `camera_settle_time` 太短 | 增大到 `8.0` 等待相机更新 |

### 6. 分步调试流程

```
Step 1: 相机 → 确认 /D455_1/color/image_raw 有数据
Step 2: YOLO → 确认 /g1/vision/detections 有检测结果
Step 3: 深度图 → 确认 /D455_1/depth/image_rect_raw 有数据
Step 4: yaw_align → 确认步进式对齐正常（观察日志）
Step 5: loco_forward → forward_speed:=0.0 观察深度距离
Step 6: loco_forward → 正常速度前进，确认到达停止
Step 7: grasp_task → 完整流程测试
```

---

![Fork 历史趋势图。](https://commit.cool/forks/1255027942/cloud/manact_ws?interval=day)
