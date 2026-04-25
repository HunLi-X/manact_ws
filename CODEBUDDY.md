# CODEBUDDY.md This file provides guidance to CodeBuddy when working with code in this repository.

## Project Overview

ROS2 Foxy colcon workspace for **Unitree G1 humanoid robot** — YOLO target detection, 3D spatial projection, and Nav2 path planning navigation.

**Target platform**: Ubuntu 20.04 / ROS2 Foxy / Python 3.8+ / colcon

## Build & Run Commands

```bash
# Build all packages
cd g1act_ws && colcon build

# Build specific package
colcon build --packages-select g1_yolo_nav_py
colcon build --packages-select g1_driver_py

# Source environment after build
. install/setup.bash

# Launch YOLO detection + navigation pipeline
ros2 launch g1_yolo_nav_py yolo_nav.launch.py

# Launch with Nav2 and depth sensor
ros2 launch g1_yolo_nav_py yolo_nav.launch.py use_nav2:=true use_depth_sensor:=true

# Launch G1 driver + RViz
ros2 launch g1_driver_py driver.launch.py

# Keyboard teleop (requires G1 connected)
ros2 run g1_teleop_ctrl_keyboard g1_teleop_keyboard

# YOLO detection + yaw align + forward approach
ros2 run g1_yolo_nav_py yolo_detector
ros2 run g1_yolo_nav_py yaw_align
ros2 run g1_yolo_nav_py loco_forward --ros-args -p forward_speed:=0.15

# One-click grasp task
ros2 launch g1_yolo_nav_py grasp_task.launch.py

# GUI control panel
ros2 run g1_yolo_nav_py control_panel

# Run arm control script (standalone, not a ROS2 package)
cd g1act_ws && python3 src/arm.py [network_interface]
```

Python dependency: `pip3 install ultralytics`

## Architecture

### Package Structure

```
src/
├── base/                          # G1 robot base packages
│   ├── g1_description/            # URDF/MJCF models (12dof, 23dof, 29dof variants)
│   ├── g1_driver_py/              # ROS2 driver: odom, TF, joint_states
│   ├── g1_teleop_ctrl_keyboard/   # Keyboard teleop via Sport API (MOVE=1008)
│   ├── g1_twist_bridge_py/        # Twist → Sport API Request bridge
│   └── ctrl_keyboard/             # Reference: auto_ctrl.py (Loco API, deprecated)
├── g1_yolo_nav_py/                # YOLO detection + navigation + motion control
│   ├── g1_yolo_nav_py/
│   │   ├── sport_client.py        # ** 统一运动控制模块 (纯 Sport API) **
│   │   ├── yolo_detector.py       # YOLO 目标检测节点
│   │   ├── spatial_target.py      # 2D→3D 空间投影节点
│   │   ├── yaw_align.py           # 偏航对齐节点
│   │   ├── loco_forward.py        # 前进控制节点
│   │   ├── grasp_task.py          # 抓取任务主控（搜索→对齐→前进→抓取→菜单）
│   │   ├── control_panel.py       # tkinter GUI 控制面板
│   │   └── detection_visualizer.py # 检测可视化节点
│   ├── arm/                       # arm SDK 脚本 (unitree_sdk2py)
│   ├── config/yolo_nav.yaml       # 节点参数配置
│   └── launch/                    # Launch 文件
└── arm.py                         # Standalone arm SDK script (unitree_sdk2py, not ROS2)
```

### Data Flow

```
Camera Image (/D455_1/color/image_raw)
    → [g1_yolo_detector_node] → Detection2DArray (/g1/vision/detections)
                                      ↓
                              [g1_yaw_align_node] → SportClient.move()
                                      ↓               → /api/sport/request
                              [g1_loco_forward_node] → SportClient.move()
                                      ↓
                                 G1 Hardware
```

抓取任务全流程（grasp_task / control_panel）：
```
Camera → YOLO Detector → Detection2DArray
                                  ↓
                    [grasp_task_node / control_panel]
                         ↓ 使用 SportClient 统一控制（纯 Sport API）
                    SEARCHING → ALIGNING → APPROACHING → GRABBING → MENU
                         ↓           ↓           ↓
                    旋转搜索    P控制偏航    MOVE前进
                         ↓           ↓           ↓
                    /api/sport/request (Sport API: MOVE 1008, STOPMOVE 1003)
```

### Motion Control Architecture

所有运动控制通过 **`SportClient`** (`sport_client.py`) 统一封装，**全部使用 Sport API**，不使用任何 Loco API。

#### SportClient 功能

| 方法 | API | 说明 |
|---|---|---|
| `init_fsm()` | `DAMP(101)` + `STANDUP(1004)` + `BALANCESTAND(1002)` + `CONTINUOUSGAIT(1019)` | 后台线程初始化: DAMP → STANDUP → BALANCESTAND → CONTINUOUSGAIT |
| `move(vx, vy, vyaw)` | `MOVE(1008)` | 运动控制（参数 {x, y, z}，与 teleop_keyboard 一致） |
| `stop()` | `STOPMOVE(1003)` | 停止运动 |
| `balance_stand()` | `BALANCESTAND(1002)` | 平衡站立 |
| `sit()` | `SIT(1009)` | 坐下 |
| `stand_up()` | `STANDUP(1004)` | 站立 |
| `stand_down()` | `STANDDOWN(1005)` | 趴下 |
| `rise_sit()` | `RISESIT(1010)` | 从坐姿恢复站立 |
| `continuous_gait()` | `CONTINUOUSGAIT(1019)` | 开启连续步态 |
| `damp()` | `DAMP(101)` | 阻尼模式 |
| `publish(api_id, params)` | 任意 API | 每次创建新 Request 对象 |

#### FSM 状态机

运动控制节点启动时通过 `init_fsm()` 自动初始化状态机：
```
DAMP(101) → STANDUP(1004) → BALANCESTAND(1002) → CONTINUOUSGAIT(1019)
```
BALANCESTAND + CONTINUOUSGAIT 模式下 `MOVE(1008)` 才生效。

#### API 体系（纯 Sport API）

| API | ID | 用途 | 参数格式 |
|---|---|---|---|
| **DAMP** | 101 | 阻尼模式 | 无 |
| **BALANCESTAND** | 1002 | 平衡站立 | 无 |
| **STOPMOVE** | 1003 | 停止运动 | 无 |
| **STANDUP** | 1004 | 站立 | 无 |
| **STANDDOWN** | 1005 | 趴下 | 无 |
| **MOVE** | 1008 | 运动控制 | `{"x": vx, "y": vy, "z": vyaw}` |
| **SIT** | 1009 | 坐下 | 无 |
| **RISESIT** | 1010 | 从坐姿恢复 | 无 |
| **CONTINUOUSGAIT** | 1019 | 连续步态 | 无 |

**不再使用的 Loco API（7xxx 系列）**：SET_FSM_ID(7101)、SET_BALANCE_MODE(7102)、SET_VELOCITY(7105)。

### Two Control Interfaces

1. **unitree_api (high-level, via SportClient)**: All motion control nodes use `SportClient` to publish `unitree_api/msg/Request` to `/api/sport/request`. `SportClient` wraps only Sport API — MOVE(1008), STOPMOVE(1003), SIT(1009), BALANCESTAND(1002), etc. This is the same API system used by `g1_teleop_ctrl_keyboard`.
2. **unitree_sdk2py (low-level)**: `src/arm.py` and `arm/*.py` use `ChannelPublisher("rt/arm_sdk", LowCmd_)` for direct joint-level arm control. This bypasses ROS2 entirely and communicates via DDS.

### G1 Joint Index Mapping (from arm.py)

```
0-5:   Left leg  (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
6-11:  Right leg
12-14: Waist (yaw, roll, pitch — roll/pitch INVALID on 29dof locked-waist)
15-21: Left arm  (shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw)
22-28: Right arm
29:    Weight (kNotUsedJoint — used as arm_sdk enable flag: 1=enable, 0=disable)
```

### URDF Joint Names (from g1_driver_py/driver.py)

Joint names follow `snake_case`: `left_hip_pitch_joint`, `right_shoulder_yaw_joint`, `waist_yaw_joint`, etc. The default URDF loaded is `g1_29dof.urdf`.

### Key ROS2 Topics

| Topic | Type | Description |
|---|---|---|
| `/api/sport/request` | `unitree_api/Request` | All motion commands (via SportClient, Sport API only) |
| `/cmd_vel` | `geometry_msgs/Twist` | Legacy velocity (bridged by twist_bridge) |
| `/g1/sensor/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/joint_states` | `sensor_msgs/JointState` | 29 joint positions |
| `/g1/vision/detections` | `vision_msgs/Detection2DArray` | YOLO 2D detections |
| `/g1/nav/target_pose` | `geometry_msgs/PoseStamped` | Navigation target (odom frame) |

### TF Tree Requirement

The navigation pipeline requires: `odom → base_link → camera_color_optical_frame`. The driver node publishes `odom → base_link` TF; camera TF depends on the camera driver being active.

### Configuration

All node parameters are declared via `declare_parameter()` then read with `get_parameter()`. YAML config file: `g1_yolo_nav_py/config/yolo_nav.yaml`. Launch parameters override YAML values.

### Coding Conventions

- Nodes inherit `rclpy.node.Node`, not `rclpy.create_node()`
- All parameters **must** be declared before reading (`declare_parameter` then `get_parameter`)
- No magic numbers — use parameters or named constants
- Explicit QoS with depth (never omit `depth`)
- Topic naming: `/g1/<module>/<signal>`
- Package naming: `g1_xxx_py`
- Node naming: `snake_case`
- **Motion control**: Always use `SportClient` from `sport_client.py`, never construct `Request` directly or use `cmd_vel`/`Twist` for motion
- **Sport API only**: 不使用 Loco API (7xxx)，全部使用 Sport API (1xxx)。运动用 MOVE(1008)，停止用 STOPMOVE(1003)，姿态用 SIT(1009)/STANDUP(1004) 等
- **Request publishing**: `SportClient.publish()` creates a new `Request()` each time (avoid DDS buffer reuse bugs)
- **MOVE parameter format**: `{"x": vx, "y": vy, "z": vyaw}` — 与 g1_teleop_ctrl_keyboard 完全一致
- **P controller sign**: `vyaw = -kp * error * fov` (negative sign: target on right → robot turns right → vyaw negative)
- Safety: velocity clamping, low default speeds, `auto_stand` parameter for FSM initialization
