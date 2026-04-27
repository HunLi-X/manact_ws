English | [简体中文](README.md)

<div align="center">
<h1>G1 NavGrasp</h1>

[![ROS2 Foxy](https://img.shields.io/badge/ROS2-Foxy-blue)](https://docs.ros.org/en/foxy/)
[![YOLOv11](https://img.shields.io/badge/YOLOv11-Object%20Detection-green)](https://docs.ultralytics.com/)
[![Vision-Only](https://img.shields.io/badge/Vision-Only-Path%20Planning-orange)](https://navigation.ros.org/)
[![Python 3.8](https://img.shields.io/badge/Python-3.8+-yellow)](https://www.python.org/)

[![Auth](https://img.shields.io/badge/Auther--HunLi-ff69b4.svg?logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAADIAAAAyCAYAAAAeP4ixAAAACXBIWXMAAAsTAAALEwEAmpwYAAADZElEQVR4nO2ZX2iPURjHP/7/aZN/E21DaZvtwoVYyQUuGXLB/LtkLRcUhSJMSVwg3KCUJPJvLmRZtMQFLvwZhUJk/saGLWaYV6eet06n9/3tfd+9531/sW89td9z3vOc8z3nPOc8zzPoQQ+yAnlAA/AbcCzKR2ClTSL7LRNwNPkB5Noi0pggEQeYbovIh4SJLLBBojfwK2EiVTaIjEyYhANsskGkNAUi+2wQmZECkRM2iCxMgUi9DSKrxPhS7GOZjHXPhvGtYlwdMduYKWO9tmH8oBgvxj6KZawOoFfcxk+L8RzsI0fzkyFhO28EvqXg0FHlK1DtReRzFkzOCSmfvIhczIKJOSHlmheRgcAK4E4WTNDpQp4Ba4P4zxRgO3ArgSQqqDQDx4AKCWBDYwSwRDP4JWECG4CpQB9iwCTNsBsJl8srvxk4CtQBN4Enkre0GKQ7RaekCXgAXAcuAHuB1bLaZcAgrd+2uK78ai0PsfLSBshG70u9IDJmyUq2AYvlbD5N4Dg9l/EnArdFV9sdIlfEiLrNzFAlqPwE2kP2OaKNlyfH8Q9QFJVIixgepukqIuTdRRI3Be1TaczjnOgXRSXyTgwUarrBIVb4htbvcMA+nR7+UCdtc6MSOS8G9hj6+gjlnHEBd+WuMVaZ9FMyJiqRydrgB4B80VcFmFBtxALfFm3nlwPv48rfK7WoWDkc8jgpYq0eE/kOnAGG+4RBh3z6qTGOyzfIJeG2nQL6EQMmaEaTgtNdv+jKcFJwbI33XxEpAM76+IArrXIbFqVFpE0MuzeXF4nmEI9ecwZbhVo6GzsaxLhnniw74YQUVdDwwjppv2yzaPbIJ7HJdJz8RIX5JgYAL6V9vg0ifYEXMsAaj3Z3cpfkmPmhQAs5vHxgh+gbbdS0XMyTQdrkbdHhTiwTCdMHTCLl8hCqeGsalnFSO2JDNb3XxOZINqhkttFmfq8W4I1PbGcFuVr21qCloe7EarRvmzT9K01fYxAZJVmgA1yNKxwh4NF4q0Wr+UZs5JIxndskoVLoEskI1e/HRu6TCEpklR1ZebPcqk/YT9cu1UL190NgNCmhQGpfTjelLo2dMNEf2BkylXVF7eL6qAU3WyiVvCHIv7A7pBY2nizGWGC3FOr0XeoQZ96VIcbqAf8K/gLNGaTJ3vwbFgAAAABJRU5ErkJggg==)](https://github.com/HunLi-X)
[![cnb](https://img.shields.io/badge/CNB-xhunli-F76945?logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAAAXNSR0IArs4c6QAAAppJREFUOE9tk01rE1EUhs+5H5lJ0iQT3FRX2Yi4avoLkv4CWxBcNt11oZiuFKGmXQhFN6kuRESSgitXKf6A5g+I6cqFLiKCiptOkraZ5H4cuZOkH9oLw+UO8z7nvGfei3DFohoEQLAKCGWyGJCSbQZsD3ei7r+f45WATV6xmjdISyAlYLaDliveux+ti5ozwLAaFPhIlk6G6f18sxuO7l9bBiWbVoncDGCMWMl++PI/gJ7IohkmD0zkBxT5oRn5LTpN7ULqBByEtFywWvRyHz8Fx3dvN0UwuMPyR0uJFycddH7tKPHZRn7BjnwwkQcUud0HO/K7oOENcTYPRCEAFERmsCpyfZDZfpdnBosOUKGx13BC60QTIdjpmSzrEWDR+UagjsgMciLXAwfhc4MNpKfQtOPE6mVh0gEOaeTXtWXXEdADpJAR/GaZ/rrM9Us82wORPd5zHbRp7JVcyxR5PRP5LTPy6uo0HXIlG9awMgLuEUABkBYI7VLyxs+yyPW3+Nyg4zqoG+UV7TDRlMBbWA/D43u3qlbLGmgRWM2BjFxjQjWQGQBuu4bYYvrm14JMnQSXcjBcny9A5DXIiDJp9/8FWCXAAOaFUEfILTgIcttOvf+2NJnLdI0fBFUyskZKBGTEJEAmBuzr9LCSUDwGQAwwgMzu+m9/VWOAeZSsk+YP48C4li/sRok1nlBd5PogBjgxNw70PfHyqBADyEVXicasZXCVpxYiLfO+HxVZDHCVYwvu2ebPT7fOLNAm65AWLnEAzvsEcOi9/lPU1aACXDfOLdhD9kxNszGdweQGYstqXgItJwDFNxKvwrp57G8h0zVwldHuA0IFt8El83yIs2FSDcpgxDJpUUTwK7gTduN3AK5iG7ehc/E2/gUPD3q3eY4awwAAAABJRU5ErkJggq==)](https://cnb.cool/u/xhunli)
<p>Let G1 see the world, walk to the target, and reach out.</p>

<img src="https://cnb.cool/66666/resource/-/git/raw/main/img/hengtiao.gif" width="100%" height="3">
</div><br>

## Introduction

Unitree G1 humanoid robot **YOLO object detection and path planning navigation** workspace. Built on ROS2 Foxy + Python, featuring real-time object detection, 3D spatial localization, and autonomous navigation.

**Tech Stack**: Ubuntu 20.04 · ROS2 Foxy · Python 3.8+ · YOLOv11 · Nav2 · colcon

**Core Capabilities**:

- YOLO real-time object detection (custom trained models supported)
- 2D detection to 3D spatial coordinate projection
- Nav2 path planning and autonomous navigation
- Emergency stop and speed limiting safety protection

### Project Structure

```
g1act_ws/
├── src/
│   ├── g1_yolo_nav_py/                      # YOLO detection & navigation package
│   │   ├── g1_yolo_nav_py/
│   │   │   ├── sport_client.py              # Unified motion control (Loco API)
│   │   │   ├── yolo_detector.py             # YOLO detection node
│   │   │   ├── spatial_target.py            # 3D spatial projection node
│   │   │   ├── detection_visualizer.py      # Detection visualizer (tkinter)
│   │   │   ├── control_panel.py             # Control panel (tkinter GUI)
│   │   │   ├── grasp_task.py                # Grasp task controller
│   │   │   ├── yaw_align.py                 # Yaw alignment node (robot rotation)
│   │   │   ├── loco_forward.py              # Forward control (Loco API)
│   │   │   ├── rgbd_capture.py              # RGBD data capture node
│   │   │   └── distence.py                  # Distance estimation utility
│   │   ├── arm/                             # Arm control scripts (unitree_sdk2py, non-ROS2)
│   │   │   ├── armup.py                     # Arm grasp (auto-exit on completion)
│   │   │   ├── armdown.py                   # Arm release (auto-exit on completion)
│   │   │   └── arm.py                       # Arm SDK base control
│   │   ├── launch/
│   │   │   └── yolo_nav.launch.py           # Navigation pipeline launch file
│   │   ├── config/
│   │   │   └── yolo_nav.yaml                # Parameter configuration
│   │   └── yolo_v11x_best.pt                # YOLOv11 custom trained model weights
│   └── base/                                # G1 robot base packages
│       ├── g1_description/                  # URDF/MJCF models (12/23/29dof variants)
│       ├── g1_driver_py/                    # ROS2 driver: odom, TF, joint_states
│       ├── g1_teleop_ctrl_keyboard/         # Keyboard teleop (Sport API)
│       ├── g1_twist_bridge_py/              # Twist → Sport API bridge
│       ├── ctrl_keyboard/                   # Keyboard+voice control (Loco API + arm actions)
│       └── h1_description/                  # H1 robot description
├── requirements.txt
└── README.md
```
```
# Update commands
cd ~/g1act_ws/manact_ws
git pull
colcon build --packages-skip h1_description
```

```
# Activate virtual environment (every session)
cd ~/g1act_ws/manact_ws
source ~/g1act_venv/bin/activate
. install/setup.bash
```

### Model Description

| File | Description |
| --- | --- |
| `yolo_v11x_best.pt` | YOLOv11x custom trained model weights for object detection inference |

## Quick Start

### Environment Setup (First Time)

> **Lab Requirement**: Each experimenter must use an independent Python virtual environment. Installing packages in the system Python is prohibited.

```bash
# 1) Create virtual environment (with access to system ROS2 / unitree_sdk2py)
python3 -m venv ~/g1act_venv --system-site-packages

# 2) Activate virtual environment
source ~/g1act_venv/bin/activate

# 3) Install project dependencies
cd ~/g1act_ws/manact_ws
pip install -r requirements.txt

# 4) Build ROS2 workspace
colcon build
```

```bash
# --- Every session ---
source ~/g1act_venv/bin/activate
cd ~/g1act_ws
. install/setup.bash
```

### Test YOLO Detection Only

```bash
# 1. Launch D455 camera (Terminal 1)
ros2 launch realsense2_camera rs_launch.py camera_namespace:=robot1 camera_name:=D455_1

# 2. Build and launch YOLO detector (Terminal 2)
source ~/g1act_venv/bin/activate
colcon build --packages-select g1_yolo_nav_py && . install/setup.bash
ros2 run g1_yolo_nav_py yolo_detector

# 3. Launch visualizer (Terminal 3)
ros2 run g1_yolo_nav_py detection_visualizer

# 4. View detections (Terminal 4)
ros2 topic echo /g1/vision/detections
```

**Custom Parameters:**

```bash
# Detect persons
ros2 run g1_yolo_nav_py yolo_detector --ros-args -p target_classes:='["person"]'

# Custom model path
ros2 run g1_yolo_nav_py yolo_detector --ros-args -p model_path:=/path/to/model.pt
```

---

## Separate Debugging: Alignment vs. Motion Control

Alignment and forward movement are **independent nodes** — they can be launched separately for debugging.

### Debug Yaw Alignment (yaw_align)

**Verify**: Can the robot rotate to keep the target centered in the camera view?

```bash
# Terminal 1: Camera
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=robot1 camera_name:=D455_1

# Terminal 2: YOLO detection
ros2 run g1_yolo_nav_py yolo_detector

# Terminal 3: Yaw alignment (Loco API SET_VELOCITY)
ros2 run g1_yolo_nav_py yaw_align

# Terminal 4 (optional): Visualizer
ros2 run g1_yolo_nav_py detection_visualizer
```

**What to observe:**

- Log shows `[对齐] 检测到目标: u=0.xxx, vyaw=x.xxx`
- Place target left/right of robot → robot should auto-rotate
- Robot stops rotating when target is centered

**Parameter Tuning:**

```bash
# Tracking too slow — increase kp
ros2 run g1_yolo_nav_py yaw_align --ros-args -p yaw_kp:=3.0

# Oscillating — decrease kp + widen tolerance
ros2 run g1_yolo_nav_py yaw_align --ros-args \
  -p yaw_kp:=1.0 -p center_tolerance:=0.12

# Limit max rotation speed
ros2 run g1_yolo_nav_py yaw_align --ros-args -p max_yaw_speed:=0.3
```

| Parameter | Default | Description |
| --- | --- | --- |
| `yaw_kp` | 2.0 | P gain: increase if slow, decrease if oscillating |
| `center_tolerance` | 0.08 | Centering tolerance (normalized), smaller = stricter |
| `max_yaw_speed` | 0.6 rad/s | Maximum rotation speed |
| `lost_timeout` | 1.0 s | Target lost timeout |

**Pass criteria**: Robot smoothly tracks target when it moves; stable when centered.

---

### Debug Forward Control (loco_forward)

**Verify**: Can the robot move forward and stop automatically when the target is centered?

```bash
# Terminal 1~3: Same as above (camera + YOLO + yaw_align)

# Terminal 4: Forward control (Loco API SET_VELOCITY)
ros2 run g1_yolo_nav_py loco_forward
```

**What to observe:**

- When target is centered for >0.8s, log shows `开始前进`
- Robot moves forward at set speed
- When bbox exceeds 45% of frame, log shows `到达目标!` and stops
- Auto-stops when target is lost or off-center

**Parameter Tuning:**

```bash
# Use low speed for first test
ros2 run g1_yolo_nav_py loco_forward --ros-args -p forward_speed:=0.15

# Get closer before stopping
ros2 run g1_yolo_nav_py loco_forward --ros-args -p arrive_bbox_ratio:=0.6
```

| Parameter | Default | Description |
| --- | --- | --- |
| `forward_speed` | 0.3 m/s | Forward speed (0.15 recommended for first test) |
| `align_stable_time` | 0.8 s | How long centered before moving forward |
| `arrive_bbox_ratio` | 0.45 | Bbox ratio threshold for arrival |
| `center_tolerance` | 0.08 | Centering tolerance |
| `lost_timeout` | 1.0 s | Target lost timeout |

**Pass criteria**: Robot moves forward correctly, stops near target, stops immediately when target is lost.

---

### Combined Operation

Once alignment and forward control are verified:

```bash
# Option A: Launch separately (recommended for debugging)
# Terminal 1: Camera
# Terminal 2: ros2 run g1_yolo_nav_py yolo_detector
# Terminal 3: ros2 run g1_yolo_nav_py yaw_align
# Terminal 4: ros2 run g1_yolo_nav_py loco_forward

# Option B: Launch file
ros2 launch g1_yolo_nav_py yolo_nav.launch.py enable_approach:=true
```

**Safety Tips:**

- First combined test: `forward_speed` ≤ 0.2 m/s
- Keep remote emergency stop ready
- Ensure clear path ahead

---

## Control Panel (GUI Integrated Control)

tkinter-based GUI integrating detection visualization + manual teleop + one-click grasp task.

**Features:**

- Dual canvas: raw camera image + YOLO detection overlay
- Direction buttons: forward/backward/left/right/stop (available in IDLE state)
- Task buttons: search/grab/put down/turn & put down
- Speed slider: 0.05~0.5 m/s adjustable
- Real-time FPS, state machine status, detection info
- Bottom log panel: arm script output displayed in real-time

**State Machine Flow:**

```
IDLE → [Search] → SEARCHING (rotate to find target)
                     ↓ Target detected
                   ALIGNING (yaw alignment to center)
                     ↓ Centered & stable
                   APPROACHING (Loco API forward)
                     ↓ Arrived at target
                   GRABBING (execute armup.py)
                     ↓ Grasp complete
                   MENU (put down / turn & put down)
                     ↓ Put down complete
                   IDLE
```

**Running:**

```bash
# Terminal 1: YOLO detection
ros2 run g1_yolo_nav_py yolo_detector

# Terminal 2: Control panel
ros2 run g1_yolo_nav_py control_panel
```

> **Note**: The control panel does not import `unitree_sdk2py` (to avoid DDS conflicts).
> Motion control is handled via `SportClient` using Loco API, publishing directly to `/api/sport/request`.

**Parameters:**

| Parameter | Default | Description |
| --- | --- | --- |
| `image_topic` | `/D455_1/color/image_raw` | Camera image topic |
| `detection_topic` | `/g1/vision/detections` | Detection result topic |
| `target_class_id` | `chair` | Target class |
| `forward_speed` | `0.2` | Forward speed m/s |
| `search_yaw_speed` | `0.3` | Search rotation speed rad/s |
| `yaw_kp` | `2.0` | Yaw alignment P gain |
| `arrive_bbox_ratio` | `0.45` | Arrival detection threshold |
| `arm_script_dir` | `~/g1act_ws/manact_ws/src/g1_yolo_nav_py/arm` | Arm script directory |

**Dependency:** `pip install pillow`

---

## Grasp Task (One-Click Full Pipeline)

Automatically executes: object detection → yaw alignment → forward approach → grasp → interactive menu.

**Prerequisites:**

- G1 robot is connected and standing
- `armup.py` / `armdown.py` are in place (under `arm/` directory, auto-exit on completion)

**Manual Step-by-Step Launch (for debugging):**

```bash
# Terminal 1: Camera
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=robot1 camera_name:=D455_1 align_depth.enable:=true

# Terminal 2: YOLO detection
ros2 run g1_yolo_nav_py yolo_detector

# Terminal 3: Grasp task (Loco API mode, no DDS required)
ros2 run g1_yolo_nav_py grasp_task
```

**Execution Flow & Key Logs:**

```
==================================================
G1 Grasp Task Started (Loco API mode)
Target class: chair
armup: ~/g1act_ws/manact_ws/src/g1_yolo_nav_py/arm/armup.py
armdown: ~/g1act_ws/manact_ws/src/g1_yolo_nav_py/arm/armdown.py
==================================================
[SEARCHING]  Rotating to search for target...
[Detect] Target found: chair, confidence=95%, u=0.32
[State] SEARCHING → ALIGNING: Target found
[State] ALIGNING → APPROACHING: Target centered
[State] APPROACHING → GRABBING: Arrived at target! bbox=0.46 >= 0.45
[Grasp] Executing armup.py ...
[Grasp] armup.py completed
```

**Interactive menu after grasp:**

```
========================================
  G1 Grasp Task — Action Menu
========================================
  1. Put down object
  2. Turn & put down object
  3. Custom control (input xyz)
  4. Exit
========================================
Select [1-4]:
```

| Option | Action | Description |
| --- | --- | --- |
| 1 | Execute `armdown.py` | Put down object in place |
| 2 | Turn 90° + `armdown.py` | Turn around and put down |
| 3 | Input `x y z` to move | e.g. `0.2 0.0 0.3`, input `q` to stop |
| 4 | Safe exit | Stop motion and exit program |

**Parameter Tuning:**

```bash
# Detect bottles, reduce speed
ros2 run g1_yolo_nav_py grasp_task --ros-args \
  -p target_class_id:=bottle \
  -p forward_speed:=0.15

# Custom arm script directory
ros2 run g1_yolo_nav_py grasp_task --ros-args \
  -p arm_script_dir:=/home/unitree/my_arm_scripts
```

| Parameter | Default | Description |
| --- | --- | --- |
| `target_class_id` | chair | YOLO detection target class |
| `forward_speed` | 0.2 m/s | Approach speed (0.15 recommended for first run) |
| `arrive_bbox_ratio` | 0.45 | Bbox ratio arrival threshold |
| `search_yaw_speed` | 0.3 rad/s | Search rotation speed |
| `yaw_kp` | 2.0 | Yaw alignment P gain |
| `lost_timeout` | 2.0 s | Target lost timeout |
| `arm_script_dir` | `~/g1act_ws/manact_ws/src/g1_yolo_nav_py/arm` | Arm script directory |

> **Safety Tip**: First run recommended with `forward_speed:=0.15`. Keep emergency stop ready.

---

## RGBD Data Capture

```bash
# Camera must be launched with depth alignment
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=robot1 camera_name:=D455_1 align_depth.enable:=true

# Run capture (default: 1 frame every 5s, 60s total)
ros2 run g1_yolo_nav_py rgbd_capture

# Custom parameters
ros2 run g1_yolo_nav_py rgbd_capture --ros-args \
  -p interval_sec:=2.0 -p duration_sec:=30.0
```

## Motion Control

All motion control nodes use the **Loco API (7xxx series)**, wrapped via `SportClient` and published to `/api/sport/request`.

| API | ID | Purpose | Parameter Format |
| --- | --- | --- | --- |
| SET_FSM_ID | 7101 | FSM state control | `{"data": fsm_id}` |
| SET_VELOCITY | 7105 | Velocity control | `{"velocity": [vx, vy, vyaw], "duration": t}` |
| SET_BALANCE_MODE | 7102 | Balance mode | `{"data": mode}` |
| SET_SPEED_MODE | 7107 | Speed mode | `{"data": mode}` |
| SET_STAND_HEIGHT | 7104 | Stand height | `{"data": height}` |

**FSM State IDs:**

| ID | State | Description |
| --- | --- | --- |
| 0 | ZERO_TORQUE | Zero torque mode (initial sitting pose) |
| 1 | DAMP | Damping mode (prepare to stand) |
| 3 | SIT | Sitting |
| 4 | STAND_UP | Locked standing |
| 500 | START | Normal locomotion |
| 801 | WALK_RUN | Walk/run locomotion |

**Balance Modes:**

| ID | Mode | Description |
| --- | --- | --- |
| 0 | BALANCE_STAND | Balance stand (stops stepping when velocity=0) |
| 1 | CONTINUOUS_GAIT | Continuous gait (keeps stepping) |

**FSM Initialization Sequence:**

```
SET_FSM_ID(DAMP=1) → SET_FSM_ID(STAND_UP=4) → SET_FSM_ID(WALK_RUN=801) → SET_BALANCE_MODE(CONTINUOUS_GAIT=1)
```

`SET_VELOCITY(7105)` only takes effect under WALK_RUN + CONTINUOUS_GAIT mode.

> **Note**: The motion control implementation references the verified `ctrl_keyboard` package, using the same API system.

---

![Fork History Trend](https://commit.cool/forks/1255027942/cloud/manact_ws?interval=day)
