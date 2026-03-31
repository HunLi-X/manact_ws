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
│   ├── g1_teleop_ctrl_keyboard/   # Keyboard teleop via unitree_api
│   ├── g1_twist_bridge_py/        # Twist → unitree_api Request bridge
│   └── h1_description/            # H1 robot description (separate robot)
├── g1_yolo_nav_py/                # YOLO detection + navigation
└── arm.py                         # Standalone arm SDK script (unitree_sdk2py, not ROS2)
```

### Data Flow

```
Camera Image (/camera/color/image_raw)
    → [g1_yolo_detector_node] → Detection2DArray (/g1/vision/detections)
                                      ↓
                                 [g1_spatial_target_node] → PoseStamped (/g1/nav/target_pose)
                                      ↓                                  (in odom frame)
                                 [g1_nav_planner_node] → Nav2 or simple approach
                                      ↓
                                 Twist (/cmd_vel)
                                      ↓
                                 [g1_twist_bridge_node] → Request (/api/sport/request)
                                      ↓
                                 G1 Hardware
```

### Two Control Interfaces

1. **unitree_api (high-level)**: `g1_teleop_ctrl_keyboard` and `g1_twist_bridge_py` publish `unitree_api/msg/Request` to `/api/sport/request` with API IDs (MOVE=1008, HELLO=1016, etc.). This is the G1's official sport mode API.
2. **unitree_sdk2py (low-level)**: `src/arm.py` uses `ChannelPublisher("rt/arm_sdk", LowCmd_)` for direct joint-level arm control. This bypasses ROS2 entirely and communicates via DDS. The `G1JointIndex` class in `arm.py` defines the motor index mapping (0-29).

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
| `/api/sport/request` | `unitree_api/Request` | High-level motion commands |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (bridged to sport API) |
| `/g1/sensor/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/joint_states` | `sensor_msgs/JointState` | 29 joint positions |
| `/g1/vision/detections` | `vision_msgs/Detection2DArray` | YOLO 2D detections |
| `/g1/nav/target_pose` | `geometry_msgs/PoseStamped` | Navigation target (odom frame) |
| `/g1/nav/emergency_stop` | `geometry_msgs/Twist` | Emergency stop (any msg triggers) |

### TF Tree Requirement

The navigation pipeline requires: `odom → base_link → camera_color_optical_frame`. The driver node publishes `odom → base_link` TF; camera TF depends on the camera driver being active.

### Configuration

All node parameters are declared via `declare_parameter()` then read with `get_parameter()`. YAML config file: `g1_yolo_nav_py/config/yolo_nav.yaml`. Launch parameters override YAML values.

### Coding Conventions (from ros2doc_skill)

- Nodes inherit `rclpy.node.Node`, not `rclpy.create_node()`
- All parameters **must** be declared before reading (`declare_parameter` then `get_parameter`)
- No magic numbers — use parameters or named constants
- Explicit QoS with depth (never omit `depth`)
- Topic naming: `/g1/<module>/<signal>`
- Package naming: `g1_xxx_py`
- Node naming: `snake_case`
- Safety: velocity clamping, low default speeds, emergency stop support
