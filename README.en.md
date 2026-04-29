### Project Structure

```
g1act_ws/
├── README.md                     # Project documentation (Chinese)
├── README.en.md                 # Project documentation (English)
├── CODEBUDDY.md                # AI coding assistant guide
├── requirements.txt             # Python dependencies
├── src/
│   ├── D455.md                 # D455 camera documentation
│   ├── g1_yolo_nav_py/        # Main ROS2 Python package
│   │   ├── setup.py                # Package setup (entry_points definition)
│   │   ├── package.xml            # ROS2 package description
│   │   ├── setup.cfg              # Package configuration
│   │   ├── yolo_v11x_best.pt     # YOLOv11x custom trained model
│   │   ├── config/                # Parameter configuration files
│   │   │   └── yolo_nav.yaml
│   │   ├── launch/                # ROS2 launch files
│   │   │   ├── grasp_task.launch.py
│   │   │   └── yolo_nav.launch.py
│   │   ├── arm/                   # Arm control scripts (unitree_sdk2py, separate process)
│   │   │   ├── arm_common.py        # Shared module: joint constants, BaseArmController base class
│   │   │   ├── arm.py              # Arm control demo (zero→lift→lower)
│   │   │   ├── armup.py            # Grab action: reach→lift→grip hold
│   │   │   └── armdown.py          # Release action: extend→lower→zero release
│   │   └── g1_yolo_nav_py/       # Python module code
│   │       ├── __init__.py
│   │       ├── _dds_compat.py      # DDS compatibility layer
│   │       ├── _detection_utils.py  # Detection utility functions
│   │       ├── _grasp_state.py     # Grasp state machine mixin
│   │       └── other functions
│   └── base/                   # Reference packages (not directly called)
│       ├── ctrl_keyboard/        # Loco API reference implementation
│       └── g1_description/      # URDF models (12/23/29 dof)
```

```