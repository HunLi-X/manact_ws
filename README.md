[English](README.en.md) | 简体中文

<div align="center">
<h1>base</h1>


[![主题](https://img.shields.io/badge/主题-red)](./)
[![HTML5](https://img.shields.io/badge/HTML示例可删除)](https://html.spec.whatwg.org/)
[![CSS3](https://img.shields.io/badge/CSS3示例可删除-3-blue)](https://www.w3.org/Style/CSS/)
[![JavaScript](https://img.shields.io/badge/JavaScript示例可删除-ES6-yellow)](https://www.ecma-international.org/)

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

| 文件 | 说明 |
|------|------|
| `yolo_v11x_best.pt` | YOLOv11x 自定义训练模型权重文件，用于目标检测推理 |

> 该模型为已训练完成的 YOLOv11 权重文件，部署时放置于 `src/g1_yolo_nav_py/` 根目录下，通过 launch 参数 `model_path` 指定加载路径。

### 🚀 快速开始

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

### 🧪 仅测试 YOLO 目标检测

如果只需验证 YOLO 检测功能，无需启动完整导航管线：

```bash
# 1. 启动 D455 相机（终端 1）
ros2 launch realsense2_camera rs_launch.py camera_namespace:=robot1 camera_name:=D455_1

# 2. 编译并启动 YOLO 检测节点（终端 2）
colcon build --packages-select g1_yolo_nav_py
. install/setup.bash
ros2 run g1_yolo_nav_py yolo_detector

# 3. 启动可视化节点，查看带检测框的图像（终端 3）
ros2 run g1_yolo_nav_py detection_visualizer
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

---

![Fork 历史趋势图。](https://commit.cool/forks/实际项目地址?interval=day)