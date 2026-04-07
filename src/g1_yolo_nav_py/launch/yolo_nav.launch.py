"""
Launch 文件：启动 YOLO 目标识别 + 空间投影 + 导航规划 + 腰部追踪

启动节点：
    1. g1_yolo_detector_node   - YOLO 目标检测
    2. g1_spatial_target_node  - 2D→3D 空间投影
    3. g1_nav_planner_node     - 路径规划与趋近控制
    4. g1_waist_tracker_node   - 腰部视觉伺服追踪（可选）
    5. d455_camera_tf_publisher - D455 相机静态 TF

使用示例：
    ros2 launch g1_yolo_nav_py yolo_nav.launch.py
    ros2 launch g1_yolo_nav_py yolo_nav.launch.py use_nav2:=true use_depth_sensor:=true
    ros2 launch g1_yolo_nav_py yolo_nav.launch.py target_class:=person model_path:=yolov8n.pt
    ros2 launch g1_yolo_nav_py yolo_nav.launch.py enable_waist_tracking:=true

TF 树结构：
    odom → base_link → pelvis → torso_link → robot1/D455_1_link → camera_optical_frame
           └──────────────────────────────────────────────────────────────────────────┘
                              （由 static_transform_publisher 连接）
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """生成 Launch 描述。"""
    pkg_dir = get_package_share_directory("g1_yolo_nav_py")
    config_file = os.path.join(pkg_dir, "config", "yolo_nav.yaml")

    # ==================================================================
    # Launch 参数声明
    # ==================================================================

    # 是否启动 RViz2 可视化
    use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="false",
        description="是否启动 RViz2 可视化界面",
    )

    # 是否使用 Nav2 导航栈（false 时使用简单 P 控制趋近）
    use_nav2 = DeclareLaunchArgument(
        name="use_nav2",
        default_value="false",
        description="是否使用 Nav2 导航（false 时使用简单趋近模式）",
    )

    # YOLO 模型文件路径
    model_path_arg = DeclareLaunchArgument(
        name="model_path",
        default_value="yolo_v11x_best.pt",
        description="YOLO 模型文件名（相对于 share/models/ 目录，或绝对路径）",
    )

    # 是否使用深度传感器获取目标距离
    use_depth = DeclareLaunchArgument(
        name="use_depth_sensor",
        default_value="false",
        description="是否使用深度传感器（false 时使用默认距离）",
    )

    # 目标检测类别
    target_class = DeclareLaunchArgument(
        name="target_class",
        default_value="chair",
        description="目标类别名称（如 chair, person，也支持数字 ID）",
    )

    # 是否启用腰部追踪
    enable_waist_tracking = DeclareLaunchArgument(
        name="enable_waist_tracking",
        default_value="false",
        description="是否启用腰部视觉伺服追踪（需要 unitree_sdk2py）",
    )

    # ==================================================================
    # D455 相机静态 TF
    # ==================================================================
    # 将相机坐标系连接到机器人本体 TF 树
    #
    # 安装参数（来自工程图）：
    #   - 前向偏移：~47.65mm（相机在头部前方）
    #   - 俯仰角：42° 向下（低头看地，适合地面目标检测）
    #   - 安装高度：~462.68mm（从地面到光学中心）
    #
    # 坐标变换：
    #   parent: torso_link（躯干坐标系）
    #   child:  robot1/D455_1_link（RealSense 基座坐标系）
    #   translation: (0.04765, 0, 0) meters
    #   rotation: pitch = -42° (向下为负)
    #
    # 四元数计算：
    #   绕 Y 轴旋转 pitch 角 → (sin(pitch/2), 0, 0, cos(pitch/2))
    # ==================================================================
    camera_pitch_deg = -42.0  # 向下俯仰角度（负值）
    camera_pitch_rad = math.radians(camera_pitch_deg)
    q_x = math.sin(camera_pitch_rad / 2.0)
    q_w = math.cos(camera_pitch_rad / 2.0)

    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="d455_camera_tf_publisher",
        arguments=[
            # 平移向量 (meters)
            "--x", "0.04765",      # 前向偏移 47.65mm
            "--y", "0.0",          # 无侧向偏移
            "--z", "0.0",          # 相对于 torso_link 无高度偏移（头部位置）
            # 四元数旋转 (绕 Y 轴俯仰 -42°)
            "--qx", str(q_x),
            "--qy", "0.0",
            "--qz", "0.0",
            "--qw", str(q_w),
            # 坐标系名称
            "--frame-id", "torso_link",
            "--child-frame-id", "robot1/D455_1_link",
        ],
    )

    # ==================================================================
    # 节点定义
    # ==================================================================

    # YOLO 目标检测节点
    # 输入: /robot1/D455_1/color/image_raw (彩色图像)
    # 输出: /g1/vision/detections (Detection2DArray)
    yolo_detector_node = Node(
        package="g1_yolo_nav_py",
        executable="yolo_detector",
        name="g1_yolo_detector_node",
        parameters=[
            config_file,
            {"model_path": LaunchConfiguration("model_path")},
            {"target_classes": [LaunchConfiguration("target_class")]},
        ],
    )

    # 空间投影节点
    # 输入: /g1/vision/detections (检测结果)
    #        /robot1/D455_1/depth/image_rect_raw (深度图，可选)
    #        /robot1/D455_1/color/camera_info (相机内参)
    # 输出: /g1/nav/target_pose (PoseStamped, odom 坐标系)
    spatial_target_node = Node(
        package="g1_yolo_nav_py",
        executable="spatial_target",
        name="g1_spatial_target_node",
        parameters=[
            config_file,
            {"use_depth_sensor": LaunchConfiguration("use_depth_sensor")},
        ],
    )

    # 导航规划节点
    # 输入: /g1/nav/target_pose (目标位姿)
    #        TF: odom → base_link (机器人位姿)
    # 输出: /cmd_vel (Twist, 速度指令)
    # 模式: use_nav2=true 使用 Nav2 栈，false 使用简单 P 控制趋近
    nav_planner_node = Node(
        package="g1_yolo_nav_py",
        executable="nav_planner",
        name="g1_nav_planner_node",
        parameters=[
            config_file,
            {"use_nav2": LaunchConfiguration("use_nav2")},
        ],
    )

    # RViz2 可视化（可选）
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=["-d", os.path.join(pkg_dir, "rviz", "yolo_nav.rviz")]
        if os.path.exists(os.path.join(pkg_dir, "rviz", "yolo_nav.rviz"))
        else [],
    )

    # 腰部追踪节点（视觉伺服，需要 unitree_sdk2py）
    # 通过旋转腰部让目标保持在画面中心
    # 输入: /g1/vision/detections (检测结果)
    # 输出: DDS LowCmd (腰部关节控制)
    waist_tracker_node = Node(
        package="g1_yolo_nav_py",
        executable="waist_tracker",
        name="g1_waist_tracker_node",
        condition=IfCondition(LaunchConfiguration("enable_waist_tracking")),
        parameters=[config_file],
    )

    # ==================================================================
    # Launch 描述
    # ==================================================================
    return LaunchDescription([
        # 参数声明
        use_rviz,
        use_nav2,
        model_path_arg,
        use_depth,
        target_class,
        enable_waist_tracking,
        # 静态 TF
        camera_static_tf,
        # 功能节点
        yolo_detector_node,
        spatial_target_node,
        nav_planner_node,
        waist_tracker_node,
        rviz_node,
    ])
