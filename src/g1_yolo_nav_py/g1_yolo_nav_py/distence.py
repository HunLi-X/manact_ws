#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ==================================================================
# 1. 标准库导入
# ==================================================================
import os  # 环境变量读取（DISPLAY）

# ==================================================================
# 2. 第三方库与 ROS2 导入
# ==================================================================
import rclpy  # ROS2 Python 客户端库
from rclpy.node import Node  # ROS2 节点基类
from sensor_msgs.msg import Image, CameraInfo  # 图像与相机内参消息
from geometry_msgs.msg import PointStamped, Point  # 3D 点消息
from std_msgs.msg import Float32  # 浮点数消息（距离值）
from cv_bridge import CvBridge  # ROS2 图像消息与 OpenCV 格式互转
import cv2  # OpenCV 图像处理与可视化
import numpy as np  # 数值计算
from ultralytics import YOLO  # YOLO 目标检测模型

# TF2 坐标变换库
import tf2_ros  # TF2 缓冲区与监听器
from tf2_geometry_msgs import do_transform_point  # 点坐标变换函数

class DistanceToG1(Node):
    def __init__(self):
        super().__init__('distance_to_g1')

        # ---------- 参数 ----------
        self.declare_parameter("model_path", "yolo_v11x_best.pt")
        self.declare_parameter("target_class", "chair")
        self.declare_parameter("confidence", 0.5)

        model_path = self.get_parameter("model_path").value
        self._target_class = self.get_parameter("target_class").value
        self._conf = float(self.get_parameter("confidence").value)

        # ---------- 1. 初始化ROS组件 ----------
        self.bridge = CvBridge()
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLO 模型加载成功: {model_path}')
        except Exception as e:
            self.get_logger().error(f'YOLO 模型加载失败: {e}')
            raise

        # ---------- 2. 初始化TF2 ----------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------- 3. 订阅相机话题 ----------
        # 订阅彩色图和深度图
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        # 订阅相机内参
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)

        # ---------- 4. 发布话题 ----------
        self.dist_pub = self.create_publisher(Float32, '/object_distance_to_g1', 10)
        self.object_pose_pub = self.create_publisher(PointStamped, '/object_pose_in_g1', 10)

        # ---------- 5. 内部变量 ----------
        self.latest_depth = None      # 存储最新的深度图
        self.camera_info = None       # 存储相机内参
        self._display = os.environ.get("DISPLAY") is not None  # X11 检测
        self.get_logger().info('Node started. Waiting for data...')

    # ---------- 回调函数 ----------
    def depth_callback(self, msg):
        """接收深度图，转换为numpy数组（单位：米）"""
        try:
            depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth = depth_mm.astype(np.float32) * 0.001  # 转换为米
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')

    def camera_info_callback(self, msg):
        """接收相机内参，只接收一次"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('Camera info received.')
            self.get_logger().info(f'fx={msg.k[0]}, fy={msg.k[4]}, cx={msg.k[2]}, cy={msg.k[5]}')

    def get_depth_at_center(self, x1, y1, x2, y2, window_size=5):
        """计算检测框中心区域的深度中位数（米）"""
        if self.latest_depth is None:
            return None
        h, w = self.latest_depth.shape
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        cx = max(0, min(cx, w - 1))
        cy = max(0, min(cy, h - 1))
        half = window_size // 2
        y_min = max(0, cy - half)
        y_max = min(h, cy + half + 1)
        x_min = max(0, cx - half)
        x_max = min(w, cx + half + 1)
        window = self.latest_depth[y_min:y_max, x_min:x_max]
        valid = window[window > 0]
        if len(valid) == 0:
            return None
        return float(np.median(valid))

    def color_callback(self, msg):
        """主回调：目标检测 + 坐标变换 + 计算距离"""
        # 等待必要数据就绪
        if self.latest_depth is None or self.camera_info is None:
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Color callback conversion error: {e}')
            return

        # 目标检测
        results = self.model(color_image, conf=self._conf, verbose=False)
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                confidence = float(box.conf[0])

                # 过滤目标类别
                if cls_name != self._target_class:
                    continue

                # 获取中心点深度
                depth_z = self.get_depth_at_center(x1, y1, x2, y2)
                if depth_z is None or depth_z <= 0:
                    continue

                # 相机内参
                fx = self.camera_info.k[0]
                fy = self.camera_info.k[4]
                cx = self.camera_info.k[2]
                cy = self.camera_info.k[5]

                # 计算检测框中心点的三维坐标（相机坐标系）
                u = (x1 + x2) // 2
                v = (y1 + y2) // 2
                x_cam = (u - cx) * depth_z / fx
                y_cam = (v - cy) * depth_z / fy
                z_cam = depth_z

                # 创建相机坐标系下的点
                point_cam = PointStamped()
                point_cam.header.frame_id = 'camera_color_optical_frame'  # 彩色相机光学坐标系
                point_cam.header.stamp = self.get_clock().now().to_msg()
                point_cam.point = Point(x=x_cam, y=y_cam, z=z_cam)

                # 等待并获取 base_link 到 camera_color_optical_frame 的变换
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'base_link',            # 目标坐标系
                        'camera_color_optical_frame',  # 源坐标系
                        rclpy.time.Time()       # 获取最新变换
                    )
                except Exception as e:
                    self.get_logger().warn(f'TF lookup failed: {e}')
                    continue

                # 执行坐标变换：camera -> base_link
                point_base = do_transform_point(point_cam, transform)

                # 计算距离（欧氏距离）
                distance = np.sqrt(point_base.point.x**2 + point_base.point.y**2 + point_base.point.z**2)

                # 发布距离和坐标
                self.dist_pub.publish(Float32(data=distance))
                self.object_pose_pub.publish(point_base)

                self.get_logger().info(f'Detected {cls_name} | Distance to base_link: {distance:.3f} m | '
                                       f'Position in base_link: x={point_base.point.x:.3f}, y={point_base.point.y:.3f}, z={point_base.point.z:.3f}')

                # 可视化（可选）
                label = f'{cls_name} {confidence:.2f} | {distance:.2f}m'
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(color_image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        if self._display:
            cv2.imshow('Distance to G1 (TF2)', color_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DistanceToG1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()