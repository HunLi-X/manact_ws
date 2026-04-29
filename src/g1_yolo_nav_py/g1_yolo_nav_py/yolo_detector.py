"""YOLO 目标检测节点 — 订阅相机图像，运行 YOLO 推理，发布 2D 检测结果。"""

# ==================================================================
# 1. 标准库导入
# ==================================================================
import os  # 路径判断与 sys.path 修改
import sys  # sys.path 修改

# ROS2 colcon 会隔离 PYTHONPATH，必须在所有 import 之前追加路径
for _p in [
    "/usr/lib/python3/dist-packages",
    os.path.expanduser("~/.local/lib/python3.8/site-packages"),
    "/usr/local/lib/python3.8/dist-packages",
]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

# ==================================================================
# 2. 第三方库与 ROS2 导入
# ==================================================================
import rclpy  # ROS2 Python 客户端库
from rclpy.node import Node  # ROS2 节点基类
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # QoS 配置
from typing import List  # 类型注解（Python 3.8 兼容）
from sensor_msgs.msg import Image  # ROS2 图像消息
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D  # 检测结果消息
from cv_bridge import CvBridge  # ROS2 图像消息与 OpenCV 格式互转
from ament_index_python.packages import get_package_share_directory  # 获取 ROS2 包共享目录

# ultralytics: YOLO 目标检测模型库（可选依赖）
try:
    from ultralytics import YOLO  # YOLO 推理模型
except Exception as _e:
    print(f"[DEBUG] ultralytics import failed: {type(_e).__name__}: {_e}")
    YOLO = None


class YoloDetectorNode(Node):
    """YOLO 目标检测节点。"""

    def __init__(self) -> None:
        super().__init__("g1_yolo_detector_node")

        # ---- 参数 ----
        self.declare_parameter("model_path", "yolo_v11x_best.pt")
        self.declare_parameter("confidence_threshold", 0.8)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("input_image_topic", "/D455_1/color/image_raw")
        self.declare_parameter("output_detection_topic", "/g1/vision/detections")
        self.declare_parameter("target_classes", ["chair"])  # 按类别名称过滤
        self.declare_parameter("max_image_size", 640)

        model_path = self.get_parameter("model_path").value
        # 若为相对路径，自动解析为 share 目录下的 models 子目录
        if not os.path.isabs(model_path):
            pkg_share = get_package_share_directory("g1_yolo_nav_py")
            model_path = os.path.join(pkg_share, "models", model_path)
        self._conf_thresh = float(self.get_parameter("confidence_threshold").value)
        self._nms_thresh = float(self.get_parameter("nms_threshold").value)
        self._max_size = int(self.get_parameter("max_image_size").value)

        self.get_logger().info(f"模型路径: {model_path}, 置信度阈值: {self._conf_thresh}")

        # ---- 加载模型 ----
        if YOLO is None:
            self.get_logger().error("ultralytics 未安装，请执行: pip3 install ultralytics")
            raise RuntimeError("ultralytics package not found")

        try:
            self._model = YOLO(model_path)
            self.get_logger().info(f"YOLO 模型加载成功: {model_path}")
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {e}")
            raise

        # ---- 解析目标类别：支持名称或数字 ID ----
        # model.names: {0: "chair", 1: "table", ...}
        self._name_to_id = {v: int(k) for k, v in self._model.names.items()}
        self._target_class_ids: List[int] = []
        raw_targets = list(self.get_parameter("target_classes").value)
        for t in raw_targets:
            if isinstance(t, (int, float)):
                self._target_class_ids.append(int(t))
            elif isinstance(t, str) and t in self._name_to_id:
                self._target_class_ids.append(self._name_to_id[t])
            else:
                self.get_logger().warn(f"未知目标类别 '{t}'，可用类别: {list(self._name_to_id.keys())}")
        self.get_logger().info(f"目标类别 ID: {self._target_class_ids} ({raw_targets})")

        # ---- CV Bridge ----
        self._bridge = CvBridge()

        # ---- QoS ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ---- 话题 ----
        input_topic = self.get_parameter("input_image_topic").value
        output_topic = self.get_parameter("output_detection_topic").value
        self._sub = self.create_subscription(
            Image, input_topic, self._image_callback, sensor_qos
        )
        self._pub = self.create_publisher(Detection2DArray, output_topic, 10)

        self.get_logger().info(f"订阅图像: {input_topic}, 发布检测: {output_topic}")

    # ------------------------------------------------------------------
    def _image_callback(self, msg: Image) -> None:
        """图像回调：执行推理并发布检测结果。"""
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        # 保持原始尺寸用于坐标还原
        orig_h, orig_w = cv_image.shape[:2]

        # 推理
        results = self._model(
            cv_image,
            conf=self._conf_thresh,
            iou=self._nms_thresh,
            classes=self._target_class_ids if self._target_class_ids else None,
            verbose=False,
        )

        # 构造检测结果
        det_array = Detection2DArray()
        det_array.header = msg.header

        if results and len(results) > 0:
            boxes = results[0].boxes
            for box in boxes:
                det = Detection2D()
                hyp = ObjectHypothesisWithPose()
                hyp.id = self._model.names[int(box.cls[0])]
                hyp.score = float(box.conf[0])

                # BoundingBox2D（归一化坐标）
                # 注意：Pose2D 只有 x, y, theta，没有 position 嵌套
                bbox = BoundingBox2D()
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                w = x2 - x1
                h = y2 - y1
                bbox.center.x = cx / orig_w
                bbox.center.y = cy / orig_h
                bbox.size_x = w / orig_w
                bbox.size_y = h / orig_h

                det.bbox = bbox
                det.results.append(hyp)
                det_array.detections.append(det)

                self.get_logger().info(
                    f'检测到目标: {hyp.id} (置信度={hyp.score:.0%}), '
                    f'中心=({bbox.center.x:.2f},{bbox.center.y:.2f})'
                )

        self._pub.publish(det_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
