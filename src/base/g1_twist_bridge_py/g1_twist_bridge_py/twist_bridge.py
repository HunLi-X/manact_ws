"""G1 Twist Bridge — 将 geometry_msgs/Twist 转换为 unitree_api/Request。"""

# ==================================================================
# 1. 标准库导入
# ==================================================================
import json  # JSON 序列化，构造 Request 参数

# ==================================================================
# 2. 第三方库与 ROS2 导入
# ==================================================================
import rclpy  # ROS2 Python 客户端库
from rclpy.node import Node  # ROS2 节点基类
from unitree_api.msg import Request  # 宇树 API 请求消息
from geometry_msgs.msg import Twist  # 速度指令消息

# ==================================================================
# 3. 本项目内部导入
# ==================================================================
from .sport_model import ROBOT_SPORT_API_IDS  # Sport API ID 常量映射


class TwistBridge(Node):
    """Twist → unitree_api Request 桥接节点。"""

    def __init__(self) -> None:
        super().__init__("g1_twist_bridge_node")
        self.request_pub = self.create_publisher(Request, "/api/sport/request", 10)
        self.twist_sub = self.create_subscription(
            Twist, "/cmd_vel", self.twist_cb, 10
        )
        self.get_logger().info("G1 Twist Bridge 节点启动")

    def twist_cb(self, twist: Twist) -> None:
        request = Request()
        x = twist.linear.x
        y = twist.linear.y
        z = twist.angular.z
        js = {"x": x, "y": y, "z": z}
        request.parameter = json.dumps(js)
        if x != 0 or y != 0 or z != 0:
            api_id = ROBOT_SPORT_API_IDS["MOVE"]
        else:
            api_id = ROBOT_SPORT_API_IDS["STOPMOVE"]
        request.header.identity.api_id = api_id
        self.request_pub.publish(request)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwistBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
