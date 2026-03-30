"""
需求：
步骤:
    1.导包
    2.初始化 ROS2 客户端
    3.定义节点类
    
    4.调用spin函数,并传入节点对象
    5.释放资源
"""

# 1.导包
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
from geometry_msgs.msg import Twist
from .sport_model import ROBOT_SPORT_API_IDS
import json

# 3.定义节点类
class TwistBridge(Node):
    def __init__(self):
        super().__init__("twist_bridge")
        self.request_pub = self.create_publisher(Request,"/api/sport/request",10)
        self.twist_sub = self.create_subscription(Twist,"/cmd_vel",self.twist_cb,10)

    def twist_cb(self,twist:Twist):
        request = Request()
        x = twist.linear.x
        y = twist.linear.y
        z = twist.angular.z
        api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]
        if x != 0 or y != 0 or z != 0:
            api_id=ROBOT_SPORT_API_IDS["MOVE"]
            js = {"x": x, "y": y, "z": z}
            request.parameter = json.dumps(js)
        request.header.identity.api_id = api_id
        self.request_pub.publish(request)
def main():
    # 2.初始化 ROS2 客户端
    rclpy.init()
    # 4.调用spin函数,并传入节点对象
    rclpy.spin(TwistBridge())
    # 5.释放资源
    rclpy.shutdown()

if __name__ == '__main__':
    main()