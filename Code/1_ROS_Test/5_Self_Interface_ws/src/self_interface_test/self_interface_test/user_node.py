"""
    客户端有两个任务：
    1. 机器人运动指令的发布
    2. 机器人实时状态接受
"""
import rclpy
from rclpy.node import Node
# 导入话题通信接口
from self_interface.msg import RobotStatus
# 导入服务通信接口
from self_interface.srv import RobotCommand

class User_Node(Node):
    def __init__(self,name):
        super().__init__(name)
        # 创建订阅方
        self.robotstatus_subscriber = self.create_subscription(RobotStatus,"robotstatus",self.robotstatus_get_callback,10)
        # 创建客户端
        self.robotmove_client = self.create_client(RobotCommand,"robotmove")

    # 订阅接受回调函数
    def robotstatus_get_callback(self,msg):
        self.get_logger().info(f"Robot Status:{msg.status};Pose:{msg.pose}")

    # 服务端响应回调函数
    def robotmove_command_callback(self,result_future):
        responce = result_future.result()
        self.get_logger().info(f"Moving Done!Consquence:{responce.pose}")

    # 服务端发送函数
    def robotmove_send_command(self,distance):
        while rclpy.ok() and self.robotmove_client.wait_for_service(1) == False:
            self.get_logger().info("Wait for service...")
        
        request = RobotCommand.Request()
        request.distance = distance
        self.get_logger().info(f"Command to move:{distance}")
        self.robotmove_client.call_async(request).add_done_callback(self.robotmove_command_callback)
    
def main(args = None):
    rclpy.init(args=args)
    node = User_Node("user")
    node.robotmove_send_command(5.0)
    rclpy.spin(node)
    rclpy.shutdown()
