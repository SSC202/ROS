"""
    机器人端有两个任务：
    1. 机器人状态消息的发布(Publisher)
    2. 接受移动请求并回传当前位置
"""
import rclpy
from rclpy.node import Node
# 消息需要的数据类型
from self_interface.msg import RobotStatus
# 服务需要的数据类型
from self_interface.srv import RobotCommand
# 导入机器人类
from self_interface_test.robot import Robot

class Robot_Node(Node):
    def __init__(self,name):
        super().__init__(name)
        # 创建机器人
        self.robot = Robot()
        # 创建服务端
        self.robotmove_server = self.create_service(RobotCommand,"robotmove",self.move_request_callback)
        # 创建发布者
        self.robotstatus_publisher = self.create_publisher(RobotStatus,"robotstatus",10)
        # 定时消息发布定时器
        self.robotstatus_timer = self.create_timer(1,self.robotstatus_timmer_callback)

    # 定时器回调函数
    def robotstatus_timmer_callback(self):
        status_msg = RobotStatus()
        status_msg.status = self.robot.get_status()
        status_msg.pose = self.robot.get_current_pose()

        self.robotstatus_publisher.publish(status_msg)

    # 服务端回调函数
    def move_request_callback(self,request,responce):
        self.robot.move_robot(request.distance)
        responce.pose = self.robot.get_current_pose()
        return responce
        
def main(args = None):
    rclpy.init(args=args)
    node = Robot_Node("robot")
    rclpy.spin(node)
    rclpy.shutdown()