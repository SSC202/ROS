import rclpy
from rclpy.node import Node
# 导入订阅者需要的消息接口
from std_msgs.msg import String


class Listener(Node):
    def __init__(self,name):
        super().__init__(name)
        # 创建订阅者，由于为触发式接收，无需定时器
        self.listener = self.create_subscription(String,"command",self.command_callback,10)
    
    def command_callback(self,msg):
        t = 0.0
        if msg.data == "Hello Listener":
            t = t + 1.0
        self.get_logger().info(f'{t}:Get Msg:{msg.data}')
    
def main(args = None):
    rclpy.init()
    listener = Listener("Listener")
    rclpy.spin(listener)
    rclpy.shutdown()