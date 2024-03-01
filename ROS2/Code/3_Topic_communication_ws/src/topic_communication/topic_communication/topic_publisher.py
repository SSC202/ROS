import rclpy
from rclpy.node import Node
# 导入发布者需要的消息接口
from std_msgs.msg import String


class Publisher(Node):
    def __init__(self,name):
        super().__init__(name)
        # 创建发布者 publisher
        self.publisher = self.create_publisher(String,"command",10)
        # 创建定时器以定时发布
        self.timer = self.create_timer(10,self.timer_callback)
    
    # 发布方本身无需回调函数，但是可以通过定时器定时发布
    def timer_callback(self):
        # 消息构建
        msg = String()
        msg.data = "Hello Listener"
        # 发布消息
        self.publisher.publish(msg)
        self.get_logger().info(f'Send Command:{msg.data}')

    

    
def main(args = None):
    rclpy.init()
    publisher = Publisher("Publisher")
    rclpy.spin(publisher)
    rclpy.shutdown()