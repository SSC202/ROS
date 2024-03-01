import rclpy
from rclpy.node import Node
# 导入Laserscan消息数据
from sensor_msgs.msg import LaserScan

class LaserScan_Read_Node(Node):
    def __init__(self, name):
        super().__init__(name)
        self.subscriber = self.create_subscription(LaserScan,'/scan',self.subscriber_callback,10)

    def subscriber_callback(self,msg):
        distance = msg.ranges[180]
        self.get_logger().info(f'distance:{distance}')

def main(args=None):
    rclpy.init()
    node = LaserScan_Read_Node("LaserscanNode")
    rclpy.spin(node)
    rclpy.shutdown()


    