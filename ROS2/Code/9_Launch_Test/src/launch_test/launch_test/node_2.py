import rclpy
from rclpy.node import Node

class Node_2(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("Hello World")

def main(args = None):
    rclpy.init(args = args)
    node = Node_2("node_2")
    rclpy.spin(node)
    rclpy.shutdown()