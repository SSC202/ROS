import rclpy
from rclpy.node import Node

class Node_1(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("Hello World")

def main(args = None):
    rclpy.init(args = args)
    node = Node_1("node_1")
    rclpy.spin(node)
    rclpy.shutdown()