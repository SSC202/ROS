import rclpy
from rclpy.node import Node

class Node_HelloWorld(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("Hello World")

def main(args = None):
    rclpy.init(args = args)
    node = Node_HelloWorld("node_build")
    rclpy.spin(node)
    rclpy.shutdown()