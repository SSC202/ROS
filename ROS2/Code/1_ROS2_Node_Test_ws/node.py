# 导入 rclpy 库
import rclpy
from rclpy.node import Node

# 调用初始化函数
rclpy.init()
# 循环运行节点
rclpy.spin(Node("node"))