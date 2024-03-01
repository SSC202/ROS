import rclpy
from rclpy.node import Node

class Param_Node(Node):
    def __init__(self,name):
        super().__init__(name)
        # 声明参数
        self.declare_parameter("log_value",0)
        # 获取参数
        log_value = self.get_parameter("log_value").value
        # 设置参数
        self.get_logger().set_level(log_value)
        # 定时修改
        self.timer = self.create_timer(1,self.timer_callback)

    def timer_callback(self):
        # 获取参数
        log_value = self.get_parameter("log_value").value
        # 设置参数
        self.get_logger().set_level(log_value)
        print(
            f"========================{log_value}=============================")
        self.get_logger().debug("我是DEBUG级别的日志，我被打印出来了!")
        self.get_logger().info("我是INFO级别的日志，我被打印出来了!")
        self.get_logger().warn("我是WARN级别的日志，我被打印出来了!")
        self.get_logger().error("我是ERROR级别的日志，我被打印出来了!")
        self.get_logger().fatal("我是FATAL级别的日志，我被打印出来了!")

def main(args = None):
    rclpy.init(args=args)
    node = Param_Node("param_node")
    rclpy.spin(node)
    rclpy.shutdown()
    
