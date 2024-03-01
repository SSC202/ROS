import rclpy
from rclpy.node import Node
# 导入服务通信消息接口
from example_interfaces.srv import AddTwoInts

class Server(Node):
    def __init__(self,name):
        super().__init__(name)
        self.server = self.create_service(AddTwoInts,"request",self.request_callback)

    # 由于为触发式响应，使用接受请求回调函数
    def request_callback(self,request,responce):
        self.get_logger().info(f"Get request:{request.a},{request.b}")
        responce.sum = request.a + request.b
        return responce
    
def main(args = None):
    rclpy.init(args=args)
    node = Server("Server")
    rclpy.spin(node)
    rclpy.shutdown()