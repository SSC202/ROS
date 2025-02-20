import rclpy
from rclpy.node import Node
# 导入服务通信的数据类型
from example_interfaces.srv import AddTwoInts

class Client(Node):
    def __init__(self,name):
        super().__init__(name)
        self.client = self.create_client(AddTwoInts,"request")
        self.send_request(3,6)

    # 由于为触发式返回，使用接受回调函数
    def responce_callback(self,result_future):
        responce = result_future.result()
        self.get_logger().info(f"Responce:{responce.sum}")

    def send_request(self,num1,num2):
        # 等待节点和服务端开始运行
        while rclpy.ok() and self.client.wait_for_service(1) == False:
            self.get_logger().info("Wait for service...")

        # 组织请求数据
        request = AddTwoInts.Request()
        request.a = num1
        request.b = num2
        # 发送请求并等待回调
        self.client.call_async(request).add_done_callback(self.responce_callback)

def main(args = None):
    rclpy.init(args=args)
    node = Client("client")
    rclpy.spin(node)
    rclpy.shutdown()