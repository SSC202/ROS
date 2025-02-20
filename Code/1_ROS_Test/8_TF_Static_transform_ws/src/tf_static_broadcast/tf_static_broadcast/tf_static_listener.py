import rclpy
from rclpy.node import Node
# 导入坐标变换监听器
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TF_Listener_Node(Node):
    def __init__(self,name):
        super().__init__(name)
        # 创建缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.timer = self.create_timer(1.0,self.timer_callback)
    
    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base','camera',now)
            print(trans)
        except TransformException as ex:
            print(f'Not found transform:{ex}')



def main(args=None):
    rclpy.init(args=args)
    node = TF_Listener_Node('TF_Listerner')
    rclpy.spin(node)
    rclpy.shutdown()
