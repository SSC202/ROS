import rclpy 
from rclpy.node import Node
# 导入TF帧
from geometry_msgs.msg import TransformStamped
# 导入TF静态坐标发布器
from tf2_ros import StaticTransformBroadcaster
import transforms3d as tfs
import numpy as np

class TF_StaticBroadcaster_Node(Node):
    def __init__(self,name):
        global pitch_degree
        super().__init__(name)
        pitch_degree = 0.0
        # 构造静态坐标广播发布器
        self.tf_publisher = StaticTransformBroadcaster(self)
        self.tf_timer = self.create_timer(1.0,self.timer_callback)

    def timer_callback(self):
        global pitch_degree
        # 构造TF帧
        tf_frame = TransformStamped()
        tf_frame.header.stamp = self.get_clock().now().to_msg()
        ## 父坐标系
        tf_frame.header.frame_id = "base"
        ## 子坐标系
        tf_frame.child_frame_id = "camera"
        ## 坐标变换数据
        tf_frame.transform.translation.x = 2.0
        tf_frame.transform.translation.y = 5.0
        tf_frame.transform.translation.z = 4.0

        pitch_degree = pitch_degree + 1.0

        pitch = float(pitch_degree * (np.pi) / 180)
        roll = float(5 * (np.pi) / 180)
        yaw = float(35 * (np.pi) / 180)

        ## 转换为四元数
        q = tfs.euler.euler2quat(pitch,roll,yaw,"sxyz")
        w = q[0]
        x = q[1]
        y = q[2]
        z = q[3]
        tf_frame.transform.rotation.w = w
        tf_frame.transform.rotation.y = y
        tf_frame.transform.rotation.z = z
        tf_frame.transform.rotation.x = x

        self.tf_publisher.sendTransform(tf_frame)


def main(args=None):
    rclpy.init(args=args)
    node = TF_StaticBroadcaster_Node("TF_StaticBroadcaster_Node")
    rclpy.spin(node)
    rclpy.shutdown()
    