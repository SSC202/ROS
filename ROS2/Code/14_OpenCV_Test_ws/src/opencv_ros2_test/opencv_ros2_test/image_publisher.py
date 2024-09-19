import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(0)  # 打开摄像头
        self.bridge = CvBridge()  # 用于将OpenCV图像转换为ROS 2消息

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # 处理图像（例如，转为灰度图）
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 使用CvBridge将OpenCV图像转为ROS 2的Image消息
            image_msg = self.bridge.cv2_to_imgmsg(gray_frame, encoding='mono8')

            # 发布处理后的图像
            self.publisher_.publish(image_msg)
            self.get_logger().info('Publishing image')

    def destroy(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    image_publisher.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
