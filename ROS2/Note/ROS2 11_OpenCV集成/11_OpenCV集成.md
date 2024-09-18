# ROS2 11_OpenCV 集成

<font color=LightGreen>1. 确认安装OpenCV</font>

```shell
$ python3 -c "import cv2; print(cv2.__version__)"
```

<font color=LightGreen>2. 创建功能包，编写以下代码</font>

```python
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
```

- `cvbridge` 包：ROS 2的图像消息（`sensor_msgs/Image`）与OpenCV的图像格式不同，需要使用`CvBridge`来转换。

```python
# 从ROS 2图像消息转换为OpenCV图像
cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

# 从OpenCV图像转换为ROS 2图像消息
image_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
```

- 可以使用`rqt_image_raw`查看发布图像：

```shell
$ ros2 run rqt_image_view rqt_image_view
```

选择`/image_raw`话题，将看到处理后的图像流。
