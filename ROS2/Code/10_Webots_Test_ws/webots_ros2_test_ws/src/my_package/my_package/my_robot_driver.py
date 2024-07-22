import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.05
WHEEL_RADIUS = 0.04

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__wheel1 = self.__robot.getDevice('wheel1')
        self.__wheel2 = self.__robot.getDevice('wheel2')
        self.__wheel3 = self.__robot.getDevice('wheel3')
        self.__wheel4 = self.__robot.getDevice('wheel4')

        self.__wheel1.setPosition(float('inf'))
        self.__wheel1.setVelocity(0)

        self.__wheel2.setPosition(float('inf'))
        self.__wheel2.setVelocity(0)

        self.__wheel3.setPosition(float('inf'))
        self.__wheel3.setVelocity(0)

        self.__wheel4.setPosition(float('inf'))
        self.__wheel4.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__wheel1.setVelocity(command_motor_left)
        self.__wheel2.setVelocity(command_motor_left)
        self.__wheel3.setVelocity(command_motor_right)
        self.__wheel4.setVelocity(command_motor_right)