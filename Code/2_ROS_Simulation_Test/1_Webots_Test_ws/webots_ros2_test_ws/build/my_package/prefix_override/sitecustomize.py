import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ssc/Desktop/webots_ros2_test_ws/install/my_package'
