from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
        launch 内容描述函数
    """
    node_01 = Node(
        package = "launch_test",
        executable = "node_1"
    )
    node_02 = Node(
        package = "launch_test",
        executable = "node_2"
    )
    # 创建LaunchDescription对象用于描述launch文件
    launch_description = LaunchDescription([node_01,node_02])
    return launch_description