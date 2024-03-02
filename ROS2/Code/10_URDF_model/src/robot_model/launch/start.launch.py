import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'robot_model'
    urdf_name = "robot_model.urdf"

    ld = LaunchDescription()
    # FindPackageShare 类是 ROS 2 中用于查找包共享文件夹路径的工具。在 ROS 2 中，一个包可以包含可供其他包使用的共享资源，例如配置文件、URDF 模型、启动文件等。这些共享资源通常存储在包的 share 文件夹中。
    # 搜索指定包的共享文件夹路径，并返回找到的路径。
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    # pkg_share 是找到的包的共享文件夹路径，而 urdf_name 是定义的 URDF 模型文件的名称。通过使用 os.path.join() 方法，将这两部分连接起来，形成了 URDF 模型文件的完整路径。
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
	# 模型发布节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
        )
	# 关节数据发布节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_model_path]
        )
	# rviz2 节点
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld