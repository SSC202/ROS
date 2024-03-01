# ROS2 5_常用工具

## 1. Launch 启动管理工具

Launch 启动管理工具用于启动多个节点。

在 ROS 中，使用`.launch`文件启动多个节点，在 ROS2 中，可以使用Python编写Launch。

Launch 文件通常在工作空间的`install`文件夹下编写。编写步骤如下：

### 在功能包下添加Launch文件夹，编写`.launch.py`文件

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
        launch 内容描述函数
    """
    # 第一步：编写节点内容
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
```

### 拷贝文件

在功能包的`setup.py`下编写以下内容：

```python
from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'launch_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加以下代码段
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='e22750706642022@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "node_1 = launch_test.node_1:main",
            "node_2 = launch_test.node_2:main"
        ],
    },
)
```

此后编译可通过。



