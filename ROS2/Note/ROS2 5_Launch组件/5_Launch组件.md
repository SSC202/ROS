# ROS2 5_Launch组件

## 1. Launch 启动管理工具

Launch 启动管理工具用于启动多个节点。

在 ROS 中，使用`.launch`文件启动多个节点，在 ROS2 中，可以使用Python编写Launch。

Launch 文件通常在工作空间的`install`文件夹下编写。编写步骤如下：

### 在功能包下添加Launch文件夹，编写`.launch.py`文件

1. 编写需要启动的节点内容

```python
"""
	Node 对象创建函数
	package：要启动节点的包名称。这是必需的参数。
	executable：要运行的可执行文件的名称。这是必需的参数。
	name：节点的名称。如果未指定，系统将为节点自动生成一个唯一的名称
	namespace：节点的命名空间。这允许在ROS图中组织节点。如果未指定，节点将处于全局命名空间。
	output：节点的输出方式。可以是“log”、“screen”或“both”。默认是“screen”，将节点的输出打印到屏幕上。使用“log”将节点的输出记录到ROS的日志系统中。
	parameters：要传递给节点的参数。这是一个字典，其中键是参数名，值是参数值。
	remappings：节点的重映射。这是一个字典，用于重映射节点的输入和输出话题、服务和参数。键是原始名称，值是新名称。
	arguments：传递给可执行文件的额外参数。这是一个字符串列表，其中每个字符串代表一个参数。
"""
Node()
```

2. 使用`LaunchDescription`对象管理启动

`LaunchDescription`对象是一个容器，用于存储将要启动的节点、参数等。它可以包含多个`Node`、参数等，以描述启动文件的整体结构。

> 1. **添加节点和其他操作**: 可以通过将Node、其他Action等添加到LaunchDescription对象来描述要在启动文件中启动的节点和执行的其他操作。
> 2. **管理节点间的依赖关系**: LaunchDescription对象允许您指定节点之间的依赖关系，确保节点按照正确的顺序启动。
> 3. **描述启动文件的整体结构**: LaunchDescription对象提供了一种组织和管理启动文件中各种操作的方式，使得整个启动文件的结构清晰可见。
> 4. **支持嵌套结构**: LaunchDescription对象可以嵌套使用，以便更好地组织和描述复杂的启动文件结构。

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
        launch 内容描述函数
    """
    # 编写节点内容
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
    # 功能包名称
    name=package_name,
    # 功能包版本号
    version='0.0.0',
    # 指定包含的包
    packages=find_packages(exclude=['test']),
    # 要安装的数据文件
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加launch文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='e22750706642022@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # 指定项目的入口点，例如命令行脚本
    entry_points={
        'console_scripts': [
            "node_1 = launch_test.node_1:main",
            "node_2 = launch_test.node_2:main"
        ],
    },
)
```

此后编译可通过。



