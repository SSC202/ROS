# ROS2 2_节点构建

## 1. Python 节点的构建方式

以下几个要素是必须的：

### A. 导入 rclpy 库

```python
import rclpy
from rclpy.node import Node
```

### B. 初始化 rclpy

```python
rclpy.init()
```

### C. 使得节点循环运行

```python
rclpy.spin(Node("node"))
```

如果是单独的 python 文件，使用以下指令开始运行：

```shell
python3 <name.py>
```

使用以下指令查看节点是否运行：

```shell
ros2 node list
```

此时终端会显示在 C 步中的节点名称。 

## 2. 节点的概念

ROS2中每一个节点也是只负责一个**单独的模块化的功能**。

> 比如一个节点负责控制车轮转动，一个节点负责从激光雷达获取数据、一个节点负责处理激光雷达的数据、一个节点负责定位等等。

### 节点的交互（通信）方式

节点之间有四种通信方式：**话题（Topic），服务（Service），参数（Param），动作（Action）**

### 节点的启动

使用以下指令启动节点

```shell
# package_name为包名，executable_name为节点名
ros2 run <package_name> <executable_name>
# 例子：ros2 run turtlesim turtlesim_node
```

### 节点相关CLI

```shell
# 查看节点列表
ros2 node list

# 查看节点信息，node_name为节点名
ros2 node info <node_name>

# 重映射节点名称
ros2 run <package_name> <node_name> --ros-args --remap __node:=<node_name>

# 运行时设置参数，param_name为参数名，param_value为参数值
ros2 run <package_name> <node_name> --ros-args -p <param_name>:=<param_value>
```

## 3. 工作空间/功能包

### 工作空间

工作空间就是一个文件夹，用于存放和调试功能包；

**一个工作空间下有多个功能包，一个功能包可以有多个节点存在。**

工作空间在创建时，应创建一个子文件夹`src`。

### 功能包

功能包用于存放节点。有三种功能包类型：

- ament_python，适用于python程序；
- cmake，适用于C++；
- ament_cmake，适用于C++程序,是cmake的增强版；

#### 安装获取

```shell
sudo apt install ros-<version>-package_name
```

#### 手动编译获取

手动编译需要下载源码然后进行编译生成相关文件。

手动编译之后，需要手动source工作空间的install目录。

#### 功能包CLI

```shell
# 功能包创建
ros2 pkg create <package-name>  --build-type  {cmake,ament_cmake,ament_python}  --dependencies <依赖名字>

# 列出可执行文件
ros2 pkg executables

# 列出功能包的可执行文件
ros2 pkg executables <node_name>

# 列出功能包
ros2 pkg list
```

## 4. Colcon 编译工具

 colcon 用于在工作空间对功能包进行编译（对应于ROS中的`catkin_make`）

```shell
# 在工作空间中编译
colcon build
```

编译通过后的目录结构：

```
.
├── build
├── install
├── log
└── src

4 directories, 0 files
```

构建完成后，在`src`同级目录应该会看到 `build` 、 `install` 和 `log` 目录:

> - `build` 目录存储的是中间文件。对于每个包，将创建一个子文件夹，在其中调用例如CMake
> - `install` 目录是每个软件包将安装到的位置。默认情况下，每个包都将安装到单独的子目录中。
> - `log` 目录包含有关每个colcon调用的各种日志信息。

## 5. 功能包的节点创建

### 建立工作空间

```shell
mkdir /1_node_build/src	# 工作空间名为1_node_build，注意创建src文件夹
cd /1_node_build		# 进入工作空间
```

### 在工作空间内创建功能包

```shell
ros2 pkg create node_build --build-type ament_python --dependencies rclpy
```

创建完毕后的目录如下：

```
.
├── node_build				# 以功能包命名的文件夹下用于存放节点文件
│   └── __init__.py			# 初始化文件
├── package.xml
├── resource
│   └── example_py
├── setup.cfg
├── setup.py				# 配置文件
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py

3 directories, 8 files
```

### 编写源码

```python
import rclpy
from rclpy.node import Node

def main(args = None):
    # 初始化客户端
    rclpy.init(args=args)
    # 节点创建
    node = Node("node_build")
    # 日志输出（节点）
    node.get_logger().info("Hello World")
    # 节点循环运行
    rclpy.spin(node)
    # 关闭客户端，节点停止运行前无法到达
    rclpy.shutdown()
```

### 修改 setup.py 文件

```python 
    entry_points={
        'console_scripts': [
            "节点名 = 功能包名.节点文件名:main"
        ],
    },
```

### Colcon 编译

回到工作空间中：

```shell
colcon build
```

### 加载环境变量

```shell
source install/setup.bash
```

### 运行节点

```shell
ros2 run node_build node_build 
```

## 6. 对象化节点

将节点作为对象进行运行，每一类节点视为一个类，使用时实例化即可。

```python
import rclpy
from rclpy.node import Node

# 继承于Node类的节点Node_HelloWorld
class Node_HelloWorld(Node):
    def __init__(self,name):
        super().__init__(name)
        # 节点运行的任务
        self.get_logger().info("Hello World")
        

def main(args = None):
    # 初始化客户端
    rclpy.init(args=args)
    # 节点创建
    node = Node_HelloWorld("node_build")
    # 节点循环运行
    rclpy.spin(node)
    # 关闭客户端，节点停止运行前无法到达
    rclpy.shutdown()
```

