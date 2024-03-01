# ROS2 3_节点通信

## 1. 话题通信（Topic）

### 1.1 话题通信原理

#### 话题通信模型

话题通信采取的是**订阅发布**模型。

![NULL](picture_1.jpg)

![NULL](picture_2.jpg)

![NULL](picture_3.jpg)

![NULL](picture_4.jpg)

![NULL](picture_5.jpg)

话题通信可以是`1对1`的，还可以是`1对n`，`n对1`，`n对n`的。

#### 消息接口

为了方便发送者和接收者进行数据的交换，ROS2在数据传递时做好了消息的序列化和反序列化，而且ROS2的消息序列化与反序列化通信是可以做到跨编程语言、跨平台和跨设备之间的。

定义好消息接口后，ROS2会根据消息接口内容生成不同语言的接口类，在不同编程语言中调用相同的类即可实现无感的消息序列化和反序列化。

**同一个话题，所有的发布者和接收者必须使用相同消息接口**。

#### 话题CLI

```shell
# 列举出当前的活动话题
ros2 topic list

# 打印话题内的内容
ros2 topic echo <topic_name>

# 查看主题信息
ros2 topic info <topic_name>

# 查看话题内的消息类型
ros2 interface show <msg_type>

# 手动命令发布
ros2 topic pub <topic_name> <msg_type> <msg_value>
```

### 1.2 话题通信实现

#### 发布方（Publisher）实现

> 1. 创建发布者
>
> ```python
> """
> 	发布者创建函数
> 	第一个参数：消息数据类型
> 	第二个参数：消息话题Topic
> 	第三个参数：消息缓冲区个数，如果溢出直接舍弃
> """
> create_publisher()
> ```
>
> 2. 创建定时器实现定时发布
>
> ```python
> """
> 	定时器创建函数
> 	第一个参数：定时器延时数（s）
> 	第二个参数：定时器回调函数
> """
> create_timer()
> ```
>
> 3. 在定时器回调函数内实现消息的创建（使用数据类型类创建），打包（实例化对象中有`data`成员），发送
>
> ```python
> """
> 	消息发送函数（为publisher的成员函数）
> 	参数：消息对象
> """
> publish()
> ```

```python
import rclpy
from rclpy.node import Node
# 导入发布者需要的消息接口
from std_msgs.msg import String


class Publisher(Node):
    def __init__(self,name):
        super().__init__(name)
        # 创建发布者 publisher
        self.publisher = self.create_publisher(String,"command",10)
        # 创建定时器以定时发布
        self.timer = self.create_timer(10,self.timer_callback)
    
    # 发布方本身无需回调函数，但是可以通过定时器定时发布
    def timer_callback(self):
        # 消息构建
        msg = String()
        msg.data = "Hello Listener"
        # 发布消息
        self.publisher.publish(msg)
        self.get_logger().info(f'Send Command:{msg.data}')

    

    
def main(args = None):
    rclpy.init()
    publisher = Publisher("Publisher")
    rclpy.spin(publisher)
    rclpy.shutdown()
```

#### 接收方（Subscriber）实现

> 1. 创建接收者
> ```python
> """
> 	接收者创建函数
> 	第一个参数：消息数据类型
> 	第二个参数：消息话题Topic
> 	第三个参数：接受回调函数
> 	第四个参数：消息缓存区
> """
> create_subscriber()
> ```
> 2. 创建接受回调函数：接受回调函数中需要接受参数`msg`用于进行消息处理。

```python
import rclpy
from rclpy.node import Node
# 导入订阅者需要的消息接口
from std_msgs.msg import String


class Listener(Node):
    def __init__(self,name):
        super().__init__(name)
        # 创建订阅者，由于为触发式接收，无需定时器
        self.listener = self.create_subscription(String,"command",self.command_callback,10)
    
    def command_callback(self,msg):
        t = 0.0
        if msg.data == "Hello Listener":
            t = t + 1.0
        self.get_logger().info(f'{t}:Get Msg:{msg.data}')
    
def main(args = None):
    rclpy.init()
    listener = Listener("Listener")
    rclpy.spin(listener)
    rclpy.shutdown()
```

#### 总结

![NULL](picture_6.jpg)

## 2. 服务通信（Service）

### 2.1 服务通信原理

#### 服务通信模型

服务通信采取**请求响应模型。**

![NULL](picture_7.jpg)

客户端发送请求给服务端，服务端可以根据客户端的请求做一些处理，然后返回结果给客户端。

- **同一个服务（名称相同）有且只能有一个节点来提供**。
- **同一个服务可以被多个客户端调用**。

即服务通信是多对一的。

#### 服务通信CLI

```shell
# 查看服务列表
ros2 service list

# 作为客户端向服务端发送请求
## service_type 为服务数据类型名
## service_value 为发送请求数据
ros2 service call <service_name> <service_type> <service_value>

# 查看服务数据类型
ros2 service type <service_name> 

# 查找使用该接口的服务
ros2 service find <service_type>
```

### 2.2 服务通信实现

#### 客户端（Client）实现

客户端要完成两个任务：发送请求和解析响应。

> 1. 创建客户端
>
> ```python
> """
> 	客户端创建函数
> 	第一个参数：服务通信消息接口
> 	第二个参数：服务名称service
> """
> create_client()
> ```
>
> 2. 创建请求发送函数
>
> 请求发送函数应该有请求中需要的所有数据作为形参；
>
> 首先需要确认节点是否运行和***服务端是否启动***：
>
> ```python
> """
> 	等待服务端启动函数（客户端成员函数）
> 	第一个参数：等待时间
> """
> wait_for_service()
> ```
>
> 然后进行请求数据的组织，在服务消息接口中有`Request`类，使用此类进行实例化。
>
> 最后发送请求并等待响应：
>
> ```python
> self.client.call_async(request).add_done_callback(self.responce_callback)
> # 第一个函数用于发送请求，第二个函数用于等待响应的回调
> ```
> 3. 创建接受回调函数
> 
> 应使用接受数据作为形参，注意，接收数据下有result类，这个类真正包含响应数据。


```python
import rclpy
from rclpy.node import Node
# 导入服务通信的数据类型
from example_interfaces.srv import AddTwoInts

class Client(Node):
    def __init__(self,name):
        super().__init__(name)
        self.client = self.create_client(AddTwoInts,"request")
        self.send_request(3,6)

    # 由于为触发式返回，使用接受回调函数
    def responce_callback(self,result_future):
        responce = result_future.result()
        self.get_logger().info(f"Responce:{responce.sum}")

    def send_request(self,num1,num2):
        # 等待节点和服务端开始运行
        while rclpy.ok() and self.client.wait_for_service(1) == False:
            self.get_logger().info("Wait for service...")

        # 组织请求数据
        request = AddTwoInts.Request()
        request.a = num1
        request.b = num2
        # 发送请求并等待回调
        self.client.call_async(request).add_done_callback(self.responce_callback)

def main(args = None):
    rclpy.init(args=args)
    node = Client("client")
    rclpy.spin(node)
    rclpy.shutdown()
```

#### 服务端（Server）实现

> 服务端主要实现接受请求并进行响应的任务
>
> 1. 创建服务端
>
> ```python
> """
> 	服务端创建函数
> 	第一个参数：服务数据类型
> 	第二个参数：服务名service
> 	第三个参数：接受回调函数
> """
> create_service()
> ```
>
> 2. 创建接受回调函数
>
> 接受回调函数应有请求和响应两部分作为形参
>
> 响应作为返回值。

```python
import rclpy
from rclpy.node import Node
# 导入服务通信消息接口
from example_interfaces.srv import AddTwoInts

class Server(Node):
    def __init__(self,name):
        super().__init__(name)
        self.server = self.create_service(AddTwoInts,"request",self.request_callback)

    # 由于为触发式响应，使用接受请求回调函数
    def request_callback(self,request,responce):
        self.get_logger().info(f"Get request:{request.a},{request.b}")
        responce.sum = request.a + request.b
        return responce
    
def main(args = None):
    rclpy.init(args=args)
    node = Server("Server")
    rclpy.spin(node)
    rclpy.shutdown()
```

#### 总结

![NULL](picture_8.jpg)

### 2.3 自定义消息接口（Interface）

**接口是一种消息规范。**

ROS2 中有**动作接口，话题接口和服务接口。**

#### 接口的基本数据类型

```shell
bool
byte
char
float32,float64
int8,uint8
int16,uint16
int32,uint32
int64,uint64
string
```

#### 通过接口生成代码

ROS2 的 IDL 模块可以通过接口生成代码，产生头文件（模块），从而在程序里导入并使用。

![NULL](picture_9.jpg)

#### 接口的定义

- 话题接口

```
<type> <name>
<type> <name>
...
```

- 服务接口

```
# request
<type> <name>
---
# responce
<type> <name>
```

#### 接口CLI

```shell
# 接口列表查看
ros2 interface list

# 查看接口内容
ros2 interface show <interface_type>
```

#### 接口包的创建

```shell
# 依赖于 rosidl 创建接口包
ros2 pkg create <self_interface> --build-type ament_cmake --dependencies rosidl_default_generators geometry_msgs
```

在创建的接口包中，新建接口文件夹`msg`，`srv`，`action`，在接口文件夹下建立接口文件`xxx.msg`，`xxx.srv`，`xxx.action`（注意，首字母必须大写）。

- 修改`CMakeLists.txt`

```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
# 添加下面的内容
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/RobotMove.srv"
  DEPENDENCIES geometry_msgs
)
```

- 修改`package.xml`

```xml
  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rosidl_default_generators</depend>
  <depend>geometry_msgs</depend>
  
  <member_of_group>rosidl_interface_packages</member_of_group> #添加这一行

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
```

- 在`<ws_name>/<install>/<interface_name>/local/lib/python3.10/`下看到Python模块文件。

```python
"""
    客户端有两个任务：
    1. 机器人运动指令的发布
    2. 机器人实时状态接受
"""
import rclpy
from rclpy.node import Node
# 导入话题通信接口
from self_interface.msg import RobotStatus
# 导入服务通信接口
from self_interface.srv import RobotCommand

class User_Node(Node):
    def __init__(self,name):
        super().__init__(name)
        # 创建订阅方
        self.robotstatus_subscriber = self.create_subscription(RobotStatus,"robotstatus",self.robotstatus_get_callback,10)
        # 创建客户端
        self.robotmove_client = self.create_client(RobotCommand,"robotmove")

    # 订阅接受回调函数
    def robotstatus_get_callback(self,msg):
        self.get_logger().info(f"Robot Status:{msg.status};Pose:{msg.pose}")

    # 服务端响应回调函数
    def robotmove_command_callback(self,result_future):
        responce = result_future.result()
        self.get_logger().info(f"Moving Done!Consquence:{responce.pose}")

    # 服务端发送函数
    def robotmove_send_command(self,distance):
        while rclpy.ok() and self.robotmove_client.wait_for_service(1) == False:
            self.get_logger().info("Wait for service...")
        
        request = RobotCommand.Request()
        request.distance = distance
        self.get_logger().info(f"Command to move:{distance}")
        self.robotmove_client.call_async(request).add_done_callback(self.robotmove_command_callback)
    
def main(args = None):
    rclpy.init(args=args)
    node = User_Node("user")
    node.robotmove_send_command(5.0)
    rclpy.spin(node)
    rclpy.shutdown()
```

```python
"""
    机器人端有两个任务：
    1. 机器人状态消息的发布(Publisher)
    2. 接受移动请求并回传当前位置
"""
import rclpy
from rclpy.node import Node
# 消息需要的数据类型
from self_interface.msg import RobotStatus
# 服务需要的数据类型
from self_interface.srv import RobotCommand
# 导入机器人类
from self_interface_test.robot import Robot

class Robot_Node(Node):
    def __init__(self,name):
        super().__init__(name)
        # 创建机器人
        self.robot = Robot()
        # 创建服务端
        self.robotmove_server = self.create_service(RobotCommand,"robotmove",self.move_request_callback)
        # 创建发布者
        self.robotstatus_publisher = self.create_publisher(RobotStatus,"robotstatus",10)
        # 定时消息发布定时器
        self.robotstatus_timer = self.create_timer(1,self.robotstatus_timmer_callback)

    # 定时器回调函数
    def robotstatus_timmer_callback(self):
        status_msg = RobotStatus()
        status_msg.status = self.robot.get_status()
        status_msg.pose = self.robot.get_current_pose()

        self.robotstatus_publisher.publish(status_msg)

    # 服务端回调函数
    def move_request_callback(self,request,responce):
        self.robot.move_robot(request.distance)
        responce.pose = self.robot.get_current_pose()
        return responce
        
def main(args = None):
    rclpy.init(args=args)
    node = Robot_Node("robot")
    rclpy.spin(node)
    rclpy.shutdown()
```

## 3. 参数通信（Param）

### 参数CLI

```shell
# 查看参数
ros2 param list


# 查看参数详细信息
ros2 param describe <node_name> <param_name>

# 查看参数值
ros2 param get <node_name> <param_name>

# 设置参数
ros2 param set <node_name> <param_name>
```

### 参数通信原理

ROS2 的参数由键值对构成。

ROS2 参数类型支持以下几种：

> 1. bool 和bool[]，布尔类型用来表示开关
>
> 2. int64 和int64[]，整形表示一个数字
>
> 3. float64 和float64[]，浮点型，可以表示小数类型的参数值
>
> 4. string 和string[]，字符串
> 5. byte[]，字节数组，用来表示图片，点云数据等信息

```python
import rclpy
from rclpy.node import Node

class Param_Node(Node):
    def __init__(self,name):
        super().__init__(name)
        # 声明参数
        self.declare_parameter("log_value",0)
        # 获取参数
        log_value = self.get_parameter("log_value").value
        # 设置参数
        self.get_logger().set_level(log_value)
        # 定时修改
        self.timer = self.create_timer(1,self.timer_callback)

    def timer_callback(self):
        # 获取参数
        log_value = self.get_parameter("log_value").value
        # 设置参数
        self.get_logger().set_level(log_value)
        print(
            f"========================{log_value}=============================")
        self.get_logger().debug("我是DEBUG级别的日志，我被打印出来了!")
        self.get_logger().info("我是INFO级别的日志，我被打印出来了!")
        self.get_logger().warn("我是WARN级别的日志，我被打印出来了!")
        self.get_logger().error("我是ERROR级别的日志，我被打印出来了!")
        self.get_logger().fatal("我是FATAL级别的日志，我被打印出来了!")

def main(args = None):
    rclpy.init(args=args)
    node = Param_Node("param_node")
    rclpy.spin(node)
    rclpy.shutdown()
```

