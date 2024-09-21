# ROS2 13_ROS1_Bridge使用

`ros1_bridge` 是一个桥接工具，用于让 ROS1 和 ROS2 之间进行双向通信。

## 1. `ros1_bridge` 安装和运行

1. 安装 `ros1_bridge`

   ```shell
   $ sudo apt install ros-foxy-ros1-bridge
   ```

2. 设置 ROS1 和 ROS2 环境

   ```shell
   $ source /opt/ros/noetic/setup.bash
   $ source /opt/ros/foxy/setup.bash
   ```

   确保已经在终端中运行了这些命令，或者将它们添加到 `.bashrc` 文件以使其永久生效。

3. 运行 `ros1_bridge` 以在ROS1和ROS2之间传递数据

   ```shell
   # 启动ROS1 Bridge，并使用--bridge-all-topics标志来启用所有主题的桥接功能
   $ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
   
   # 如果仅需要特定类型话题传输
   $ ros2 run ros1_bridge static_bridge --ros1-bridge-std-msgs-String std_msgs/String
   
   # 如果仅需要特定话题传输
   $ ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics /chatter  std_msgs/String
   
   ---
   
   # 需要打开 roscore
   $ roscore
   ```

4. 静态桥接配置文件

   可以通过一个配置文件来定义你想要桥接的特定消息和主题。`ros1_bridge` 支持 YAML 配置文件来静态定义桥接规则。

   ```yaml
   topics:
     - topic: /chatter
       type: std_msgs/String
     - topic: /my_custom_topic
       type: custom_msgs/MyCustomMessage
   
   ```
	```shell
	$ ros2 run ros1_bridge static_bridge --config-file bridge_topics.yaml
	```


## 2. 验证通信传输

1. ROS1 发布消息，ROS2 接收消息

   ```shell
   # 加载 ROS1 环境
   $ source /opt/ros/noetic/setup.bash
   
   # 消息发布
   $ rostopic pub /chatter std_msgs/String "data: 'Hello from ROS1'" --once
   
   ---
   
   # 加载 ROS2 环境
   $ source /opt/ros/foxy/setup.bash
   
   # 消息接收
   $ ros2 topic echo /chatter
   ```

2. ROS2 发布消息，ROS1 接收消息

   ```shell
   # 加载 ROS2 环境
   $ source /opt/ros/foxy/setup.bash
   
   # 消息发布
   $ ros2 topic pub /chatter std_msgs/String "data: 'Hello from ROS2'" --once
   
   ---
   
   # 加载 ROS1 环境
   $ source /opt/ros/noetic/setup.bash
   
   # 消息接收
   $ rostopic echo /chatter
   ```

   