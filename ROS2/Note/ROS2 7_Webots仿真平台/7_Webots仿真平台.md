# ROS2 7_Webots 仿真平台

机器人仿真其实就是通过软件来模仿硬件的特性，用于验证机器人算法、架构等。

> 1. 仿真可以解决真机资源不足;
> 2. 仿真可以保证环境的一致和稳定;
> 3. 仿真场景可以更加灵活，在测试机器人算法时可以通过仿真软件快速更改仿真环境，验证算法。
> 4. 仿真的主要缺陷就是仿不全，现实世界中的环境非常复杂，光线、材质、电磁干扰等等，仿真平台无法做到100%的仿真。

常用仿真平台：`Gazebo`，`WeBots`，`Ignition`，`Unity`

## 1. Webots 简介

Webots 是一个开源的三维移动机器人模拟器，它与gazebo类似都是ROS2中仿真环境，但是对于gazebo而言，需要比较复杂的配置，尤其是涉及到使用GPU的时候，对初学者并不友好。Webots在2018年以前是一款商业软件，商业软件的好处就是安装简单，在windows和Ubuntu上都可以实现一键安装，对用户很友好，Webots从2018年以后Webots进行了开源。

Webots支持C/C++、Python、MATLAB、Java、ROS和TCP/IP等多种方式实现模型的仿真控制。Webots内置了接近100种机器人模型，包括轮式机器人、人形机器人、爬行移动机器人、单臂移动机器人、双臂移动机器人、无人机、狗、飞艇等等。

## 2. Webots 安装

[Webots 软件安装官方文档](https://cyberbotics.com/doc/guide/installation-procedure#installing-the-debian-package-with-the-advanced-packaging-tool-apt)

[Webots-ROS2官方文档](https://docs.ros.org/en/iron/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html)

> Webots 在 2021a 版本后使用在线Github下载的方式进行环境导入，通常会导致导入环境时卡死。
>
> 使用 Webots 2021a ：[Webots 老版本下载](https://github.com/cyberbotics/webots/releases)
>
> ```shell
> sudo apt install ./webots_2021a_amd64.deb
> ```
>
> 此版本仅支持Ubuntu 20.04和18.04。
>
> > 此方法仍然存在问题：Webots - ROS2 接口是按照 Webots 最新版本安装的，使用示例时会产生版本不兼容问题。

1. Webots 软件安装（2023a）

首先，Webots 应该使用 `Cyberbotics.asc` 签名文件进行身份验证。使用以下命令安装 `Cyberbotics.asc` 签名文件：

```shell
sudo mkdir -p /etc/apt/keyrings
cd /etc/apt/keyrings
sudo wget -q https://cyberbotics.com/Cyberbotics.asc
```

然后，通过添加 `Cyberbotics` 存储库来配置 APT 包管理器。

```shell
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
sudo apt update
```

最后，安装Webots：

```shell
sudo apt install webots
```

2. Webots - ROS2 接口安装

使用以下命令行安装`webots_ros2`：

```shell
sudo apt-get install ros-<ROS_DISTRO>-webots-ros2
```

3. 示例运行

```shell
source /opt/ros/$ROS_DISTRO/setup.bash
export WEBOTS_HOME=/usr/local/webots
ros2 launch webots_ros2_universal_robot multirobot_launch.py
```

> 示例运行使用在线Github下载的方式进行环境导入，通常会导致导入环境时卡死。建议科学上网，多多等待，等待网络环境较好时运行示例。

## 3. Webots 入门

[Webots 官方手册](https://cyberbotics.com/doc/guide/introduction-to-webots)

### Webots 仿真基本概念

Webots 仿真工程由以下部分组成：

> 1. 定义一个或多个机器人及其环境的 Webots 世界文件 （`.wbt`）。 `.wbt` 文件有时依赖于外部 `PROTO` 文件 （`.proto`） 和纹理。
> 2. 用于上述机器人的一个或多个控制器程序（C/C++/Java/Python/MATLAB）。
> 3. 一个可选的物理插件，可用于修改 Webots 的常规物理行为（在 C/C++ 中）。

1. 世界文件

在 Webots 中，**世界是对机器人属性及其环境的3D描述。** 

它包含对每个对象的描述：位置、方向、几何形状、外观（如颜色或亮度）、物理属性、对象类型等。 世界被组织为分层结构，其中对象可以包含其他对象。 

例如，机器人可以包含两个轮子、一个距离传感器和一个本身包含摄像头的关节等。 

**世界文件不包含机器人的控制器代码；它仅指定每个机器人所需的控制器名称。**

世界保存在`.wbt`文件中。 `.wbt`文件存储在每个 Webots 项目的`worlds`子目录中。

2. 控制器文件

**控制器文件是控制世界文件中指定的机器人的计算机程序。 **

控制器文件可以用Webots支持的任何编程语言编写。 当仿真开始时，Webots 会启动指定的控制器，每个控制器都是一个单独的进程，并将控制器进程与机器人相关联。 请注意，多个机器人可以使用相同的控制器代码，但是将为每个机器人启动不同的进程。

每个控制器的源文件和二进制文件一起存储在控制器目录中。 控制器目录位于每个 Webots 项目的`controllers`子目录中。

> **Supervisor控制器**
>
> Supervisor控制器可以执行通常只能由人类操作员执行的操作，而不能由真正的机器人执行。 Supervisor控制器可以用支持的任何一种编程语言编写。 但是与常规控制器相比，Supervisor控制器将有权访问特权操作。 
>
> 特权操作包括模拟控制，例如将机器人移动到随机位置，对模拟进行视频捕获等。

### Webots 窗口介绍

![NULL](./assets/picture_1.jpg)

> **工具栏**
>
> 1. 场景树的第一个图标是隐藏/显示场景树模块，第二个图标是添加场景树的节点。
> 2. 3D视角第一个图标是返回初始视角，第二个图标是跳到一些常用视角，如俯视，正视等。
> 3. 世界文件控制的第一个图标是打开一个已有的世界，第二个是保存当前世界，第三个是从新加载这个世界。
> 4. 仿真控制的第一块是显示仿真世界的时间以及仿真世界的时间与现实世界的比例，后面的图标分别是初始化、单步前进，开始/暂停，加速，显示渲染/关闭渲染。
> 5. 仿真记录第一个图标是记录仿真视频，第二个是生成一个HTML5的animation，第三个是快照。

> **场景树**
>
> 场景树就体现了 Webots文件的基本构成逻辑——树状结构，场景树中的每一个文件都叫一个节点`Node`，节点下可能还有子节点。节点可以嵌套，还拥有属性`Field`。
>
> **场景树中的节点和物体，是一一对应的关系。**

### Webots 新建世界和控制器
