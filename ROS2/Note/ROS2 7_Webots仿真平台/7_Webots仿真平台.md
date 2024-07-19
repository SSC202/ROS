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
> 场景树就体现了 Webots 文件的基本构成逻辑——树状结构，场景树中的每一个文件都叫一个节点`Node`，节点下可能还有子节点。节点可以嵌套，还拥有属性`Field`。
>
> **场景树中的节点和物体，是一一对应的关系。**

### Webots 新建世界和控制器

<font color=LightGreen>1. 新建世界</font>

世界是一个包含物体信息的文件，例如物体的位置、它们的外观、它们如何相互作用、天空的颜色以及重力、摩擦力、物体质量等的定义。 它定义了仿真的初始状态。 

不同的物体称为节点，并在场景树中按层次结构组织。 因此，一个节点可以包含子节点。

> 1. 通过单击工具栏的按钮暂停当前仿真。
> 2. 从`File / New / New Project Directory...`中新建项目：
>
> > 1. 命名项目目录
> > 2. 命名世界文件
> > 3. 单击所有复选框，包括默认情况下未勾选的`Add a rectangle arena`
> > 
>
> ![NULL](./assets/picture_2.jpg)


存储在世界文件中的 Webots 节点以称为场景树的树结构进行组织。 

可以在主窗口的两个子窗口中查看场景树：3D 视图（位于主窗口的中心）是场景树的 3D 表示，场景树视图（左侧）是场景树的分层表示。 场景树视图是可以修改节点和字段的地方。

> 新建世界后应当存在以下的节点：
>
> - `WorldInfo`：包含仿真的全局参数。
> - `ViewPoints`：定义主视点相机参数。
> - `TexturedBackground`：定义场景的背景。
> - `TexturedBackgroundLight`：定义与上述背景关联的光源。

通过添加场景树节点图标以添加障碍物：

![NULL](./assets/picture_3.jpg)

用鼠标选中地图中的障碍物，会出现三个颜色的的箭头，用鼠标点中箭头可实现障碍物的拖动，也可以通过左侧的节点属性栏设置障碍物的尺寸、位置、质量等参数。

![NULL](./assets/picture_4.jpg)

<font color=LightGreen>2. 添加机器人</font>

首先以自带的e-puck电子冰球机器人为例，e-puck是一个小型机器人，具有差速器轮，10个LED和多个传感器，包括8个距离传感器和一个摄像头。

当一个 Webots 世界被修改以保存时，首先暂停仿真并重新加载到其初始状态是必不可少的，即主工具栏上的虚拟时间计数器应显示 `0：00：00：000`。 否则，在每次保存时，每个 3D 对象的位置都会累积错误。 因此，对世界的任何修改都应按以下顺序执行：**暂停、重置、修改和保存仿真**。

<font color=LightGreen>3. 控制器创建</font>

**控制器是定义机器人行为的程序。** Webots控制器可以用以下编程语言编写：C，C++，Java，Python，MATLAB，ROS等。 C、C++ 和 Java 控制器需要先编译，然后才能作为机器人控制器运行。 Python 和 MATLAB 控制器是解释型语言，因此它们无需编译即可运行。

> 1. 首先创建控制器`File / New / New Robot Controller...`
> 2. 选择编程语言。
> 3. 如果选择C、C++ 和 Java，编写文件完成后必须进行**保存->编译->保存->运行仿真**，其余语言无需编译。
> 4. 运行仿真前**将机器人的控制器与编写的控制器相关联**。（`controller`属性）

示例程序，将机器人运动到固定位置：

```c
#include <webots/robot.h>

// Added a new include file
#include <webots/motor.h>

#define TIME_STEP 64

int main(int argc, char **argv) {
     wb_robot_init();

     // get the motor devices
     WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
     WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
     // set the target position of the motors
     wb_motor_set_position(left_motor, 10.0);
     wb_motor_set_position(right_motor, 10.0);

     while (wb_robot_step(TIME_STEP) != -1);

     wb_robot_cleanup();

     return 0;
}
```

```python
from controller import Robot, Motor

TIME_STEP = 64

# create the Robot instance.
robot = Robot()

# get the motor devices
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
# set the target position of the motors
leftMotor.setPosition(10.0)
rightMotor.setPosition(10.0)

while robot.step(TIME_STEP) != -1:
   pass
```

### Webots 世界

<font color=LightGreen>1. Solid 节点</font>

Solid 节点表示刚体。 Webots 的物理引擎仅用于模拟刚体。 在设计仿真时，一个重要的步骤是将各种实体分解为单独的刚体。

![NULL](./assets/picture_5.jpg)

要定义刚体，必须创建一个 Solid 节点。在该节点内，可以根据刚体的特性设置不同的子节点。Solid 节点的图形描述由 children 列表的 Shape 节点定义。碰撞边界在`boundingObject` 字段中定义。图形描述和碰撞边界通常但不一定相同。最后，物理属性定义对象属于动态环境还是静态环境。定义物理属性时，需要定义`boundingObject`属性。Geometry 代表任何类型的几何形状。如果物理属性为NULL，刚体将被冻结。

> **创建一个刚体**
>
> 1. 在世界的地板上添加一个 Solid 节点；
>
> ![NULL](./assets/picture_6.jpg)
>
> 2. 向children列表中加入Shape节点，添加刚体的纹理和形状：
>
> ![NULL](./assets/picture_7.jpg)
>
> 3. 添加碰撞边界和物理属性
>
> ![NULL](./assets/picture_8.jpg)
>
> 4. 更多的属性请参考 Webots 参考手册。

**`DEF-USE`机制**

`DEF-USE` 机制允许在一个地方定义一个节点，然后在场景树的其他地方重复使用该定义。这对于避免在世界文件中重复相同的节点很有用。此外，它还允许用户同时修改多个对象。

首先用 `DEF` 字符串标记一个节点。然后可以使用 `USE` 关键字在其他地方重复使用此节点的副本。只有 `DEF` 节点的字段可以编辑，`USE` 的字段从 `DEF` 节点继承，不能更改。此机制取决于世界文件中节点的顺序。`DEF` 节点应在任何相应的 `USE` 节点之前定义。

![NULL](./assets/picture_9.jpg)

> 尽可能在 Shape 级别而不是 Geometry 级别使用 `DEF-USE` 机制。实际上，在Solid 节点字段中添加中间 Shape 节点更为方便。

### Webots 控制器

