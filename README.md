# ROS2

ROS2 学习笔记

ROS（Robot Operating System）是一个适用于机器人的开源框架，这个框架把原本松散的零部件耦合在了一起，为它们提供了通信架构。ROS虽然叫做操作系统，但是它却要安装在如Linux这种操作系统上才能运行。它的作用只是连接真正的操作系统（如Linux）和使用者自己开发的ROS应用程序（比如自动驾驶的感知、规划、决策等模块），所以它也算是个中间件，在基于ROS的应用程序之间建立起了沟通的桥梁。

ROS 主要分为 ROS1 和 ROS2 两大版本，ROS2 相比与 ROS1 改进了系统架构，同时延续了 ROS1 的基本概念。

## 使用此库的前置条件

1. Linux 操作系统(特别是Ubuntu)的基本使用；
2. 查阅官方文档的能力。

## 对应Note和相关Code说明

| Note                                                         | Note说明                            | Code                                                         |
| ------------------------------------------------------------ | ----------------------------------- | ------------------------------------------------------------ |
| [1_环境配置](https://github.com/SSC202/ROS/tree/main/ROS2/Note/ROS2%201_环境配置) | ROS2 基础环境配置                   | [环境测试](https://github.com/SSC202/ROS/tree/main/ROS2/Code/1_ROS2_Node_Test) |
| [2_节点构建](https://github.com/SSC202/ROS/tree/main/ROS2/Note/ROS2%202_节点构建) | ROS2 节点介绍                       | [节点构建测试](https://github.com/SSC202/ROS/tree/main/ROS2/Code/2_Node_Build_ws/src/node_build) |
| [3_节点通信](https://github.com/SSC202/ROS/tree/main/ROS2/Note/ROS2%203_节点通信) | ROS2 话题，服务，动作，参数通信介绍 | [话题通信测试](https://github.com/SSC202/ROS/tree/main/ROS2/Code/3_Topic_communication_ws/src/topic_communication)<br/>[服务通信测试](https://github.com/SSC202/ROS/tree/main/ROS2/Code/4_Service_communication_ws/src/service_communication)<br/>[自定义通信接口测试](https://github.com/SSC202/ROS/tree/main/ROS2/Code/5_Self_Interface_ws/src)<br/>[参数通信测试](https://github.com/SSC202/ROS/tree/main/ROS2/Code/6_Param_ws/src/param_test) |
| [4_TF组件](https://github.com/SSC202/ROS/tree/main/ROS2/Note/ROS2%204_TF组件) | 坐标变换和TF2组件介绍               | [坐标变换测试](https://github.com/SSC202/ROS/tree/main/ROS2/Code/7_Pose_transform)<br/>[TF静态坐标广播测试](https://github.com/SSC202/ROS/tree/main/ROS2/Code/8_TF_Static_transform/src/tf_static_broadcast) |
| [5_Launch组件](https://github.com/SSC202/ROS/tree/main/ROS2/Note/ROS2%205_Launch组件) | Launch文件介绍                      | [Launch文件编写测试](https://github.com/SSC202/ROS/tree/main/ROS2/Code/9_Launch_Test/src/launch_test) |
| [7_Webots仿真平台](https://github.com/SSC202/ROS/tree/main/ROS2/Note/ROS2%207_Webots仿真平台) | Webots机器人仿真平台介绍            | [自建四轮机器人测试](https://github.com/SSC202/ROS/tree/main/ROS2/Code/10_Webots_Test_ws/wheel_robot)<br/>[自建supervisor节点测试](https://github.com/SSC202/ROS/tree/main/ROS2/Code/10_Webots_Test_ws/my_supervisor)<br/>[Webots-ROS2使用测试](https://github.com/SSC202/ROS/tree/main/ROS2/Code/10_Webots_Test_ws/webots_ros2_test_ws) |

