#!/usr/bin/env python3
# coding=utf-8
'''
@company: Copyright (C) 2022, LD Robot, WHEELTEC (Dongguan) Co., Ltd
@product: ld14
@filename: ld14.launch.py
@brief:
@version:       date:       author:            comments:
@v2.0           22-4-25      Tues              ROS2

'''
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
'''
禁用角度裁剪: {'flag_parted' : False}
启用角度裁剪: {'flag_parted' : True}

角度裁剪：顺时针设置，屏蔽雷达的270°~90°扫描范围
{'angle1_start' : 270}
{'angle1_end' : 90}
增加角度裁剪间隔：
parameters=[{
'port_name' : '/dev/LD14',
'flag_parted' : False,
'angle1_start' : 10,
'angle1_end' : 20,

'angle2_start' : 40,
'angle2_end' : 50,

'angle3_start' : 70,
'angle3_end' : 80,

'angle4_start' : 100,
'angle4_end' : 110,
}],
'''            

def generate_launch_description():

    ld = LaunchDescription()

    ld_14_node = Node(
        package='ld14',
        executable='ld14_node',
        parameters=[{
        'port_name' : '/dev/ttyUSB0',
        'flag_parted' : False,
        'angle1_start' : 270,
        'angle1_end' : 90,
        }],
        output='screen'       
    )

    ld.add_action(ld_14_node)

    return ld

