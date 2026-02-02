#!/usr/bin/env python3
"""
Xbox手柄伺服控制启动文件
启动joy节点和xbox_servo_teleop节点
此版本使用MoveIt Servo进行实时控制
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明启动参数
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('rs_a3_teleop'),
            'config',
            'xbox_servo_teleop.yaml'
        ]),
        description='Path to config file'
    )
    
    # Joy节点 - 读取手柄输入
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': LaunchConfiguration('device'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )
    
    # Xbox Servo Teleop节点 - 处理手柄输入并发布Twist命令
    xbox_servo_teleop_node = Node(
        package='rs_a3_teleop',
        executable='xbox_servo_teleop_node',
        name='xbox_servo_teleop_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )
    
    return LaunchDescription([
        device_arg,
        config_file_arg,
        joy_node,
        xbox_servo_teleop_node,
    ])





