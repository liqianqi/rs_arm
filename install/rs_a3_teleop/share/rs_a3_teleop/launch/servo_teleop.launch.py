#!/usr/bin/env python3
"""
使用MoveIt Servo的Xbox遥操作启动文件
Servo使用雅可比矩阵进行增量控制，避免IK解跳变问题
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 声明参数
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    can_interface = LaunchConfiguration('can_interface')
    
    # 获取包路径
    rs_a3_description_share = get_package_share_directory('rs_a3_description')
    rs_a3_moveit_config_share = get_package_share_directory('rs_a3_moveit_config')
    rs_a3_teleop_share = get_package_share_directory('rs_a3_teleop')
    
    # 配置文件路径
    servo_config = os.path.join(rs_a3_moveit_config_share, 'config', 'servo_config.yaml')
    
    # 参数声明
    declare_mock_hardware = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Use mock hardware'
    )
    
    declare_can_interface = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name'
    )
    
    # 包含基础启动文件 (rs_a3_control.launch.py)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rs_a3_description_share, 'launch', 'rs_a3_control.launch.py')
        ),
        launch_arguments={
            'use_mock_hardware': use_mock_hardware,
            'can_interface': can_interface,
        }.items(),
    )
    
    # 包含MoveIt启动文件
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rs_a3_moveit_config_share, 'launch', 'moveit.launch.py')
        ),
    )
    
    # MoveIt Servo节点
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        output='screen',
        parameters=[
            servo_config,
            {'use_sim_time': False},
        ],
    )
    
    # Xbox Servo遥操作节点 - 延迟启动确保Servo准备就绪
    xbox_servo_node = TimerAction(
        period=8.0,  # 延迟8秒启动
        actions=[
            Node(
                package='rs_a3_teleop',
                executable='xbox_servo_node',
                name='xbox_servo_node',
                output='screen',
                parameters=[{
                    'update_rate': 50.0,
                    'linear_scale': 0.15,
                    'angular_scale': 0.5,
                    'deadzone': 0.15,
                    'debug_input': False,
                }],
            )
        ]
    )
    
    # Joy节点
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
    )
    
    return LaunchDescription([
        declare_mock_hardware,
        declare_can_interface,
        bringup_launch,
        moveit_launch,
        joy_node,
        # Servo节点延迟启动
        TimerAction(
            period=5.0,
            actions=[servo_node]
        ),
        xbox_servo_node,
    ])

