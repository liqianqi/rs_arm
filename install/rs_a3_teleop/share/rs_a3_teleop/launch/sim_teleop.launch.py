#!/usr/bin/env python3
"""
仿真环境Xbox手柄控制启动文件
使用Mock硬件进行仿真测试
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2'
    )
    
    # 包含MoveIt仿真启动文件（使用mock硬件）
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rs_a3_moveit_config'),
                'launch',
                'demo.launch.py'  # 使用仿真demo launch文件
            ])
        ]),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items()
    )
    
    # Joy节点
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )
    
    # Xbox Teleop节点
    xbox_teleop_node = Node(
        package='rs_a3_teleop',
        executable='xbox_teleop_node',
        name='xbox_teleop_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('rs_a3_teleop'),
                'config',
                'xbox_teleop.yaml'
            ])
        ],
        output='screen'
    )
    
    return LaunchDescription([
        device_arg,
        use_rviz_arg,
        moveit_launch,
        joy_node,
        xbox_teleop_node,
    ])
