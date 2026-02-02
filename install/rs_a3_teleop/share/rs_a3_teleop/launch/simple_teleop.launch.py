#!/usr/bin/env python3
"""
简化的Xbox手柄控制启动文件（不需要MoveIt Servo）
使用MoveGroup规划器进行控制
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明启动参数
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        description='Use mock hardware for simulation'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    # 包含MoveIt启动文件（包含robot和RViz）
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rs_a3_moveit_config'),
                'launch',
                'demo.launch.py' if LaunchConfiguration('use_mock_hardware') else 'robot.launch.py'
            ])
        ])
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
    
    # Xbox Teleop节点（使用MoveGroup版本）
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
        use_mock_hardware_arg,
        device_arg,
        moveit_launch,
        joy_node,
        xbox_teleop_node,
    ])

