#!/usr/bin/env python3
"""
实机Xbox手柄控制启动文件
使用真实硬件而非仿真
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明启动参数
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name'
    )
    
    host_can_id_arg = DeclareLaunchArgument(
        'host_can_id',
        default_value='253',
        description='Host CAN ID (0xFD = 253)'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    # 包含MoveIt实机启动文件
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rs_a3_moveit_config'),
                'launch',
                'robot.launch.py'  # 使用实机launch文件
            ])
        ]),
        launch_arguments={
            'can_interface': LaunchConfiguration('can_interface'),
            'host_can_id': LaunchConfiguration('host_can_id'),
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
        can_interface_arg,
        host_can_id_arg,
        device_arg,
        moveit_launch,
        joy_node,
        xbox_teleop_node,
    ])


