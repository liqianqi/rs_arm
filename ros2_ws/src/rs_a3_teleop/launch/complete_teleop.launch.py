#!/usr/bin/env python3
"""
完整的Xbox手柄控制启动文件
包含机器人、MoveIt、Servo和手柄控制的一体化启动

使用方法：
  # 仿真模式
  ros2 launch rs_a3_teleop complete_teleop.launch.py use_mock_hardware:=true
  
  # 真实硬件模式
  ros2 launch rs_a3_teleop complete_teleop.launch.py use_mock_hardware:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # 声明启动参数
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Use mock hardware for simulation'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    # 包含MoveIt启动文件
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rs_a3_moveit_config'),
                'launch',
                'robot.launch.py'
            ])
        ]),
        launch_arguments={
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items()
    )
    
    # MoveIt Servo节点
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('rs_a3_teleop'),
                'config',
                'moveit_servo_config.yaml'
            ])
        ],
        output='screen'
    )
    
    # Joy节点
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
    
    # Xbox Servo Teleop节点
    xbox_servo_teleop_node = Node(
        package='rs_a3_teleop',
        executable='xbox_servo_teleop_node',
        name='xbox_servo_teleop_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('rs_a3_teleop'),
                'config',
                'xbox_servo_teleop.yaml'
            ])
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_mock_hardware_arg,
        use_rviz_arg,
        device_arg,
        moveit_launch,
        servo_node,
        joy_node,
        xbox_servo_teleop_node,
    ])





