#!/usr/bin/env python3
"""
Joy-Con IMU 遥控 Launch 文件

启动方式:
  仿真模式:
    ros2 launch rs_a3_teleop joycon_imu_teleop.launch.py

  真实硬件:
    ros2 launch rs_a3_teleop joycon_imu_teleop.launch.py use_mock_hardware:=false can_interface:=can0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # 获取包路径
    teleop_pkg = FindPackageShare('rs_a3_teleop')
    moveit_pkg = FindPackageShare('rs_a3_moveit_config')
    
    # 声明启动参数
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        description='Use mock hardware for simulation'
    )
    
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name'
    )
    
    joy_device_arg = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js0',
        description='Joy-Con device path'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )
    
    # 获取参数
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    can_interface = LaunchConfiguration('can_interface')
    joy_device = LaunchConfiguration('joy_device')
    debug = LaunchConfiguration('debug')
    
    # 配置文件路径
    teleop_config = PathJoinSubstitution([
        teleop_pkg, 'config', 'joycon_imu_teleop.yaml'
    ])
    
    # ==================== 仿真模式 ====================
    # 启动 MoveIt demo (包含 mock 硬件)
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_pkg, 'launch', 'demo.launch.py'])
        ]),
        condition=IfCondition(use_mock_hardware)
    )
    
    # ==================== 真实硬件模式 ====================
    # 启动真实硬件 MoveIt
    moveit_real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_pkg, 'launch', 'robot.launch.py'])
        ]),
        launch_arguments={
            'can_interface': can_interface
        }.items(),
        condition=UnlessCondition(use_mock_hardware)
    )
    
    # ==================== Joy 节点 ====================
    # 发布手柄按键数据
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_name': '',
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        remappings=[
            ('joy', 'joy'),
        ],
    )
    
    # ==================== Joy-Con IMU 驱动节点 ====================
    # 读取 Joy-Con IMU 数据并发布为 sensor_msgs/Imu
    joycon_imu_driver_node = Node(
        package='rs_a3_teleop',
        executable='joycon_imu_driver',
        name='joycon_imu_driver',
        parameters=[{
            'device_path': joy_device,
            'publish_rate': 60.0,
        }],
        output='screen',
    )
    
    # ==================== IMU 遥控节点 ====================
    joycon_imu_teleop_node = Node(
        package='rs_a3_teleop',
        executable='joycon_imu_teleop',
        name='joycon_imu_teleop_node',
        parameters=[teleop_config, {'debug_mode': debug}],
        output='screen',
    )
    
    return LaunchDescription([
        # 参数声明
        use_mock_hardware_arg,
        can_interface_arg,
        joy_device_arg,
        debug_arg,
        
        # MoveIt 启动
        moveit_demo_launch,
        moveit_real_launch,
        
        # Joy-Con 节点
        joy_node,
        joycon_imu_driver_node,
        
        # 遥控节点
        joycon_imu_teleop_node,
    ])
