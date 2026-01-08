#!/bin/bash
# 安装 RS-A3 机械臂控制系统所需的依赖包
# 运行方法: sudo ./install_deps.sh

echo "Installing ROS2 Control and MoveIt dependencies for RS-A3..."

# ROS2 Control
apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-hardware-interface \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-controller-interface

# MoveIt2
apt install -y \
    ros-humble-moveit \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-kinematics

# Tools
apt install -y \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2

# CAN tools
apt install -y can-utils

echo "Dependencies installed successfully!"
echo ""
echo "Next steps:"
echo "1. cd /home/wy/RS/A3/ros2_ws"
echo "2. source /opt/ros/humble/setup.bash"
echo "3. colcon build --symlink-install"
echo "4. source install/setup.bash"
echo "5. ros2 launch rs_a3_moveit_config demo.launch.py"


