#!/bin/bash
# Xbox手柄控制测试脚本

echo "=================================="
echo "Xbox手柄控制系统测试"
echo "=================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查ROS2环境
echo -n "1. 检查ROS2环境... "
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}失败${NC}"
    echo "   请先source ROS2环境: source /opt/ros/humble/setup.bash"
    exit 1
else
    echo -e "${GREEN}通过${NC} (ROS_DISTRO=$ROS_DISTRO)"
fi

# 检查工作空间
echo -n "2. 检查工作空间... "
if [ -f "/home/wy/RS/A3/ros2_ws/install/setup.bash" ]; then
    echo -e "${GREEN}通过${NC}"
else
    echo -e "${RED}失败${NC}"
    echo "   工作空间未编译，请运行: colcon build"
    exit 1
fi

# 检查rs_a3_teleop包
echo -n "3. 检查rs_a3_teleop包... "
source /home/wy/RS/A3/ros2_ws/install/setup.bash
if ros2 pkg list | grep -q rs_a3_teleop; then
    echo -e "${GREEN}通过${NC}"
else
    echo -e "${RED}失败${NC}"
    echo "   rs_a3_teleop包未找到"
    exit 1
fi

# 检查可执行文件
echo -n "4. 检查可执行文件... "
if ros2 pkg executables rs_a3_teleop | grep -q xbox_servo_teleop_node; then
    echo -e "${GREEN}通过${NC}"
else
    echo -e "${RED}失败${NC}"
    echo "   xbox_servo_teleop_node未找到"
    exit 1
fi

# 检查启动文件
echo -n "5. 检查启动文件... "
LAUNCH_DIR="/home/wy/RS/A3/ros2_ws/install/rs_a3_teleop/share/rs_a3_teleop/launch"
if [ -f "$LAUNCH_DIR/complete_teleop.launch.py" ]; then
    echo -e "${GREEN}通过${NC}"
else
    echo -e "${RED}失败${NC}"
    echo "   启动文件未找到"
    exit 1
fi

# 检查配置文件
echo -n "6. 检查配置文件... "
CONFIG_DIR="/home/wy/RS/A3/ros2_ws/install/rs_a3_teleop/share/rs_a3_teleop/config"
if [ -f "$CONFIG_DIR/xbox_servo_teleop.yaml" ]; then
    echo -e "${GREEN}通过${NC}"
else
    echo -e "${RED}失败${NC}"
    echo "   配置文件未找到"
    exit 1
fi

# 检查joy包
echo -n "7. 检查joy包... "
if ros2 pkg list | grep -q joy; then
    echo -e "${GREEN}通过${NC}"
else
    echo -e "${YELLOW}警告${NC}"
    echo "   joy包未安装，请运行: sudo apt install ros-humble-joy"
fi

# 检查moveit_servo包
echo -n "8. 检查moveit_servo包... "
if ros2 pkg list | grep -q moveit_servo; then
    echo -e "${GREEN}通过${NC}"
else
    echo -e "${YELLOW}警告${NC}"
    echo "   moveit_servo包未安装，请运行: sudo apt install ros-humble-moveit-servo"
fi

# 检查手柄设备
echo -n "9. 检查手柄设备... "
if [ -e "/dev/input/js0" ]; then
    echo -e "${GREEN}通过${NC}"
    echo "   设备: /dev/input/js0"
    
    # 检查权限
    if [ -r "/dev/input/js0" ]; then
        echo "   权限: ${GREEN}可读${NC}"
    else
        echo "   权限: ${YELLOW}不可读${NC}"
        echo "   解决方法: sudo chmod 666 /dev/input/js0"
    fi
else
    echo -e "${YELLOW}未检测到${NC}"
    echo "   请连接Xbox手柄"
fi

echo ""
echo "=================================="
echo "测试完成!"
echo "=================================="
echo ""
echo "可用的启动命令："
echo ""
echo "1. 完整系统（推荐）："
echo "   ${GREEN}ros2 launch rs_a3_teleop complete_teleop.launch.py${NC}"
echo ""
echo "2. 仅手柄控制（需要先启动机器人和MoveIt Servo）："
echo "   ${GREEN}ros2 launch rs_a3_teleop xbox_servo_teleop.launch.py${NC}"
echo ""
echo "3. 测试手柄输入："
echo "   ${GREEN}ros2 run joy joy_node${NC}"
echo "   ${GREEN}ros2 topic echo /joy${NC}"
echo ""
echo "详细使用说明："
echo "   cat /home/wy/RS/A3/ros2_ws/src/rs_a3_teleop/QUICK_START.md"
echo ""





