#!/bin/bash
# 启动Xbox手柄控制的一键脚本

echo "========================================"
echo "   启动Xbox手柄控制系统"
echo "========================================"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 进入工作空间
cd /home/wy/RS/A3/ros2_ws

# Source环境
echo -e "${BLUE}设置ROS2环境...${NC}"
source /opt/ros/humble/setup.bash
source install/setup.bash
echo -e "${GREEN}✓${NC} 环境已设置"
echo ""

# 检查手柄连接
echo -e "${BLUE}检查手柄连接...${NC}"

if [ -e /dev/input/js0 ]; then
    echo -e "${GREEN}✓${NC} 找到手柄设备: /dev/input/js0"
    DEVICE_PARAM="device_name:=/dev/input/js0"
    echo ""
elif [ -e /dev/input/js1 ]; then
    echo -e "${GREEN}✓${NC} 找到手柄设备: /dev/input/js1"
    DEVICE_PARAM="device_name:=/dev/input/js1"
    echo ""
else
    echo -e "${YELLOW}!${NC} 未找到/dev/input/js设备"
    echo "尝试查找event设备..."
    
    # 查找Xbox event设备
    XBOX_EVENT=$(ls -t /dev/input/event* | head -1)
    if [ -n "$XBOX_EVENT" ]; then
        echo -e "${GREEN}✓${NC} 使用event设备: $XBOX_EVENT"
        DEVICE_PARAM="device_name:=$XBOX_EVENT"
        echo ""
    else
        echo -e "${RED}✗${NC} 未找到任何输入设备"
        echo ""
        echo "请执行以下操作："
        echo "1. 确保Xbox手柄已连接（查看蓝牙设置）"
        echo "2. 按下手柄上的任意按钮激活它"
        echo "3. 运行: sudo modprobe joydev"
        echo "4. 重新运行此脚本"
        echo ""
        exit 1
    fi
fi

# 测试手柄输入
echo -e "${BLUE}测试手柄输入（3秒）...${NC}"
echo "请移动摇杆或按下按钮..."
echo ""

timeout 3 ros2 run joy joy_node --ros-args -p $DEVICE_PARAM &
JOY_PID=$!
sleep 1

# 检查是否有joy数据
if ros2 topic list | grep -q "/joy"; then
    echo -e "${GREEN}✓${NC} 手柄节点正在运行"
    
    # 显示一些joy数据
    timeout 2 ros2 topic echo /joy --once 2>/dev/null && echo -e "${GREEN}✓${NC} 手柄输入正常" || echo -e "${YELLOW}!${NC} 未检测到手柄输入，请移动摇杆"
fi

# 停止测试节点
kill $JOY_PID 2>/dev/null
sleep 1

echo ""
echo "========================================"
echo -e "${GREEN}启动完整控制系统${NC}"
echo "========================================"
echo ""
echo "控制说明："
echo "  左摇杆: X/Y平移"
echo "  LT/RT: Z轴上下"
echo "  右摇杆: Yaw/Pitch旋转"
echo "  LB/RB: Roll旋转"
echo ""
echo "按 Ctrl+C 停止系统"
echo ""
sleep 2

# 启动完整系统
ros2 launch rs_a3_teleop simple_teleop.launch.py





