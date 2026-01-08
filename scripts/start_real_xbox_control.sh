#!/bin/bash
# RS-A3 实机Xbox手柄控制启动脚本
# 
# 使用方法: ./start_real_xbox_control.sh [can_interface]
#   can_interface: CAN接口名称 (默认: can0)

CAN_INTERFACE=${1:-can0}

echo "============================================"
echo "RS-A3 实机Xbox手柄控制启动"
echo "============================================"

# 检查CAN接口
echo "检查CAN接口 ${CAN_INTERFACE}..."
if ! ip link show ${CAN_INTERFACE} &> /dev/null; then
    echo "错误: CAN接口 ${CAN_INTERFACE} 不存在!"
    echo ""
    echo "请先设置CAN接口:"
    echo "  sudo ./setup_can.sh ${CAN_INTERFACE}"
    echo ""
    echo "或检查CAN适配器是否已连接"
    exit 1
fi

# 检查CAN接口是否UP
if ! ip link show ${CAN_INTERFACE} | grep -q "UP"; then
    echo "CAN接口 ${CAN_INTERFACE} 未启动，正在启动..."
    sudo ip link set ${CAN_INTERFACE} up
    if [ $? -ne 0 ]; then
        echo "错误: 无法启动CAN接口"
        exit 1
    fi
fi

echo "CAN接口 ${CAN_INTERFACE} 已就绪"

# 检查手柄
echo ""
echo "检查Xbox手柄..."
if [ ! -e /dev/input/js0 ]; then
    echo "警告: 未检测到手柄 (/dev/input/js0)"
    echo "请确保Xbox手柄已连接"
fi

# 启动ROS2控制
echo ""
echo "启动实机控制..."
echo "控制说明:"
echo "  左摇杆Y: X方向移动"
echo "  左摇杆X: Y方向移动"
echo "  LT/RT:   Z方向移动"
echo "  右摇杆:  Yaw/Pitch旋转"
echo "  LB/RB:   Roll旋转"
echo "  A键:    切换速度档位"
echo "  B键:    回到初始位置"
echo ""
echo "按 Ctrl+C 停止"
echo "============================================"

cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rs_a3_teleop real_teleop.launch.py can_interface:=${CAN_INTERFACE}


