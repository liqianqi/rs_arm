#!/bin/bash
# 等待Xbox手柄设备出现并自动启动控制系统

echo "========================================"
echo "   等待Xbox手柄就绪..."
echo "========================================"
echo ""

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${YELLOW}请在Xbox手柄上执行以下操作：${NC}"
echo "1. 按住Xbox按钮3秒（中央大按钮）"
echo "2. 按几下A/B/X/Y按钮"
echo "3. 移动左右摇杆"
echo ""
echo "正在监控设备..."
echo ""

# 等待js0设备出现（最多等待30秒）
for i in {1..30}; do
    if [ -e /dev/input/js0 ]; then
        echo -e "${GREEN}✓ 检测到手柄设备！${NC}"
        ls -l /dev/input/js0
        echo ""
        break
    fi
    
    # 每5秒显示一次提示
    if [ $((i % 5)) -eq 0 ]; then
        echo "等待中... (${i}/30秒) - 请按手柄上的按钮"
    fi
    
    sleep 1
done

if [ ! -e /dev/input/js0 ]; then
    echo -e "${YELLOW}⚠ 30秒内未检测到手柄设备${NC}"
    echo ""
    echo "可能的解决方案："
    echo ""
    echo "1. 安装xpadneo驱动："
    echo "   bash /home/wy/RS/A3/scripts/install_xpadneo.sh"
    echo ""
    echo "2. 重新连接手柄："
    echo "   - 蓝牙设置 → 断开Xbox手柄"
    echo "   - 按住Xbox按钮重新连接"
    echo ""
    echo "3. 查看详细排查指南："
    echo "   cat /home/wy/RS/A3/XBOX_QUICK_FIX.md"
    echo ""
    exit 1
fi

# 设置权限
echo "2501" | sudo -S chmod 666 /dev/input/js0 2>/dev/null
echo -e "${GREEN}✓ 权限已设置${NC}"
echo ""

# 测试手柄
echo -e "${BLUE}测试手柄输入（2秒）...${NC}"
echo "请移动摇杆或按按钮..."
echo ""

cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

timeout 2 ros2 run joy joy_node --ros-args -p device_name:=/dev/input/js0 > /dev/null 2>&1 &
JOY_PID=$!
sleep 1.5

if ros2 topic list 2>/dev/null | grep -q "/joy"; then
    if timeout 1 ros2 topic echo /joy --once > /dev/null 2>&1; then
        echo -e "${GREEN}✓ 手柄输入正常！${NC}"
    else
        echo -e "${YELLOW}! 手柄节点运行中，但未检测到输入${NC}"
        echo "  请移动摇杆或按按钮"
    fi
else
    echo -e "${YELLOW}! Joy节点未能启动${NC}"
fi

kill $JOY_PID 2>/dev/null
wait $JOY_PID 2>/dev/null
sleep 1

echo ""
echo "========================================"
echo -e "${GREEN}   启动机器人控制系统${NC}"
echo "========================================"
echo ""
echo "控制说明："
echo "  • 左摇杆: X/Y平移（前后左右）"
echo "  • LT扳机: Z轴向下"
echo "  • RT扳机: Z轴向上"
echo "  • 右摇杆: Yaw/Pitch旋转"
echo "  • LB/RB: Roll旋转（左右翻滚）"
echo ""
echo "RViz将显示机器人模型"
echo "按 Ctrl+C 停止系统"
echo ""
sleep 3

# 启动完整系统
exec ros2 launch rs_a3_teleop simple_teleop.launch.py





