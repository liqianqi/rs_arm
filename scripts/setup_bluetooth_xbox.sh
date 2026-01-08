#!/bin/bash
# 蓝牙Xbox手柄设置脚本

echo "========================================"
echo "   蓝牙Xbox手柄设置向导"
echo "========================================"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 步骤1: 检查并加载joydev模块
echo -e "${BLUE}步骤1: 加载joydev内核模块${NC}"
if lsmod | grep -q joydev; then
    echo -e "${GREEN}✓${NC} joydev模块已加载"
else
    echo -e "${YELLOW}⟳${NC} 正在加载joydev模块..."
    sudo modprobe joydev
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓${NC} joydev模块加载成功"
        
        # 设置开机自动加载
        if ! grep -q "^joydev" /etc/modules 2>/dev/null; then
            echo "joydev" | sudo tee -a /etc/modules > /dev/null
            echo -e "${GREEN}✓${NC} 已设置开机自动加载"
        fi
    else
        echo -e "${RED}✗${NC} 加载失败，请检查权限"
    fi
fi
echo ""

# 步骤2: 检查蓝牙服务
echo -e "${BLUE}步骤2: 检查蓝牙服务${NC}"
if systemctl is-active --quiet bluetooth; then
    echo -e "${GREEN}✓${NC} 蓝牙服务正在运行"
else
    echo -e "${YELLOW}⟳${NC} 启动蓝牙服务..."
    sudo systemctl start bluetooth
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓${NC} 蓝牙服务已启动"
    else
        echo -e "${RED}✗${NC} 蓝牙服务启动失败"
    fi
fi
echo ""

# 步骤3: 检查手柄设备
echo -e "${BLUE}步骤3: 检查手柄设备${NC}"
if [ -e /dev/input/js0 ]; then
    echo -e "${GREEN}✓${NC} 找到手柄设备: /dev/input/js0"
    ls -l /dev/input/js0
else
    echo -e "${YELLOW}!${NC} 未找到 /dev/input/js0 设备"
    echo ""
    echo "请按照以下步骤连接手柄："
    echo "1. 按住Xbox手柄顶部的配对按钮（Xbox logo旁边）"
    echo "2. 手柄的Xbox logo应该开始快速闪烁"
    echo "3. 在系统蓝牙设置中找到 'Xbox Wireless Controller'"
    echo "4. 点击连接"
    echo ""
    echo "或者运行以下命令使用命令行配对："
    echo "  bluetoothctl"
    echo "  然后输入: scan on"
    echo "  找到手柄MAC地址后: pair XX:XX:XX:XX:XX:XX"
    echo ""
fi
echo ""

# 步骤4: 检查用户权限
echo -e "${BLUE}步骤4: 检查用户权限${NC}"
if groups | grep -q input; then
    echo -e "${GREEN}✓${NC} 用户已在input组中"
else
    echo -e "${YELLOW}⟳${NC} 添加用户到input组..."
    sudo usermod -a -G input $USER
    echo -e "${GREEN}✓${NC} 已添加到input组"
    echo -e "${YELLOW}!${NC} 请注销并重新登录后生效"
fi
echo ""

# 步骤5: 查找所有输入设备
echo -e "${BLUE}步骤5: 当前输入设备${NC}"
echo "Joystick设备:"
ls -l /dev/input/js* 2>/dev/null || echo "  (无)"
echo ""
echo "最近的输入事件设备:"
ls -lt /dev/input/event* | head -5
echo ""

# 步骤6: 扫描Xbox手柄
echo -e "${BLUE}步骤6: 扫描Xbox手柄${NC}"
XBOX_DEVICE=$(cat /proc/bus/input/devices 2>/dev/null | grep -i "xbox\|wireless controller" -A 5)
if [ -n "$XBOX_DEVICE" ]; then
    echo -e "${GREEN}✓${NC} 找到Xbox手柄："
    echo "$XBOX_DEVICE"
else
    echo -e "${YELLOW}!${NC} 未在/proc/bus/input/devices中找到Xbox手柄"
    echo "可能的原因："
    echo "  - 手柄未连接"
    echo "  - 需要重新配对"
    echo "  - 蓝牙驱动问题"
fi
echo ""

# 检查joystick包是否安装
echo -e "${BLUE}步骤7: 检查测试工具${NC}"
if command -v jstest &> /dev/null; then
    echo -e "${GREEN}✓${NC} jstest已安装"
else
    echo -e "${YELLOW}!${NC} jstest未安装"
    echo "安装命令: sudo apt install joystick"
fi
echo ""

# 总结
echo "========================================"
echo "   设置完成"
echo "========================================"
echo ""

if [ -e /dev/input/js0 ]; then
    echo -e "${GREEN}✓ 手柄设备已就绪！${NC}"
    echo ""
    echo "下一步："
    echo "1. 测试手柄："
    echo "   jstest /dev/input/js0"
    echo ""
    echo "2. 测试ROS2 joy节点："
    echo "   ros2 run joy joy_node"
    echo "   ros2 topic echo /joy"
    echo ""
    echo "3. 启动完整控制系统："
    echo "   cd /home/wy/RS/A3/ros2_ws"
    echo "   source install/setup.bash"
    echo "   ros2 launch rs_a3_teleop simple_teleop.launch.py"
else
    echo -e "${YELLOW}⚠ 手柄设备尚未就绪${NC}"
    echo ""
    echo "请执行以下操作："
    echo ""
    echo "1. 确保Xbox手柄已通过蓝牙配对："
    echo "   - 打开系统设置 → 蓝牙"
    echo "   - 按住手柄配对按钮（Xbox logo旁边）"
    echo "   - 连接 'Xbox Wireless Controller'"
    echo ""
    echo "2. 连接后重新运行此脚本："
    echo "   bash $0"
    echo ""
    echo "3. 查看详细设置指南："
    echo "   cat /home/wy/RS/A3/BLUETOOTH_XBOX_SETUP.md"
fi
echo ""





