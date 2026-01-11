#!/bin/bash
# Joy-Con 驱动安装脚本
# 
# 此脚本安装 Nintendo Switch Joy-Con 手柄所需的驱动和工具
#
# 使用方法:
#   sudo ./install_joycon_driver.sh

set -e

echo "=========================================="
echo "  Joy-Con 驱动安装脚本"
echo "=========================================="
echo ""

# 检查是否为 root
if [ "$EUID" -ne 0 ]; then
    echo "请使用 sudo 运行此脚本"
    exit 1
fi

# 1. 检查内核版本
echo "=== 1. 检查内核版本 ==="
KERNEL_VERSION=$(uname -r | cut -d. -f1-2)
echo "内核版本: $(uname -r)"

if [[ $(echo "$KERNEL_VERSION >= 5.16" | bc -l) -eq 1 ]]; then
    echo "✓ 内核版本 >= 5.16，已内置 hid-nintendo 模块"
else
    echo "⚠ 内核版本 < 5.16，可能需要手动安装 hid-nintendo 模块"
fi
echo ""

# 2. 加载 hid-nintendo 模块
echo "=== 2. 加载 hid-nintendo 模块 ==="
if lsmod | grep -q hid_nintendo; then
    echo "✓ hid-nintendo 模块已加载"
else
    echo "正在加载 hid-nintendo 模块..."
    modprobe hid-nintendo || {
        echo "⚠ 无法加载 hid-nintendo 模块"
        echo "尝试安装 dkms 版本..."
        apt install -y dkms
        # 如果模块不存在，可能需要从源码编译
    }
    
    # 设置开机自动加载
    if ! grep -q "hid-nintendo" /etc/modules-load.d/hid-nintendo.conf 2>/dev/null; then
        echo "hid-nintendo" > /etc/modules-load.d/hid-nintendo.conf
        echo "✓ 已设置开机自动加载 hid-nintendo 模块"
    fi
fi
echo ""

# 3. 安装 joycond
echo "=== 3. 安装 joycond ==="
if which joycond >/dev/null 2>&1; then
    echo "✓ joycond 已安装"
else
    echo "正在安装 joycond..."
    
    # 尝试从包管理器安装
    if apt install -y joycond 2>/dev/null; then
        echo "✓ joycond 从 apt 安装成功"
    else
        # 从源码编译安装
        echo "apt 安装失败，尝试从源码编译..."
        
        # 安装依赖
        apt install -y cmake libevdev-dev libudev-dev
        
        # 克隆并编译
        TEMP_DIR=$(mktemp -d)
        cd "$TEMP_DIR"
        git clone https://github.com/DanielOgorworking/joycond.git || \
            git clone https://gitlab.com/jcoffland/joycond.git
        cd joycond
        cmake .
        make
        make install
        
        # 复制 udev 规则
        cp udev/89-joycond.rules /etc/udev/rules.d/
        udevadm control --reload-rules
        udevadm trigger
        
        # 安装 systemd 服务
        cp systemd/joycond.service /etc/systemd/system/
        
        cd /
        rm -rf "$TEMP_DIR"
        
        echo "✓ joycond 从源码编译安装成功"
    fi
fi

# 启用 joycond 服务
echo "启用 joycond 服务..."
systemctl enable joycond
systemctl start joycond
echo "✓ joycond 服务已启动"
echo ""

# 4. 安装 Python 依赖
echo "=== 4. 安装 Python 依赖 ==="
pip3 install evdev
echo "✓ evdev 已安装"
echo ""

# 5. 配置 udev 规则（允许普通用户访问）
echo "=== 5. 配置 udev 规则 ==="
cat > /etc/udev/rules.d/50-nintendo.rules << 'EOF'
# Nintendo Switch Joy-Con / Pro Controller udev rules

# Joy-Con (L)
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="057e", ATTRS{idProduct}=="2006", MODE="0666"
# Joy-Con (R)
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="057e", ATTRS{idProduct}=="2007", MODE="0666"
# Pro Controller
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="057e", ATTRS{idProduct}=="2009", MODE="0666"
# Combined Joy-Con
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="057e", ATTRS{idProduct}=="200e", MODE="0666"

# evdev 访问
KERNEL=="event*", SUBSYSTEM=="input", ATTRS{name}=="*Joy-Con*", MODE="0666"
KERNEL=="event*", SUBSYSTEM=="input", ATTRS{name}=="*Nintendo*", MODE="0666"
EOF

udevadm control --reload-rules
udevadm trigger
echo "✓ udev 规则已配置"
echo ""

# 6. 提示蓝牙配对方法
echo "=========================================="
echo "  安装完成!"
echo "=========================================="
echo ""
echo "=== Joy-Con 蓝牙配对步骤 ==="
echo ""
echo "1. 长按 Joy-Con 侧边的同步按钮进入配对模式"
echo "   (LED 灯会快速闪烁)"
echo ""
echo "2. 使用 bluetoothctl 配对:"
echo "   $ bluetoothctl"
echo "   [bluetooth]# scan on"
echo "   (等待发现 Joy-Con...)"
echo "   [bluetooth]# pair XX:XX:XX:XX:XX:XX"
echo "   [bluetooth]# trust XX:XX:XX:XX:XX:XX"
echo "   [bluetooth]# connect XX:XX:XX:XX:XX:XX"
echo ""
echo "3. 验证连接:"
echo "   $ ls /dev/input/js*"
echo "   $ cat /proc/bus/input/devices | grep -A 5 Joy-Con"
echo ""
echo "=== 启动 Joy-Con IMU 遥控 ==="
echo ""
echo "仿真模式:"
echo "  ros2 launch rs_a3_teleop joycon_imu_teleop.launch.py"
echo ""
echo "真实硬件:"
echo "  ros2 launch rs_a3_teleop joycon_imu_teleop.launch.py use_mock_hardware:=false can_interface:=can0"
echo ""
