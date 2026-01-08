#!/bin/bash
# 安装xpadneo驱动（专门用于蓝牙Xbox手柄）

echo "========================================"
echo "   安装xpadneo蓝牙Xbox手柄驱动"
echo "========================================"
echo ""

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 检查是否已安装
if dkms status | grep -q hid-xpadneo; then
    echo -e "${GREEN}xpadneo驱动已安装${NC}"
    echo ""
    echo "重新加载驱动..."
    echo "2501" | sudo -S modprobe -r hid-xpadneo 2>/dev/null
    echo "2501" | sudo -S modprobe hid-xpadneo
    echo -e "${GREEN}✓ 驱动已重新加载${NC}"
    exit 0
fi

echo -e "${BLUE}下载xpadneo驱动...${NC}"
cd /tmp
rm -rf xpadneo
git clone https://github.com/atar-axis/xpadneo.git

if [ $? -ne 0 ]; then
    echo "Git克隆失败，尝试下载release版本..."
    wget https://github.com/atar-axis/xpadneo/archive/refs/tags/v0.9.5.tar.gz
    tar xzf v0.9.5.tar.gz
    cd xpadneo-0.9.5
else
    cd xpadneo
fi

echo ""
echo -e "${BLUE}安装xpadneo驱动...${NC}"
echo "2501" | sudo -S ./install.sh

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}✓ xpadneo驱动安装成功！${NC}"
    echo ""
    echo "重启蓝牙服务..."
    echo "2501" | sudo -S systemctl restart bluetooth
    echo ""
    echo -e "${YELLOW}请断开并重新连接Xbox手柄：${NC}"
    echo "1. 在蓝牙设置中断开Xbox手柄"
    echo "2. 按住Xbox按钮重新连接"
    echo "3. 然后运行: ls /dev/input/js*"
else
    echo ""
    echo "安装失败，请检查错误信息"
fi





