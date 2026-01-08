#!/bin/bash
# 批量测试不同 Kp 值的回零误差
# 用法: ./batch_kp_test.sh

WORKSPACE="/home/wy/RS/A3/ros2_ws"
XACRO_FILE="${WORKSPACE}/src/rs_a3_description/urdf/rs_a3_ros2_control.xacro"
RESULT_FILE="/home/wy/RS/A3/kp_test_results.txt"

# 测试的 Kp 值
KP_VALUES=(50 100 200 300)
KD_VALUE=1.0

echo "========================================" | tee $RESULT_FILE
echo "  Kp 回零误差测试" | tee -a $RESULT_FILE
echo "  Kd = ${KD_VALUE}" | tee -a $RESULT_FILE
echo "  测试时间: $(date)" | tee -a $RESULT_FILE
echo "========================================" | tee -a $RESULT_FILE

for KP in "${KP_VALUES[@]}"; do
    echo ""
    echo "----------------------------------------"
    echo "测试 Kp = ${KP}"
    echo "----------------------------------------"
    
    # 停止现有进程
    pkill -f "ros2" 2>/dev/null
    sleep 3
    
    # 修改 xacro 文件中的 Kp 值
    sed -i "s/<param name=\"position_kp\">.*<\/param>/<param name=\"position_kp\">${KP}.0<\/param>/" $XACRO_FILE
    sed -i "s/<param name=\"position_kd\">.*<\/param>/<param name=\"position_kd\">${KD_VALUE}<\/param>/" $XACRO_FILE
    
    echo "已设置 Kp=${KP}, Kd=${KD_VALUE}"
    
    # 重新编译
    cd $WORKSPACE
    source /opt/ros/humble/setup.bash
    colcon build --packages-select rs_a3_description > /dev/null 2>&1
    source install/setup.bash
    
    # 启动机器人
    echo "启动控制器..."
    ros2 launch rs_a3_moveit_config robot.launch.py can_interface:=can1 > /tmp/ros_launch_${KP}.log 2>&1 &
    LAUNCH_PID=$!
    
    # 等待启动
    sleep 25
    
    # 运行归零测试
    echo "运行归零测试..."
    timeout 60 /usr/bin/python3 /home/wy/RS/A3/scripts/move_to_zero.py > /tmp/zero_test_${KP}.log 2>&1
    
    # 等待稳定
    sleep 3
    
    # 采集最终位置
    echo "" >> $RESULT_FILE
    echo "Kp = ${KP}:" >> $RESULT_FILE
    
    # 获取最后的位置数据
    LAST_POS=$(grep "当前位置" /tmp/zero_test_${KP}.log | tail -1)
    echo "  ${LAST_POS}" >> $RESULT_FILE
    
    # 从日志提取误差
    echo "  结果: $(tail -5 /tmp/zero_test_${KP}.log | head -1)" >> $RESULT_FILE
    
    echo "Kp=${KP} 测试完成"
    
    # 停止进程
    kill $LAUNCH_PID 2>/dev/null
    pkill -f "ros2" 2>/dev/null
    sleep 2
done

echo ""
echo "========================================"
echo "  所有测试完成!"
echo "  结果保存在: $RESULT_FILE"
echo "========================================"
cat $RESULT_FILE


















