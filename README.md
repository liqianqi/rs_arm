# RS-A3 机械臂 ROS2 控制系统

基于 ROS2 Control 和 MoveIt2 的 RS-A3 六自由度机械臂控制系统，支持 Xbox 手柄实时遥操作。

## 📋 目录

- [系统概述](#系统概述)
- [硬件要求](#硬件要求)
- [软件环境](#软件环境)
- [功能包说明](#功能包说明)
- [安装配置](#安装配置)
- [快速开始](#快速开始)
- [控制参数](#控制参数)
- [ROS2 接口](#ros2-接口)
- [电机通信协议](#电机通信协议)
- [故障排除](#故障排除)
- [目录结构](#目录结构)

---

## 系统概述

### 主要特性

- **S曲线轨迹规划**: 标准7段S曲线速度规划，实现平滑无冲击运动
- **重力补偿**: 支持关节重力前馈补偿，提升运动精度
- **速度前馈**: 位置差分计算速度前馈，配合低通滤波减少运动抖动
- **关节限位保护**: 软限位减速+硬限位停止，保护机械臂安全
- **实时笛卡尔控制**: 50Hz Xbox手柄笛卡尔空间实时遥控
- **MoveIt2集成**: 支持运动规划和避障功能
- **笛卡尔速度映射**: 摇杆输入直接映射为末端执行器速度
- **多级平滑滤波**: 输入平滑 + 关节输出平滑 + 加速度限制，消除抖动
- **奇异点保护**: 自动检测并拒绝导致剧烈跳变的IK解，连续拒绝50帧后自动恢复
- **碰撞检测**: 启动后自动开启自碰撞和环境碰撞检测
- **Home/零点精确回归**: 回到预设位置后从目标点精确开始控制

### 机械臂参数

| 属性 | 规格 |
|------|------|
| 自由度 | 6 DOF |
| 末端执行器 | 可定制 |
| 通信协议 | 灵足时代私有协议 (CAN 2.0, 扩展帧29位ID) |
| 波特率 | 1Mbps |
| 控制模式 | 运控模式 (MIT-like PD Control) |

### 电机配置

| 关节 | 电机ID | 型号 | 力矩限制 | 速度限制 | 位置限制 | 方向 |
|------|--------|------|----------|----------|----------|------|
| L1_joint | 1 | RS00 | ±14 Nm | ±33 rad/s | ±160° (±2.79 rad) | -1 |
| L2_joint | 2 | RS00 | ±14 Nm | ±33 rad/s | ±180° (±3.14 rad) | +1 |
| L3_joint | 3 | RS00 | ±14 Nm | ±33 rad/s | ±180° (±3.14 rad) | -1 |
| L4_joint | 4 | RS05 | ±5.5 Nm | ±50 rad/s | ±180° (±3.14 rad) | +1 |
| L5_joint | 5 | RS05 | ±5.5 Nm | ±50 rad/s | ±180° (±3.14 rad) | -1 |
| L6_joint | 6 | RS05 | ±5.5 Nm | ±50 rad/s | ±180° (±3.14 rad) | -1 |

---

## 硬件要求

### 必需硬件

- **RS-A3 机械臂** (含 6 个 Robstride 电机)
- **CAN 适配器**: CANdle / gs_usb 兼容设备
- **电源**: 24V/48V 直流电源 (根据电机规格)
- **PC**: Ubuntu 22.04 x86_64

### 可选硬件

- **Xbox 手柄**: 支持有线或蓝牙连接
  - Xbox One 控制器
  - Xbox Series X|S 控制器
  - 其他 XInput 兼容手柄

---

## 软件环境

### 系统要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS 版本**: ROS 2 Humble Hawksbill
- **内核模块**: `gs_usb` (用于 CANdle 适配器)

### 依赖安装

```bash
# 一键安装所有依赖
cd /home/wy/RS/A3/scripts
sudo ./install_deps.sh
```

或手动安装：

```bash
# ROS2 Control 相关包
sudo apt install ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-hardware-interface \
                 ros-humble-controller-manager \
                 ros-humble-joint-state-broadcaster \
                 ros-humble-joint-trajectory-controller

# MoveIt2 相关包
sudo apt install ros-humble-moveit \
                 ros-humble-moveit-ros-move-group \
                 ros-humble-moveit-ros-planning-interface \
                 ros-humble-moveit-ros-visualization \
                 ros-humble-moveit-planners-ompl \
                 ros-humble-moveit-kinematics

# 工具和其他依赖
sudo apt install ros-humble-xacro \
                 ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-rviz2 \
                 ros-humble-joy \
                 can-utils

# Python 依赖
pip3 install python-can scipy
```

### Xbox 手柄蓝牙驱动 (可选)

```bash
# 安装 xpadneo 驱动以获得更好的蓝牙支持
cd /home/wy/RS/A3/scripts
./install_xpadneo.sh
```

---

## 功能包说明

| 功能包 | 说明 |
|--------|------|
| `rs_a3_hardware` | ROS2 Control 硬件接口，实现 CAN 通信驱动 |
| `rs_a3_description` | URDF 机器人描述、ros2_control 配置、控制器参数 |
| `rs_a3_moveit_config` | MoveIt2 运动规划配置 |
| `rs_a3_teleop` | Xbox 手柄实时笛卡尔空间控制 |

---

## 安装配置

### 1. 克隆/复制项目

```bash
# 项目已位于 /home/wy/RS/A3
cd /home/wy/RS/A3
```

### 2. 编译工作空间

```bash
cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. 设置 CAN 接口

```bash
# 连接 CAN 适配器后执行
sudo /home/wy/RS/A3/scripts/setup_can.sh can0 1000000

# 验证接口状态
ip link show can0
candump can0  # 监听 CAN 总线
```

### 4. 配置手柄 (可选)

**有线连接**:
```bash
# 插入 USB 后自动识别
ls /dev/input/js*
```

**蓝牙连接**:
```bash
# 使用配置脚本
./scripts/setup_bluetooth_xbox.sh
```

---

## 快速开始

### 🎮 Xbox 手柄实时控制 (推荐)

**真实硬件模式**:
```bash
# 终端1: 设置 CAN 接口
sudo ./scripts/setup_can.sh can0

# 终端2: 启动控制系统
cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rs_a3_teleop real_teleop.launch.py can_interface:=can0
```

或使用一键脚本:
```bash
./scripts/start_real_xbox_control.sh can0
```

**手柄控制映射**:

| 按键/摇杆 | 功能 |
|-----------|------|
| 左摇杆 Y | X 方向平移 |
| 左摇杆 X | Y 方向平移 |
| LT/RT | Z 方向上下移动 |
| 右摇杆 X | Yaw 旋转 |
| 右摇杆 Y | Pitch 旋转 |
| LB/RB | Roll 旋转 |
| A 键 | 切换速度档位 (5档) |
| B 键 | 回到初始位置 (home) |
| X 键 | 回到零点位置 (所有关节归零) |

**速度档位**:

| 档位 | 名称 | 平移速度 | 旋转速度 |
|------|------|----------|----------|
| 1 | 超慢 | 25 mm/s | 0.25 rad/s |
| 2 | 慢速 | 60 mm/s | 0.6 rad/s |
| 3 | 中速 | 120 mm/s | 1.2 rad/s |
| 4 | 快速 | 240 mm/s | 2.4 rad/s |
| 5 | 极速 | 600 mm/s | 6 rad/s |

### 仿真模式 (MoveIt Demo)

无需真实硬件，使用 mock 硬件进行测试：

```bash
ros2 launch rs_a3_moveit_config demo.launch.py
```

### 仿真模式 + Xbox 手柄控制

使用仿真硬件配合 Xbox 手柄进行测试（无需真实机械臂）：

```bash
ros2 launch rs_a3_teleop sim_teleop.launch.py
```

### 真实硬件 + MoveIt 控制

```bash
# 设置 CAN 接口
sudo ./scripts/setup_can.sh can0

# 启动 MoveIt 控制系统
ros2 launch rs_a3_moveit_config robot.launch.py can_interface:=can0
```

### 仅启动 ros2_control (不带 MoveIt)

```bash
ros2 launch rs_a3_description rs_a3_control.launch.py use_mock_hardware:=false can_interface:=can0
```

---

## 控制参数

### 硬件接口参数 (`rs_a3_ros2_control.xacro`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | can0 | CAN 接口名称 |
| `host_can_id` | 253 (0xFD) | 主机 CAN ID |
| `position_kp` | 80.0 | 位置 PD 控制 Kp 增益 |
| `position_kd` | 4.0 | 位置 PD 控制 Kd 增益 |
| `velocity_limit` | 10.0 | 速度限制 (rad/s) |
| `velocity_filter_alpha` | 0.1 | 速度前馈滤波系数 (0-1，减少起步反冲) |
| `smoothing_alpha` | 0.08 | 低通滤波系数 (0-1) |
| `max_velocity` | 2.0 | 最大速度限制 (rad/s) |
| `max_acceleration` | 8.0 | 最大加速度限制 (rad/s²) |
| `max_jerk` | 50.0 | 最大加加速度限制 (rad/s³) - S曲线规划 |
| `s_curve_enabled` | true | 启用S曲线轨迹规划 |
| `gravity_feedforward_ratio` | 0.5 | 重力补偿前馈比例 (0-1)，50%重力作为前馈力矩 |
| `gravity_comp_L{n}_sin` | 见下表 | 关节n重力补偿sin系数 (Nm) |
| `gravity_comp_L{n}_cos` | 0.0 | 关节n重力补偿cos系数 (Nm) |
| `gravity_comp_L{n}_offset` | 0.0 | 关节n重力补偿偏移量 (Nm) |
| `limit_margin` | 0.15 | 关节限位减速区域 (rad, ~8.6°) |
| `limit_stop_margin` | 0.02 | 关节限位硬停止区域 (rad, ~1.1°) |
| `can_frame_delay_us` | 50 | CAN帧间发送延迟 (μs，防止缓冲区拥塞) |

**重力补偿参数默认值**:

| 关节 | sin_coeff | 说明 |
|------|-----------|------|
| L1 | 0.0 | 基座绕Z轴旋转，无重力影响 |
| L2 | 3.5 | 大臂俯仰，承受主要重力负载 |
| L3 | 2.0 | 小臂俯仰 |
| L4 | 0.0 | 腕部Roll |
| L5 | 0.3 | 腕部Pitch |
| L6 | 0.0 | 腕部Yaw |

重力补偿公式: `τ_ff = (sin_coeff × sin(θ) + cos_coeff × cos(θ) + offset) × gravity_feedforward_ratio`

### 控制器参数 (`rs_a3_controllers.yaml`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `update_rate` | 200 Hz | 控制器更新频率 |
| `state_publish_rate` | 200 Hz | 状态发布频率 |
| `interpolation_method` | splines | 轨迹插值方法 |
| `goal` | 0.03 rad | 目标到达位置容差 |

### Xbox 遥控参数 (`xbox_teleop.yaml`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `update_rate` | 50.0 Hz | 控制循环频率 |
| `use_fast_ik_mode` | true | 使用快速 IK 模式 |
| `max_linear_velocity` | 0.15 | 最大线速度 (m/s) |
| `max_angular_velocity` | 1.5 | 最大角速度 (rad/s) |
| `joint_smoothing_alpha` | 0.15 | 关节输出平滑系数 (0-1，越小越平滑) |
| `max_joint_velocity` | 1.5 | 单关节最大速度 (rad/s) |
| `max_joint_acceleration` | 5.0 | 单关节最大加速度 (rad/s²) |
| `input_smoothing_factor` | 0.3 | 输入平滑滤波系数 |
| `deadzone` | 0.15 | 摇杆死区阈值 |
| `max_ik_jump_threshold` | 0.5 | 单关节最大允许跳变 (rad) |
| `singularity_warning_count` | 5 | 连续拒绝IK解警告阈值 |
| `enable_collision_check` | true | 启用碰撞检测 |

---

## ROS2 接口

### Topics

#### 发布 (Published)

| Topic | 类型 | 频率 | 说明 |
|-------|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 200 Hz | 关节状态 (位置/速度/力矩) |
| `/robot_description` | `std_msgs/String` | latched | URDF 描述 |
| `/target_pose` | `geometry_msgs/PoseStamped` | 50 Hz | 目标末端位姿 |
| `/debug/ik_solution` | `sensor_msgs/JointState` | 50 Hz | IK 解调试信息 |

#### 订阅 (Subscribed)

| Topic | 类型 | 说明 |
|-------|------|------|
| `/joy` | `sensor_msgs/Joy` | Xbox 手柄输入 |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 关节轨迹指令 |

### Services

| Service | 类型 | 说明 |
|---------|------|------|
| `/compute_ik` | `moveit_msgs/GetPositionIK` | 逆运动学求解 |
| `/compute_cartesian_path` | `moveit_msgs/GetCartesianPath` | 笛卡尔路径规划 |

### Actions

| Action | 类型 | 说明 |
|--------|------|------|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | 关节轨迹执行 |
| `/move_action` | `moveit_msgs/MoveGroup` | MoveIt 运动规划 |

### TF Frames

```
base_link
├── L1_joint → l1_link_urdf_asm
│   └── L2_joint → l2_l3_urdf_asm
│       └── L3_joint → l3_lnik_urdf_asm
│           └── L4_joint → l4_l5_urdf_asm
│               └── L5_joint → part_9
│                   └── L6_joint → l5_l6_urdf_asm
│                       └── end_effector
```

---

## 电机通信协议

系统使用灵足时代私有协议与电机通信，采用 **运控模式 (MIT-like)** 进行实时控制。

### 运控模式原理

运控模式下，每个控制周期发送以下参数：

```
τ = Kp × (θ_target - θ_actual) + Kd × (ω_target - ω_actual) + τ_ff
```

| 参数 | 范围 | 说明 |
|------|------|------|
| θ_target | ±12.57 rad | 目标位置 |
| ω_target | 见电机规格 | 目标速度 (速度前馈，位置差分计算) |
| Kp | 0~500 (RS00/RS05) | 位置刚度 |
| Kd | 0~5 (RS00/RS05) | 阻尼系数 |
| τ_ff | 见电机规格 | 前馈力矩 (重力补偿) |

### 通信类型

| 类型 | 功能 | 说明 |
|------|------|------|
| 1 | 运控模式控制 | 发送位置/速度/Kp/Kd/力矩 |
| 2 | 电机反馈 | 接收位置/速度/力矩/温度 |
| 3 | 电机使能 | 启动电机 |
| 4 | 电机停止 | 停止电机 |
| 6 | 设置零位 | 设置当前位置为零点 |
| 18 | 参数写入 | 写入运行模式/位置等参数 |

详细协议说明请参考 [`电机通信协议汇总.md`](./电机通信协议汇总.md)。

---

## 故障排除

### CAN 接口问题

```bash
# 检查 USB 设备
lsusb | grep -i can

# 加载内核模块
sudo modprobe can
sudo modprobe can_raw
sudo modprobe gs_usb

# 检查接口
ip link show type can

# 重启接口
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 监听 CAN 数据
candump can0
```

### 电机无响应

1. 检查 CAN 接线和终端电阻
2. 确认电机 ID 配置正确 (1-6)
3. 检查电源供电
4. 使用 candump 监听数据
5. 检查主机 CAN ID (默认 253/0xFD)

### Xbox 手柄问题

```bash
# 检查手柄设备
ls -la /dev/input/js*
jstest /dev/input/js0

# 检查 joy 节点
ros2 topic echo /joy
```

### 编译错误

```bash
# 清理并重新编译
cd /home/wy/RS/A3/ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 末端抖动问题

1. 调整 `position_kp` 和 `position_kd` 参数
2. 增加 `smoothing_alpha` 值 (更平滑但响应变慢)
3. 检查关节是否有机械间隙
4. 降低控制频率

### 抖动抑制机制

系统采用多级平滑策略消除机械臂运动抖动：

**1. 输入平滑 (Input Smoothing)**
- 参数: `input_smoothing_factor` (默认 0.3)
- 对摇杆原始输入进行一阶低通滤波
- 消除手抖和传感器噪声

**2. 关节输出平滑 (Joint Output Smoothing)**
- 参数: `joint_smoothing_alpha` (默认 0.15)
- 对IK求解结果进行低通滤波
- 公式: `filtered = α × target + (1-α) × previous`

**3. 速度限制 (Velocity Limiting)**
- 参数: `max_joint_velocity` (默认 1.5 rad/s)
- 限制单关节每周期最大位移
- 防止IK解跳变导致的急速运动

**4. 加速度限制 (Acceleration Limiting)**
- 参数: `max_joint_acceleration` (默认 5.0 rad/s²)
- 限制速度变化率，确保平滑加减速
- 避免电机力矩突变

**5. 奇异点保护 (Singularity Protection)**
- 参数: `max_ik_jump_threshold` (默认 0.5 rad)
- 检测并拒绝导致关节大幅跳变的IK解
- 在奇异点附近保持稳定

---

## 目录结构

```
/home/wy/RS/A3/
├── README.md                      # 本文档
├── 电机通信协议汇总.md              # 电机通信协议详细说明
│
├── ros2_ws/                       # ROS2 工作空间
│   └── src/
│       ├── rs_a3_description/     # 机器人描述包
│       │   ├── urdf/
│       │   │   ├── rs_a3.urdf.xacro           # URDF 主文件
│       │   │   └── rs_a3_ros2_control.xacro   # ros2_control 配置
│       │   ├── config/
│       │   │   ├── rs_a3_controllers.yaml     # 控制器参数
│       │   │   └── rs_a3_view.rviz            # RViz 配置
│       │   ├── launch/
│       │   │   └── rs_a3_control.launch.py
│       │   └── meshes/                        # 3D 模型文件
│       │
│       ├── rs_a3_hardware/        # 硬件接口包
│       │   ├── include/rs_a3_hardware/
│       │   │   ├── rs_a3_hardware.hpp         # 硬件接口头文件
│       │   │   ├── robstride_can_driver.hpp   # CAN 驱动头文件
│       │   │   └── s_curve_generator.hpp      # S曲线轨迹生成器
│       │   ├── src/
│       │   │   ├── rs_a3_hardware.cpp         # 硬件接口实现
│       │   │   ├── robstride_can_driver.cpp   # CAN 驱动实现
│       │   │   └── s_curve_generator.cpp      # S曲线轨迹生成器实现
│       │   └── rs_a3_hardware_plugin.xml      # 插件描述
│       │
│       ├── rs_a3_moveit_config/   # MoveIt 配置包
│       │   ├── config/
│       │   │   ├── rs_a3.srdf                 # 语义机器人描述
│       │   │   ├── kinematics.yaml            # 运动学求解器配置
│       │   │   ├── joint_limits.yaml          # 关节限制
│       │   │   ├── ompl_planning.yaml         # OMPL 规划器配置
│       │   │   └── moveit_controllers.yaml    # MoveIt 控制器配置
│       │   └── launch/
│       │       ├── demo.launch.py             # 仿真演示
│       │       └── robot.launch.py            # 真实硬件
│       │
│       └── rs_a3_teleop/          # 手柄遥控包
│           ├── config/
│           │   └── xbox_teleop.yaml           # 手柄参数配置
│           ├── launch/
│           │   ├── real_teleop.launch.py      # 真实硬件遥控
│           │   ├── sim_teleop.launch.py       # 仿真环境遥控
│           │   └── complete_teleop.launch.py  # 完整遥控启动
│           └── rs_a3_teleop/
│               └── xbox_teleop_node.py        # 手柄控制节点
│
├── scripts/                       # 实用脚本
│   ├── setup_can.sh               # CAN 接口设置
│   ├── install_deps.sh            # 依赖安装
│   ├── install_xpadneo.sh         # Xbox 蓝牙驱动安装
│   ├── setup_bluetooth_xbox.sh    # 蓝牙手柄配置
│   ├── start_real_xbox_control.sh # 一键启动脚本
│   ├── move_to_zero.py            # 移动到零位
│   ├── simple_motion_test.py      # 简单运动测试
│   └── foxglove_bridge.service    # Foxglove 远程可视化服务
│
├── RS_A3_urdf/                    # 原始 URDF 和 mesh 文件
│
└── 文档/
    ├── XBOX_CONTROL_SETUP.md      # Xbox 控制详细设置
    ├── XBOX_HOW_TO_USE.md         # Xbox 使用指南
    ├── BLUETOOTH_XBOX_SETUP.md    # 蓝牙设置指南
    └── XBOX_QUICK_FIX.md          # 快速修复指南
```

---

## 脚本使用说明

| 脚本 | 功能 | 使用方法 |
|------|------|----------|
| `setup_can.sh` | 设置 CAN 接口 | `sudo ./setup_can.sh can0 1000000` |
| `install_deps.sh` | 安装 ROS2 依赖 | `sudo ./install_deps.sh` |
| `install_xpadneo.sh` | 安装 Xbox 蓝牙驱动 | `./install_xpadneo.sh` |
| `setup_bluetooth_xbox.sh` | 配置蓝牙手柄 | `./setup_bluetooth_xbox.sh` |
| `start_real_xbox_control.sh` | 一键启动实机控制 | `./start_real_xbox_control.sh can0` |
| `move_to_zero.py` | 移动机械臂到零位 | `python3 move_to_zero.py` |
| `simple_motion_test.py` | 简单运动测试 | `python3 simple_motion_test.py` |

---

## Launch 文件参数

### `rs_a3_teleop/real_teleop.launch.py`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | can0 | CAN 接口名称 |
| `host_can_id` | 253 | 主机 CAN ID |
| `device` | /dev/input/js0 | 手柄设备路径 |

### `rs_a3_moveit_config/robot.launch.py`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | can0 | CAN 接口名称 |
| `host_can_id` | 253 | 主机 CAN ID |
| `use_rviz` | true | 是否启动 RViz |

### `rs_a3_moveit_config/demo.launch.py`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_mock_hardware` | true | 使用仿真硬件 |
| `use_rviz` | true | 是否启动 RViz |

---

## 许可证

Apache-2.0

## 联系方式

如有问题，请联系维护者。

---

**最后更新**: 2026-01-20
