
#!/usr/bin/env python3
"""
Xbox手柄实时控制节点
使用笛卡尔空间控制机械臂末端执行器

控制映射：
- 左摇杆: X/Y 平移
- LT/RT: Z 平移
- 右摇杆: Yaw/Roll 旋转
- LB/RB: Pitch 旋转
- A键: 切换速度档位（5档）
- B键: 回到初始位置（home）
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
import socket
import struct
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    PlanningOptions,
    RobotState,
    JointConstraint,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
import copy

# 使用scipy替代tf_transformations
from scipy.spatial.transform import Rotation as R

def quaternion_from_euler(roll, pitch, yaw):
    """将欧拉角转换为四元数 (x, y, z, w)"""
    rot = R.from_euler('xyz', [roll, pitch, yaw])
    quat = rot.as_quat()  # 返回 (x, y, z, w)
    return quat

def euler_from_quaternion(quaternion):
    """将四元数 (x, y, z, w) 转换为欧拉角"""
    rot = R.from_quat(quaternion)
    euler = rot.as_euler('xyz')
    return euler[0], euler[1], euler[2]  # roll, pitch, yaw


class XboxTeleopNode(Node):
    def __init__(self):
        super().__init__('xbox_teleop_node')
        
        # 声明参数
        self.declare_parameter('update_rate', 50.0)  # Hz - 50Hz提高平滑度
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('end_effector_frame', 'end_effector')
        self.declare_parameter('deadzone', 0.15)  # 摇杆死区
        self.declare_parameter('debug_input', False)  # 调试模式
        self.declare_parameter('use_fast_ik_mode', True)  # 使用快速IK模式实现50Hz控制
        
        # 笛卡尔速度参数（实际物理单位）
        self.declare_parameter('max_linear_velocity', 0.15)   # m/s 最大线速度
        self.declare_parameter('max_angular_velocity', 1.5)   # rad/s 最大角速度
        
        # 关节输出平滑参数
        self.declare_parameter('joint_smoothing_alpha', 0.15)  # 关节平滑系数
        self.declare_parameter('max_joint_velocity', 1.5)      # rad/s 单关节最大速度
        self.declare_parameter('max_joint_acceleration', 5.0)  # rad/s² 单关节最大加速度
        
        # 输入平滑参数
        self.declare_parameter('input_smoothing_factor', 0.5)  # 输入平滑系数
        
        # 奇异点保护参数
        self.declare_parameter('max_ik_jump_threshold', 0.5)   # rad 单关节最大允许跳变
        self.declare_parameter('singularity_warning_count', 5) # 连续拒绝警告阈值
        
        # 碰撞检测参数
        self.declare_parameter('enable_collision_check', True)  # 是否在初始化后启用碰撞检测
        
        # 获取参数
        self.update_rate = self.get_parameter('update_rate').value
        self.planning_group = self.get_parameter('planning_group').value
        self.base_frame = self.get_parameter('base_frame').value
        self.end_effector_frame = self.get_parameter('end_effector_frame').value
        self.deadzone = self.get_parameter('deadzone').value
        self.debug_input = self.get_parameter('debug_input').value
        self.use_fast_ik_mode = self.get_parameter('use_fast_ik_mode').value
        
        # 笛卡尔速度参数
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        
        # 关节平滑参数
        self.joint_smoothing_alpha = self.get_parameter('joint_smoothing_alpha').value
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').value
        self.max_joint_acceleration = self.get_parameter('max_joint_acceleration').value
        self.last_joint_velocities = None  # 上一帧关节速度（用于加速度限制）
        
        # 输入平滑参数
        self.input_smoothing_factor = self.get_parameter('input_smoothing_factor').value
        
        # 奇异点保护参数
        self.max_ik_jump_threshold = self.get_parameter('max_ik_jump_threshold').value
        self.singularity_warning_count = self.get_parameter('singularity_warning_count').value
        self.consecutive_ik_rejects = 0  # 连续拒绝计数器
        self.ik_seed_just_initialized = False  # IK种子刚初始化标志，跳过首帧跳变检测
        
        # 碰撞检测参数
        self.enable_collision_check = self.get_parameter('enable_collision_check').value
        self.collision_check_active = False  # 初始化期间关闭，完成后开启
        
        # 计算时间步长
        self.dt = 1.0 / self.update_rate
        
        # 调试计数器 - 避免日志刷屏
        self.debug_counter = 0
        
        # 速度档位设置 (5档) - 使用速度比例因子
        # 实际速度 = max_velocity * factor
        self.speed_levels = [
            {'name': '超慢', 'factor': 0.1},   # 10% 最大速度
            {'name': '慢速', 'factor': 0.25},  # 25% 最大速度
            {'name': '中速', 'factor': 0.5},   # 50% 最大速度
            {'name': '快速', 'factor': 0.75},  # 75% 最大速度
            {'name': '极速', 'factor': 1.0},   # 100% 最大速度
        ]
        self.current_speed_level = 2  # 默认中速 (索引从0开始)
        self.speed_factor = self.speed_levels[self.current_speed_level]['factor']
        
        # 按钮状态跟踪（用于防抖动）
        self.last_a_button = 0
        self.last_b_button = 0
        self.last_x_button = 0
        self.is_going_home = False  # 是否正在回home/zero
        
        # 方向键控制夹爪电机（ID=7）
        self.last_dpad_up = 0
        self.last_dpad_down = 0
        self.gripper_torque = 0.0  # 当前夹爪力矩
        self.gripper_torque_step = 0.4  # 每次按键增加的力矩 (Nm)
        self.gripper_motor_id = 7
        
        # 初始化CAN socket用于直接发送夹爪命令
        self.can_socket = None
        self.gripper_enabled = False
        try:
            self.can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.can_socket.bind(('can0',))
            self.get_logger().info('CAN socket初始化成功，夹爪控制已启用')
            # 使能夹爪电机
            self.enable_gripper_motor()
        except Exception as e:
            self.get_logger().warn(f'CAN socket初始化失败，夹爪控制禁用: {e}')
        
        # 输入平滑滤波器状态（指数移动平均）
        self.smoothed_vx = 0.0  # 平滑后的X速度
        self.smoothed_vy = 0.0  # 平滑后的Y速度
        self.smoothed_vz = 0.0  # 平滑后的Z速度
        self.smoothed_vroll = 0.0  # 平滑后的Roll角速度
        self.smoothed_vpitch = 0.0  # 平滑后的Pitch角速度
        self.smoothed_vyaw = 0.0  # 平滑后的Yaw角速度
        
        # 关节输出平滑状态
        self.smoothed_joint_positions = None  # 平滑后的关节位置
        
        # 累积增量（用于在移动时累积输入）
        self.pending_dx = 0.0
        self.pending_dy = 0.0
        self.pending_dz = 0.0
        self.pending_droll = 0.0
        self.pending_dpitch = 0.0
        self.pending_dyaw = 0.0
        
        # 订阅手柄输入
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # TF2监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 目标位姿发布器
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/target_pose',
            10
        )
        
        # 笛卡尔路径服务客户端
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )
        
        # IK服务客户端 - 用于快速控制
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )
        
        # 直接关节轨迹发布 - 用于快速控制
        # 使用BEST_EFFORT QoS匹配arm_controller的订阅设置
        trajectory_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            trajectory_qos
        )
        
        # 关节轨迹Action客户端 - 用于直接发送轨迹
        self.follow_joint_trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        # 订阅当前关节状态
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 关节名称
        self.joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
        
        # 【调试】IK解发布器 - 用于Foxglove跟踪
        self.ik_solution_pub = self.create_publisher(
            JointState,
            '/debug/ik_solution',
            10
        )
        
        # 【调试】原始IK解发布器（处理前）- 用于对比
        self.ik_raw_solution_pub = self.create_publisher(
            JointState,
            '/debug/ik_raw_solution',
            10
        )
        
        # IK请求状态跟踪（用于快速IK模式）
        self.pending_ik_request = False  # 是否有待处理的IK请求
        self.last_ik_joint_positions = None  # 缓存上一次的IK关节位置
        
        # 执行轨迹Action客户端
        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )
        
        # MoveGroup Action客户端（用于回home）
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        
        # 状态变量
        self.current_joy = None
        self.target_pose = None
        self.is_moving = False
        self.last_update_time = self.get_clock().now()
        self.pose_initialized = False  # 标记位姿是否已正确初始化
        self.last_sent_pose = None  # 上次发送的位姿，用于检测变化
        self.min_move_threshold = 0.00005  # 最小移动阈值(米)，比translation_scale小
        self.move_start_time = None  # 运动开始时间，用于超时检测
        self.move_timeout = 2.0  # 运动超时时间(秒)
        
        # 等待服务
        if self.use_fast_ik_mode:
            self.get_logger().info('等待IK服务（快速模式）...')
            self.ik_client.wait_for_service()
            self.get_logger().info('IK服务已连接 - 使用50Hz快速IK控制模式')
        else:
            self.get_logger().info('等待笛卡尔路径服务...')
            self.cartesian_path_client.wait_for_service()
        self.get_logger().info('等待执行轨迹服务...')
        self.execute_trajectory_client.wait_for_server()
        self.get_logger().info('等待MoveGroup服务...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('服务已连接')
        
        # 【启动时自动回home】
        # 1. 先等待关节状态可用（确保读取到电机位置）
        self.get_logger().info('等待关节状态（读取电机位置）...')
        self.wait_for_joint_states()
        
        # 2. 打印当前关节位置
        if self.current_joint_state:
            self.get_logger().info('当前电机位置已读取:')
            for i, name in enumerate(self.current_joint_state.name):
                if i < len(self.current_joint_state.position):
                    self.get_logger().info(f'  {name}: {self.current_joint_state.position[i]:.4f} rad')
        
        # 3. 自动运动到零点位置
        self.get_logger().info('正在规划运动至零点位置...')
        self.startup_go_zero()
        
        # 创建定时器，定期更新目标位姿
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_callback)
        
        self.get_logger().info('Xbox手柄控制节点已启动')
        self.get_logger().info(f'控制组: {self.planning_group}')
        self.get_logger().info(f'末端执行器: {self.end_effector_frame}')
        self.get_logger().info(f'更新频率: {self.update_rate} Hz (dt={self.dt*1000:.1f}ms)')
        self.get_logger().info(f'最大线速度: {self.max_linear_velocity} m/s')
        self.get_logger().info(f'最大角速度: {self.max_angular_velocity} rad/s')
        self.get_logger().info(f'关节平滑: alpha={self.joint_smoothing_alpha}, max_vel={self.max_joint_velocity} rad/s')
        if self.use_fast_ik_mode:
            self.get_logger().info('控制模式: 快速IK模式（50Hz笛卡尔控制）')
        else:
            self.get_logger().info('控制模式: 传统路径规划模式')
        self.get_logger().info('=== 控制说明 ===')
        self.get_logger().info('左摇杆: XY平移 | LT/RT: Z平移')
        self.get_logger().info('右摇杆: Yaw/Roll | LB/RB: Pitch')
        self.get_logger().info('A键: 切换速度档位 | B键: 回到home位置 | X键: 回到零点')
        self.log_speed_level()
        
    def sync_current_pose(self):
        """从TF同步当前末端执行器位姿作为目标位姿起点
        
        每次调用都会获取最新的实际位姿，确保增量运动基于正确的起点。
        
        Returns:
            bool: 成功返回True，失败返回False
        """
        try:
            # 尝试获取TF变换（使用较短的超时时间）
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.end_effector_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)  # 缩短超时，提高响应速度
            )
            
            # 每次都更新目标位姿为当前实际位姿
            self.target_pose = PoseStamped()
            self.target_pose.header.frame_id = self.base_frame
            self.target_pose.pose.position.x = transform.transform.translation.x
            self.target_pose.pose.position.y = transform.transform.translation.y
            self.target_pose.pose.position.z = transform.transform.translation.z
            self.target_pose.pose.orientation = transform.transform.rotation
            
            if not self.pose_initialized:
                self.get_logger().info(f'首次位姿同步: x={self.target_pose.pose.position.x:.4f}, '
                                     f'y={self.target_pose.pose.position.y:.4f}, '
                                     f'z={self.target_pose.pose.position.z:.4f}')
                # 【关键】如果IK种子已经被预设（如从home/zero回调中），则保留它
                # 否则从当前关节状态初始化
                if self.last_ik_joint_positions is None:
                    # 只有IK种子未设置时，才从joint_states初始化
                    if self.current_joint_state:
                        seed_positions = []
                        for joint_name in self.joint_names:
                            if joint_name in self.current_joint_state.name:
                                idx = self.current_joint_state.name.index(joint_name)
                                seed_positions.append(self.current_joint_state.position[idx])
                            else:
                                seed_positions.append(0.0)  # 默认值
                        self.last_ik_joint_positions = seed_positions
                        self.smoothed_joint_positions = list(seed_positions)
                        self.last_joint_velocities = [0.0] * len(seed_positions)
                        self.get_logger().info(f'从关节状态初始化IK种子: {[f"{p:.3f}" for p in self.last_ik_joint_positions]}')
                        self.ik_seed_just_initialized = True
                        self.consecutive_ik_rejects = 0
                else:
                    # IK种子已预设（来自home/zero回调），只需同步笛卡尔位姿
                    self.get_logger().info(f'使用预设IK种子: {[f"{p:.3f}" for p in self.last_ik_joint_positions]}')
                self.pose_initialized = True
            
            return True
            
        except Exception as e:
            self.get_logger().debug(f'TF查询失败: {str(e)}')
            return False
            
    def joint_state_callback(self, msg):
        """关节状态回调"""
        self.current_joint_state = msg
    
    def wait_for_joint_states(self, timeout_sec=10.0):
        """等待关节状态可用（确保已读取电机位置）"""
        import time
        start_time = time.time()
        while self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout_sec:
                self.get_logger().warn('等待关节状态超时！使用默认位置')
                return False
        return True
    
    def startup_go_zero(self):
        """启动时运动到零点位置（同步执行），完成后开启碰撞检测"""
        import time
        
        self.is_going_home = True
        
        # 等待MoveGroup服务完全准备好（包括规划场景）
        self.get_logger().info('等待MoveGroup服务准备就绪...')
        time.sleep(3.0)  # 等待MoveGroup初始化完成
        
        # 重试机制
        max_retries = 3
        for retry in range(max_retries):
            # 创建MoveGroup请求
            goal_msg = MoveGroup.Goal()
            goal_msg.request.group_name = self.planning_group
            goal_msg.request.num_planning_attempts = 10
            goal_msg.request.allowed_planning_time = 10.0
            goal_msg.request.max_velocity_scaling_factor = 0.15  # 启动时更慢速运动更安全
            goal_msg.request.max_acceleration_scaling_factor = 0.15
            
            # 设置目标关节位置 (零点位置)
            # 零点位置: 所有关节归零
            joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
            joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 所有关节归零
            
            constraints = Constraints()
            for name, value in zip(joint_names, joint_values):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = name
                joint_constraint.position = value
                joint_constraint.tolerance_above = 0.01
                joint_constraint.tolerance_below = 0.01
                joint_constraint.weight = 1.0
                constraints.joint_constraints.append(joint_constraint)
            
            goal_msg.request.goal_constraints = [constraints]
            goal_msg.planning_options.plan_only = False
            goal_msg.planning_options.replan = True
            goal_msg.planning_options.replan_attempts = 3
            
            # 发送请求并等待结果
            self.get_logger().info(f'发送零点运动请求... (尝试 {retry + 1}/{max_retries})')
            send_goal_future = self.move_group_client.send_goal_async(goal_msg)
            
            # 等待goal被接受
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
            goal_handle = send_goal_future.result()
            
            if goal_handle and goal_handle.accepted:
                self.get_logger().info('零点运动请求已接受，正在缓慢执行...')
                
                # 等待执行完成
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
                
                result = result_future.result()
                self.is_going_home = False
                
                # 检查结果
                try:
                    if result and result.result and result.result.error_code.val == result.result.error_code.SUCCESS:
                        self.get_logger().info('✓ 已缓慢运动到零点位置！')
                        # 【改进】直接使用零点位置作为起点，避免TF查询误差
                        zero_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                        self.last_ik_joint_positions = zero_joint_positions
                        self.smoothed_joint_positions = list(zero_joint_positions)
                        self.last_joint_velocities = [0.0] * 6
                        self.ik_seed_just_initialized = True
                        self.consecutive_ik_rejects = 0
                        self.pose_initialized = False  # 仍需从TF获取笛卡尔位姿
                        self.get_logger().info(f'IK种子已设置为零点位置')
                        # 运动成功后开启碰撞检测
                        if self.enable_collision_check:
                            self.collision_check_active = True
                            self.get_logger().info('✓ 碰撞检测已开启')
                        return True
                    else:
                        error_code = 'unknown'
                        if result and result.result and hasattr(result.result, 'error_code'):
                            error_code = result.result.error_code.val
                        self.get_logger().warn(f'零点运动执行失败: error_code={error_code}')
                        # 失败时重置状态
                        self.pose_initialized = False
                        self.smoothed_joint_positions = None
                        self.last_ik_joint_positions = None
                except Exception as e:
                    self.get_logger().warn(f'零点运动结果解析失败: {e}')
                    self.pose_initialized = False
                    self.smoothed_joint_positions = None
                    self.last_ik_joint_positions = None
            else:
                self.get_logger().warn(f'零点运动请求被拒绝，等待重试...')
                time.sleep(2.0)  # 等待后重试
        
        self.get_logger().error('零点运动多次尝试后失败，请手动控制机械臂')
        self.is_going_home = False
        self.pose_initialized = False
        self.smoothed_joint_positions = None
        self.last_ik_joint_positions = None
        return False
    
    def joy_callback(self, msg):
        """手柄输入回调"""
        self.current_joy = msg
        
        # 调试模式：周期性打印原始输入
        if self.debug_input:
            self.debug_counter += 1
            if self.debug_counter >= 40:  # 每2秒打印一次（假设20Hz）
                self.debug_counter = 0
                if len(msg.axes) >= 6:
                    self.get_logger().info(
                        f'摇杆原始值: 左X={msg.axes[0]:.3f} 左Y={msg.axes[1]:.3f} '
                        f'LT={msg.axes[2]:.3f} 右X={msg.axes[3]:.3f} '
                        f'右Y={msg.axes[4]:.3f} RT={msg.axes[5]:.3f}'
                    )
        
    def apply_deadzone(self, value):
        """应用摇杆死区"""
        if abs(value) < self.deadzone:
            return 0.0
        # 线性映射死区外的值
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def apply_trigger_deadzone(self, trigger_value):
        """应用扳机死区 - LT/RT专用"""
        # 扳机范围是 1.0(未按) 到 -1.0(完全按下)
        # 转换为 0.0(未按) 到 1.0(完全按下)
        normalized = (1.0 - trigger_value) / 2.0
        
        # 应用死区 (扳机死区稍大一些)
        trigger_deadzone = self.deadzone * 1.5  # 扳机需要更大的死区
        if normalized < trigger_deadzone:
            return 0.0
        # 线性映射
        return (normalized - trigger_deadzone) / (1.0 - trigger_deadzone)
    
    def log_speed_level(self):
        """打印当前速度档位"""
        level = self.speed_levels[self.current_speed_level]
        actual_linear = self.max_linear_velocity * self.speed_factor
        actual_angular = self.max_angular_velocity * self.speed_factor
        self.get_logger().info(
            f'当前速度档位: {self.current_speed_level + 1}/5 [{level["name"]}] '
            f'(线速度={actual_linear*1000:.0f}mm/s, 角速度={actual_angular:.2f}rad/s)'
        )
    
    def switch_speed_level(self):
        """切换速度档位"""
        self.current_speed_level = (self.current_speed_level + 1) % len(self.speed_levels)
        level = self.speed_levels[self.current_speed_level]
        self.speed_factor = level['factor']
        self.log_speed_level()
    
    def go_home(self):
        """回到初始位置（home）"""
        if self.is_moving or self.is_going_home:
            self.get_logger().warn('正在执行其他动作，请稍后再试')
            return
        
        self.is_going_home = True
        self.get_logger().info('正在回到初始位置...')
        
        # 创建MoveGroup请求
        goal_msg = MoveGroup.Goal()
        
        # 设置运动请求
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        
        # 设置目标为命名位置 "home"
        goal_msg.request.goal_constraints = []
        
        # 使用关节约束来定义home位置
        # home位置: L1=0, L2=45°, L3=-45°, L4=0, L5=0, L6=0
        joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
        joint_values = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]  # 弧度
        
        constraints = Constraints()
        for name, value in zip(joint_names, joint_values):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = value
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        goal_msg.request.goal_constraints = [constraints]
        
        # 设置规划选项
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3
        
        # 发送请求
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.home_response_callback)
    
    def home_response_callback(self, future):
        """回home响应回调"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('回home请求被拒绝')
            self.is_going_home = False
            return
        
        self.get_logger().info('回home请求已接受，正在执行...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.home_result_callback)
    
    def home_result_callback(self, future):
        """回home结果回调"""
        result = future.result().result
        self.is_going_home = False
        
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('已回到初始位置！')
            # 【改进】直接使用home位置作为起点，避免TF查询误差
            home_joint_positions = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]
            self.last_ik_joint_positions = home_joint_positions
            self.smoothed_joint_positions = list(home_joint_positions)
            self.last_joint_velocities = [0.0] * 6
            self.ik_seed_just_initialized = True
            self.consecutive_ik_rejects = 0
            # 仍然让sync_current_pose()获取笛卡尔位姿（从TF）
            self.pose_initialized = False
            self.get_logger().info(f'IK种子已设置为home位置: {[f"{p:.3f}" for p in home_joint_positions]}')
        else:
            self.get_logger().warn(f'回home失败: error_code={result.error_code.val}')
            self.pose_initialized = False
            self.smoothed_joint_positions = None
            self.last_ik_joint_positions = None
    
    def go_zero(self):
        """回到零点位置（所有关节归零）"""
        if self.is_moving or self.is_going_home:
            self.get_logger().warn('正在执行其他动作，请稍后再试')
            return
        
        self.is_going_home = True
        self.get_logger().info('正在回到零点位置...')
        
        # 创建MoveGroup请求
        goal_msg = MoveGroup.Goal()
        
        # 设置运动请求
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        
        # 设置目标为零点位置
        # 零点位置: 所有关节都为0
        joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
        joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 弧度
        
        constraints = Constraints()
        for name, value in zip(joint_names, joint_values):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = value
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        goal_msg.request.goal_constraints = [constraints]
        
        # 设置规划选项
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3
        
        # 发送请求
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.zero_response_callback)
    
    def zero_response_callback(self, future):
        """回零点响应回调"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('回零点请求被拒绝')
            self.is_going_home = False
            return
        
        self.get_logger().info('回零点请求已接受，正在执行...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.zero_result_callback)
    
    def zero_result_callback(self, future):
        """回零点结果回调"""
        result = future.result().result
        self.is_going_home = False
        
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('已回到零点位置！')
            # 【改进】直接使用零点位置作为起点，避免TF查询误差
            zero_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.last_ik_joint_positions = zero_joint_positions
            self.smoothed_joint_positions = list(zero_joint_positions)
            self.last_joint_velocities = [0.0] * 6
            self.ik_seed_just_initialized = True
            self.consecutive_ik_rejects = 0
            # 仍然让sync_current_pose()获取笛卡尔位姿（从TF）
            self.pose_initialized = False
            self.get_logger().info(f'IK种子已设置为零点位置: {[f"{p:.3f}" for p in zero_joint_positions]}')
        else:
            self.get_logger().warn(f'回零点失败: error_code={result.error_code.val}')
            self.pose_initialized = False
            self.smoothed_joint_positions = None
            self.last_ik_joint_positions = None
        
    def update_callback(self):
        """定期更新回调"""
        if self.current_joy is None:
            return
        
        joy = self.current_joy
        
        # 确保索引有效
        if len(joy.axes) < 6 or len(joy.buttons) < 6:
            return
        
        # 处理A键 - 切换速度档位（按下触发，防抖动）
        a_button = joy.buttons[0]
        if a_button == 1 and self.last_a_button == 0:
            self.switch_speed_level()
        self.last_a_button = a_button
        
        # 处理B键 - 回到home位置（按下触发，防抖动）
        b_button = joy.buttons[1]
        if b_button == 1 and self.last_b_button == 0:
            self.go_home()
        self.last_b_button = b_button
        
        # 处理X键 - 回到零点（按下触发，防抖动）
        x_button = joy.buttons[2]
        if x_button == 1 and self.last_x_button == 0:
            self.go_zero()
        self.last_x_button = x_button
        
        # 处理方向键 - 控制夹爪电机(ID=7)力矩
        # Xbox手柄方向键: axes[6]=左右, axes[7]=上下 (上=1, 下=-1)
        if len(joy.axes) >= 8 and self.can_socket is not None:
            dpad_vertical = joy.axes[7]  # 上=1, 下=-1
            
            # 方向键上 - 增加力矩
            dpad_up = 1 if dpad_vertical > 0.5 else 0
            if dpad_up == 1 and self.last_dpad_up == 0:
                self.gripper_torque = self.gripper_torque_step
                self.send_gripper_torque(self.gripper_torque)
                self.get_logger().info(f'夹爪力矩: +{self.gripper_torque:.2f} Nm')
            elif dpad_up == 0 and self.last_dpad_up == 1:
                # 松开按键时发送0力矩
                self.gripper_torque = 0.0
                self.send_gripper_torque(0.0)
            self.last_dpad_up = dpad_up
            
            # 方向键下 - 减少力矩
            dpad_down = 1 if dpad_vertical < -0.5 else 0
            if dpad_down == 1 and self.last_dpad_down == 0:
                self.gripper_torque = -self.gripper_torque_step
                self.send_gripper_torque(self.gripper_torque)
                self.get_logger().info(f'夹爪力矩: {self.gripper_torque:.2f} Nm')
            elif dpad_down == 0 and self.last_dpad_down == 1:
                # 松开按键时发送0力矩
                self.gripper_torque = 0.0
                self.send_gripper_torque(0.0)
            self.last_dpad_down = dpad_down
            
        # 如果正在回home/zero，跳过普通运动控制
        if self.is_going_home:
            return
            
        # 解析手柄输入 -> 笛卡尔速度映射
        # Xbox手柄映射:
        # 左摇杆 X: axes[0], 左摇杆 Y: axes[1]
        # LT: axes[2], RT: axes[5] (范围 1.0 到 -1.0，按下为-1.0)
        # 右摇杆 X: axes[3], 右摇杆 Y: axes[4]
        # LB: buttons[4], RB: buttons[5]
        # A: buttons[0], B: buttons[1]
        
        # ============ 笛卡尔速度映射 ============
        # 摇杆输入 → 速度(m/s, rad/s) → 位置增量
        
        # 计算当前档位下的实际最大速度
        current_max_linear = self.max_linear_velocity * self.speed_factor
        current_max_angular = self.max_angular_velocity * self.speed_factor
        
        # 平移速度 - 左摇杆XY对调并反向
        raw_vx = -self.apply_deadzone(joy.axes[1]) * current_max_linear  # 左摇杆Y -> X速度 (m/s)
        raw_vy = -self.apply_deadzone(joy.axes[0]) * current_max_linear  # 左摇杆X -> Y速度 (m/s)
        
        # LT/RT控制Z速度
        lt = self.apply_trigger_deadzone(joy.axes[2])  # 转换为0-1范围并应用死区
        rt = self.apply_trigger_deadzone(joy.axes[5])  # 转换为0-1范围并应用死区
        raw_vz = (rt - lt) * current_max_linear  # RT向上，LT向下 (m/s)
        
        # 旋转速度
        raw_vyaw = self.apply_deadzone(joy.axes[3]) * current_max_angular   # 右摇杆X -> Yaw速度 (rad/s)
        raw_vroll = self.apply_deadzone(joy.axes[4]) * current_max_angular  # 右摇杆Y -> Roll速度 (rad/s)
        
        # LB/RB控制Pitch速度
        raw_vpitch = (joy.buttons[5] - joy.buttons[4]) * current_max_angular  # RB正，LB负 (rad/s)
        
        # 应用输入平滑滤波器（指数移动平均 EMA）
        alpha = self.input_smoothing_factor
        self.smoothed_vx = alpha * raw_vx + (1 - alpha) * self.smoothed_vx
        self.smoothed_vy = alpha * raw_vy + (1 - alpha) * self.smoothed_vy
        self.smoothed_vz = alpha * raw_vz + (1 - alpha) * self.smoothed_vz
        self.smoothed_vroll = alpha * raw_vroll + (1 - alpha) * self.smoothed_vroll
        self.smoothed_vpitch = alpha * raw_vpitch + (1 - alpha) * self.smoothed_vpitch
        self.smoothed_vyaw = alpha * raw_vyaw + (1 - alpha) * self.smoothed_vyaw
        
        # 速度 × dt = 位置增量
        dx = self.smoothed_vx * self.dt
        dy = self.smoothed_vy * self.dt
        dz = self.smoothed_vz * self.dt
        droll = self.smoothed_vroll * self.dt
        dpitch = self.smoothed_vpitch * self.dt
        dyaw = self.smoothed_vyaw * self.dt
        
        # 检查是否有输入 - 基于速度阈值
        velocity_threshold = 0.001  # 1mm/s 以下视为无输入
        angular_threshold = 0.01    # 0.01 rad/s 以下视为无输入
        translation_threshold = velocity_threshold * self.dt
        rotation_threshold = angular_threshold * self.dt
        
        has_translation = abs(dx) > translation_threshold or abs(dy) > translation_threshold or abs(dz) > translation_threshold
        has_rotation = abs(dyaw) > rotation_threshold or abs(dpitch) > rotation_threshold or abs(droll) > rotation_threshold
        
        if not has_translation and not has_rotation:
            # 【修改】即使没有输入，也持续计算IK，确保调试话题有最新数据
            # 同时可以观察IK解在静止状态下的稳定性
            if self.use_fast_ik_mode and self.pose_initialized:
                self.send_cartesian_goal()  # 持续调用IK计算当前目标位姿
            return  # 没有有效输入，不更新目标位姿
        
        # 累积增量
        self.pending_dx += dx
        self.pending_dy += dy
        self.pending_dz += dz
        self.pending_droll += droll
        self.pending_dpitch += dpitch
        self.pending_dyaw += dyaw
        
        # 【关键修改】只在首次时同步位姿，之后增量累加到目标位姿上
        # 这样避免因为实机跟踪误差（如重力导致下垂）导致z轴持续下降
        if not self.pose_initialized:
            if not self.sync_current_pose():
                self.get_logger().warn('等待TF树建立...')
                return  # TF还没准备好，跳过这次更新
        
        # 使用累积的增量
        dx = self.pending_dx
        dy = self.pending_dy
        dz = self.pending_dz
        droll = self.pending_droll
        dpitch = self.pending_dpitch
        dyaw = self.pending_dyaw
        
        # 清空累积
        self.pending_dx = 0.0
        self.pending_dy = 0.0
        self.pending_dz = 0.0
        self.pending_droll = 0.0
        self.pending_dpitch = 0.0
        self.pending_dyaw = 0.0
            
        # 保存上一次的目标位置用于检测跳变
        if hasattr(self, 'last_target_x'):
            position_jump = math.sqrt(
                (self.target_pose.pose.position.x - self.last_target_x) ** 2 +
                (self.target_pose.pose.position.y - self.last_target_y) ** 2 +
                (self.target_pose.pose.position.z - self.last_target_z) ** 2
            )
            # 检测位置跳变（超过50mm警告）
            if position_jump > 0.05:
                self.get_logger().warn(f'检测到位置跳变！距离={position_jump*1000:.1f}mm')
        
        # 调试：打印实际输入值（仅当有输入时）
        if self.debug_input:
            self.get_logger().info(
                f'输入: dx={dx:.6f} dy={dy:.6f} dz={dz:.6f} '
                f'droll={droll:.6f} dpitch={dpitch:.6f} dyaw={dyaw:.6f}'
            )
            
        # 更新目标位姿
        # 平移
        self.target_pose.pose.position.x += dx
        self.target_pose.pose.position.y += dy
        self.target_pose.pose.position.z += dz
        
        # 旋转 - 将当前四元数转换为欧拉角
        q = self.target_pose.pose.orientation
        current_roll, current_pitch, current_yaw = euler_from_quaternion([
            q.x, q.y, q.z, q.w
        ])
        
        # 更新欧拉角
        new_roll = current_roll + droll
        new_pitch = current_pitch + dpitch
        new_yaw = current_yaw + dyaw
        
        # 转换回四元数
        new_q = quaternion_from_euler(new_roll, new_pitch, new_yaw)
        self.target_pose.pose.orientation.x = new_q[0]
        self.target_pose.pose.orientation.y = new_q[1]
        self.target_pose.pose.orientation.z = new_q[2]
        self.target_pose.pose.orientation.w = new_q[3]
        
        # 保存当前目标位置用于下次跳变检测
        self.last_target_x = self.target_pose.pose.position.x
        self.last_target_y = self.target_pose.pose.position.y
        self.last_target_z = self.target_pose.pose.position.z
        
        # 发布目标位姿用于可视化
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose_pub.publish(self.target_pose)
        
        # 发送运动指令
        self.send_cartesian_goal()
        
    def pose_changed_significantly(self):
        """检查目标位姿是否发生了明显变化"""
        if self.last_sent_pose is None:
            return True
        
        # 计算位置变化
        dx = abs(self.target_pose.pose.position.x - self.last_sent_pose.pose.position.x)
        dy = abs(self.target_pose.pose.position.y - self.last_sent_pose.pose.position.y)
        dz = abs(self.target_pose.pose.position.z - self.last_sent_pose.pose.position.z)
        
        # 如果位置变化超过阈值，认为发生了明显变化
        if dx > self.min_move_threshold or dy > self.min_move_threshold or dz > self.min_move_threshold:
            return True
        
        # 检查姿态变化（简化检查四元数w分量的变化）
        dw = abs(self.target_pose.pose.orientation.w - self.last_sent_pose.pose.orientation.w)
        if dw > 0.01:  # 约1度的变化
            return True
        
        return False
    
    def send_cartesian_goal(self):
        """发送笛卡尔目标 - 根据模式选择快速IK或路径规划"""
        if self.use_fast_ik_mode:
            self.send_ik_goal()
        else:
            self.send_cartesian_path_goal()
    
    def send_ik_goal(self):
        """使用IK服务计算关节位置，直接发布轨迹 - 50Hz快速模式"""
        # 允许覆盖旧请求，但避免请求堆积
        if self.pending_ik_request:
            return  # 上一个IK请求还在处理中
        
        # 调试日志
        self.get_logger().debug('发送IK请求...')
        
        # 构建IK请求
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.planning_group
        request.ik_request.robot_state.is_diff = False  # 使用完整状态而非差分
        
        # 【关键修复】使用上一次成功的IK解作为种子，保持解的连续性
        # 这样避免因重力下垂导致的位姿跳变
        if self.last_ik_joint_positions is not None:
            # 使用上一次IK解作为种子
            seed_state = JointState()
            seed_state.name = self.joint_names
            seed_state.position = self.last_ik_joint_positions
            request.ik_request.robot_state.joint_state = seed_state
        elif self.current_joint_state:
            # 首次使用当前关节状态
            request.ik_request.robot_state.joint_state = self.current_joint_state
        
        # 设置目标末端位姿
        request.ik_request.pose_stamped.header.frame_id = self.base_frame
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = self.target_pose.pose
        
        # 设置超时（尽量短以保持高频率）
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = 10_000_000  # 10ms超时
        
        # 设置碰撞检测
        request.ik_request.avoid_collisions = self.collision_check_active
        
        # 异步调用IK服务
        self.pending_ik_request = True
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_callback)
    
    def ik_callback(self, future):
        """IK计算完成回调 - 发布关节轨迹"""
        self.pending_ik_request = False
        
        try:
            response = future.result()
            
            # 检查IK是否成功 (SUCCESS = 1)
            if response.error_code.val != 1:
                # IK失败，可能是目标不可达
                self.get_logger().debug(f'IK失败: error_code={response.error_code.val}')
                return
            
            # 提取关节位置
            solution = response.solution.joint_state
            ik_positions = []
            
            for joint_name in self.joint_names:
                if joint_name in solution.name:
                    idx = solution.name.index(joint_name)
                    ik_positions.append(solution.position[idx])
                else:
                    return  # 缺少关节数据
            
            # 【调试】发布原始IK解（任何处理之前）
            self.publish_ik_raw_debug(ik_positions)
            
            # 检查IK解变化是否太小或太大
            if self.last_ik_joint_positions is not None:
                max_diff = max(abs(ik_positions[i] - self.last_ik_joint_positions[i]) for i in range(len(ik_positions)))
                
                # 变化太小，跳过
                if max_diff < 0.0001:
                    return
                
                # 【奇异点保护】变化过大，拒绝该IK解
                # 但如果IK种子刚初始化，跳过首帧检测以建立正确基准
                if max_diff > self.max_ik_jump_threshold:
                    if self.ik_seed_just_initialized:
                        # 首帧：接受IK解，清除标志
                        self.get_logger().info(f'IK种子初始化后首帧，接受IK解 (diff={max_diff:.3f}rad)')
                        self.ik_seed_just_initialized = False
                    else:
                        self.consecutive_ik_rejects += 1
                        if self.consecutive_ik_rejects >= self.singularity_warning_count:
                            self.get_logger().warn(
                                f'检测到奇异点区域，IK解跳变={max_diff:.3f}rad，已保护{self.consecutive_ik_rejects}帧'
                            )
                        # 【自动恢复机制】连续拒绝超过50帧后，重新同步IK种子
                        if self.consecutive_ik_rejects >= 50:
                            self.get_logger().warn('连续拒绝IK解超过50帧，自动重新同步位姿...')
                            self.pose_initialized = False
                            self.smoothed_joint_positions = None
                            self.last_ik_joint_positions = None
                            self.consecutive_ik_rejects = 0
                        # 拒绝该解，继续使用上一次的关节位置
                        return
                else:
                    # IK解正常，重置计数器和标志
                    if self.consecutive_ik_rejects > 0:
                        self.consecutive_ik_rejects = 0
                    if self.ik_seed_just_initialized:
                        self.ik_seed_just_initialized = False
            
            # 【关键】应用关节输出平滑滤波
            # 避免IK解跳变导致的机械冲击
            target_positions = self.smooth_joint_positions(ik_positions)
            self.last_ik_joint_positions = target_positions
            
            # 【调试】发布IK解到/debug/ik_solution
            self.publish_ik_debug(target_positions)
            
            # 构建并发布关节轨迹
            trajectory = JointTrajectory()
            trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory.joint_names = self.joint_names
            
            # 单点轨迹 - 短执行时间让硬件层做平滑
            point = JointTrajectoryPoint()
            point.positions = target_positions
            point.velocities = [0.0] * 6  # 目标速度为0
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 20_000_000  # 20ms到达（50Hz周期）
            
            trajectory.points = [point]
            
            # 直接发布轨迹到控制器
            self.joint_trajectory_pub.publish(trajectory)
            
            # 统计IK成功次数
            if not hasattr(self, 'ik_success_count'):
                self.ik_success_count = 0
            self.ik_success_count += 1
            if self.ik_success_count % 50 == 0:  # 每50次打印一次
                self.get_logger().info(f'IK控制中: 已发送{self.ik_success_count}次轨迹指令')
            
            # 更新上次发送的位姿（用于变化检测）
            self.last_sent_pose = copy.deepcopy(self.target_pose)
            
        except Exception as e:
            self.get_logger().error(f'IK回调异常: {e}')
    
    def smooth_joint_positions(self, target_positions):
        """对IK输出的关节位置进行平滑滤波（改进版：临界阻尼+死区）
        
        使用临界阻尼设计 + 位置死区 + 非对称减速，避免关节跳变和静止震荡。
        
        Args:
            target_positions: IK解出的目标关节位置列表
            
        Returns:
            平滑后的关节位置列表
        """
        # 首次调用时初始化
        if self.smoothed_joint_positions is None:
            self.smoothed_joint_positions = list(target_positions)
            self.last_joint_velocities = [0.0] * len(target_positions)
            return target_positions
        
        # 确保速度数组已初始化
        if self.last_joint_velocities is None:
            self.last_joint_velocities = [0.0] * len(target_positions)
        
        smoothed = []
        new_velocities = []
        alpha = self.joint_smoothing_alpha
        max_delta_v = self.max_joint_acceleration * self.dt  # 单周期最大速度变化量
        
        # 【新增】位置误差死区（小于此值视为到达目标）
        position_deadzone = 0.0005  # 0.5mrad ≈ 0.03度
        
        for i in range(len(target_positions)):
            pos_error = target_positions[i] - self.smoothed_joint_positions[i]
            current_velocity = self.last_joint_velocities[i]
            
            # 【改进1】位置死区：误差极小且速度极小时直接归零
            if abs(pos_error) < position_deadzone and abs(current_velocity) < 0.01:
                smoothed.append(self.smoothed_joint_positions[i])
                new_velocities.append(0.0)
                continue
            
            # 【改进2】临界阻尼设计：desired_velocity考虑当前速度
            # 使用类似PD控制：v_desired = Kp * pos_error - Kd * current_velocity
            # 这里 Kp 对应 alpha/dt，Kd 是阻尼系数
            damping = 0.7  # 阻尼系数，0.7接近临界阻尼
            
            # 期望速度 = 位置误差项 - 阻尼项
            desired_velocity = alpha * pos_error / self.dt - damping * current_velocity
            
            # 【改进3】非对称加速度限制：减速时允许2倍加速度（快速停止）
            velocity_change = desired_velocity - current_velocity
            is_decelerating = (current_velocity * desired_velocity < 0) or \
                              (abs(desired_velocity) < abs(current_velocity))
            
            if is_decelerating:
                effective_max_delta_v = max_delta_v * 2.0  # 减速时加速度翻倍
            else:
                effective_max_delta_v = max_delta_v
            
            if abs(velocity_change) > effective_max_delta_v:
                desired_velocity = current_velocity + effective_max_delta_v * (1 if velocity_change > 0 else -1)
            
            # 速度限制
            if abs(desired_velocity) > self.max_joint_velocity:
                desired_velocity = self.max_joint_velocity * (1 if desired_velocity > 0 else -1)
            
            # 计算最终位置
            final_pos = self.smoothed_joint_positions[i] + desired_velocity * self.dt
            
            smoothed.append(final_pos)
            new_velocities.append(desired_velocity)
        
        self.smoothed_joint_positions = smoothed
        self.last_joint_velocities = new_velocities
        return smoothed
    
    def send_joint_positions(self, positions):
        """直接发送关节位置到控制器（不经过IK）"""
        # 构建并发布关节轨迹
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names
        
        # 单点轨迹 - 短执行时间
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.velocities = [0.0] * 6
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 20_000_000  # 20ms到达
        
        trajectory.points = [point]
        
        # 发布轨迹
        self.joint_trajectory_pub.publish(trajectory)
    
    def publish_ik_debug(self, positions):
        """发布IK解调试信息到/debug/ik_solution（实时发布，包括无输入时）"""
        ik_debug_msg = JointState()
        ik_debug_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 【关键修改】如果已知joint_states顺序，则按该顺序发布，确保Foxglove中索引对应
        if self.current_joint_state is not None:
            ik_debug_msg.name = self.current_joint_state.name
            
            # 创建名称到位置的映射
            pos_map = {name: pos for name, pos in zip(self.joint_names, positions)}
            
            # 按joint_states的顺序填充位置
            ordered_positions = []
            for name in ik_debug_msg.name:
                if name in pos_map:
                    ordered_positions.append(pos_map[name])
                else:
                    ordered_positions.append(0.0) # 异常情况
            ik_debug_msg.position = ordered_positions
        else:
            # 降级方案：使用默认顺序
            ik_debug_msg.name = self.joint_names
            ik_debug_msg.position = list(positions)
            
        self.ik_solution_pub.publish(ik_debug_msg)
    
    def publish_ik_raw_debug(self, positions):
        """发布原始IK解（处理前）到/debug/ik_raw_solution，用于对比"""
        ik_raw_msg = JointState()
        ik_raw_msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.current_joint_state is not None:
            ik_raw_msg.name = self.current_joint_state.name
            pos_map = {name: pos for name, pos in zip(self.joint_names, positions)}
            ordered_positions = []
            for name in ik_raw_msg.name:
                if name in pos_map:
                    ordered_positions.append(pos_map[name])
                else:
                    ordered_positions.append(0.0)
            ik_raw_msg.position = ordered_positions
        else:
            ik_raw_msg.name = self.joint_names
            ik_raw_msg.position = list(positions)
            
        self.ik_raw_solution_pub.publish(ik_raw_msg)
    
    def send_cartesian_path_goal(self):
        """使用MoveIt笛卡尔路径规划 - 保证RViz显示更新和精确控制（传统模式）"""
        # 检查是否正在移动（带超时检测）
        if self.is_moving:
            # 超时检测 - 防止is_moving永久阻塞
            if self.move_start_time is not None:
                elapsed = (self.get_clock().now() - self.move_start_time).nanoseconds / 1e9
                if elapsed > self.move_timeout:
                    self.get_logger().warn(f'运动超时({elapsed:.1f}s)，重置状态')
                    self.is_moving = False
                else:
                    return
            else:
                return
        
        # 标记为正在移动
        self.is_moving = True
        self.move_start_time = self.get_clock().now()
        
        # 创建笛卡尔路径请求
        request = GetCartesianPath.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = self.base_frame
        request.group_name = self.planning_group
        request.link_name = self.end_effector_frame
        
        # 设置路径点 - 只有目标点
        waypoint = Pose()
        waypoint.position = self.target_pose.pose.position
        waypoint.orientation = self.target_pose.pose.orientation
        request.waypoints = [waypoint]
        
        # 设置参数
        request.max_step = 0.01  # 10mm步长
        request.jump_threshold = 0.0  # 禁用跳跃检测
        request.avoid_collisions = self.collision_check_active  # 根据状态启用碰撞检测
        
        # 异步调用服务
        future = self.cartesian_path_client.call_async(request)
        future.add_done_callback(self.cartesian_path_callback)
        
    def cartesian_path_callback(self, future):
        """笛卡尔路径规划回调"""
        try:
            response = future.result()
            
            if response.fraction < 0.9:
                self.is_moving = False
                return
            
            # 获取规划的轨迹
            trajectory = response.solution
            
            # 加速轨迹执行
            self.scale_trajectory_speed(trajectory, 5.0)
            
            # 执行轨迹
            goal_msg = ExecuteTrajectory.Goal()
            goal_msg.trajectory = trajectory
            
            send_goal_future = self.execute_trajectory_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.execute_response_callback)
            
        except Exception as e:
            self.is_moving = False
    
    def scale_trajectory_speed(self, trajectory, speed_scale):
        """缩放轨迹速度 - speed_scale > 1 表示加快，< 1 表示减慢"""
        if speed_scale <= 0:
            return
        
        for point in trajectory.joint_trajectory.points:
            # 计算新的时间（speed_scale > 1 时间变短，< 1 时间变长）
            total_nsec = point.time_from_start.sec * 1e9 + point.time_from_start.nanosec
            new_total_nsec = total_nsec / speed_scale
            point.time_from_start.sec = int(new_total_nsec // 1e9)
            point.time_from_start.nanosec = int(new_total_nsec % 1e9)
            
            # 缩放速度（speed_scale > 1 速度变大）
            point.velocities = [v * speed_scale for v in point.velocities]
            
            # 缩放加速度（speed_scale > 1 加速度变大）
            if point.accelerations:
                point.accelerations = [a * speed_scale * speed_scale for a in point.accelerations]
    
    def execute_response_callback(self, future):
        """执行轨迹响应回调"""
        try:
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().warn('轨迹执行被拒绝')
                self.is_moving = False
                return
                
            # 等待执行结果
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.execute_result_callback)
        except Exception as e:
            self.get_logger().error(f'执行响应回调异常: {e}')
            self.is_moving = False
        
    def execute_result_callback(self, future):
        """执行结果回调"""
        try:
            result = future.result().result
            self.is_moving = False
            
            if result.error_code.val == result.error_code.SUCCESS:
                # 更新上次发送的位姿
                self.last_sent_pose = copy.deepcopy(self.target_pose)
            else:
                self.get_logger().warn(f'运动失败: error_code={result.error_code.val}')
        except Exception as e:
            self.get_logger().error(f'执行结果回调异常: {e}')
            self.is_moving = False


    def enable_gripper_motor(self):
        """使能夹爪电机(ID=7)"""
        if self.can_socket is None:
            return False
        
        try:
            motor_id = self.gripper_motor_id  # 7
            host_can_id = 253  # 0xFD - 主机CAN ID
            comm_type = 3  # 通信类型3：电机使能运行
            
            # 构建29位扩展CAN ID: (comm_type << 24) | (host_can_id << 8) | motor_id
            can_id = (comm_type << 24) | (host_can_id << 8) | motor_id
            can_id |= 0x80000000  # 设置扩展帧标志
            
            # 构建8字节数据（全0）
            data = bytes(8)
            
            # 构建CAN帧
            frame = struct.pack('=IB3x8s', can_id, 8, data)
            
            # 发送
            self.can_socket.send(frame)
            self.gripper_enabled = True
            self.get_logger().info(f'夹爪电机(ID={motor_id})已使能')
            return True
            
        except Exception as e:
            self.get_logger().error(f'使能夹爪电机失败: {e}')
            return False

    def send_gripper_torque(self, torque):
        """
        发送力矩命令到夹爪电机(ID=7)
        position=0, velocity=0, kp=0, kd=0, torque=指定值
        """
        if self.can_socket is None:
            return False
        
        try:
            motor_id = self.gripper_motor_id
            
            # RS05电机参数范围
            P_MIN, P_MAX = -12.57, 12.57  # rad
            V_MIN, V_MAX = -50.0, 50.0    # rad/s
            KP_MIN, KP_MAX = 0.0, 500.0
            KD_MIN, KD_MAX = 0.0, 5.0
            T_MIN, T_MAX = -12.0, 12.0    # Nm
            
            # 转换函数
            def float_to_uint16(x, x_min, x_max):
                x = max(x_min, min(x_max, x))
                return int((x - x_min) * 65535.0 / (x_max - x_min))
            
            # 计算原始值 (position=0, velocity=0, kp=0, kd=0)
            pos_raw = float_to_uint16(0.0, P_MIN, P_MAX)
            vel_raw = float_to_uint16(0.0, V_MIN, V_MAX)
            kp_raw = float_to_uint16(0.0, KP_MIN, KP_MAX)
            kd_raw = float_to_uint16(0.0, KD_MIN, KD_MAX)
            torque_raw = float_to_uint16(torque, T_MIN, T_MAX)
            
            # 构建29位扩展CAN ID
            # 通信类型1（运控模式），torque在bit23~8，motor_id在bit7~0
            comm_type = 1
            can_id = (comm_type << 24) | (torque_raw << 8) | motor_id
            can_id |= 0x80000000  # 设置扩展帧标志
            
            # 构建8字节数据
            data = struct.pack('>HHHH', pos_raw, vel_raw, kp_raw, kd_raw)
            
            # 构建CAN帧
            frame = struct.pack('=IB3x8s', can_id, 8, data)
            
            # 发送
            self.can_socket.send(frame)
            return True
            
        except Exception as e:
            self.get_logger().error(f'发送夹爪命令失败: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = XboxTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
