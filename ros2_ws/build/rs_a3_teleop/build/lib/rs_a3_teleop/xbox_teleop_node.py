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
        self.declare_parameter('translation_scale', 0.0004)  # m per update (50Hz下)
        self.declare_parameter('rotation_scale', 0.004)  # rad per update (50Hz下)
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('end_effector_frame', 'end_effector')
        self.declare_parameter('deadzone', 0.20)  # 摇杆死区
        self.declare_parameter('debug_input', False)  # 调试模式
        self.declare_parameter('smoothing_factor', 0.5)  # 平滑滤波系数 (50Hz下)
        self.declare_parameter('use_fast_ik_mode', True)  # 使用快速IK模式实现50Hz控制
        
        # 获取参数
        self.update_rate = self.get_parameter('update_rate').value
        self.translation_scale = self.get_parameter('translation_scale').value
        self.rotation_scale = self.get_parameter('rotation_scale').value
        self.planning_group = self.get_parameter('planning_group').value
        self.base_frame = self.get_parameter('base_frame').value
        self.end_effector_frame = self.get_parameter('end_effector_frame').value
        self.deadzone = self.get_parameter('deadzone').value
        self.debug_input = self.get_parameter('debug_input').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        self.use_fast_ik_mode = self.get_parameter('use_fast_ik_mode').value
        
        # 调试计数器 - 避免日志刷屏
        self.debug_counter = 0
        
        # 速度档位设置 (5档) - 适配50Hz更新频率
        # 实际速度 = translation * update_rate
        self.speed_levels = [
            {'name': '超慢', 'translation': 0.0005, 'rotation': 0.005},   # 25mm/s, 0.25rad/s
            {'name': '慢速', 'translation': 0.0012, 'rotation': 0.012},  # 60mm/s, 0.6rad/s
            {'name': '中速', 'translation': 0.0024, 'rotation': 0.024},  # 120mm/s, 1.2rad/s
            {'name': '快速', 'translation': 0.0048, 'rotation': 0.048},  # 240mm/s, 2.4rad/s
            {'name': '极速', 'translation': 0.012, 'rotation': 0.12},    # 600mm/s, 6rad/s
        ]
        self.current_speed_level = 2  # 默认中速 (索引从0开始)
        # 设置初始速度
        self.translation_scale = self.speed_levels[self.current_speed_level]['translation']
        self.rotation_scale = self.speed_levels[self.current_speed_level]['rotation']
        
        # 按钮状态跟踪（用于防抖动）
        self.last_a_button = 0
        self.last_b_button = 0
        self.is_going_home = False  # 是否正在回home
        
        # 平滑滤波器状态（指数移动平均）
        self.smoothed_dx = 0.0
        self.smoothed_dy = 0.0
        self.smoothed_dz = 0.0
        self.smoothed_droll = 0.0
        self.smoothed_dpitch = 0.0
        self.smoothed_dyaw = 0.0
        
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
        
        # 3. 自动运动到home位置
        self.get_logger().info('正在规划运动至home位置...')
        self.startup_go_home()
        
        # 创建定时器，定期更新目标位姿
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_callback)
        
        self.get_logger().info('Xbox手柄控制节点已启动')
        self.get_logger().info(f'控制组: {self.planning_group}')
        self.get_logger().info(f'末端执行器: {self.end_effector_frame}')
        self.get_logger().info(f'更新频率: {self.update_rate} Hz')
        self.get_logger().info(f'平滑滤波系数: {self.smoothing_factor}')
        if self.use_fast_ik_mode:
            self.get_logger().info('控制模式: 快速IK模式（50Hz笛卡尔控制）')
        else:
            self.get_logger().info('控制模式: 传统路径规划模式')
        self.get_logger().info('=== 控制说明 ===')
        self.get_logger().info('左摇杆: XY平移 | LT/RT: Z平移')
        self.get_logger().info('右摇杆: Yaw/Roll | LB/RB: Pitch')
        self.get_logger().info('A键: 切换速度档位 | B键: 回到初始位置')
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
                # 【关键】同时初始化IK种子为当前关节状态，确保首次IK解的连续性
                if self.current_joint_state:
                    self.last_ik_joint_positions = list(self.current_joint_state.position[:6])
                    self.get_logger().info(f'初始化IK种子: {[f"{p:.3f}" for p in self.last_ik_joint_positions]}')
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
    
    def startup_go_home(self):
        """启动时运动到home位置（同步执行）"""
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
            
            # 设置目标关节位置 (home位置)
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
            goal_msg.planning_options.plan_only = False
            goal_msg.planning_options.replan = True
            goal_msg.planning_options.replan_attempts = 3
            
            # 发送请求并等待结果
            self.get_logger().info(f'发送home运动请求... (尝试 {retry + 1}/{max_retries})')
            send_goal_future = self.move_group_client.send_goal_async(goal_msg)
            
            # 等待goal被接受
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
            goal_handle = send_goal_future.result()
            
            if goal_handle and goal_handle.accepted:
                self.get_logger().info('Home运动请求已接受，正在缓慢执行...')
                
                # 等待执行完成
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
                
                result = result_future.result()
                self.is_going_home = False
                self.pose_initialized = False  # 重置位姿初始化标记
                
                # 检查结果
                try:
                    if result and result.result and result.result.error_code.val == result.result.error_code.SUCCESS:
                        self.get_logger().info('✓ 已缓慢运动到home位置！')
                        return True
                    else:
                        error_code = 'unknown'
                        if result and result.result and hasattr(result.result, 'error_code'):
                            error_code = result.result.error_code.val
                        self.get_logger().warn(f'Home运动执行失败: error_code={error_code}')
                except Exception as e:
                    self.get_logger().warn(f'Home运动结果解析失败: {e}')
            else:
                self.get_logger().warn(f'Home运动请求被拒绝，等待重试...')
                time.sleep(2.0)  # 等待后重试
        
        self.get_logger().error('Home运动多次尝试后失败，请手动控制机械臂')
        self.is_going_home = False
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
        self.get_logger().info(f'当前速度档位: {self.current_speed_level + 1}/5 [{level["name"]}]')
    
    def switch_speed_level(self):
        """切换速度档位"""
        self.current_speed_level = (self.current_speed_level + 1) % len(self.speed_levels)
        level = self.speed_levels[self.current_speed_level]
        self.translation_scale = level['translation']
        self.rotation_scale = level['rotation']
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
        self.pose_initialized = False  # 重置位姿初始化标记
        
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('已回到初始位置！')
        else:
            self.get_logger().warn(f'回home失败: error_code={result.error_code.val}')
        
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
        
        # 处理B键 - 回到初始位置（按下触发，防抖动）
        b_button = joy.buttons[1]
        if b_button == 1 and self.last_b_button == 0:
            self.go_home()
        self.last_b_button = b_button
            
        # 如果正在回home，跳过普通运动控制
        if self.is_going_home:
            return
            
        # 解析手柄输入
        # Xbox手柄映射:
        # 左摇杆 X: axes[0]
        # 左摇杆 Y: axes[1]
        # LT: axes[2] (范围 1.0 到 -1.0，按下为-1.0)
        # RT: axes[5] (范围 1.0 到 -1.0，按下为-1.0)
        # 右摇杆 X: axes[3]
        # 右摇杆 Y: axes[4]
        # LB: buttons[4]
        # RB: buttons[5]
        # A: buttons[0]
        # B: buttons[1]
            
        # 平移控制 - 左摇杆XY对调并反向
        raw_dx = -self.apply_deadzone(joy.axes[1]) * self.translation_scale  # 左摇杆Y -> X (反向)
        raw_dy = -self.apply_deadzone(joy.axes[0]) * self.translation_scale  # 左摇杆X -> Y (反向)
        
        # LT/RT控制Z - 使用带死区的扳机处理
        lt = self.apply_trigger_deadzone(joy.axes[2])  # 转换为0-1范围并应用死区
        rt = self.apply_trigger_deadzone(joy.axes[5])  # 转换为0-1范围并应用死区
        raw_dz = (rt - lt) * self.translation_scale  # RT向上，LT向下
        
        # 旋转控制
        raw_dyaw = self.apply_deadzone(joy.axes[3]) * self.rotation_scale  # 右摇杆X -> Yaw
        raw_droll = self.apply_deadzone(joy.axes[4]) * self.rotation_scale  # 右摇杆Y -> Roll
        
        # LB/RB控制Pitch
        raw_dpitch = (joy.buttons[5] - joy.buttons[4]) * self.rotation_scale  # RB正，LB负
        
        # 应用平滑滤波器（指数移动平均 EMA）
        # smoothed = alpha * raw + (1 - alpha) * smoothed_prev
        alpha = self.smoothing_factor
        self.smoothed_dx = alpha * raw_dx + (1 - alpha) * self.smoothed_dx
        self.smoothed_dy = alpha * raw_dy + (1 - alpha) * self.smoothed_dy
        self.smoothed_dz = alpha * raw_dz + (1 - alpha) * self.smoothed_dz
        self.smoothed_droll = alpha * raw_droll + (1 - alpha) * self.smoothed_droll
        self.smoothed_dpitch = alpha * raw_dpitch + (1 - alpha) * self.smoothed_dpitch
        self.smoothed_dyaw = alpha * raw_dyaw + (1 - alpha) * self.smoothed_dyaw
        
        # 使用平滑后的值
        dx = self.smoothed_dx
        dy = self.smoothed_dy
        dz = self.smoothed_dz
        droll = self.smoothed_droll
        dpitch = self.smoothed_dpitch
        dyaw = self.smoothed_dyaw
        
        # 检查是否有输入 - 使用更大的阈值避免漂移
        # 阈值设为缩放因子的 1/10，确保只有明显的输入才会触发运动
        translation_threshold = self.translation_scale * 0.01  # 非常小的移动忽略
        rotation_threshold = self.rotation_scale * 0.01
        
        has_translation = abs(dx) > translation_threshold or abs(dy) > translation_threshold or abs(dz) > translation_threshold
        has_rotation = abs(dyaw) > rotation_threshold or abs(dpitch) > rotation_threshold or abs(droll) > rotation_threshold
        
        if not has_translation and not has_rotation:
            # 【关键修复】即使没有输入，在IK模式下也要持续发送当前目标位置
            # 这样可以防止机械臂因重力下垂而偏离目标位置
            if self.use_fast_ik_mode and self.pose_initialized:
                self.send_ik_goal()  # 持续发送当前目标位置保持位置
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
            target_positions = []
            
            for joint_name in self.joint_names:
                if joint_name in solution.name:
                    idx = solution.name.index(joint_name)
                    target_positions.append(solution.position[idx])
                else:
                    return  # 缺少关节数据
            
            # 与上一次位置比较，避免发送相同命令
            if self.last_ik_joint_positions is not None:
                max_diff = max(abs(a - b) for a, b in zip(target_positions, self.last_ik_joint_positions))
                if max_diff < 0.0001:  # 变化太小，跳过
                    return
            
            self.last_ik_joint_positions = target_positions
            
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
        request.avoid_collisions = False  # 关闭碰撞检测加速
        
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

