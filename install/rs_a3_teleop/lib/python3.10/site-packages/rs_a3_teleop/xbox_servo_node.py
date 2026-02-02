#!/usr/bin/env python3
"""
Xbox手柄实时控制节点 - 使用MoveIt Servo进行增量控制
使用雅可比矩阵方法，避免IK解跳变问题

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
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import TwistStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from std_srvs.srv import Trigger
import math


class XboxServoNode(Node):
    """使用MoveIt Servo的Xbox遥操作节点"""

    def __init__(self):
        super().__init__('xbox_servo_node')
        
        # 声明参数
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('linear_scale', 0.15)      # 线速度 m/s
        self.declare_parameter('angular_scale', 0.5)      # 角速度 rad/s
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'end_effector')
        self.declare_parameter('debug_input', False)
        
        # 获取参数
        self.update_rate = self.get_parameter('update_rate').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.deadzone = self.get_parameter('deadzone').value
        self.planning_group = self.get_parameter('planning_group').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.debug_input = self.get_parameter('debug_input').value
        
        # 速度档位设置 (5档)
        self.speed_levels = [0.2, 0.4, 0.6, 0.8, 1.0]  # 速度倍率
        self.current_speed_level = 1  # 默认第2档 (40%)
        
        # 状态变量
        self.current_joy = None
        self.last_a_button = 0
        self.last_b_button = 0
        self.is_going_home = False
        self.servo_enabled = False
        self.current_joint_state = None
        self.joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
        
        # 平滑滤波状态
        self.smoothed_vx = 0.0
        self.smoothed_vy = 0.0
        self.smoothed_vz = 0.0
        self.smoothed_wx = 0.0
        self.smoothed_wy = 0.0
        self.smoothed_wz = 0.0
        self.smoothing_factor = 0.3  # EMA滤波系数
        
        # QoS配置
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅手柄输入
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, reliable_qos)
        
        # 订阅关节状态
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # 发布Twist命令到MoveIt Servo
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', reliable_qos)
        
        # MoveGroup Action客户端 (用于home运动)
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Servo启动/停止服务客户端
        self.servo_start_client = self.create_client(
            Trigger, '/servo_node/start_servo')
        self.servo_stop_client = self.create_client(
            Trigger, '/servo_node/stop_servo')
        
        # 定时器 - 更新循环
        self.update_timer = self.create_timer(
            1.0 / self.update_rate, self.update_callback)
        
        self.get_logger().info(f'Xbox Servo节点已启动')
        self.get_logger().info(f'  - 更新频率: {self.update_rate} Hz')
        self.get_logger().info(f'  - 线速度范围: ±{self.linear_scale} m/s')
        self.get_logger().info(f'  - 角速度范围: ±{self.angular_scale} rad/s')
        self.get_logger().info(f'  - 速度档位: {self.current_speed_level + 1}/{len(self.speed_levels)}')
        
        # 等待关节状态
        self.wait_for_joint_states()
        
        # 启动Servo
        self.start_servo()
        
        # 启动时运动到home位置
        self.startup_go_home()
    
    def wait_for_joint_states(self):
        """等待关节状态可用"""
        self.get_logger().info('等待关节状态（读取电机位置）...')
        
        timeout_sec = 10.0
        start_time = self.get_clock().now()
        
        while self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().warn('等待关节状态超时！继续启动...')
                return False
        
        self.get_logger().info('当前电机位置已读取')
        return True
    
    def start_servo(self):
        """启动MoveIt Servo"""
        if not self.servo_start_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Servo启动服务不可用，使用直接发布模式')
            self.servo_enabled = True  # 假设可以直接发布
            return
        
        request = Trigger.Request()
        future = self.servo_start_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            self.servo_enabled = True
            self.get_logger().info('MoveIt Servo已启动')
        else:
            self.get_logger().warn('MoveIt Servo启动失败，使用直接发布模式')
            self.servo_enabled = True
    
    def stop_servo(self):
        """停止MoveIt Servo"""
        if not self.servo_stop_client.wait_for_service(timeout_sec=2.0):
            return
        
        request = Trigger.Request()
        future = self.servo_stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        self.servo_enabled = False
        self.get_logger().info('MoveIt Servo已停止')
    
    def startup_go_home(self):
        """启动时运动到home位置"""
        import time
        
        self.is_going_home = True
        self.get_logger().info('正在规划运动至home位置...')
        
        # 创建MoveGroup请求
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2
        
        # 设置目标关节位置 (home位置)
        joint_values = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]
        
        constraints = Constraints()
        for name, value in zip(self.joint_names, joint_values):
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
        
        # 尝试发送home请求
        max_attempts = 3
        for attempt in range(1, max_attempts + 1):
            if not self.move_group_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().warn(f'MoveGroup服务未准备就绪 (尝试 {attempt}/{max_attempts})')
                time.sleep(2.0)
                continue
            
            self.get_logger().info(f'发送home运动请求... (尝试 {attempt}/{max_attempts})')
            send_goal_future = self.move_group_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
            goal_handle = send_goal_future.result()
            
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn('Home运动请求被拒绝，等待重试...')
                time.sleep(2.0)
                continue
            
            self.get_logger().info('Home运动请求已接受，正在缓慢执行...')
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
            
            result = result_future.result()
            self.is_going_home = False
            
            try:
                if result and result.result and result.result.error_code.val == result.result.error_code.SUCCESS:
                    self.get_logger().info('✓ 已缓慢运动到home位置！')
                    return True
            except Exception as e:
                self.get_logger().warn(f'Home运动结果解析失败: {e}')
            
            time.sleep(2.0)
        
        self.get_logger().error('Home运动多次尝试后失败')
        self.is_going_home = False
        return False
    
    def joy_callback(self, msg):
        """手柄输入回调"""
        self.current_joy = msg
    
    def joint_state_callback(self, msg):
        """关节状态回调"""
        self.current_joint_state = msg
    
    def apply_deadzone(self, value):
        """应用死区"""
        if abs(value) < self.deadzone:
            return 0.0
        # 重新映射，死区外的值从0开始
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def apply_trigger_deadzone(self, value):
        """扳机死区处理（LT/RT）"""
        # 原始范围 1.0 到 -1.0，按下为-1.0
        normalized = (1.0 - value) / 2.0  # 转换为0-1
        if normalized < self.deadzone:
            return 0.0
        return (normalized - self.deadzone) / (1.0 - self.deadzone)
    
    def switch_speed_level(self):
        """切换速度档位"""
        self.current_speed_level = (self.current_speed_level + 1) % len(self.speed_levels)
        level_percent = int(self.speed_levels[self.current_speed_level] * 100)
        self.get_logger().info(f'速度档位: {self.current_speed_level + 1}/{len(self.speed_levels)} ({level_percent}%)')
    
    def update_callback(self):
        """定期更新回调"""
        if self.current_joy is None or self.is_going_home:
            return
        
        joy = self.current_joy
        
        if len(joy.axes) < 6 or len(joy.buttons) < 6:
            return
        
        # A键 - 切换速度档位
        a_button = joy.buttons[0]
        if a_button == 1 and self.last_a_button == 0:
            self.switch_speed_level()
        self.last_a_button = a_button
        
        # B键 - 回到home
        b_button = joy.buttons[1]
        if b_button == 1 and self.last_b_button == 0:
            self.go_home()
        self.last_b_button = b_button
        
        # 获取速度倍率
        speed_mult = self.speed_levels[self.current_speed_level]
        
        # 解析手柄输入 - 直接计算速度
        # 平移控制 - 左摇杆XY对调并反向
        raw_vx = -self.apply_deadzone(joy.axes[1]) * self.linear_scale * speed_mult
        raw_vy = -self.apply_deadzone(joy.axes[0]) * self.linear_scale * speed_mult
        
        # LT/RT控制Z
        lt = self.apply_trigger_deadzone(joy.axes[2])
        rt = self.apply_trigger_deadzone(joy.axes[5])
        raw_vz = (rt - lt) * self.linear_scale * speed_mult
        
        # 旋转控制 - 右摇杆
        raw_wz = self.apply_deadzone(joy.axes[3]) * self.angular_scale * speed_mult  # Yaw
        raw_wx = self.apply_deadzone(joy.axes[4]) * self.angular_scale * speed_mult  # Roll (右摇杆Y)
        
        # LB/RB控制Pitch
        raw_wy = (joy.buttons[5] - joy.buttons[4]) * self.angular_scale * speed_mult
        
        # 应用平滑滤波器
        alpha = self.smoothing_factor
        self.smoothed_vx = alpha * raw_vx + (1 - alpha) * self.smoothed_vx
        self.smoothed_vy = alpha * raw_vy + (1 - alpha) * self.smoothed_vy
        self.smoothed_vz = alpha * raw_vz + (1 - alpha) * self.smoothed_vz
        self.smoothed_wx = alpha * raw_wx + (1 - alpha) * self.smoothed_wx
        self.smoothed_wy = alpha * raw_wy + (1 - alpha) * self.smoothed_wy
        self.smoothed_wz = alpha * raw_wz + (1 - alpha) * self.smoothed_wz
        
        # 使用平滑后的值
        vx = self.smoothed_vx
        vy = self.smoothed_vy
        vz = self.smoothed_vz
        wx = self.smoothed_wx
        wy = self.smoothed_wy
        wz = self.smoothed_wz
        
        # 检测是否有有效输入
        velocity_threshold = 0.001
        has_input = (abs(vx) > velocity_threshold or abs(vy) > velocity_threshold or 
                     abs(vz) > velocity_threshold or abs(wx) > velocity_threshold or 
                     abs(wy) > velocity_threshold or abs(wz) > velocity_threshold)
        
        # 调试输出
        if self.debug_input and has_input:
            self.get_logger().info(
                f'Twist: vx={vx:.3f} vy={vy:.3f} vz={vz:.3f} '
                f'wx={wx:.3f} wy={wy:.3f} wz={wz:.3f}'
            )
        
        # 发布Twist命令
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.ee_frame  # 末端坐标系
        
        twist_msg.twist.linear.x = vx
        twist_msg.twist.linear.y = vy
        twist_msg.twist.linear.z = vz
        twist_msg.twist.angular.x = wx
        twist_msg.twist.angular.y = wy
        twist_msg.twist.angular.z = wz
        
        # 只有在有输入时才发布，避免不必要的运动
        if has_input:
            self.twist_pub.publish(twist_msg)
    
    def go_home(self):
        """回到home位置"""
        self.get_logger().info('回到home位置...')
        # 暂停Servo
        self.stop_servo()
        # 执行home运动
        self.startup_go_home()
        # 重新启动Servo
        self.start_servo()


def main(args=None):
    rclpy.init(args=args)
    node = XboxServoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_servo()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



