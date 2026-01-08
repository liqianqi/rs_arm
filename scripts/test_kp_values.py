#!/usr/bin/env python3
"""
测试不同 Kp 值的归零误差
Kp: 50, 100, 200, 300, Kd=1
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time
import sys

class KpTester(Node):
    def __init__(self):
        super().__init__('kp_tester')
        
        self.joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
        self.current_positions = [0.0] * 6
        self.position_received = False
        
        # 订阅关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Action client
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
    def joint_state_callback(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
        self.position_received = True
    
    def wait_for_position(self, timeout=5.0):
        start = time.time()
        while not self.position_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.position_received
    
    def move_to_position(self, target_positions, duration=3.0):
        """移动到目标位置"""
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return False
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal_msg.trajectory.points.append(point)
        
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 5.0)
        
        return True
    
    def get_current_errors(self, target=[0.0]*6):
        """获取当前位置误差"""
        # 更新位置
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
        
        errors = []
        for i in range(6):
            error = abs(self.current_positions[i] - target[i])
            errors.append(error)
        return errors
    
    def run_zero_test(self):
        """运行归零测试"""
        self.get_logger().info('开始归零测试...')
        
        # 等待获取初始位置
        if not self.wait_for_position():
            self.get_logger().error('无法获取关节位置')
            return None
        
        # 先移动到非零位置
        self.get_logger().info('移动到初始位置 [0.5, -0.3, 0.4, -0.2, 0.3, -0.1]...')
        self.move_to_position([0.5, -0.3, 0.4, -0.2, 0.3, -0.1], duration=3.0)
        time.sleep(1.0)
        
        # 移动到零点
        self.get_logger().info('移动到零点...')
        self.move_to_position([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration=3.0)
        
        # 等待稳定
        time.sleep(2.0)
        
        # 采集多次误差取平均
        all_errors = []
        for _ in range(10):
            errors = self.get_current_errors()
            all_errors.append(errors)
            time.sleep(0.1)
        
        # 计算平均误差
        avg_errors = []
        for j in range(6):
            avg = sum(e[j] for e in all_errors) / len(all_errors)
            avg_errors.append(avg)
        
        return avg_errors, self.current_positions


def main():
    rclpy.init()
    
    tester = KpTester()
    
    # 等待连接
    tester.get_logger().info('等待 action server...')
    if not tester.action_client.wait_for_server(timeout_sec=10.0):
        tester.get_logger().error('Action server 不可用')
        rclpy.shutdown()
        return
    
    tester.get_logger().info('开始归零测试 (当前 Kp/Kd 设置)')
    
    # 运行测试
    result = tester.run_zero_test()
    
    if result:
        avg_errors, final_pos = result
        
        print("\n" + "="*60)
        print("归零测试结果")
        print("="*60)
        print(f"最终位置:")
        for i, name in enumerate(['L1', 'L2', 'L3', 'L4', 'L5', 'L6']):
            print(f"  {name}: {final_pos[i]:.4f} rad ({final_pos[i]*180/3.14159:.2f}°)")
        
        print(f"\n误差 (|目标-实际|):")
        for i, name in enumerate(['L1', 'L2', 'L3', 'L4', 'L5', 'L6']):
            print(f"  {name}: {avg_errors[i]:.4f} rad ({avg_errors[i]*180/3.14159:.2f}°)")
        
        max_error = max(avg_errors)
        avg_error = sum(avg_errors) / len(avg_errors)
        print(f"\n最大误差: {max_error:.4f} rad ({max_error*180/3.14159:.2f}°)")
        print(f"平均误差: {avg_error:.4f} rad ({avg_error*180/3.14159:.2f}°)")
        print("="*60)
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


















