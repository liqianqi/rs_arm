#!/usr/bin/env python3
"""
ROS2 Bridge Node for RS-A3 Web UI.
Handles communication between ROS2 and WebSocket.
"""

import threading
import time
import subprocess
import signal
import os
from typing import Callable, Dict, List, Optional
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import SetBool
from std_msgs.msg import String
from builtin_interfaces.msg import Duration


class ROS2Bridge(Node):
    """ROS2 Bridge node for web interface."""

    # Joint names in order
    JOINT_NAMES = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
    
    # Joint limits (from URDF)
    JOINT_LIMITS = {
        'L1_joint': {'lower': -2.79253, 'upper': 2.79253},
        'L2_joint': {'lower': -3.14159, 'upper': 3.14159},
        'L3_joint': {'lower': -3.14159, 'upper': 3.14159},
        'L4_joint': {'lower': -3.14159, 'upper': 3.14159},
        'L5_joint': {'lower': -3.14159, 'upper': 3.14159},
        'L6_joint': {'lower': -3.14159, 'upper': 3.14159},
    }

    def __init__(self):
        super().__init__('rs_a3_web_bridge')
        
        # State storage
        self._joint_states: Dict[str, Dict] = {}
        self._gravity_torques: Dict[str, float] = {}
        self._system_status = 'disconnected'
        self._controller_status = 'unknown'
        self._last_update_time = 0.0
        
        # Motor enable state
        self._motors_enabled = False
        
        # Teleop process management
        self._teleop_process: Optional[subprocess.Popen] = None
        self._teleop_status = 'stopped'
        
        # CAN interface configuration
        self._current_can_interface = 'can0'
        
        # Callbacks for WebSocket updates
        self._state_callback: Optional[Callable] = None
        self._log_callback: Optional[Callable] = None
        
        # Lock for thread safety
        self._lock = threading.Lock()
        
        # Initialize joint states
        for name in self.JOINT_NAMES:
            self._joint_states[name] = {
                'position': 0.0,
                'velocity': 0.0,
                'effort': 0.0
            }
            self._gravity_torques[name] = 0.0
        
        # Motor temperatures (°C)
        self._motor_temperatures: Dict[str, float] = {}
        for name in self.JOINT_NAMES:
            self._motor_temperatures[name] = 0.0
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        self.gravity_torque_sub = self.create_subscription(
            JointState,
            '/debug/gravity_torque',
            self._gravity_torque_callback,
            10
        )
        
        # Motor temperature subscriber
        self.temperature_sub = self.create_subscription(
            JointState,
            '/debug/motor_temperature',
            self._temperature_callback,
            10
        )
        
        # Publisher for trajectory commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        # Service clients
        self.zero_torque_client = self.create_client(
            SetBool,
            '/rs_a3/set_zero_torque_mode'
        )
        
        # Status timer (check connection every 2 seconds)
        self.status_timer = self.create_timer(2.0, self._check_status)
        
        # Teleop process check timer
        self.teleop_check_timer = self.create_timer(1.0, self._check_teleop_process)
        
        self.get_logger().info('ROS2 Bridge initialized')

    def set_state_callback(self, callback: Callable):
        """Set callback for state updates."""
        self._state_callback = callback

    def set_log_callback(self, callback: Callable):
        """Set callback for log messages."""
        self._log_callback = callback

    def _log(self, message: str, level: str = 'info'):
        """Log message and send to callback."""
        if level == 'info':
            self.get_logger().info(message)
        elif level == 'warn':
            self.get_logger().warn(message)
        elif level == 'error':
            self.get_logger().error(message)
        
        if self._log_callback:
            self._log_callback({
                'timestamp': time.time(),
                'level': level,
                'message': message
            })

    def _joint_state_callback(self, msg: JointState):
        """Handle joint state messages."""
        with self._lock:
            self._last_update_time = time.time()
            self._system_status = 'connected'
            
            for i, name in enumerate(msg.name):
                if name in self._joint_states:
                    if i < len(msg.position):
                        self._joint_states[name]['position'] = msg.position[i]
                    if i < len(msg.velocity):
                        self._joint_states[name]['velocity'] = msg.velocity[i]
                    if i < len(msg.effort):
                        self._joint_states[name]['effort'] = msg.effort[i]
        
        # Notify WebSocket
        if self._state_callback:
            self._state_callback(self.get_state())

    def _gravity_torque_callback(self, msg: JointState):
        """Handle gravity torque messages."""
        with self._lock:
            for i, name in enumerate(msg.name):
                if name in self._gravity_torques and i < len(msg.effort):
                    self._gravity_torques[name] = msg.effort[i]

    def _temperature_callback(self, msg: JointState):
        """Handle motor temperature messages.
        
        Note: Temperature is published in the effort field of JointState message.
        """
        with self._lock:
            for i, name in enumerate(msg.name):
                if name in self._motor_temperatures and i < len(msg.effort):
                    self._motor_temperatures[name] = msg.effort[i]

    def _check_status(self):
        """Check system connection status."""
        with self._lock:
            if time.time() - self._last_update_time > 3.0:
                if self._system_status != 'disconnected':
                    self._system_status = 'disconnected'
                    self._log('Lost connection to robot', 'warn')
            else:
                if self._system_status != 'connected':
                    self._system_status = 'connected'
                    self._log('Connected to robot', 'info')

    def get_state(self) -> Dict:
        """Get current robot state."""
        with self._lock:
            joints = []
            for name in self.JOINT_NAMES:
                state = self._joint_states[name]
                joints.append({
                    'name': name,
                    'position': state['position'],
                    'position_deg': math.degrees(state['position']),
                    'velocity': state['velocity'],
                    'effort': state['effort'],
                    'gravity_torque': self._gravity_torques.get(name, 0.0),
                    'temperature': self._motor_temperatures.get(name, 0.0),
                    'limits': self.JOINT_LIMITS[name]
                })
            
            return {
                'timestamp': time.time(),
                'status': self._system_status,
                'controller': self._controller_status,
                'motors_enabled': self._motors_enabled,
                'teleop_status': self.get_teleop_status(),
                'can_interface': self._current_can_interface,
                'joints': joints
            }

    def send_joint_command(self, positions: List[float], duration: float = 2.0) -> bool:
        """Send joint position command.
        
        Args:
            positions: List of 6 joint positions in radians
            duration: Time to reach the position in seconds
        
        Returns:
            True if command was sent successfully
        """
        if len(positions) != 6:
            self._log(f'Invalid position count: {len(positions)}, expected 6', 'error')
            return False
        
        # Clamp positions to limits
        clamped_positions = []
        for i, (name, pos) in enumerate(zip(self.JOINT_NAMES, positions)):
            limits = self.JOINT_LIMITS[name]
            clamped = max(limits['lower'], min(limits['upper'], pos))
            if clamped != pos:
                self._log(f'{name} clamped from {math.degrees(pos):.1f}° to {math.degrees(clamped):.1f}°', 'warn')
            clamped_positions.append(clamped)
        
        # Create trajectory message
        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = clamped_positions
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        
        traj.points.append(point)
        
        self.trajectory_pub.publish(traj)
        self._log(f'Sent trajectory command (duration: {duration:.1f}s)', 'info')
        
        return True

    def send_single_joint_command(self, joint_index: int, position: float, duration: float = 1.0) -> bool:
        """Send command for a single joint while keeping others at current position.
        
        Args:
            joint_index: Index of joint (0-5)
            position: Target position in radians
            duration: Time to reach the position
        
        Returns:
            True if command was sent successfully
        """
        if not 0 <= joint_index < 6:
            self._log(f'Invalid joint index: {joint_index}', 'error')
            return False
        
        with self._lock:
            positions = [self._joint_states[name]['position'] for name in self.JOINT_NAMES]
        
        positions[joint_index] = position
        return self.send_joint_command(positions, duration)

    def go_home(self, duration: float = 3.0) -> bool:
        """Send robot to home position (all zeros)."""
        self._log('Going to home position', 'info')
        return self.send_joint_command([0.0] * 6, duration)

    def _call_service_cli(self, service_name: str, data: bool) -> bool:
        """Call a SetBool service using ros2 cli to avoid threading issues."""
        try:
            cmd = [
                'ros2', 'service', 'call',
                service_name,
                'std_srvs/srv/SetBool',
                f'{{data: {str(data).lower()}}}'
            ]
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=5.0
            )
            # Check if service call succeeded
            if 'success=True' in result.stdout or 'success: true' in result.stdout.lower():
                return True
            elif result.returncode == 0:
                # Service was called but check response
                return 'success' in result.stdout.lower()
            return False
        except subprocess.TimeoutExpired:
            self._log(f'Service {service_name} call timeout', 'error')
            return False
        except Exception as e:
            self._log(f'Service call error: {e}', 'error')
            return False

    def set_zero_torque_mode(self, enable: bool) -> bool:
        """Enable or disable zero torque mode.
        
        Args:
            enable: True to enable zero torque mode
        
        Returns:
            True if successful
        """
        success = self._call_service_cli('/rs_a3/set_zero_torque_mode', enable)
        
        if success:
            self._log(f'Zero torque mode: {"enabled" if enable else "disabled"}', 'info')
        else:
            self._log('Failed to set zero torque mode', 'error')
        
        return success

    def emergency_stop(self):
        """Emergency stop - send current position as command."""
        self._log('EMERGENCY STOP', 'warn')
        with self._lock:
            positions = [self._joint_states[name]['position'] for name in self.JOINT_NAMES]
        
        # Send immediate command to hold current position
        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1 seconds
        
        traj.points.append(point)
        self.trajectory_pub.publish(traj)

    # ==================== Motor Enable/Disable ====================
    # Note: Uses zero_torque_mode service as backend via command line
    # Enable motors = Disable zero torque mode (normal position control)
    # Disable motors = Enable zero torque mode (free movement)
    
    def enable_motors(self) -> bool:
        """Enable all motors (disable zero torque mode).
        
        Returns:
            True if successful
        """
        success = self._call_service_cli('/rs_a3/set_zero_torque_mode', False)
        
        if success:
            self._motors_enabled = True
            self._log('Motors enabled (position control active)', 'info')
        else:
            self._log('Failed to enable motors', 'error')
        
        return success

    def disable_motors(self) -> bool:
        """Disable all motors (enable zero torque mode for free movement).
        
        Returns:
            True if successful
        """
        success = self._call_service_cli('/rs_a3/set_zero_torque_mode', True)
        
        if success:
            self._motors_enabled = False
            self._log('Motors disabled (zero torque mode)', 'info')
        else:
            self._log('Failed to disable motors', 'error')
        
        return success

    def get_motors_enabled(self) -> bool:
        """Get motor enable state."""
        return self._motors_enabled

    # ==================== Teleop Control ====================
    
    def _check_teleop_process(self):
        """Check teleop process status periodically."""
        if self._teleop_process is not None:
            poll = self._teleop_process.poll()
            if poll is not None:
                # Process has terminated
                self._teleop_status = 'stopped'
                self._teleop_process = None
                self._log('Teleop process terminated', 'info')

    def start_teleop(self) -> bool:
        """Start xbox teleop node.
        
        Returns:
            True if started successfully
        """
        if self._teleop_process is not None:
            poll = self._teleop_process.poll()
            if poll is None:
                self._log('Teleop already running', 'warn')
                return True
        
        try:
            # Start teleop node using ros2 run
            cmd = [
                'ros2', 'run', 'rs_a3_teleop', 'xbox_teleop_node',
                '--ros-args',
                '-p', 'use_fast_ik_mode:=true'
            ]
            
            self._teleop_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group
            )
            
            self._teleop_status = 'running'
            self._log('Teleop started', 'info')
            return True
            
        except Exception as e:
            self._log(f'Failed to start teleop: {e}', 'error')
            self._teleop_status = 'error'
            return False

    def stop_teleop(self) -> bool:
        """Stop xbox teleop node.
        
        Returns:
            True if stopped successfully
        """
        if self._teleop_process is None:
            self._teleop_status = 'stopped'
            return True
        
        try:
            # Send SIGTERM to process group
            os.killpg(os.getpgid(self._teleop_process.pid), signal.SIGTERM)
            
            # Wait for process to terminate
            try:
                self._teleop_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                # Force kill if not responding
                os.killpg(os.getpgid(self._teleop_process.pid), signal.SIGKILL)
                self._teleop_process.wait()
            
            self._teleop_process = None
            self._teleop_status = 'stopped'
            self._log('Teleop stopped', 'info')
            return True
            
        except Exception as e:
            self._log(f'Failed to stop teleop: {e}', 'error')
            self._teleop_process = None
            self._teleop_status = 'error'
            return False

    def get_teleop_status(self) -> str:
        """Get teleop node status.
        
        Returns:
            Status string: 'running', 'stopped', or 'error'
        """
        if self._teleop_process is not None:
            poll = self._teleop_process.poll()
            if poll is None:
                return 'running'
        return self._teleop_status

    # ==================== CAN Interface Configuration ====================
    
    def get_can_interface(self) -> str:
        """Get current CAN interface name."""
        return self._current_can_interface

    def set_can_interface(self, interface: str) -> bool:
        """Set CAN interface.
        
        Note: This requires restarting the hardware driver to take effect.
        
        Args:
            interface: CAN interface name (e.g., 'can0', 'can1')
        
        Returns:
            True if configuration saved (actual switch requires restart)
        """
        # Validate interface name
        if not interface.startswith('can'):
            self._log(f'Invalid CAN interface name: {interface}', 'error')
            return False
        
        self._current_can_interface = interface
        self._log(f'CAN interface set to {interface} (requires restart)', 'info')
        return True

    @staticmethod
    def get_available_can_interfaces() -> List[str]:
        """Get list of available CAN interfaces on the system.
        
        Returns:
            List of CAN interface names
        """
        interfaces = []
        try:
            result = subprocess.run(
                ['ip', 'link', 'show', 'type', 'can'],
                capture_output=True,
                text=True,
                timeout=5.0
            )
            
            # Parse output: lines like "3: can0: <NOARP,UP,LOWER_UP>..."
            for line in result.stdout.split('\n'):
                if ':' in line and 'can' in line:
                    parts = line.split(':')
                    if len(parts) >= 2:
                        iface = parts[1].strip()
                        if iface.startswith('can'):
                            interfaces.append(iface)
        except Exception:
            pass
        
        # If no interfaces found, return default
        if not interfaces:
            interfaces = ['can0']
        
        return interfaces


class ROS2BridgeThread:
    """Thread wrapper for ROS2 Bridge node."""
    
    def __init__(self):
        self.node: Optional[ROS2Bridge] = None
        self._thread: Optional[threading.Thread] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._running = False

    def start(self):
        """Start the ROS2 bridge in a background thread."""
        if self._running:
            return
        
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        
        # Wait for node to be created
        timeout = 5.0
        start_time = time.time()
        while self.node is None and time.time() - start_time < timeout:
            time.sleep(0.1)
        
        if self.node is None:
            raise RuntimeError('Failed to start ROS2 bridge')

    def _run(self):
        """Run the ROS2 node."""
        try:
            rclpy.init()
            self.node = ROS2Bridge()
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self.node)
            
            while self._running:
                self._executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            print(f'ROS2 Bridge error: {e}')
        finally:
            if self.node:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def stop(self):
        """Stop the ROS2 bridge."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=5.0)
        self.node = None


# Singleton instance
_bridge_thread: Optional[ROS2BridgeThread] = None


def get_bridge() -> ROS2Bridge:
    """Get or create the ROS2 bridge instance."""
    global _bridge_thread
    
    if _bridge_thread is None:
        _bridge_thread = ROS2BridgeThread()
        _bridge_thread.start()
    
    return _bridge_thread.node


def shutdown_bridge():
    """Shutdown the ROS2 bridge."""
    global _bridge_thread
    
    if _bridge_thread:
        _bridge_thread.stop()
        _bridge_thread = None
