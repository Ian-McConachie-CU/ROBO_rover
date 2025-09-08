#!/usr/bin/env python3
"""
ROS2 ArduPilot Rover Node
Combines steering/throttle control and IMU data publishing
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import time
import threading
from pymavlink import mavutil
import numpy as np
from scipy.spatial.transform import Rotation


class ArduPilotRoverNode(Node):
    def __init__(self):
        super().__init__('rover_node')
        
        # Parameters
        self.declare_parameter('connection_string', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('default_throttle', 0.0)
        self.declare_parameter('default_steering', 0.0)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('imu_frequency', 20.0)
        
        # Get parameters
        self.connection_string = self.get_parameter('connection_string').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.default_throttle = self.get_parameter('default_throttle').value
        self.default_steering = self.get_parameter('default_steering').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.imu_freq = self.get_parameter('imu_frequency').value
        
        # Control variables
        self.current_throttle = self.default_throttle
        self.current_steering = self.default_steering
        self.last_cmd_time = time.time()
        self.cmd_timeout = 1.0  # 1 second timeout for commands
        
        # Connection variables
        self.master = None
        self.connected = False
        self.armed = False
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', sensor_qos)
        self.armed_pub = self.create_publisher(Bool, 'rover/armed', control_qos)
        
        # Subscribers
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, control_qos)
        
        # Timers
        self.control_timer = self.create_timer(
            1.0 / self.control_freq, self.control_loop)
        self.imu_timer = self.create_timer(
            1.0 / self.imu_freq, self.imu_loop)
        self.status_timer = self.create_timer(1.0, self.status_loop)
        
        # Initialize connection
        self.get_logger().info('Initializing ArduPilot Rover Node...')
        self.connect_to_rover()
        
    def connect_to_rover(self):
        """Connect to the rover via MAVLink"""
        try:
            self.get_logger().info(f'Connecting to rover on {self.connection_string}...')
            
            self.master = mavutil.mavlink_connection(
                self.connection_string, 
                baud=self.baud_rate,
                timeout=10
            )
            
            # Wait for heartbeat
            self.get_logger().info('Waiting for heartbeat...')
            heartbeat = self.master.wait_heartbeat(timeout=10)
            
            if heartbeat is None:
                self.get_logger().error('No heartbeat received')
                return False
                
            self.get_logger().info(
                f'Connected to system {self.master.target_system} '
                f'component {self.master.target_component}'
            )
            
            self.connected = True
            
            # Set mode to ACRO
            if self.set_mode('ACRO'):
                time.sleep(2)
                # Arm the rover
                self.arm_rover()
                # Request IMU data
                self.request_imu_data()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')
            return False
    
    def set_mode(self, mode_name):
        """Set the rover flight mode"""
        if not self.connected:
            return False
            
        mode_mapping = self.master.mode_mapping()
        if mode_name not in mode_mapping:
            self.get_logger().error(f'Mode {mode_name} not available')
            return False
            
        mode_id = mode_mapping[mode_name]
        
        self.get_logger().info(f'Setting mode to {mode_name}')
        
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
        # Wait for mode change confirmation
        start_time = time.time()
        while time.time() - start_time < 5:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                current_mode = mavutil.mode_string_v10(msg)
                if current_mode == mode_name:
                    self.get_logger().info(f'Mode changed to {mode_name}')
                    return True
            time.sleep(0.1)
            
        self.get_logger().error(f'Failed to change mode to {mode_name}')
        return False
    
    def arm_rover(self):
        """Arm the rover"""
        if not self.connected:
            return False
            
        self.get_logger().info('Arming rover...')
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        
        # Wait for arming confirmation
        start_time = time.time()
        while time.time() - start_time < 10:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    self.get_logger().info('Rover armed successfully')
                    self.armed = True
                    return True
            time.sleep(0.1)
            
        self.get_logger().error('Failed to arm rover')
        return False
    
    def disarm_rover(self):
        """Disarm the rover"""
        if not self.connected:
            return
            
        self.get_logger().info('Disarming rover...')
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        
        self.armed = False
    
    def request_imu_data(self):
        """Request IMU messages from the autopilot"""
        if not self.connected:
            return
            
        try:
            # Calculate interval in microseconds
            interval_us = int(1000000 / self.imu_freq)
            
            # Request SCALED_IMU messages
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                26,  # SCALED_IMU message ID
                interval_us,
                0, 0, 0, 0, 0
            )
            
            # Request HIGHRES_IMU messages as backup
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                105,  # HIGHRES_IMU message ID
                interval_us,
                0, 0, 0, 0, 0
            )
            
            self.get_logger().info(f'Requested IMU data at {self.imu_freq} Hz')
            
        except Exception as e:
            self.get_logger().error(f'Failed to request IMU data: {e}')
    
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands"""
        # Convert Twist message to throttle and steering
        # msg.linear.x: forward/backward speed (-1.0 to 1.0)
        # msg.angular.z: turning rate (-1.0 to 1.0)
        
        # Scale to MAVLink range (-1000 to 1000)
        self.current_throttle = int(np.clip(msg.linear.x * 1000, -1000, 1000))
        self.current_steering = int(np.clip(msg.angular.z * 1000, -1000, 1000))
        
        self.last_cmd_time = time.time()
        
        self.get_logger().debug(
            f'Received cmd_vel: throttle={self.current_throttle}, '
            f'steering={self.current_steering}'
        )
    
    def control_loop(self):
        """Main control loop - sends commands at fixed rate"""
        if not self.connected or not self.armed:
            return
        
        # Check for command timeout
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            # Use default values if no recent commands
            throttle = int(self.default_throttle * 1000)
            steering = int(self.default_steering * 1000)
        else:
            throttle = self.current_throttle
            steering = self.current_steering
        
        # Send manual control command
        try:
            self.master.mav.manual_control_send(
                self.master.target_system,
                steering,    # x (steering)
                throttle,    # y (throttle)
                0,           # z (not used)
                0,           # r (not used)
                0            # buttons
            )
        except Exception as e:
            self.get_logger().error(f'Failed to send control command: {e}')
    
    def imu_loop(self):
        """IMU data processing loop"""
        if not self.connected:
            return
        
        # Try to get SCALED_IMU first (preferred)
        scaled_imu = self.master.recv_match(type='SCALED_IMU', blocking=False)
        if scaled_imu is not None:
            self.publish_scaled_imu(scaled_imu)
            return
        
        # Fallback to HIGHRES_IMU
        highres_imu = self.master.recv_match(type='HIGHRES_IMU', blocking=False)
        if highres_imu is not None:
            self.publish_highres_imu(highres_imu)
    
    def publish_scaled_imu(self, scaled_imu_msg):
        """Publish SCALED_IMU data as ROS2 IMU message"""
        imu_msg = Imu()
        
        # Header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Convert gyro from mrad/s to rad/s
        imu_msg.angular_velocity.x = scaled_imu_msg.xgyro / 1000.0
        imu_msg.angular_velocity.y = scaled_imu_msg.ygyro / 1000.0
        imu_msg.angular_velocity.z = scaled_imu_msg.zgyro / 1000.0
        
        # Convert accelerometer from mG to m/s²
        imu_msg.linear_acceleration.x = (scaled_imu_msg.xacc / 1000.0) * 9.80665
        imu_msg.linear_acceleration.y = (scaled_imu_msg.yacc / 1000.0) * 9.80665
        imu_msg.linear_acceleration.z = (scaled_imu_msg.zacc / 1000.0) * 9.80665
        
        # ArduPilot doesn't provide quaternion in SCALED_IMU, set to unknown
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        
        # Set covariance matrices (unknown/identity)
        imu_msg.orientation_covariance = [-1.0] + [0.0] * 8  # -1 means unknown
        imu_msg.angular_velocity_covariance = [0.01] + [0.0] * 8  # Small variance
        imu_msg.linear_acceleration_covariance = [0.01] + [0.0] * 8  # Small variance
        
        self.imu_pub.publish(imu_msg)
    
    def publish_highres_imu(self, highres_imu_msg):
        """Publish HIGHRES_IMU data as ROS2 IMU message"""
        imu_msg = Imu()
        
        # Header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Gyro data already in rad/s
        imu_msg.angular_velocity.x = highres_imu_msg.xgyro
        imu_msg.angular_velocity.y = highres_imu_msg.ygyro
        imu_msg.angular_velocity.z = highres_imu_msg.zgyro
        
        # Accelerometer data already in m/s²
        imu_msg.linear_acceleration.x = highres_imu_msg.xacc
        imu_msg.linear_acceleration.y = highres_imu_msg.yacc
        imu_msg.linear_acceleration.z = highres_imu_msg.zacc
        
        # ArduPilot doesn't provide quaternion in HIGHRES_IMU, set to unknown
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        
        # Set covariance matrices
        imu_msg.orientation_covariance = [-1.0] + [0.0] * 8  # -1 means unknown
        imu_msg.angular_velocity_covariance = [0.01] + [0.0] * 8
        imu_msg.linear_acceleration_covariance = [0.01] + [0.0] * 8
        
        self.imu_pub.publish(imu_msg)
    
    def status_loop(self):
        """Publish status information"""
        # Publish armed status
        armed_msg = Bool()
        armed_msg.data = self.armed
        self.armed_pub.publish(armed_msg)
        
        # Check connection health
        if self.connected:
            heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if heartbeat is not None:
                # Update armed status from heartbeat
                self.armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    
    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.get_logger().info('Shutting down rover node...')
        
        if self.connected and self.armed:
            # Stop the rover
            try:
                self.master.mav.manual_control_send(
                    self.master.target_system,
                    0, 0, 0, 0, 0  # All zeros to stop
                )
                time.sleep(0.1)
                self.disarm_rover()
            except:
                pass
        
        if self.master:
            self.master.close()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ArduPilotRoverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
