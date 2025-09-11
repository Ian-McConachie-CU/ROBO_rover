#!/usr/bin/env python3
"""
Simple ROS2 script to control rover movement
Moves forward, turns, goes straight, turns again, then straight
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')
        
        # Create publisher for cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Wait a moment for publisher to be ready
        time.sleep(0.5)
        
        self.get_logger().info('Starting rover movement sequence...')
        
        # Execute the movement sequence
        self.execute_movement_sequence()
        
    def publish_velocity(self, linear_x, angular_z, duration):
        """Publish velocity command for specified duration"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        # Calculate how many messages to send (at ~10 Hz)
        rate = 10  # Hz
        num_messages = int(duration * rate)
        
        for i in range(num_messages):
            self.cmd_vel_pub.publish(msg)
            time.sleep(1.0 / rate)
            
        self.get_logger().info(f'Sent {num_messages} velocity commands: linear={linear_x}, angular={angular_z}')
    
    def stop_rover(self):
        """Send stop command"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info('Rover stopped')
    
    def execute_movement_sequence(self):
        """Execute the complete movement sequence"""
        
        # 1. Move forward at 0.3 for 1 second
        self.get_logger().info('Step 1: Moving forward at 0.3 m/s for 1 second')
        self.publish_velocity(0.3, 0.0, 1.0)
        
        # 2. Turn at 1.5 while moving forward at 0.3 for 1 second  
        self.get_logger().info('Step 2: Turning at 1.5 rad/s while moving forward at 0.3 m/s for 1 second')
        self.publish_velocity(0.3, 1.5, 1.0)
        
        # 3. Go straight at 0.3 for 1 second
        self.get_logger().info('Step 3: Moving straight at 0.3 m/s for 1 second')
        self.publish_velocity(0.3, 0.0, 1.0)
        
        # 4. Turn at 1.5 while moving forward at 0.3 for 1 second
        self.get_logger().info('Step 4: Turning at 1.5 rad/s while moving forward at 0.3 m/s for 1 second')
        self.publish_velocity(0.3, 1.5, 1.0)
        
        # 5. Go straight at 0.3 for 1 second
        self.get_logger().info('Step 5: Moving straight at 0.3 m/s for 1 second')
        self.publish_velocity(0.3, 0.0, 1.0)
        
        # Stop the rover
        self.stop_rover()
        self.get_logger().info('Movement sequence complete!')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = RoverController()
        # Keep the node alive briefly to ensure all messages are sent
        time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()