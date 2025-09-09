# ArduPilot Rover ROS2 Package

This ROS2 package provides integrated control and IMU data publishing for a Pixhawk 4 Mini running ArduPilot Rover firmware.

## Features

- **Unified Node**: Combines steering/throttle control and IMU data publishing in a single node
- **Fixed Rate Control**: Sends commands at 20Hz and publishes IMU data at 20Hz
- **Automatic Connection**: Establishes MAVLink connection and arms the rover
- **Safety Features**: Falls back to default values when no commands received
- **ROS2 Integration**: Full ROS2 node with proper QoS profiles and message types

## Topics

### Published Topics
- `/imu/data` (`sensor_msgs/Imu`): IMU data (gyroscope and accelerometer)
- `/rover/armed` (`std_msgs/Bool`): Rover armed status

### Subscribed Topics
- `/cmd_vel` (`geometry_msgs/Twist`): Velocity commands
  - `linear.x`: Forward/backward speed (-1.0 to 1.0)
  - `angular.z`: Turning rate (-1.0 to 1.0)

## Package Structure

```
ardupilot_rover/
├── ardupilot_rover/
│   ├── __init__.py
│   └── rover_node.py          # Main rover node
├── launch/
│   └── rover.launch.py        # Launch file
├── scripts/
│   └── rover_node.py          # Executable script
├── CMakeLists.txt
├── package.xml
├── setup.py
├── requirements.txt
└── README.md
```

## Installation

1. **Install Python dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Create ROS2 workspace (if not already done):**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

3. **Copy this package to your workspace:**
   ```bash
   # Copy the ardupilot_rover folder to ~/ros2_ws/src/
   ```

4. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ardupilot_rover
   source install/setup.bash
   ```

5. **Set up permissions for serial access:**
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in for changes to take effect
   ```

## Usage

### Basic Launch

Launch with default parameters:
```bash
ros2 launch ardupilot_rover rover.launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch ardupilot_rover rover.launch.py \
    connection_string:=/dev/ttyACM0 \
    baud_rate:=57600 \
    control_frequency:=25.0 \
    imu_frequency:=25.0 \
    default_throttle:=0.0 \
    default_steering:=0.0
```

### Available Launch Parameters

- `connection_string`: MAVLink connection (default: `/dev/ttyACM1`)
- `baud_rate`: Serial baud rate (default: `115200`)
- `control_frequency`: Control command rate in Hz (default: `20.0`)
- `imu_frequency`: IMU publishing rate in Hz (default: `20.0`)
- `default_throttle`: Default throttle when no commands (-1.0 to 1.0, default: `0.0`)
- `default_steering`: Default steering when no commands (-1.0 to 1.0, default: `0.0`)
- `namespace`: Namespace for the rover node (default: empty)

## Controlling the Rover

### Using Command Line

Send velocity commands using the command line:

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}'

# Turn left while moving forward
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}, angular: {z: 0.5}}'

# Turn right
ros2 topic pub /cmd_vel geometry_msgs/Twist '{angular: {z: -0.5}}'

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist '{}'
```

### Using Python Script

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def move_forward(self, speed=0.5):
        msg = Twist()
        msg.linear.x = speed
        self.cmd_pub.publish(msg)
        
    def turn_left(self, rate=0.5):
        msg = Twist()
        msg.angular.z = rate
        self.cmd_pub.publish(msg)
        
    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    controller = RoverController()
    
    # Example usage
    controller.move_forward(0.3)
    
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Monitoring Data

### View IMU Data

```bash
# Show IMU messages
ros2 topic echo /imu/data

# Show IMU data rate
ros2 topic hz /imu/data

# Plot IMU data (requires rqt)
rqt_plot /imu/data/angular_velocity/x /imu/data/angular_velocity/y /imu/data/angular_velocity/z
```

### View Rover Status

```bash
# Check if rover is armed
ros2 topic echo /rover/armed

# View all available topics
ros2 topic list

# Get topic info
ros2 topic info /cmd_vel
ros2 topic info /imu/data
```

## Safety Features

1. **Command Timeout**: If no velocity commands are received for 1 second, the rover will use default throttle and steering values (usually 0,0 to stop).

2. **Automatic Arming**: The node automatically sets ACRO mode and arms the rover on startup.

3. **Clean Shutdown**: When the node is terminated, it stops the rover and disarms it.

4. **Connection Monitoring**: The node monitors the MAVLink connection and reports status.

## Troubleshooting

### Connection Issues

1. **Check device permissions:**
   ```bash
   ls -l /dev/ttyACM*
   # Should show your user in the group, or try:
   sudo chmod 666 /dev/ttyACM1
   ```

2. **Verify ArduPilot is running:**
   ```bash
   # Connect with MAVProxy to test
   mavproxy.py --master=/dev/ttyACM1 --baudrate=115200
   ```

3. **Check available serial ports:**
   ```bash
   dmesg | grep tty
   ls /dev/ttyACM* /dev/ttyUSB*
   ```

### Node Issues

1. **Check ROS2 setup:**
   ```bash
   source /opt/ros/humble/setup.bash  # or your ROS2 distro
   source ~/ros2_ws/install/setup.bash
   ```

2. **Verify package installation:**
   ```bash
   ros2 pkg list | grep ardupilot_rover
   ros2 run ardupilot_rover rover_node --help
   ```

3. **Check node logs:**
   ```bash
   ros2 launch ardupilot_rover rover.launch.py --log-level DEBUG
   ```

### IMU Data Issues

1. **Verify message types:**
   ```bash
   ros2 interface show sensor_msgs/Imu
   ```

2. **Check if IMU messages are being received:**
   ```bash
   # In another terminal, monitor MAVLink messages
   mavproxy.py --master=/dev/ttyACM1 --baudrate=115200
   # Then type: output list
   ```

## Development

### File Organization

Place files in the following structure:

```
~/ros2_ws/src/ardupilot_rover/
├── ardupilot_rover/
│   ├── __init__.py
│   └── rover_node.py
├── launch/
│   └── rover.launch.py
├── scripts/
│   └── rover_node.py  # Copy of rover_node.py, made executable
├── resource/
│   └── ardupilot_rover  # Empty marker file
├── CMakeLists.txt
├── package.xml
├── setup.py
├── requirements.txt
└── README.md
```

### Making the Script Executable

```bash
cp ardupilot_rover/rover_node.py scripts/rover_node.py
chmod +x scripts/rover_node.py
```

### Building and Testing

```bash
cd ~/ros2_ws
colcon build --packages-select ardupilot_rover
source install/setup.bash

# Test the node directly
ros2 run ardupilot_rover rover_node

# Test the launch file
ros2 launch ardupilot_rover rover.launch.py
```
