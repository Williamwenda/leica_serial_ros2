# Leica Serial ROS2

This package provides a ROS2 node for streaming data from a Leica total station via serial port.

## Overview

The `leica_serial_ros2` package converts the original ROS1 Leica streaming application to ROS2. It connects to a Leica total station via serial port and publishes position data and TF transforms.

## Dependencies

- ROS2 (tested with Jazzy on Ubuntu 24.04)
- Boost libraries (system, thread)
- Standard ROS2 packages: rclcpp, std_msgs, geometry_msgs, nav_msgs, tf2, tf2_ros

## Building

```bash
cd /path/to/your/ros2_workspace
colcon build --packages-select leica_serial_ros2
source install/setup.bash
```

## Usage

### Basic Launch

Launch the node with default settings (using `/dev/ttyUSB0`):

```bash
ros2 launch leica_serial_ros2 leica_serial.launch.py
```

### Custom Serial Port

Launch with a different serial port:

```bash
ros2 launch leica_serial_ros2 leica_serial.launch.py comport:=/dev/ttyUSB1
```

### Full-featured Launch

Launch with additional configuration options:

```bash
ros2 launch leica_serial_ros2 leica_serial_full.launch.py comport:=/dev/ttyUSB0 log_level:=debug
```

### Demo Launch

Launch with additional helper nodes for testing and visualization:

```bash
ros2 launch leica_serial_ros2 leica_demo.launch.py comport:=/dev/ttyUSB0
```

## Topics

### Published Topics

- `/leica/position` (geometry_msgs/PointStamped): Prism position in 3D space
- `/tf` (tf2_msgs/TFMessage): Transform from "world" to "leica_pos" frame

### Subscribed Topics

- `/paintcopter/position` (nav_msgs/Odometry): External position input for prism tracking
- `/leica/start_stop` (std_msgs/Bool): Command to start/stop measurements

## Parameters

- `comport` (string, default: "/dev/ttyUSB0"): Serial port device path

## Launch File Options

### leica_serial.launch.py
Basic launch file with essential parameters:
- `comport`: Serial port device path
- `log_level`: ROS2 logging level

### leica_serial_full.launch.py
Full-featured launch file with additional options:
- `comport`: Serial port device path
- `log_level`: ROS2 logging level
- `namespace`: Node namespace
- `use_sim_time`: Use simulation time
- `frame_id`: Frame ID for published position
- `child_frame_id`: Child frame ID for TF transform

### leica_demo.launch.py
Demo launch file with helper nodes:
- `comport`: Serial port device path
- Includes static transform publisher
- Optional RViz2 configuration (commented out)

## Troubleshooting

### Serial Port Permissions

If you get permission errors accessing the serial port, add your user to the dialout group:

```bash
sudo usermod -a -G dialout $USER
```

Then log out and back in for the changes to take effect.

### Device Not Found

Check if your device is connected and detected:

```bash
ls -la /dev/ttyUSB*
# or
dmesg | grep tty
```

### Testing Communication

You can test the topics using ROS2 command line tools:

```bash
# Monitor position messages
ros2 topic echo /leica/position

# Send start command
ros2 topic pub /leica/start_stop std_msgs/msg/Bool "data: true"

# Send stop command  
ros2 topic pub /leica/start_stop std_msgs/msg/Bool "data: false"

# View TF tree
ros2 run tf2_tools view_frames
```