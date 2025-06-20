# CAN Gateway

A ROS2 package that provides a gateway between CAN bus and ROS topics for the Evabot platform.

## Features

- Reads CAN messages and publishes them to ROS topics
- Sends CAN messages from ROS topic subscriptions
- Supports virtual CAN interface for testing and simulation

## Dependencies

- ROS2 (humble/iron)
- `can-utils` for CAN interface utilities
- SocketCAN support

## Build

From the ROS workspace root:

```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select can_gateway

# Source the workspace
source install/setup.bash
```

## Usage

### Setup CAN Interface

For real hardware:
```bash
# Configure your CAN interface (adjust bitrate as needed)
sudo ip link set can0 up type can bitrate 500000
```

For simulation/testing:
```bash
# Create virtual CAN interface
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

### Run the Gateway

```bash
# Launch the CAN gateway
ros2 launch can_gateway gateway.launch.py

# Or run the node directly
ros2 run can_gateway can_gateway_node
```

### Testing

```bash
# Send a test CAN frame
cansend vcan0 123#DEADBEEF

# Monitor ROS topics
ros2 topic echo /can/raw

# Monitor CAN interface
candump vcan0
```

## Topics

### Published Topics

- `/can/raw` (can_msgs/Frame) - Raw CAN frames received from the bus

### Subscribed Topics

- `/can/send` (can_msgs/Frame) - CAN frames to send to the bus

## Parameters

- `interface` (string, default: "vcan0") - CAN interface name
- `loop_rate` (double, default: 100.0) - Loop rate in Hz

## License

[Add your license information here]