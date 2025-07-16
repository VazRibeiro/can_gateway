# CAN Gateway

ROS 2 lifecycle node that bridges CAN bus and ROS 2 topics.

## Features

- Bidirectional CAN ↔ ROS 2 communication
- Lifecycle node with systemd integration
- Supports hardware and virtual CAN interfaces
- High-performance epoll-based I/O

## Topics

- `/can/raw` (can_msgs/Frame) - CAN frames from bus
- `/can/tx` (can_msgs/Frame) - CAN frames to bus

## Parameters

- `interface` (string, default: "vcan0") - CAN interface name
- `rx_rate_hz` (int, default: 10000) - Polling rate

## Build

```bash
colcon build --packages-select can_gateway
```

## Usage

```bash
# Launch with default interface
ros2 launch can_gateway gateway.launch.py

# Launch with specific interface
ros2 launch can_gateway gateway.launch.py interface:=can0

# Manual lifecycle control
ros2 service call /can_gateway/configure lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
ros2 service call /can_gateway/activate lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
```
