# Evabot

This repository contains the workspace for the navigation software controlling the Evabot platform.

Packages included:
- can_gateway, reads ands publishes CAN messages in topics. Sends CAN messages from topics.


## Install

From the root of the ROS workspace:

```bash
# ---- build ----
colcon build
source install/setup.bash

# ---- simulate CAN on desktop ----
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# ---- run gateway ----
ros2 launch can_gateway gateway.launch.py

# ---- send a test frame ----
cansend vcan0 123#DEADBEEF

# ---- echo ROS topic ----
ros2 topic echo /can/raw