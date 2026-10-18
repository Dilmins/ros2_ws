#!/bin/bash
# Keyboard teleop for the drone. Run in its own terminal (needs raw keyboard input).
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")/.."
source install/setup.bash
ros2 run drone_pkg drone_teleop.py
