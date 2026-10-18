#!/bin/bash
# Build drone_pkg and launch Gazebo + drone + controller (opens the Gazebo window via WSLg).
set -e
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")/.."
pkill -f gzserver || true
pkill -f gzclient || true
colcon build --packages-select drone_pkg
source install/setup.bash
ros2 launch drone_pkg sim.launch.py
