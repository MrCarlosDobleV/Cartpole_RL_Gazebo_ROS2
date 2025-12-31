#!/usr/bin/env bash
set -euo pipefail

# Stop any existing Gazebo / ROS 2 launch processes.
pkill -f "ign gazebo" || true
pkill -f "gz sim" || true
pkill -f "ros2 launch cartpole_bringup cartpole.launch.py" || true

set +u
source "${HOME}/cartpole_ws/install/setup.bash"
set -u
exec ros2 launch cartpole_bringup cartpole.launch.py
