#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

ros2 pkg prefix airsim_interfaces >/dev/null 2>&1 || { echo "ROS2 workspace is not ready: airsim_interfaces not found."; exit 1; }
ros2 pkg prefix airsim_ros_pkgs >/dev/null 2>&1 || { echo "ROS2 workspace is not ready: airsim_ros_pkgs not found."; exit 1; }

echo "ROS2 workspace is ready. Starting Cosys-AirSim wrapper."

exec ros2 launch airsim_ros_pkgs airsim_node.launch.py "$@"
