#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Pass host_ip:=<ip> below if AirSim runs on another host (default: see airsim_ros_wrapper.h).
"$SCRIPT_DIR/start_wrapper.sh" namespace:=ugv host_port:=41451 enable_api_control:=True enable_object_transforms_list:=False "$@" &
WRAPPER_PID=$!
trap 'kill "$WRAPPER_PID" 2>/dev/null || true' EXIT

sleep 2
ros2 run rviz2 rviz2 -d "$SCRIPT_DIR/rviz/agv.rviz"
