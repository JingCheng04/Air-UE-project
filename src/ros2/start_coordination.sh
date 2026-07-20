#!/usr/bin/env bash
# start_coordination.sh
#
# Brings up the dual-instance UAV+UGV joint simulation:
#   /uav/airsim_node  -> drone UE instance on port 41451
#   /ugv/airsim_node  -> AGV   UE instance on port 41452
#
# Prerequisites:
#   - Two UE instances are already in Play with the matching settings:
#       src/test/settings/settings_joint_drone.json   (Multirotor,  41451)
#       src/test/settings/settings_joint_agv.json     (SkidVehicle, 41452)
#   - The ROS2 workspace was built (install/setup.bash exists).
#
# Usage:
#   ./start_coordination.sh
#   tail -f /tmp/uav_wrapper.log /tmp/ugv_wrapper.log
#   Ctrl+C to stop both wrappers.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [[ ! -f install/setup.bash ]]; then
  echo "ROS2 workspace is not ready: build it first with 'colcon build'."
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

ros2 pkg prefix airsim_ros_pkgs >/dev/null 2>&1 || {
  echo "ROS2 workspace is not ready: airsim_ros_pkgs not found."; exit 1; }

UAV_LOG="/tmp/uav_wrapper.log"
UGV_LOG="/tmp/ugv_wrapper.log"

cleanup() {
  if [[ -n "${UAV_PID:-}" ]]; then kill "$UAV_PID" 2>/dev/null || true; fi
  if [[ -n "${UGV_PID:-}" ]]; then kill "$UGV_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

echo "Starting UAV wrapper (/uav, host_port=41451). Logs: $UAV_LOG"
ros2 launch airsim_ros_pkgs airsim_node.launch.py \
  namespace:=uav host_port:=41451 \
  enable_api_control:=True enable_object_transforms_list:=False \
  > "$UAV_LOG" 2>&1 &
UAV_PID=$!

echo "Starting AGV wrapper (/ugv, host_port=41452). Logs: $UGV_LOG"
ros2 launch airsim_ros_pkgs airsim_node.launch.py \
  namespace:=ugv host_port:=41452 \
  enable_api_control:=True enable_object_transforms_list:=False \
  > "$UGV_LOG" 2>&1 &
UGV_PID=$!

echo "Both wrappers launched. Waiting until either exits (Ctrl+C to stop)."
wait -n "$UAV_PID" "$UGV_PID"
echo "One wrapper exited; shutting the other down."
