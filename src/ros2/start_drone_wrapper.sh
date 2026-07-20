#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# In PX4/MAVROS mode the wrapper is used for ROS-side sensors only. Do not let
# it take ApiControl, otherwise it races with PX4 initialization and can crash
# while trying to control the vehicle before the PX4-backed pawn is ready.
# Pass host_ip:=<ip> below if AirSim runs on another host (default: see airsim_ros_wrapper.h).
"$SCRIPT_DIR/start_wrapper.sh" namespace:=uav host_port:=41451 enable_api_control:=False enable_object_transforms_list:=False "$@" &
WRAPPER_PID=$!

# AirSim publishes LiDAR as PointCloud2. For standalone UAV wrapper usage we also
# provide a LaserScan conversion here. If launched from coordination_demo, launch
# will set DISABLE_POINTCLOUD_TO_LASERSCAN=1 and supervise the converter itself.
SCAN_PID=""
FILTER_PID=""
if [[ "${DISABLE_POINTCLOUD_TO_LASERSCAN:-0}" != "1" ]]; then
  ros2 run navigation_bringup lidar_self_filter_node --ros-args \
    -r __node:=uav_lidar_self_filter \
    -p input_topic:=/uav/airsim_node/UAV_1/lidar/points/UAV_1_Lidar1 \
    -p output_topic:=/uav/lidar/points_filtered &
  FILTER_PID=$!

  ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
    -r __node:=uav_pointcloud_to_laserscan \
    -r cloud_in:=/uav/lidar/points_filtered \
    -r scan:=/uav/scan \
    -p target_frame:=UAV_1 \
    -p transform_tolerance:=0.01 \
    -p min_height:=-8.0 \
    -p max_height:=8.0 \
    -p angle_min:=-3.14159 \
    -p angle_max:=3.14159 \
    -p angle_increment:=0.0087 \
    -p scan_time:=0.1 \
    -p range_min:=0.65 \
    -p range_max:=120.0 \
    -p use_inf:=True &
  SCAN_PID=$!
fi

trap 'kill "$WRAPPER_PID" ${FILTER_PID:+"$FILTER_PID"} ${SCAN_PID:+"$SCAN_PID"} 2>/dev/null || true' EXIT

sleep 2
ros2 run rviz2 rviz2 -d "$SCRIPT_DIR/rviz/uav.rviz"
