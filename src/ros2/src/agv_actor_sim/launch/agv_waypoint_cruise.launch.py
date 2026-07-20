"""一次性拉起完整的伪 AGV 仿真栈 (actor + imu/odom/gps + 巡航).

通过 ``IncludeLaunchDescription`` 把 ``agv_actor_sim.launch.py`` 嵌进来,
所以这一条 launch 会同时启动:

  - ``agv_actor_node``        把 ``car_cmd`` 转成 actor 运动
  - ``agv_imu_odom_node``     发布 IMU / Odom / GPS topic
  - ``agv_waypoint_cruise_node``  按 ``AGV_Traj.yaml`` 巡航

如果用户已经在别处单独跑了 actor + imu_odom (例如复用既有的 ROS topic),
可以加 ``stack:=false`` 只启动巡航节点.

典型用法:
    # 一条命令拉起 actor + imu_odom + 巡航
    ros2 launch agv_actor_sim agv_waypoint_cruise.launch.py

    # 已经在别处启动了 actor + imu_odom, 只起巡航
    ros2 launch agv_actor_sim agv_waypoint_cruise.launch.py stack:=false

主要参数:
    venv_python         运行节点的 Python 解释器, 默认 ~/.venv/bin/python
    waypoints_file      YAML 路径; 默认指向已安装的 share/agv_actor_sim/AGV_Traj.yaml
    topic_prefix        与 actor / imu_odom 节点保持一致
    object              UE actor 名 (传给 agv_actor_node, 默认 BP_HuskyVisual_C_1)
    target_speed        巡航线速度, 默认 2.0 m/s
    min_turn_radius     最小转弯半径, 默认 3.0 m
    waypoint_tolerance  到点判定容差, 默认 1.0 m
    max_speed           actor throttle=1 对应线速度, 默认 2.5 m/s
    max_yaw_rate        actor steering=1 对应角速度, 默认 90 deg/s
    stack               是否同时启动 actor + imu_odom, 默认 true
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


_DEFAULT_VENV_PYTHON = os.path.expanduser("~/Air-UE-project/.venv/bin/python")
if not os.path.exists(_DEFAULT_VENV_PYTHON):
    _DEFAULT_VENV_PYTHON = shutil.which("python3") or "python3"


def _default_waypoints_file() -> str:
    src_yaml = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "src", "agv_actor_sim", "AGV_Traj.yaml"))
    if os.path.exists(src_yaml):
        return src_yaml
    try:
        share = get_package_share_directory("agv_actor_sim")
        return os.path.join(share, "AGV_Traj.yaml")
    except Exception:
        return ""


def _stack_launch_path() -> str:
    share = get_package_share_directory("agv_actor_sim")
    return os.path.join(share, "launch", "agv_actor_sim.launch.py")


def generate_launch_description() -> LaunchDescription:
    venv_arg = DeclareLaunchArgument(
        "venv_python", default_value=_DEFAULT_VENV_PYTHON,
        description="Python interpreter to run nodes; default points at the project venv")
    wp_arg = DeclareLaunchArgument(
        "waypoints_file", default_value=_default_waypoints_file(),
        description="YAML waypoints path; defaults to installed share/AGV_Traj.yaml")
    prefix_arg = DeclareLaunchArgument(
        "topic_prefix", default_value="/sim_ugv/airsim_node/UGV_1",
        description="ROS2 topic namespace, must match agv_actor_node and agv_imu_odom_node")
    object_arg = DeclareLaunchArgument(
        "object", default_value="BP_HuskyVisual_C_1",
        description="UE actor name in the World Outliner")
    speed_arg = DeclareLaunchArgument(
        "target_speed", default_value="2.0",
        description="cruise linear speed in m/s")
    radius_arg = DeclareLaunchArgument(
        "min_turn_radius", default_value="3.0",
        description="minimum turning radius in meters")
    max_speed_arg = DeclareLaunchArgument(
        "max_speed", default_value="2.5",
        description="m/s when throttle=1.0; passed to both agv_actor_node and the cruise node")
    max_yaw_rate_arg = DeclareLaunchArgument(
        "max_yaw_rate", default_value="90.0",
        description="deg/s when steering=1.0; passed to both agv_actor_node and the cruise node")
    tol_arg = DeclareLaunchArgument(
        "waypoint_tolerance", default_value="1.0",
        description="distance threshold to mark a waypoint reached (m)")
    stack_arg = DeclareLaunchArgument(
        "stack", default_value="true",
        description="if true, also start agv_actor_node + agv_imu_odom_node via "
                    "agv_actor_sim.launch.py; set false if you've already started them elsewhere")
    imu_rate_arg = DeclareLaunchArgument(
        "imu_rate", default_value="30.0",
        description="IMU + Odom publish rate in Hz; capped at 30 (passed through to imu_odom node)")

    venv = LaunchConfiguration("venv_python")
    wp = LaunchConfiguration("waypoints_file")
    prefix = LaunchConfiguration("topic_prefix")
    obj = LaunchConfiguration("object")
    target_speed = LaunchConfiguration("target_speed")
    min_turn_radius = LaunchConfiguration("min_turn_radius")
    max_speed = LaunchConfiguration("max_speed")
    max_yaw_rate = LaunchConfiguration("max_yaw_rate")
    tol = LaunchConfiguration("waypoint_tolerance")
    stack = LaunchConfiguration("stack")
    imu_rate = LaunchConfiguration("imu_rate")

    # 把 actor + imu_odom 那条 launch 整体嵌进来; stack:=false 时跳过.
    stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_stack_launch_path()),
        launch_arguments={
            "venv_python": venv,
            "object": obj,
            "topic_prefix": prefix,
            "imu_rate": imu_rate,
            "max_speed": max_speed,
            "max_yaw_rate": max_yaw_rate,
        }.items(),
        condition=IfCondition(stack),
    )

    cruise_proc = ExecuteProcess(
        cmd=[
            venv, "-m", "agv_actor_sim.agv_waypoint_cruise_node",
            "--ros-args",
            "-r", "__node:=agv_waypoint_cruise_node",
            "-p", ["waypoints_file:=", wp],
            "-p", ["topic_prefix:=", prefix],
            "-p", ["target_speed:=", target_speed],
            "-p", ["min_turn_radius:=", min_turn_radius],
            "-p", ["max_speed:=", max_speed],
            "-p", ["max_yaw_rate:=", max_yaw_rate],
            "-p", ["waypoint_tolerance:=", tol],
        ],
        output="screen",
    )

    return LaunchDescription([
        venv_arg,
        wp_arg,
        prefix_arg,
        object_arg,
        speed_arg,
        radius_arg,
        max_speed_arg,
        max_yaw_rate_arg,
        tol_arg,
        stack_arg,
        imu_rate_arg,
        stack_launch,
        cruise_proc,
    ])
