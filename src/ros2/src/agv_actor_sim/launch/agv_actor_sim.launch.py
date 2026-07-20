"""一次拉起 agv_actor_sim 的两个常驻节点.

为什么用 ExecuteProcess 而不是 Node:
  ROS 2 ament_python 生成的入口脚本会写死 setuptools 的 sys.executable,
  在 venv 上 colcon 会把 shebang 写成 /usr/bin/python3, 导致节点找不到 venv 里
  装的 cosysairsim. 这里直接调 venv 解释器, 用 -m 启动模块, 完全绕过 shebang.

约定:
- 单实例 UE + AirSim RPC 固定跑在 127.0.0.1:41451
- 因此 launch 文件里直接把 host_ip / host_port 写死, 用户无需每次手动传入

需要的环境:
  source ~/Air-UE-project/.venv/bin/activate
  source /opt/ros/jazzy/setup.bash
  source ~/Air-UE-project/src/ros2/install/setup.bash
"""

import os
import shutil

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


# 默认查找 venv python; 可以在调用 launch 时用 venv_python:= 覆盖.
_DEFAULT_VENV_PYTHON = os.path.expanduser("~/Air-UE-project/.venv/bin/python")
if not os.path.exists(_DEFAULT_VENV_PYTHON):
    _fallback = shutil.which("python3") or "python3"
    _DEFAULT_VENV_PYTHON = _fallback


def generate_launch_description() -> LaunchDescription:
    venv_arg = DeclareLaunchArgument(
        "venv_python", default_value=_DEFAULT_VENV_PYTHON,
        description="Python interpreter to run nodes; default points at the project venv")
    object_arg = DeclareLaunchArgument(
        "object", default_value="BP_HuskyVisual_C_1",
        description="UE actor name (World Outliner)")
    prefix_arg = DeclareLaunchArgument(
        "topic_prefix", default_value="/sim_ugv/airsim_node/UGV_1",
        description="ROS2 topic namespace, mirrors Cosys-AirSim wrapper layout")
    rate_arg = DeclareLaunchArgument(
        "imu_rate", default_value="30.0",
        description="IMU + Odom publish rate in Hz; capped at 30")
    speed_arg = DeclareLaunchArgument(
        "max_speed", default_value="2.0",
        description="m/s when throttle = 1.0")
    yaw_rate_arg = DeclareLaunchArgument(
        "max_yaw_rate", default_value="90.0",
        description="deg/s when steering = 1.0")

    venv = LaunchConfiguration("venv_python")
    obj = LaunchConfiguration("object")
    prefix = LaunchConfiguration("topic_prefix")
    imu_rate = LaunchConfiguration("imu_rate")
    max_speed = LaunchConfiguration("max_speed")
    max_yaw_rate = LaunchConfiguration("max_yaw_rate")

    host_ip = "127.0.0.1"
    host_port = "41451"

    actor_proc = ExecuteProcess(
        cmd=[
            venv, "-m", "agv_actor_sim.agv_actor_node",
            "--ros-args",
            "-r", "__node:=agv_actor_node",
            "-p", ["object:=", obj],
            "-p", ["topic_prefix:=", prefix],
            "-p", ["host_ip:=", host_ip],
            "-p", ["host_port:=", host_port],
            "-p", ["max_speed:=", max_speed],
            "-p", ["max_yaw_rate:=", max_yaw_rate],
        ],
        output="screen",
    )

    imu_proc = ExecuteProcess(
        cmd=[
            venv, "-m", "agv_actor_sim.agv_imu_odom_node",
            "--ros-args",
            "-r", "__node:=agv_imu_odom_node",
            "-p", ["object:=", obj],
            "-p", ["topic_prefix:=", prefix],
            "-p", ["host_ip:=", host_ip],
            "-p", ["host_port:=", host_port],
            "-p", ["rate:=", imu_rate],
        ],
        output="screen",
    )

    return LaunchDescription([
        venv_arg,
        object_arg,
        prefix_arg,
        rate_arg,
        speed_arg,
        yaw_rate_arg,
        actor_proc,
        imu_proc,
    ])
