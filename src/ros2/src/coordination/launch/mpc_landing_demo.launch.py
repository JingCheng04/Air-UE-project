"""coni-mpc 降落 demo launch (PX4 原生版本).

整体流程:
  1. 复用现有 agv_chase_demo: 起飞 -> 巡航追 AGV GPS -> 距离 keep_distance
     处悬停在 AGV 上方 (chase 状态机自己保持高度).
  2. 本 launch 在 chase demo 之上额外拉起:
       a) coni_mpc_controller : 论文里 "受控对象" 的低层 MPC.
       b) mpc_attitude_bridge_node : AttitudeTarget 仲裁转发到 /mavros/setpoint_raw/attitude.
       c) mpc_land_coordinator_node : 等悬停 -> 启用桥接 -> 降落.
  3. AGV 仍由 agv_figure_eight_node 控制做 8 字, 不受影响.

约束:
  * 不修改 chase demo / agv 仿真栈 / coni-mpc 控制律.
  * mpc_attitude_bridge_node 启用时转发 AttitudeTarget 到 PX4, 失能时停止,
    让 state_machine 的 velocity setpoint 继续控制飞机. PX4 Offboard 自动
    在 attitude/velocity 多路 setpoint 间切换, 无需手动抢夺控制权.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _coordination_share() -> str:
    return get_package_share_directory("coordination")


def _coni_mpc_share() -> str:
    return get_package_share_directory("coni-mpc")


def generate_launch_description() -> LaunchDescription:
    object_arg = DeclareLaunchArgument(
        "object", default_value="BP_HuskyVisual_C_1",
        description="UE actor name for the AGV (Unreal World Outliner)")
    uav_height_arg = DeclareLaunchArgument(
        "uav_height", default_value="10.0",
        description="UAV cruise altitude (m above takeoff hover_z)")
    keep_distance_arg = DeclareLaunchArgument(
        "keep_distance", default_value="1.5",
        description="Horizontal distance to AGV at which UAV stops and hovers (m). "
                    "降落 demo 里需要悬停在 AGV 正上方, 所以这里默认设小")
    hover_seconds_arg = DeclareLaunchArgument(
        "hover_seconds", default_value="15.0",
        description="Hover duration (s) after reaching cruise altitude before MPC takes over (first attempt)")
    cruise_altitude_arg = DeclareLaunchArgument(
        "cruise_altitude", default_value="9.0",
        description="UAV odom z (ENU, m) threshold to consider cruise altitude reached")

    object_name = LaunchConfiguration("object")
    uav_height = LaunchConfiguration("uav_height")
    keep_distance = LaunchConfiguration("keep_distance")
    hover_seconds = LaunchConfiguration("hover_seconds")
    cruise_altitude = LaunchConfiguration("cruise_altitude")

    # ---- 1. 复用 chase demo: drone_wrapper + AGV 8 字 + nav2 + chase 节点 ----
    chase_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_coordination_share(), "launch", "agv_chase_demo.launch.py")
        ),
        launch_arguments={
            "object": object_name,
            "uav_height": uav_height,
            "keep_distance": keep_distance,
        }.items(),
    )

    # ---- 2. coni_mpc_controller: 通过 remap 把 IO 接到协调器 ----
    # 输入 : /uav/coni_mpc/imu, car_odom, quad_odom (协调器发布)
    # 输出 : /uav/coni_mpc/attitude_target (桥接转发到 MAVROS)
    # MPC 参数 (Q_p_z, min_thrust, max_v_z 等) 已在 yaml 中调为平缓下降.
    coni_mpc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_coni_mpc_share(), "launch", "coni_mpc_controller.launch.py")
        ),
        launch_arguments={
            "imu_topic": "/uav/coni_mpc/imu",
            "car_odom_topic": "/uav/coni_mpc/car_odom",
            "quad_odom_topic": "/uav/coni_mpc/quad_odom",
            "control_topic": "/uav/coni_mpc/attitude_target",
        }.items(),
    )

    # ---- 3. AttitudeTarget 仲裁转发桥接 (coni-mpc -> MAVROS) ----
    bridge_node = Node(
        package="coordination",
        executable="mpc_attitude_bridge_node",
        name="mpc_attitude_bridge_node",
        output="screen",
        parameters=[{
            "rate": 30.0,
            "cmd_timeout": 0.5,
            "attitude_input_topic": "/uav/coni_mpc/attitude_target",
            "attitude_output_topic": "/mavros/setpoint_raw/attitude",
            "enable_topic": "/uav/coni_mpc/bridge_enable",
        }],
    )

    # ---- 4. 降落协调器: 悬停 10s -> 启用桥接 -> 降落 ----
    coordinator_node = Node(
        package="coordination",
        executable="mpc_land_coordinator_node",
        name="mpc_land_coordinator_node",
        output="screen",
        parameters=[{
            "uav_prefix": "/uav/airsim_node/UAV_1",
            "agv_prefix": "/sim_ugv/airsim_node/UGV_1",
            "agv_imu_name": "UGV_1_Imu",
            "yoloe_prefix": "/uav/yoloe",
            "camera_pitch_deg": -45.0,
            "hover_seconds": hover_seconds,
            "cruise_altitude": cruise_altitude,
            "publish_rate": 30.0,
            "touch_down_dz": 0.6,
            "yoloe_stale_sec": 1.5,
            "mpc_timeout_sec": 30.0,
            "bridge_enable_topic": "/uav/coni_mpc/bridge_enable",
            "coni_mpc_prefix": "/uav/coni_mpc",
        }],
    )

    # coni-mpc 的 fixed_z 由协调器动态调整 (分段下降), 初始值见 coordinator 参数.
    # 如需手动调整可在运行期: ros2 param set /coni_mpc_controller fixed_z 1.0

    return LaunchDescription([
        object_arg,
        uav_height_arg,
        keep_distance_arg,
        hover_seconds_arg,
        cruise_altitude_arg,
        chase_demo,
        coni_mpc,
        bridge_node,
        coordinator_node,
    ])
