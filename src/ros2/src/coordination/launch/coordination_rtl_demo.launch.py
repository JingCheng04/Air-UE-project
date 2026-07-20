"""RTL demo launch.

流程:
1. 启动 AGV waypoint 巡航栈。
2. 起飞前用 follow 节点把 UAV 绑定在 AGV 上。
3. 去程仍走原协调节点巡航与避障。
4. 回程在 home 附近切 coni-mpc 跟踪 AGV 并降落。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    object_arg = DeclareLaunchArgument(
        "object", default_value="BP_HuskyVisual_C_1",
        description="UE actor name for the pseudo UGV",
    )
    ugv_distance_arg = DeclareLaunchArgument("ugv_distance", default_value="0.0")
    uav_height_arg = DeclareLaunchArgument("uav_height", default_value="8.0")
    target_latitude_arg = DeclareLaunchArgument(
        # "target_latitude", default_value="45.719851223043335",
        "target_latitude", default_value="45.72090148925781",
        description="Target latitude in decimal degrees (WGS84)",
    )
    target_longitude_arg = DeclareLaunchArgument(
        # "target_longitude", default_value="-123.93299569829097",
        "target_longitude", default_value="-123.9339370727539",
        description="Target longitude in decimal degrees (WGS84)",
    )
    target_prompt_arg = DeclareLaunchArgument(
        "target_prompt",
        default_value="yellow and black robot,ground robot,small four-wheeled robot",
        description=(
            "YOLOE text prompts (comma-separated). Multiple prompts give YOLOE "
            "a richer text embedding set; the union of detections is returned. "
            "Default targets a black/yellow Husky-class AGV with black wheels."
        ),
    )
    yoloe_model_arg = DeclareLaunchArgument(
        "yoloe_model", default_value="yoloe-26x-seg.pt",
        description="YOLOE checkpoint name resolved by ultralytics",
    )
    settings_path_arg = DeclareLaunchArgument(
        "settings_path", default_value="~/Documents/AirSim/settings.json",
        description="AirSim settings.json (used to read camera intrinsics)",
    )

    object_name = LaunchConfiguration("object")
    repo_root = os.path.expanduser("~/Air-UE-project")
    drone_script = os.path.join(repo_root, "src", "ros2", "start_drone_wrapper.sh")

    drone_wrapper = ExecuteProcess(
        cmd=["bash", drone_script],
        output="screen",
        # 让 launch 自己接管 pointcloud_to_laserscan, 与原 demo 保持一致.
        additional_env={"DISABLE_POINTCLOUD_TO_LASERSCAN": "1"},
    )

    agv_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("agv_actor_sim"),
                         "launch", "agv_actor_sim.launch.py")
        ),
        launch_arguments={
            "object": object_name,
            "topic_prefix": "/sim_ugv/airsim_node/UGV_1",
        }.items(),
    )

    agv_cruise_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("agv_actor_sim"),
                         "launch", "agv_waypoint_cruise.launch.py")
        ),
        launch_arguments={
            "object": object_name,
            "topic_prefix": "/sim_ugv/airsim_node/UGV_1",
            "stack": "false",
        }.items(),
    )

    # Keep the original startup sequence: the UAV remains forcibly bound to
    # the stationary AGV during warmup, then the AGV starts moving after 15s.
    agv_launch = TimerAction(period=15.0, actions=[agv_cruise_launch])

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("navigation_bringup"),
                         "launch", "nav2_dwb.launch.py")
        ),
    )

    uav_state_machine_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("uav_state_machine"),
                "launch",
                "uav_state_machine_with_mavros.launch.py",
            )
        ),
    )

    coni_mpc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("coni-mpc"), "launch", "coni_mpc_controller.launch.py")
        ),
        launch_arguments={
            "imu_topic": "/uav/coni_mpc/imu",
            "car_odom_topic": "/uav/coni_mpc/car_odom",
            "quad_odom_topic": "/uav/coni_mpc/quad_odom",
            "control_topic": "/uav/coni_mpc/attitude_target",
        }.items(),
    )

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

    coordination_node = Node(
        package="coordination",
        executable="uav_with_rtl_node",
        name="uav_with_rtl_node",
        output="screen",
        parameters=[{
            "ugv_prefix": "/sim_ugv/airsim_node/UGV_1",
            "uav_prefix": "/uav/airsim_node/UAV_1",
            "uav_name": "UAV_1",
            "ugv_distance": LaunchConfiguration("ugv_distance"),
            "uav_height": LaunchConfiguration("uav_height"),
            # Reuse mpc_landing -> agv_chase binding, but keep the RTL-required
            # 15s bound warmup. The follow node force-writes UAV kinematics at
            # 60Hz throughout this wait, even if PX4 is already producing
            # setpoints. Release happens only when UAV_TAKEOFF starts.
            "ugv_start_delay": 15.0,
            "target_latitude": ParameterValue(
                LaunchConfiguration("target_latitude"), value_type=float),
            "target_longitude": ParameterValue(
                LaunchConfiguration("target_longitude"), value_type=float),
            # Match coordination_demo: lower horizontal speed avoids the
            # aggressive roll/pitch transients seen in RTL AVOID.
            "cruise_speed": 3.0,
            "obstacle_distance": 7.0,
            "obstacle_clear_distance": 9.0,
            "obstacle_clear_hold": 0.4,
            # 与 coordination_demo/agv_chase_demo/RTL backup 一致: 尽快结束
            # UAV_ASCEND, 进入 FLY_TO_TARGET/RTL_FLY_AGV 后避障逻辑才会运行。
            "uav_ascend_speed": 3.0,
            # Match mpc_landing -> agv_chase startup binding.
            "bind_on_startup": True,
            "bind_resend_rate": 5.0,
            # bind 15s -> release -> 0.3s grace -> OFFBOARD/ARM -> ASCEND.
            # release_delay_seconds=0 means release on UAV_TAKEOFF; grace 让
            # follow_node 的 simSetKinematics 真正停下, 再放父类做 ARM,
            # 否则 OFFBOARD/ARM 与 kinematics 抢控制会引起 release 瞬间的上抛。
            "release_delay_seconds": 0.0,
            "release_grace_seconds": 0.3,
            "rtl_landing_pause": 2.0,
            "home_tolerance": 1.5,
            # 10m: 进入视觉门限后停止继续贴近, 持续 yaw 对准 AGV,
            # 等待 YOLOE/MPC 条件; 超出 10m 才重新跟踪。
            "approach_enable_distance": 10.0,
            # 10m enters detection; 12m exits it. This prevents GPS noise at
            # the boundary from resetting the 15s MPC handoff timer.
            "approach_exit_distance": 12.0,
            "camera_pitch_deg": -45.0,
            "yoloe_prefix": "/uav/yoloe",
            "bridge_enable_topic": "/uav/coni_mpc/bridge_enable",
            "coni_mpc_prefix": "/uav/coni_mpc",
            # 以下 5 项与 mpc_landing_demo.launch.py 的 mpc_land_coordinator_node
            # 保持同名同值, 保证 "接近车辆后的降落逻辑" 与 mpc_landing_demo 同源。
            "touch_down_dz": 0.6,
            "yoloe_stale_sec": 1.5,
            "target_lost_timeout": 2.0,
            "descend_final": 0.30,
            "hover_seconds": 15.0,
            "yaw_align_tolerance": 0.35,
            # imu_blind_alt 未在 mpc_landing_demo 里显式配置, 用 coordinator 默认 2.0m。
            "imu_blind_alt": 2.0,
        }],
    )

    # YOLOE 节点: 订阅 AirSim 相机 Scene + Depth, 发布检测 + 标注图.
    yoloe_node = Node(
        package="coordination",
        executable="yoloe_detector_node",
        name="yoloe_detector_node",
        output="screen",
        parameters=[{
            "uav_prefix": "/uav/airsim_node/UAV_1",
            "camera_name": "front_center",
            "out_prefix": "/uav/yoloe",
            "target_prompt": LaunchConfiguration("target_prompt"),
            "model_path": LaunchConfiguration("yoloe_model"),
            "conf": 0.25,
            "iou": 0.7,
            "settings_path": LaunchConfiguration("settings_path"),
            "vehicle_name": "UAV_1",
            "yoloe_script_path": os.path.join(
                repo_root, "src", "ros2", "src", "coordination",
                "coordination", "yoloe-position.py",
            ),
        }],
    )

    # 以下 LiDAR/scan 子图与原 coordination_demo 完全相同, 保证父类的避障逻辑可用.
    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="uav_lidar_static_tf",
        output="screen",
        arguments=[
            "0", "0", "-0.15",
            "0", "0", "0",
            "UAV_1", "UAV_1/UAV_1_Lidar1",
        ],
    )

    lidar_self_filter_node = Node(
        package="navigation_bringup",
        executable="lidar_self_filter_node",
        name="uav_lidar_self_filter",
        output="screen",
        parameters=[{
            "input_topic": "/uav/airsim_node/UAV_1/lidar/points/UAV_1_Lidar1",
            "output_topic": "/uav/lidar/points_filtered",
        }],
    )

    pointcloud_to_scan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="uav_pointcloud_to_laserscan",
        output="screen",
        remappings=[
            ("cloud_in", "/uav/lidar/points_filtered"),
            ("scan", "/uav/scan"),
        ],
        parameters=[{
            "target_frame": "UAV_1",
            "transform_tolerance": 0.01,
            "min_height": -8.0,
            "max_height": 8.0,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.0087,
            "scan_time": 0.1,
            "range_min": 0.65,
            "range_max": 120.0,
            "use_inf": True,
        }],
    )

    follow_node = Node(
        package="coordination",
        executable="uav_follow_agv_node",
        name="uav_follow_agv_node",
        output="screen",
        parameters=[{
            "uav_prefix": "/uav/airsim_node/UAV_1",
            "uav_name": "UAV_1",
            "agv_object": object_name,
            "rate": 60.0,
        }],
    )

    return LaunchDescription([
        object_arg,
        ugv_distance_arg,
        uav_height_arg,
        target_latitude_arg,
        target_longitude_arg,
        target_prompt_arg,
        yoloe_model_arg,
        settings_path_arg,
        drone_wrapper,
        uav_state_machine_launch,
        agv_stack_launch,
        agv_launch,
        nav2_launch,
        coni_mpc,
        bridge_node,
        coordination_node,
        yoloe_node,
        lidar_static_tf,
        lidar_self_filter_node,
        pointcloud_to_scan_node,
        follow_node,
    ])
