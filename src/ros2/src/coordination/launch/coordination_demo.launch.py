"""统一拉起:
1. UAV wrapper (start_drone_wrapper.sh)
2. 伪 AGV wrapper (agv_actor_sim.launch.py)
3. 协调节点 (ugv_then_uav_node)
4. UAV 状态机 (通过 uav_state_machine 包内 launch 统一拉起)

本包不处理 Python 虚拟环境.
用户负责在 colcon build 与 ros2 launch 之前激活项目 venv (.venv),
否则 colcon 写出的 entry_point shebang 会指向系统 python,
其下没有 cosysairsim, follow 节点会启动失败.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    object_arg = DeclareLaunchArgument(
        "object", default_value="BP_HuskyVisual_C_1",
        description="UE actor name for the pseudo UGV")
    ugv_distance_arg = DeclareLaunchArgument(
        "ugv_distance", default_value="0.0")
    uav_height_arg = DeclareLaunchArgument(
        "uav_height", default_value="8.0")
    # 坐标设定
    target_latitude_arg = DeclareLaunchArgument(
        "target_latitude", default_value="45.719851223043335",
        description="Target latitude in decimal degrees (WGS84)")
    target_longitude_arg = DeclareLaunchArgument(
        "target_longitude", default_value="-123.93299569829097",
        description="Target longitude in decimal degrees (WGS84)")
    object_name = LaunchConfiguration("object")

    repo_root = os.path.expanduser("~/Air-UE-project")
    drone_script = os.path.join(repo_root, "src", "ros2", "start_drone_wrapper.sh")

    drone_wrapper = ExecuteProcess(
        cmd=["bash", drone_script],
        output="screen",
        additional_env={"DISABLE_POINTCLOUD_TO_LASERSCAN": "1"},
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

    agv_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("agv_actor_sim"), "launch", "agv_actor_sim.launch.py")
        ),
        launch_arguments={
            "object": object_name,
            "topic_prefix": "/sim_ugv/airsim_node/UGV_1",
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("navigation_bringup"), "launch", "nav2_dwb.launch.py")
        ),
    )

    coordination_node = Node(
        package="coordination",
        executable="ugv_then_uav_node",
        name="ugv_then_uav_node",
        output="screen",
        parameters=[{
            "ugv_prefix": "/sim_ugv/airsim_node/UGV_1",
            "uav_prefix": "/uav/airsim_node/UAV_1",
            "uav_name": "UAV_1",
            "ugv_distance": LaunchConfiguration("ugv_distance"),
            "uav_height": LaunchConfiguration("uav_height"),
            "ugv_start_delay": 2.0,
            # 由 launch 参数传入; 默认值与节点内默认坐标一致, 可在
            # `ros2 launch` 时用 target_latitude:=... target_longitude:=...
            # 进行覆盖. ParameterValue 强制按 double 解析, 防止节点收到
            # 字符串导致类型不匹配.
            "target_latitude": ParameterValue(
                LaunchConfiguration("target_latitude"), value_type=float),
            "target_longitude": ParameterValue(
                LaunchConfiguration("target_longitude"), value_type=float),
            # Cruise speed: 3.0 m/s. Lower than APF max_speed=3.5 so APF
            # always has the authority to override cruise when stuck.
            # Lower than DWB max=2.5 not by much: DWB provides finer
            # avoidance velocity control inside AVOID, cruise only runs
            # in GO_TO_TARGET.
            "cruise_speed": 3.0,
            # AVOID hysteresis: enter at 7m, exit at 9m. Hold time reduced
            # to 0.4s so the UAV does not loiter in AVOID after it has
            # already opened a 9m+ gap to the obstacle; the previous 0.8s
            # window kept it stuck in AVOID-but-DWB-silent for too long.
            "obstacle_distance": 7.0,
            "obstacle_clear_distance": 9.0,
            "obstacle_clear_hold": 0.4,
            # Reach 10 m cruise altitude faster.
            "uav_ascend_speed": 3.0,
        }],
    )

    # Binding is temporarily disabled to keep the flight/navigation chain isolated.
    # Keep the node definitions here for later re-enable instead of deleting them.
    # recovery_state_node = Node(
    #     package="coordination",
    #     executable="uav_recovery_state_node",
    #     name="uav_recovery_state_node",
    #     output="screen",
    #     parameters=[{
    #         "uav_prefix": "/uav/airsim_node/UAV_1",
    #         "uav_name": "UAV_1",
    #         "agv_object": object_name,
    #         "host_ip": "127.0.0.1",
    #         "host_port": 41451,
    #     }],
    # )

    # AirSim pointcloud frame name is UAV_1/UAV_1_Lidar1. Publish its static extrinsic
    # so RViz, SLAM Toolbox and Nav2 can transform scan data into the UAV base frame.
    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="uav_lidar_static_tf",
        output="screen",
        arguments=[
            "0", "0", "-0.15",  # x y z
            "0", "0", "0",      # yaw pitch roll
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
            # Vertical slice ±8m around the UAV body. At cruise altitude 8m
            # AGL this captures most of the obstacle envelope: tree canopies
            # whose tops dip into the slice, low building roofs, and the
            # body of mid-rise buildings. The wider window gives DWB a much
            # denser 2D scan than ±4.5m used to produce.
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

    # follow_agv_node = Node(
    #     package="coordination",
    #     executable="uav_follow_agv_node",
    #     name="uav_follow_agv_node",
    #     output="screen",
    #     parameters=[{
    #         "uav_prefix": "/uav/airsim_node/UAV_1",
    #         "uav_name": "UAV_1",
    #         "agv_object": object_name,
    #         "rate": 60.0,
    #     }],
    # )

    return LaunchDescription([
        object_arg,
        ugv_distance_arg,
        uav_height_arg,
        target_latitude_arg,
        target_longitude_arg,
        drone_wrapper,
        uav_state_machine_launch,
        agv_launch,
        nav2_launch,
        coordination_node,
        lidar_static_tf,
        lidar_self_filter_node,
        pointcloud_to_scan_node,
        # recovery_state_node,
        # follow_agv_node,
    ])
