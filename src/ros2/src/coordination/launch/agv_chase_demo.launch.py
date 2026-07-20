"""无人机追踪 AGV demo launch.

并联拉起:
  - AirSim UAV wrapper (仅保留传感器/仿真数据, 飞控走 PX4/MAVROS)
  - AGV 仿真栈 (agv_actor_sim + imu_odom + figure-eight cruise),
    通过 IncludeLaunchDescription 直接复用 agv_figure_eight.launch.py
  - Nav2 (DWB) + LiDAR/scan 子图, 与原 coordination_demo 一致
  - uav_chase_agv_node: 等 AGV 跑 5s -> UAV 起飞 -> 巡航追踪 AGV GPS
                       距离 15m 处对地悬停
  - uav_state_machine_node: 状态机, 转发候选指令
  - yoloe_detector_node: 给图像话题持续供应可视化帧
  - uav_follow_agv_node: 起飞前把 UAV 钉在 AGV 上, 让两者一起预热;
    一旦 chase 节点进入 UAV_TAKEOFF, 它会立刻发 release, follow 在
    同一 tick 解绑 (uav_follow_agv_node._on_cmd 对 release 即时响应),
    随后 chase 还会强制 release_grace_seconds 秒空窗才让父类调
    takeoff RPC, 防止 simSetKinematics 与 PX4 Offboard 同时抢控制.

不修改 agv_actor_sim 任何代码, 只订阅它的 /global_gps 话题.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    object_arg = DeclareLaunchArgument(
        "object", default_value="BP_HuskyVisual_C_1",
        description="UE actor name for the AGV",
    )
    uav_height_arg = DeclareLaunchArgument(
        "uav_height", default_value="8.0",
        description="UAV cruise altitude (m above takeoff hover_z)",
    )
    keep_distance_arg = DeclareLaunchArgument(
        "keep_distance", default_value="10.0",
        description="Horizontal distance to AGV at which UAV stops and hovers (m)",
    )
    agv_warmup_arg = DeclareLaunchArgument(
        "agv_warmup_seconds", default_value="5.0",
        description="Wait this long after launch before UAV starts chasing",
    )
    target_prompt_arg = DeclareLaunchArgument(
        "target_prompt",
        # 与 yoloe_test_demo.launch.py 保持一致.
        default_value="yellow and black robot,ground robot,small four-wheeled robot",
        description=(
            "YOLOE text prompts (comma-separated). Multiple prompts give YOLOE "
            "a richer text embedding set; the union of detections is returned. "
            "Default targets a black/yellow Husky-class AGV with black wheels."
        ),
    )
    yoloe_model_arg = DeclareLaunchArgument(
        # 与 yoloe_test_demo 一致, 默认用更大的 26x 模型.
        "yoloe_model", default_value="yoloe-26x-seg.pt",
    )
    settings_path_arg = DeclareLaunchArgument(
        "settings_path", default_value="~/Documents/AirSim/settings.json",
    )

    object_name = LaunchConfiguration("object")
    repo_root = os.path.expanduser("~/Air-UE-project")
    drone_script = os.path.join(repo_root, "src", "ros2", "start_drone_wrapper.sh")

    drone_wrapper = ExecuteProcess(
        cmd=["bash", drone_script],
        output="screen",
        # 与原 coordination_demo 一致: launch 自己接管 pointcloud_to_laserscan,
        # wrapper 内不再起一份, 否则会和下面的 pointcloud_to_scan_node 冲突.
        additional_env={"DISABLE_POINTCLOUD_TO_LASERSCAN": "1"},
    )

    # 直接 include AGV 圆形巡航 launch (含 actor + imu_odom + circle control).
    agv_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("agv_actor_sim"),
                         "launch", "agv_circle.launch.py")
        ),
        launch_arguments={
            "object": object_name,
            "topic_prefix": "/sim_ugv/airsim_node/UGV_1",
        }.items(),
    )

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

    # 主追踪节点. 复用 ugv_then_uav_node 的全部巡航/避障算法和参数.
    chase_node = Node(
        package="coordination",
        executable="uav_chase_agv_node",
        name="uav_chase_agv_node",
        output="screen",
        parameters=[{
            "ugv_prefix": "/sim_ugv/airsim_node/UGV_1",
            "uav_prefix": "/uav/airsim_node/UAV_1",
            "uav_gps_topic": "/mavros/global_position/global",
            "uav_name": "UAV_1",
            # ugv_distance=0 -> 父类自动跳过 UGV_FORWARD 阶段.
            "ugv_distance": 0.0,
            # 父类的 ugv_start_delay = 入 UAV_TAKEOFF 前的等待时长. 我们直接
            # 把它当作"AGV 起步预热 + UAV 跟车 5s"的窗口, 期间 follow 节点把
            # UAV 钉在 AGV 上同步移动. 进入 UAV_TAKEOFF 时父类会发 release.
            "ugv_start_delay": LaunchConfiguration("agv_warmup_seconds"),
            "uav_height": LaunchConfiguration("uav_height"),
            # 与原 coordination_demo 完全一致的巡航/避障参数.
            "cruise_speed": 3.5,
            "obstacle_distance": 7.0,
            "obstacle_clear_distance": 9.0,
            "obstacle_clear_hold": 0.4,
            "uav_ascend_speed": 3.0,
            # 追踪专属参数.
            "agv_prefix": "/sim_ugv/airsim_node/UGV_1",
            "keep_distance": LaunchConfiguration("keep_distance"),
            "hold_band": 1.5,
            # 启动期把 UAV 钉到 AGV 上, 让两者同步预热移动. 进入 UAV_TAKEOFF
            # 时本节点 (chase) 会发 release, follow 节点会立刻解绑.
            "bind_on_startup": True,
            "bind_resend_rate": 5.0,
            # release_grace_seconds 使用节点默认 0.3s, 不再覆盖.
            # 父类的 target_lat/lon 在追踪期间会被本节点动态覆盖, 这里只是
            # 给个合理初值避免 _publish_target_cmd 在第一帧除零.
            "target_latitude": 45.72060377096292,
            "target_longitude": -123.93305245338378,
        }],
    )

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
            # 与 yoloe_test_demo.launch.py 保持一致.
            "conf": 0.15,
            "iou": 0.7,
            "settings_path": LaunchConfiguration("settings_path"),
            "vehicle_name": "UAV_1",
            "yoloe_script_path": os.path.join(
                repo_root, "src", "ros2", "src", "coordination",
                "coordination", "yoloe-position.py",
            ),
        }],
    )

    # 与 coordination_demo 一致的 LiDAR/scan 子图, 让父类 _scan_cb 拿到数据,
    # 避障迟滞才有效; 即便整个飞行无障碍, 这条链路在也安全.
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

    # UAV<->AGV 绑定/解绑节点. 起飞前把 UAV 钉到 AGV 上预热, 进入起飞阶段
    # 后由 chase 节点发 release 立即解绑, 控制权交还 PX4 Offboard.
    follow_node = Node(
        package="coordination",
        executable="uav_follow_agv_node",
        name="uav_follow_agv_node",
        output="screen",
        parameters=[{
            "uav_prefix": "/uav/airsim_node/UAV_1",
            "uav_name": "UAV_1",
            "agv_object": object_name,
            # 60 Hz simSetKinematics, 跟 follow 节点默认一致.
            "rate": 60.0,
        }],
    )

    return LaunchDescription([
        object_arg,
        uav_height_arg,
        keep_distance_arg,
        agv_warmup_arg,
        target_prompt_arg,
        yoloe_model_arg,
        settings_path_arg,
        drone_wrapper,
        uav_state_machine_launch,
        agv_launch,
        nav2_launch,
        chase_node,
        yoloe_node,
        lidar_static_tf,
        lidar_self_filter_node,
        pointcloud_to_scan_node,
        follow_node,
    ])
