"""YOLOE 识别测试 launch.

最小启动: AirSim UAV wrapper + YOLOE 检测节点 + 测试节点.
不拉 Nav2 / 状态机 / agv_actor_sim, 因为本测试不需要避障 / 跟车.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    takeoff_height_arg = DeclareLaunchArgument(
        "takeoff_height", default_value="10.0",
        description="Cruise height above takeoff point (m)",
    )
    backup_distance_arg = DeclareLaunchArgument(
        "backup_distance", default_value="10.0",
        description="Distance to back up along body -x (m)",
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
    )
    settings_path_arg = DeclareLaunchArgument(
        "settings_path", default_value="~/Documents/AirSim/settings.json",
    )

    repo_root = os.path.expanduser("~/Air-UE-project")
    drone_script = os.path.join(repo_root, "src", "ros2", "start_drone_wrapper.sh")

    # AirSim wrapper. 测试不依赖 LaserScan, 关掉 pointcloud_to_laserscan 子进程.
    drone_wrapper = ExecuteProcess(
        cmd=["bash", drone_script],
        output="screen",
        additional_env={"DISABLE_POINTCLOUD_TO_LASERSCAN": "1"},
    )

    test_node = Node(
        package="coordination",
        executable="uav_yoloe_test_node",
        name="uav_yoloe_test_node",
        output="screen",
        parameters=[{
            "uav_prefix": "/uav/airsim_node/UAV_1",
            "uav_name": "UAV_1",
            "takeoff_height": LaunchConfiguration("takeoff_height"),
            "backup_distance": LaunchConfiguration("backup_distance"),
            "ascend_speed": 2.0,
            "backup_speed": 1.0,
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

    return LaunchDescription([
        takeoff_height_arg,
        backup_distance_arg,
        target_prompt_arg,
        yoloe_model_arg,
        settings_path_arg,
        drone_wrapper,
        test_node,
        yoloe_node,
    ])
