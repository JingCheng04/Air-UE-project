"""Launch MAVROS and the UAV state machine together.

This package-level launch centralizes the PX4/MAVROS control bridge so other
demo launches can simply include it instead of duplicating MAVROS startup.

Control path:
  uav_state_machine_node -> /mavros/setpoint_raw/attitude -> MAVROS -> PX4
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        # Do not use PX4's onboard MAVLink channel (14540/14580) here because
        # AirSim PX4Multirotor already uses that pair for simulator-side MAVLink
        # traffic. Reusing it with MAVROS causes request/ack conflicts (for
        # example ARM command timeouts). Instead attach MAVROS to PX4's normal
        # MAVLink/GCS endpoint on port 14570 using a separate local UDP port.
        default_value="udp://:14550@127.0.0.1:14570",
        description="MAVROS FCU URL for PX4 SITL (separate from AirSim onboard channel)",
    )
    gcs_url_arg = DeclareLaunchArgument(
        "gcs_url",
        default_value="",
        description="Optional MAVROS GCS URL",
    )
    tgt_system_arg = DeclareLaunchArgument(
        "tgt_system",
        default_value="1",
        description="PX4 MAVLink system id",
    )
    tgt_component_arg = DeclareLaunchArgument(
        "tgt_component",
        default_value="1",
        description="PX4 MAVLink component id",
    )

    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mavros"), "launch", "px4.launch")
        ),
        launch_arguments={
            "fcu_url": LaunchConfiguration("fcu_url"),
            "gcs_url": LaunchConfiguration("gcs_url"),
            "tgt_system": LaunchConfiguration("tgt_system"),
            "tgt_component": LaunchConfiguration("tgt_component"),
        }.items(),
    )

    state_machine_node = Node(
        package="uav_state_machine",
        executable="uav_state_machine_node",
        name="uav_state_machine_node",
        output="screen",
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        mavros_launch,
        state_machine_node,
    ])
