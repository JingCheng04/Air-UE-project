"""Launch the pseudo AGV stack with a keyboard teleop node.

It reuses agv_actor_sim.launch.py for the AGV actor + odom/imu nodes, and adds a
keyboard teleop publisher on <topic_prefix>/car_cmd. Speed ramps avoid sudden
changes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    object_arg = DeclareLaunchArgument('object', default_value='BP_HuskyVisual_C_1')
    prefix_arg = DeclareLaunchArgument('topic_prefix', default_value='/sim_ugv/airsim_node/UGV_1')

    object_name = LaunchConfiguration('object')
    prefix = LaunchConfiguration('topic_prefix')

    agv_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('agv_actor_sim'), 'launch', 'agv_actor_sim.launch.py')
        ),
        launch_arguments={
            'object': object_name,
            'topic_prefix': prefix,
            # Match teleop's intended physical scales.
            'max_speed': '4.0',
            'max_yaw_rate': '28.6478897565',
        }.items(),
    )

    teleop = Node(
        package='agv_actor_sim',
        executable='agv_keyboard_teleop_node',
        name='agv_keyboard_teleop_node',
        output='screen',
        parameters=[{
            'topic_prefix': prefix,
            'linear_speed': 4.0,
            'angular_speed': 0.5,
            'linear_accel': 2.0,
            'angular_accel': 1.0,
            'rate': 20.0,
        }],
    )

    return LaunchDescription([object_arg, prefix_arg, agv_stack, teleop])
