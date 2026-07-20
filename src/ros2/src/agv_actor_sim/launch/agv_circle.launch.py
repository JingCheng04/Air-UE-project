"""Launch the pseudo AGV stack with a circular trajectory node.

It reuses agv_actor_sim.launch.py for the AGV actor + odom/imu nodes, and adds a
circular trajectory publisher on <topic_prefix>/car_cmd. The trajectory is a
continuous circle (radius 5 m) at constant linear speed 2 m/s, with a fixed
turn direction.
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
    speed_arg = DeclareLaunchArgument('target_speed', default_value='2.0')
    radius_arg = DeclareLaunchArgument('radius', default_value='5.0')
    turn_arg = DeclareLaunchArgument('turn_direction', default_value='1.0')

    object_name = LaunchConfiguration('object')
    prefix = LaunchConfiguration('topic_prefix')
    target_speed = LaunchConfiguration('target_speed')
    radius = LaunchConfiguration('radius')
    turn_direction = LaunchConfiguration('turn_direction')

    agv_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('agv_actor_sim'), 'launch', 'agv_actor_sim.launch.py')
        ),
        launch_arguments={
            'object': object_name,
            'topic_prefix': prefix,
            # Match the circle node's expected actuator scales.
            'max_speed': '2.0',
            'max_yaw_rate': '90.0',
        }.items(),
    )

    circle = Node(
        package='agv_actor_sim',
        executable='agv_circle_node',
        name='agv_circle_node',
        output='screen',
        parameters=[{
            'topic_prefix': prefix,
            'target_speed': target_speed,
            'radius': radius,
            'turn_direction': turn_direction,
            'max_speed': 2.0,
            'max_yaw_rate': 90.0,
            'rate': 30.0,
        }],
    )

    return LaunchDescription([object_arg, prefix_arg, speed_arg, radius_arg, turn_arg, agv_stack, circle])
