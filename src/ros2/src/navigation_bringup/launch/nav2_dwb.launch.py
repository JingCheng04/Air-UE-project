from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('navigation_bringup'),
            'config',
            'nav2_dwb_params.yaml',
        ),
        description='Nav2 DWB parameter file',
    )
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('navigation_bringup'),
            'config',
            'slam_toolbox.yaml',
        ),
        description='SLAM Toolbox parameter file',
    )

    # SLAM Toolbox consumes /uav/scan and publishes map/odom transforms.
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('slam_params_file')],
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[('/cmd_vel', '/uav/dwb_cmd_vel')],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['controller_server'],
        }],
    )

    # AirSim wrapper publishes odom_local with BEST_EFFORT QoS, which Nav2's
    # internal OdomSmoother cannot consume. Relay to a RELIABLE topic for DWB.
    odom_relay = Node(
        package='navigation_bringup',
        executable='odom_qos_relay_node',
        name='odom_qos_relay_node',
        output='screen',
    )

    path_feeder = Node(
        package='navigation_bringup',
        executable='dwb_goal_path_node',
        name='dwb_goal_path_node',
        output='screen',
    )

    return LaunchDescription([params_arg, slam_params_arg, slam, controller, lifecycle_manager, odom_relay, path_feeder])
