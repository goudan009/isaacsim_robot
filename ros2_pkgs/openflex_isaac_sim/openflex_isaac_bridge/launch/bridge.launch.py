#!/usr/bin/env python3
"""Launch Isaac Sim bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Isaac Sim bridge."""

    # Arguments
    isaac_host_arg = DeclareLaunchArgument(
        'isaac_host',
        default_value='127.0.0.1',
        description='Isaac Sim host address'
    )

    isaac_port_arg = DeclareLaunchArgument(
        'isaac_port',
        default_value='24102',
        description='Isaac Sim port'
    )

    enable_sensors_arg = DeclareLaunchArgument(
        'enable_sensors',
        default_value='true',
        description='Enable sensor data forwarding'
    )

    enable_control_arg = DeclareLaunchArgument(
        'enable_control',
        default_value='true',
        description='Enable control command forwarding'
    )

    # Config file
    config_file = PathJoinSubstitution([
        FindPackageShare('openflex_isaac_bridge'),
        'config',
        'bridge_params.yaml'
    ])

    # Bridge node
    bridge_node = Node(
        package='openflex_isaac_bridge',
        executable='sim_bridge_node.py',
        name='isaac_sim_bridge',
        output='screen',
        parameters=[
            config_file,
            {
                'isaac_sim_host': LaunchConfiguration('isaac_host'),
                'isaac_sim_port': LaunchConfiguration('isaac_port'),
                'enable_sensors': LaunchConfiguration('enable_sensors'),
                'enable_control': LaunchConfiguration('enable_control'),
            }
        ]
    )

    return LaunchDescription([
        isaac_host_arg,
        isaac_port_arg,
        enable_sensors_arg,
        enable_control_arg,
        bridge_node,
    ])
