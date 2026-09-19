#!/usr/bin/env python3
"""
Main launch file for OpenFleX in Isaac Sim.

This launches:
- Robot description (URDF)
- Sensors (cameras, LiDAR, IMU)
- Controllers (ros2_control)
- RViz is launched separately with rviz_only.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the main launch description."""

    # Arguments
    sensor_profile_arg = DeclareLaunchArgument(
        'sensor_profile',
        default_value='full',
        choices=['none', 'minimal', 'full'],
        description='Sensor configuration profile'
    )

    start_controllers_arg = DeclareLaunchArgument(
        'ros2_control',
        default_value='true',
        description='Start ros2_control controllers'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Robot description
    robot_description_file = PathJoinSubstitution([
        FindPackageShare('openflex_isaac_description'),
        'urdf',
        'openflex_robot.urdf.xacro'
    ])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_file,
        }]
    )

    # Sensors launch
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('openflex_isaac_sensors'),
                'launch',
                'sensors.launch.py'
            ])
        ]),
        launch_arguments={
            'sensor_profile': LaunchConfiguration('sensor_profile'),
        }.items()
    )

    # Controllers launch
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('openflex_isaac_controllers'),
                'launch',
                'controllers.launch.py'
            ])
        ])
    )

    # TF static transforms (if needed for sensors)
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        sensor_profile_arg,
        start_controllers_arg,
        use_sim_time_arg,
        robot_state_publisher,
        static_tf_publisher,
        sensors_launch,
        controllers_launch,
    ])
