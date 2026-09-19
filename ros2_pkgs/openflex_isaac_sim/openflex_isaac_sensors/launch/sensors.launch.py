#!/usr/bin/env python3
"""Launch all sensor publishers for Isaac Sim."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for all sensors."""

    # Arguments
    sensor_profile_arg = DeclareLaunchArgument(
        'sensor_profile',
        default_value='full',
        choices=['none', 'minimal', 'full'],
        description='Sensor configuration profile'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'camera_rate',
        default_value='30.0',
        description='Camera publish rate in Hz'
    )

    # Head camera
    head_camera_node = Node(
        package='openflex_isaac_sensors',
        executable='camera_publisher',
        name='head_camera_publisher',
        output='screen',
        parameters=[{
            'camera_name': 'head_camera',
            'frame_id': 'head_camera_optical_frame',
            'publish_rate': LaunchConfiguration('camera_rate'),
            'width': 640,
            'height': 480,
        }]
    )

    # Chest camera
    chest_camera_node = Node(
        package='openflex_isaac_sensors',
        executable='camera_publisher',
        name='chest_camera_publisher',
        output='screen',
        parameters=[{
            'camera_name': 'chest_camera',
            'frame_id': 'chest_camera_optical_frame',
            'publish_rate': LaunchConfiguration('camera_rate'),
            'width': 640,
            'height': 480,
        }]
    )

    # Left wrist camera
    left_wrist_camera_node = Node(
        package='openflex_isaac_sensors',
        executable='camera_publisher',
        name='left_wrist_camera_publisher',
        output='screen',
        parameters=[{
            'camera_name': 'left_wrist_camera',
            'frame_id': 'left_wrist_camera_optical_frame',
            'publish_rate': LaunchConfiguration('camera_rate'),
            'width': 320,
            'height': 240,
        }]
    )

    # Right wrist camera
    right_wrist_camera_node = Node(
        package='openflex_isaac_sensors',
        executable='camera_publisher',
        name='right_wrist_camera_publisher',
        output='screen',
        parameters=[{
            'camera_name': 'right_wrist_camera',
            'frame_id': 'right_wrist_camera_optical_frame',
            'publish_rate': LaunchConfiguration('camera_rate'),
            'width': 320,
            'height': 240,
        }]
    )

    # LiDAR
    lidar_node = Node(
        package='openflex_isaac_sensors',
        executable='lidar_publisher',
        name='lidar_publisher',
        output='screen',
        parameters=[{
            'lidar_name': 'mid360',
            'frame_id': 'mid360_lidar_link',
            'publish_rate': 10.0,
            'points_per_scan': 24000,
        }]
    )

    # IMU
    imu_node = Node(
        package='openflex_isaac_sensors',
        executable='imu_publisher',
        name='imu_publisher',
        output='screen',
        parameters=[{
            'frame_id': 'imu_link',
            'publish_rate': 100.0,
        }]
    )

    return LaunchDescription([
        sensor_profile_arg,
        publish_rate_arg,
        head_camera_node,
        chest_camera_node,
        left_wrist_camera_node,
        right_wrist_camera_node,
        lidar_node,
        imu_node,
    ])
