#!/usr/bin/env python3
"""Run the Pico/Quest VR control path against the Isaac Sim controllers."""

import subprocess
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def _start_vr_stack(context, nodes):
    if IfCondition(LaunchConfiguration("stop_existing_vr")).evaluate(context):
        cleanup_script = PathJoinSubstitution(
            [
                FindPackagePrefix("isaacsim_bringup"),
                "lib",
                "isaacsim_bringup",
                "stop_stale_vr_processes.py",
            ]
        ).perform(context)
        subprocess.run([sys.executable, cleanup_script], check=True)
    return nodes


def generate_launch_description() -> LaunchDescription:
    isaac_urdf = PathJoinSubstitution(
        [
            FindPackageShare("isaacsim_bringup"),
            "assets",
            "generated",
            "openflex_isaac_robot.urdf",
        ]
    )
    arm_config = PathJoinSubstitution(
        [FindPackageShare("openarmx_teleop_vr"), "config", "teleop_params.yaml"]
    )

    arguments = [
        DeclareLaunchArgument("listen_address", default_value="0.0.0.0"),
        DeclareLaunchArgument("listen_port", default_value="5100"),
        DeclareLaunchArgument("stop_existing_vr", default_value="true"),
        DeclareLaunchArgument("enable_arms", default_value="true"),
        DeclareLaunchArgument("enable_head", default_value="true"),
        DeclareLaunchArgument("enable_chassis", default_value="true"),
        DeclareLaunchArgument("enable_lift", default_value="true"),
        DeclareLaunchArgument("max_linear_speed", default_value="0.35"),
        DeclareLaunchArgument("boost_linear_speed", default_value="0.50"),
        DeclareLaunchArgument("max_angular_speed", default_value="0.60"),
        DeclareLaunchArgument("acceleration_time", default_value="2.0"),
    ]

    bridge = Node(
        package="openflex_vr_bridge",
        executable="pico_pose_bridge_node",
        name="pico_pose_bridge",
        output="screen",
        parameters=[
            {
                "listen_address": LaunchConfiguration("listen_address"),
                "listen_port": ParameterValue(LaunchConfiguration("listen_port"), value_type=int),
                "use_sim_time": False,
            }
        ],
    )
    arms = Node(
        package="isaacsim_bringup",
        executable="isaacsim_vr_arm_node.py",
        name="openarmx_teleop_vr_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_arms")),
        parameters=[
            arm_config,
            {
                "urdf_path": isaac_urdf,
                "use_sim_time": False,
                # Robot base frame: +X forward, +Y left, +Z up.
                # Desired VR mapping: +X right, +Y up, +Z toward robot.
                "left_axis_matrix": [0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                "right_axis_matrix": [0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            },
        ],
    )
    head = Node(
        package="openarmx_head_teleop_vr_pico",
        executable="head_teleop_node",
        name="openarmx_head_teleop_vr_pico_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_head")),
        parameters=[{"use_sim_time": False, "publish_visualization_tf": False}],
    )
    chassis = Node(
        package="swerve_bringup",
        executable="vr_teleop_node",
        name="vr_teleop_chassis",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_chassis")),
        parameters=[
            {
                "cmd_vel_topic": "/cmd_vel",
                "max_linear_speed": ParameterValue(
                    LaunchConfiguration("max_linear_speed"), value_type=float
                ),
                "boost_linear_speed": ParameterValue(
                    LaunchConfiguration("boost_linear_speed"), value_type=float
                ),
                "max_angular_speed": ParameterValue(
                    LaunchConfiguration("max_angular_speed"), value_type=float
                ),
                "enable_scurve": True,
                "acceleration_time": ParameterValue(
                    LaunchConfiguration("acceleration_time"), value_type=float
                ),
                "smoothness": 0.5,
                "estop_toggle_topic": "",
                "use_vr_chassis_speed_config": False,
                "use_sim_time": False,
            }
        ],
    )
    lift = Node(
        package="swerve_bringup",
        executable="vr_lift_control_node",
        name="vr_lift_control",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_lift")),
        parameters=[
            {
                "jog_command_topic": "/lift_manual_position_controller/jog_command",
                "lift_speed": 0.05,
                "use_sim_time": False,
            }
        ],
    )

    start_vr_stack = OpaqueFunction(
        function=_start_vr_stack,
        args=[[bridge, arms, head, chassis, lift]],
    )
    return LaunchDescription([*arguments, start_vr_stack])
