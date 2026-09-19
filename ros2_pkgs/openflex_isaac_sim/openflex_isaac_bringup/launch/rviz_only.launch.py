#!/usr/bin/env python3
"""Launch only the OpenFleX RViz interface."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _rviz_display_error(qt_qpa_platform: str) -> str | None:
    if qt_qpa_platform.strip():
        return None
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        return None
    if os.environ.get("QT_QPA_PLATFORM"):
        return None
    return (
        "RViz needs a GUI display, but DISPLAY and WAYLAND_DISPLAY are unset. "
        "Run this launch from a desktop terminal or SSH with X forwarding, or pass "
        "qt_qpa_platform:=offscreen/vnc for non-interactive diagnostics."
    )


def _rviz_node_environment(qt_qpa_platform: str) -> dict[str, str]:
    value = qt_qpa_platform.strip()
    if not value:
        return {}
    return {"QT_QPA_PLATFORM": value}


def launch_setup(context, *args, **kwargs):
    bringup_share = Path(get_package_share_directory("openflex_isaac_bringup"))
    rviz_config = bringup_share / "rviz" / "robot_control_only.rviz"
    qt_qpa_platform = LaunchConfiguration("qt_qpa_platform").perform(context)

    if not rviz_config.is_file():
        raise RuntimeError(f"RViz config does not exist: {rviz_config}")
    display_error = _rviz_display_error(qt_qpa_platform)
    if display_error is not None:
        raise RuntimeError(display_error)

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="both",
            arguments=["-d", str(rviz_config)],
            additional_env=_rviz_node_environment(qt_qpa_platform),
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "pos_min": ParameterValue(LaunchConfiguration("pos_min"), value_type=float),
                    "pos_max": ParameterValue(LaunchConfiguration("pos_max"), value_type=float),
                    "simulation_mode": ParameterValue(
                        LaunchConfiguration("simulation_mode"), value_type=bool
                    ),
                    "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                    "position_command_topic": LaunchConfiguration("position_command_topic"),
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the simulation clock when RViz is attached to an Isaac Sim session.",
            ),
            DeclareLaunchArgument(
                "qt_qpa_platform",
                default_value="",
                description=(
                    "Optional Qt platform override for RViz, e.g. offscreen or vnc. "
                    "Leave empty for normal desktop xcb/Wayland detection."
                ),
            ),
            DeclareLaunchArgument(
                "pos_min",
                default_value="-0.65",
                description="Minimum simulated lift_joint position in metres.",
            ),
            DeclareLaunchArgument(
                "pos_max",
                default_value="0.3",
                description="Maximum simulated lift_joint position in metres.",
            ),
            DeclareLaunchArgument("simulation_mode", default_value="true"),
            DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
            DeclareLaunchArgument(
                "position_command_topic",
                default_value="/lift_position_controller/commands",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="49",
                description="ROS 2 domain of the running Isaac Sim session.",
            ),
            DeclareLaunchArgument(
                "ros_localhost_only",
                default_value="1",
                description=(
                    "ROS_LOCALHOST_ONLY value used by the Isaac Sim session. "
                    "The default local-only mode matches the simulation launch environment."
                ),
            ),
            SetEnvironmentVariable(
                name="ROS_DOMAIN_ID",
                value=LaunchConfiguration("ros_domain_id"),
            ),
            SetEnvironmentVariable(
                name="ROS_LOCALHOST_ONLY",
                value=LaunchConfiguration("ros_localhost_only"),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
