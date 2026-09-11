#!/usr/bin/env python3
"""Launch only the OpenFleX RViz interface."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_rviz_config(value: str, bringup_share: Path) -> Path:
    normalized = value.strip().lower()
    if normalized in ("", "default", "integrated", "real"):
        integrated_share = Path(get_package_share_directory("openarmx_integrated_description"))
        return integrated_share / "rviz" / "integrated_robot.rviz"

    config_path = Path(value).expanduser()
    if not config_path.is_absolute():
        config_path = Path.cwd() / config_path
    return config_path


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
    bringup_share = Path(get_package_share_directory("isaacsim_bringup"))
    rviz_config = _resolve_rviz_config(
        LaunchConfiguration("rviz_config").perform(context),
        bringup_share,
    )
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
                    "position_command_topic": LaunchConfiguration("position_command_topic"),
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value="integrated",
                description=(
                    "RViz config to load: 'integrated'/'real' for the real robot UI, "
                    "or a file path whose panel dependencies are built in this workspace."
                ),
            ),
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
            DeclareLaunchArgument("pos_min", default_value="-0.75"),
            DeclareLaunchArgument("pos_max", default_value="0.4"),
            DeclareLaunchArgument(
                "position_command_topic",
                default_value="/lift_position_controller/commands",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
