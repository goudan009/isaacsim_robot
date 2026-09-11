#!/usr/bin/env python3
"""Start the robot and ROS 2 control path without any sensor component."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _camera_profile(context) -> str:
    profile = LaunchConfiguration("camera_profile").perform(context).strip().lower()
    if profile not in ("none", "quad"):
        raise RuntimeError("camera_profile must be 'none' or 'quad'")
    return profile


def _lidar_profile(context) -> str:
    profile = LaunchConfiguration("lidar_profile").perform(context).strip().lower()
    if profile not in ("none", "performance", "full"):
        raise RuntimeError("lidar_profile must be 'none', 'performance', or 'full'")
    return profile


def _default_sensor_asset_dir() -> Path:
    configured = os.environ.get("ISAACSIM_SENSOR_ASSET_DIR", "").strip()
    if configured:
        return Path(configured).expanduser()
    # The consolidated repository keeps the MID360 contract in
    # isaac_sim_core/config/sensor_params.  Passing the workspace root through
    # the Isaac child environment makes the same launch work with either the
    # consolidated tree or the historical standalone sensor repository.
    return Path(__file__).resolve().parents[4]


def _default_isaac_path() -> Path:
    for candidate in (
        Path.home() / "isaacsim-6.0",
        Path.home() / "isaacsim",
        Path("/opt/isaac-sim"),
        Path("/opt/isaac-sim-5.1"),
        Path("/isaac-sim"),
    ):
        if (candidate / "python.sh").is_file():
            return candidate
    return Path("/isaac-sim")


def _default_generated_urdf() -> Path:
    configured = os.environ.get("ISAACSIM_ROBOT_GENERATED_DIR", "").strip()
    if configured:
        return Path(configured).expanduser() / "robot_control_only.urdf"
    # Keep generated launch artifacts inside the workspace by default.  This
    # makes the launch usable in containers and managed runners where the
    # user's home directory may be read-only, while still allowing callers to
    # override the location with ISAACSIM_ROBOT_GENERATED_DIR.
    workspace_root = Path(__file__).resolve().parents[4]
    return workspace_root / "reports" / "runtime" / "generated" / "robot_control_only.urdf"


def _resolve_source_xacro(value: str) -> Path:
    normalized = value.strip().lower()
    if normalized in ("", "auto", "default"):
        share = Path(get_package_share_directory("openarmx_integrated_description"))
        return share / "urdf" / "openarmx_integrated_robot.urdf.xacro"
    path = Path(value).expanduser()
    return path if path.is_absolute() else Path.cwd() / path


def _resolve_path(value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else Path.cwd() / path


def _generate_sensor_free_urdf(context, output: Path) -> str:
    source = _resolve_source_xacro(LaunchConfiguration("source_xacro").perform(context))
    generator = (
        Path(get_package_prefix("isaacsim_description"))
        / "lib"
        / "isaacsim_description"
        / "generate_isaac_urdf.py"
    )
    if not source.is_file():
        raise RuntimeError(f"source_xacro does not exist: {source}")
    if not generator.is_file():
        raise RuntimeError(f"URDF generator is not installed: {generator}")

    output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        str(generator),
        "--source",
        str(source),
        "--output",
        str(output),
        "--robot-name",
        LaunchConfiguration("robot_name").perform(context),
        "--command-topic",
        LaunchConfiguration("command_topic").perform(context),
        "--state-topic",
        LaunchConfiguration("state_topic").perform(context),
        "--enable-head",
        LaunchConfiguration("enable_head").perform(context),
        "--sensor-profile",
        "none",
        "--disable-sensors",
        "--preserve-visual-materials",
    ]
    result = subprocess.run(command, check=False, text=True, capture_output=True)
    if result.returncode != 0:
        raise RuntimeError(
            "sensor-free URDF generation failed\n"
            f"command: {' '.join(command)}\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )

    urdf = output.read_text(encoding="utf-8")
    if "<sensor" in urdf or "<isaac:sensor" in urdf:
        raise RuntimeError(f"generated URDF contains sensor definitions: {output}")
    return urdf


def _controller_chain(context, controller_file: Path):
    controllers = ["joint_state_broadcaster", "swerve_drive_controller"]
    if _as_bool(LaunchConfiguration("start_upper_body").perform(context)):
        controllers.extend(
            (
                "left_forward_position_controller",
                "right_forward_position_controller",
                "head_forward_position_controller",
                "lift_position_controller",
            )
        )
    upper_body_param_files = {
        "left_forward_position_controller": controller_file.parent / "left_arm_controller.yaml",
        "right_forward_position_controller": controller_file.parent / "right_arm_controller.yaml",
        "head_forward_position_controller": controller_file.parent / "head_controller.yaml",
        "lift_position_controller": controller_file.parent / "lift_controller.yaml",
    }
    nodes = []
    for controller in controllers:
        parameter_file = upper_body_param_files.get(controller, controller_file)
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller,
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "60",
                    "--service-call-timeout",
                    "60",
                    "--param-file",
                    str(parameter_file),
                ],
                output="both",
            )
        )

    actions = []
    for previous, current in zip(nodes, nodes[1:]):
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=previous,
                    on_exit=lambda _event, _context, next_controller=current: [next_controller],
                )
            )
        )
    actions.append(TimerAction(period=1.0, actions=[nodes[0]]))
    return actions


def launch_setup(context, *args, **kwargs):
    bringup_share = Path(get_package_share_directory("isaacsim_bringup"))
    camera_profile = _camera_profile(context)
    lidar_profile = _lidar_profile(context)
    if camera_profile == "quad" and lidar_profile != "none":
        sensor_profile = "data"
    elif camera_profile == "quad":
        sensor_profile = "rgb_depth"
    elif lidar_profile != "none":
        sensor_profile = "lidar"
    else:
        sensor_profile = "none"
    output_urdf = _resolve_path(LaunchConfiguration("generated_urdf").perform(context))
    robot_description = _generate_sensor_free_urdf(context, output_urdf)
    controller_file = bringup_share / "config" / "controllers.isaac.mobile_base.yaml"
    stage = LaunchConfiguration("stage").perform(context).strip()
    if not stage or stage.lower() in ("auto", "default"):
        # The robot is imported by the control-only spawn step. Do not open a
        # stage that already references the robot, otherwise Isaac creates a
        # second /openflex ActionGraph and invalidates the physics view.
        stage_path = bringup_share / "config" / "empty_stage.usd"
    else:
        stage_path = _resolve_path(stage)
    if not stage_path.is_file():
        raise RuntimeError(f"robot control stage does not exist: {stage_path}")
    if not controller_file.is_file():
        raise RuntimeError(f"controller configuration does not exist: {controller_file}")

    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    controller_use_sim_time = ParameterValue(
        LaunchConfiguration("controller_use_sim_time"), value_type=bool
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="both",
        parameters=[
            {
                "robot_description": robot_description,
                # Keep controller-manager timing independently selectable. A
                # wall-clock controller loop can still publish at its 90 Hz
                # target when the Isaac scene runs below RTF 1; the rest of
                # the visualization/state-publisher stack may continue using
                # simulation time.
                "use_sim_time": controller_use_sim_time,
            },
            ParameterFile(str(controller_file)),
        ],
        remappings=[("~/robot_description", "/robot_description")],
        on_exit=Shutdown(),
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )

    scripts_share = Path(get_package_share_directory("isaac_ros2_scripts"))
    isaac_python = _resolve_path(LaunchConfiguration("isaac_path").perform(context)) / "python.sh"
    if not isaac_python.is_file():
        raise RuntimeError(f"Isaac Sim python.sh does not exist: {isaac_python}")
    start_script = (
        Path(get_package_prefix("isaacsim_bringup"))
        / "lib"
        / "isaacsim_bringup"
        / "start_robot_control_sim.py"
    )
    if not start_script.is_file():
        raise RuntimeError(f"control-only Isaac launcher is not installed: {start_script}")
    api_port = LaunchConfiguration("api_port").perform(context)
    physics_hz = LaunchConfiguration("physics_hz").perform(context)
    render_hz = LaunchConfiguration("render_hz").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    sim_process = ExecuteProcess(
        cmd=[
            str(isaac_python),
            str(start_script),
            str(scripts_share),
            str(stage_path),
            render_hz,
            physics_hz,
            physics_hz,
            headless,
            api_port,
        ],
        output="both",
        name="isaac_robot_control_sim",
        additional_env={
            "OPENFLEX_ISAAC_SENSOR_ASSET_DIR": str(
                _resolve_path(LaunchConfiguration("sensor_asset_dir").perform(context))
            ),
            "OPENFLEX_ISAAC_MID360_ASSET_DIR": str(
                _resolve_path(LaunchConfiguration("sensor_asset_dir").perform(context))
            ),
            "OPENFLEX_ROBOT_SENSOR_PROFILE": sensor_profile,
            "OPENFLEX_MID360_LIDAR_PROFILE": (
                "MID360_PERFORMANCE" if lidar_profile == "performance" else "MID360_APPROX"
            ),
            "OPENFLEX_MID360_LIDAR_TRANSPORT": LaunchConfiguration(
                "lidar_transport"
            ).perform(context),
            "OPENFLEX_MID360_MOUNT_MODE": LaunchConfiguration(
                "lidar_mount_mode"
            ).perform(context),
        },
    )

    spawn_node = Node(
        package="isaacsim_bringup",
        executable="spawn_robot_when_ready.py",
        name="spawn_robot_control_only",
        output="both",
        parameters=[
            {
                "urdf_path": str(output_urdf),
                "x": float(LaunchConfiguration("x").perform(context)),
                "y": float(LaunchConfiguration("y").perform(context)),
                "z": float(LaunchConfiguration("z").perform(context)),
                "roll": float(LaunchConfiguration("roll").perform(context)),
                "pitch": float(LaunchConfiguration("pitch").perform(context)),
                "yaw": float(LaunchConfiguration("yaw").perform(context)),
                "fixed": _as_bool(LaunchConfiguration("fixed").perform(context)),
                # The default remains strictly sensor-free. ``quad`` is an
                # explicit performance probe using the repository's four
                # released RealSense mounts; it is not the old all-body stack.
                "enable_sensors": sensor_profile != "none",
                "apply_appearance": False,
                "api_host": LaunchConfiguration("api_host"),
                "api_port": int(api_port),
                "wait_timeout": float(LaunchConfiguration("spawn_wait_timeout").perform(context)),
                "poll_period": 2.0,
                "auto_play": True,
                "play_timeout": 60.0,
                # MID360's first RTX frame must complete before controller
                # spawners are released; zero-cost for the sensor-free path.
                "post_play_settle_sec": 8.0 if sensor_profile != "none" else 0.0,
            }
        ],
    )
    sim_ready = Node(
        package="isaacsim_bringup",
        executable="wait_for_sim_ready.py",
        name="wait_for_robot_control_ready",
        output="both",
        parameters=[
            {
                "clock_topic": "/clock",
                "joint_states_topic": LaunchConfiguration("state_topic"),
                "timeout_sec": float(LaunchConfiguration("sim_ready_timeout").perform(context)),
                "log_period_sec": 5.0,
                "require_nonzero_clock": True,
                "require_nonzero_joint_stamp": True,
            }
        ],
    )

    def start_control(event, _context):
        if event.returncode != 0:
            return [Shutdown()]
        return [controller_manager, *_controller_chain(context, controller_file)]

    def start_ready_gate(event, _context):
        if event.returncode != 0:
            return [Shutdown()]
        return [sim_ready]

    qt_platform = LaunchConfiguration("qt_qpa_platform").perform(context).strip()
    rviz_env = {
        # The workspace is commonly launched from a Snap-hosted IDE. Keep its
        # private glibc/GTK paths out of the system RViz process.
        "LD_LIBRARY_PATH": "/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu",
        "LOCPATH": "",
        "GTK_PATH": "",
        "GTK_EXE_PREFIX": "",
        "GIO_MODULE_DIR": "",
        "GSETTINGS_SCHEMA_DIR": "",
    }
    if qt_platform:
        rviz_env["QT_QPA_PLATFORM"] = qt_platform
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_robot_control_only",
        output="both",
        arguments=["-d", str(_resolve_path(LaunchConfiguration("rviz_config").perform(context)))],
        additional_env=rviz_env,
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return [
        state_publisher,
        sim_process,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_node, on_exit=start_ready_gate)
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=sim_ready, on_exit=start_control)
        ),
        TimerAction(period=2.0, actions=[spawn_node]),
        rviz,
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("isaacsim_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("source_xacro", default_value="auto"),
            DeclareLaunchArgument("generated_urdf", default_value=str(_default_generated_urdf())),
            DeclareLaunchArgument("robot_name", default_value="openflex"),
            DeclareLaunchArgument("command_topic", default_value="/openflex/joint_command"),
            DeclareLaunchArgument("state_topic", default_value="/openflex/joint_states"),
            DeclareLaunchArgument("enable_head", default_value="true"),
            DeclareLaunchArgument(
                "camera_profile",
                default_value="none",
                description="none: sensor-free control; quad: four 640x480@30Hz RGB-D camera probe",
            ),
            DeclareLaunchArgument(
                "lidar_profile",
                default_value="none",
                description="none: no MID360; performance: historical ~30k point scan; full: full-density scan",
            ),
            DeclareLaunchArgument(
                "lidar_transport",
                default_value="helper",
                description="MID360 ROS transport; helper uses the historical LidarSensor render-product chain",
            ),
            DeclareLaunchArgument(
                "lidar_mount_mode",
                default_value="fixed_kinematic",
                description="MID360 pose mode: fixed_kinematic (safe default), fixed, or experimental follow",
            ),
            DeclareLaunchArgument(
                "sensor_asset_dir",
                default_value=str(_default_sensor_asset_dir()),
                description="consolidated sensor asset root or historical sensor repository root",
            ),
            DeclareLaunchArgument("stage", default_value="auto"),
            DeclareLaunchArgument("isaac_path", default_value=str(_default_isaac_path())),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            # This is the no-sensor *complete robot* entry point. Keep the
            # mobile-base-only variant available for performance isolation,
            # but require it to opt out explicitly.
            DeclareLaunchArgument("start_upper_body", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(bringup_share / "config" / "robot_control_only.rviz"),
            ),
            DeclareLaunchArgument("qt_qpa_platform", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # The real robot controller contract is a wall-clock 90 Hz loop.
            # Isaac /clock remains available to the rest of the simulated
            # graph, but tying controller-manager scheduling to it would make
            # the externally observed /joint_states rate fall with RTF.
            DeclareLaunchArgument("controller_use_sim_time", default_value="false"),
            DeclareLaunchArgument("api_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("api_port", default_value="8085"),
            DeclareLaunchArgument("physics_hz", default_value="120.0"),
            # No sensor render product is required for the control-only path.
            # 30 Hz is sufficient for diagnostic capture and keeps the
            # complete robot above the real-time threshold. Use 60 Hz only
            # when an interactive Isaac viewport specifically needs it.
            DeclareLaunchArgument("render_hz", default_value="30.0"),
            DeclareLaunchArgument("spawn_wait_timeout", default_value="900.0"),
            DeclareLaunchArgument("sim_ready_timeout", default_value="180.0"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.25"),
            DeclareLaunchArgument("roll", default_value="0.0"),
            DeclareLaunchArgument("pitch", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("fixed", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
