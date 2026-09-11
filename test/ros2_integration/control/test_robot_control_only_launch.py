#!/usr/bin/env python3
"""Static checks for the sensor-free robot control entry point."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[3]
LAUNCH_PATH = ROOT / "ros2_pkgs/control/bringup/launch/robot_control_only.launch.py"
START_PATH = ROOT / "ros2_pkgs/control/bringup/scripts/start_robot_control_sim.py"


class _FakeAction:
    def __init__(self, *args, **kwargs) -> None:
        self.args = args
        self.kwargs = kwargs


def _load_launch_module():
    launch = types.ModuleType("launch")
    launch.LaunchDescription = _FakeAction
    actions = types.ModuleType("launch.actions")
    for name in ("DeclareLaunchArgument", "ExecuteProcess", "OpaqueFunction", "RegisterEventHandler", "Shutdown", "TimerAction"):
        setattr(actions, name, _FakeAction)
    conditions = types.ModuleType("launch.conditions")
    conditions.IfCondition = _FakeAction
    event_handlers = types.ModuleType("launch.event_handlers")
    event_handlers.OnProcessExit = _FakeAction
    substitutions = types.ModuleType("launch.substitutions")
    substitutions.LaunchConfiguration = _FakeAction
    launch_ros = types.ModuleType("launch_ros")
    launch_ros_actions = types.ModuleType("launch_ros.actions")
    launch_ros_actions.Node = _FakeAction
    launch_ros_params = types.ModuleType("launch_ros.parameter_descriptions")
    launch_ros_params.ParameterFile = _FakeAction
    launch_ros_params.ParameterValue = _FakeAction
    ament = types.ModuleType("ament_index_python")
    ament_packages = types.ModuleType("ament_index_python.packages")
    ament_packages.get_package_prefix = lambda _name: "/tmp"
    ament_packages.get_package_share_directory = lambda _name: "/tmp"
    modules = {
        "launch": launch,
        "launch.actions": actions,
        "launch.conditions": conditions,
        "launch.event_handlers": event_handlers,
        "launch.substitutions": substitutions,
        "launch_ros": launch_ros,
        "launch_ros.actions": launch_ros_actions,
        "launch_ros.parameter_descriptions": launch_ros_params,
        "ament_index_python": ament,
        "ament_index_python.packages": ament_packages,
    }
    spec = importlib.util.spec_from_file_location("robot_control_only_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    with mock.patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class RobotControlOnlyLaunchTest(unittest.TestCase):
    def test_launch_keeps_sensor_free_default_and_has_explicit_quad_probe(self) -> None:
        text = LAUNCH_PATH.read_text(encoding="utf-8")
        self.assertIn('"--disable-sensors"', text)
        self.assertIn('"--sensor-profile",\n        "none"', text)
        self.assertIn('"enable_sensors": sensor_profile != "none"', text)
        self.assertIn('DeclareLaunchArgument(\n                "camera_profile"', text)
        self.assertIn('"empty_stage.usd"', text)
        self.assertNotIn('"robot_control_stage.usda"', text)
        self.assertNotIn("isaacsim_compat_bridge", text)
        self.assertNotIn("launch_sensor", text)

    def test_isaac_adapter_only_imports_camera_component_for_explicit_probe(self) -> None:
        text = START_PATH.read_text(encoding="utf-8")
        self.assertIn("reusing imported control graph", text)
        self.assertIn("_ensure_clock_graph", text)
        self.assertIn('"OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"', text)
        self.assertNotIn("launch_sensor", text)
        self.assertIn('sensor_profile=sensor_profile', text)
        self.assertIn("_installed_sensor_share", text)
        self.assertIn("create_robot_sensor_suite", text)

    def test_launch_description_declares_three_runtime_switches(self) -> None:
        module = _load_launch_module()
        description = module.generate_launch_description()
        actions = description.args[0]
        arguments = [item.args[0] for item in actions if item.args]
        self.assertIn("headless", arguments)
        self.assertIn("rviz", arguments)
        self.assertIn("start_upper_body", arguments)
        self.assertIn("camera_profile", arguments)
        self.assertIn("qt_qpa_platform", arguments)

    def test_sensor_free_default_is_complete_control_at_120_hz(self) -> None:
        launch_text = LAUNCH_PATH.read_text(encoding="utf-8")
        self.assertIn('DeclareLaunchArgument("start_upper_body", default_value="true")', launch_text)
        self.assertIn('DeclareLaunchArgument("physics_hz", default_value="120.0")', launch_text)
        self.assertIn('DeclareLaunchArgument("render_hz", default_value="30.0")', launch_text)
        self.assertIn('default_value="fixed_kinematic"', launch_text)

    def test_mid360_safe_mount_mode_is_the_performance_default(self) -> None:
        launch_text = LAUNCH_PATH.read_text(encoding="utf-8")
        performance_text = (
            ROOT / "test" / "performance" / "robot_control_full_performance.py"
        ).read_text(encoding="utf-8")
        self.assertIn("fixed_kinematic (safe default)", launch_text)
        self.assertIn('default="fixed_kinematic"', performance_text)
        self.assertIn("OPENFLEX_MID360_UNSAFE_FOLLOW", (
            ROOT / "ros2_pkgs" / "simulation_bridge" / "sensor_pkg" /
            "isaacsim_sensors" / "mid360.py"
        ).read_text(encoding="utf-8"))

    def test_controller_target_is_90_hz_for_public_joint_states(self) -> None:
        controller_text = (
            ROOT / "ros2_pkgs" / "control" / "bringup" / "config" / "controllers.isaac.mobile_base.yaml"
        ).read_text(encoding="utf-8")
        self.assertIn("update_rate: 90", controller_text)

    def test_control_only_does_not_start_duplicate_lift_state_broadcaster(self) -> None:
        launch_text = LAUNCH_PATH.read_text(encoding="utf-8")
        self.assertNotIn('"lift_state_controller",', launch_text)

    def test_empty_scene_benchmark_uses_control_adapter_without_robot_spawn(self) -> None:
        benchmark_text = (ROOT / "test" / "performance" / "empty_scene_performance.py").read_text(encoding="utf-8")
        self.assertIn("OPENFLEX_EMPTY_SCENE", benchmark_text)
        self.assertIn("empty_stage.usd", benchmark_text)
        self.assertNotIn("robot_control_only.launch.py", benchmark_text)

    def test_rviz_exposes_all_robot_control_panels(self) -> None:
        rviz_text = (ROOT / "ros2_pkgs" / "control" / "bringup" / "config" / "robot_control_only.rviz").read_text(encoding="utf-8")
        for panel in (
            "swerve_base_panel/SwerveBasePanel",
            "openarmx_joint_slider_panel/JointSliderPanel",
            "openarmx_head_joint_slider_panel/HeadJointSliderPanel",
            "lift_slide_panel/LiftPanel",
        ):
            self.assertIn(panel, rviz_text)


if __name__ == "__main__":
    unittest.main()
