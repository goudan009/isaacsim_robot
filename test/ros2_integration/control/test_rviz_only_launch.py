#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import re
import sys
import types
import unittest
from unittest import mock


REPO_DIR = Path(__file__).resolve().parents[3]
PACKAGE_DIR = REPO_DIR / "ros2_pkgs" / "control" / "bringup"
LAUNCH_PATH = PACKAGE_DIR / "launch" / "rviz_only.launch.py"
RVIZ_CONFIG_PATH = (
    REPO_DIR.parent
    / "openflex_integrated"
    / "openarmx_integrated_description"
    / "rviz"
    / "integrated_robot.rviz"
)


class _FakeAction:
    def __init__(self, *args, **kwargs) -> None:
        self.args = args
        self.kwargs = kwargs


def load_launch_module():
    launch_module = types.ModuleType("launch")
    launch_module.LaunchDescription = _FakeAction

    launch_actions_module = types.ModuleType("launch.actions")
    launch_actions_module.DeclareLaunchArgument = _FakeAction
    launch_actions_module.OpaqueFunction = _FakeAction

    launch_substitutions_module = types.ModuleType("launch.substitutions")
    launch_substitutions_module.LaunchConfiguration = _FakeAction

    launch_ros_module = types.ModuleType("launch_ros")
    launch_ros_actions_module = types.ModuleType("launch_ros.actions")
    launch_ros_actions_module.Node = _FakeAction

    launch_ros_parameters_module = types.ModuleType("launch_ros.parameter_descriptions")
    launch_ros_parameters_module.ParameterValue = _FakeAction

    ament_module = types.ModuleType("ament_index_python")
    ament_packages_module = types.ModuleType("ament_index_python.packages")
    ament_packages_module.get_package_share_directory = lambda _name: "/tmp"

    modules = {
        "ament_index_python": ament_module,
        "ament_index_python.packages": ament_packages_module,
        "launch": launch_module,
        "launch.actions": launch_actions_module,
        "launch.substitutions": launch_substitutions_module,
        "launch_ros": launch_ros_module,
        "launch_ros.actions": launch_ros_actions_module,
        "launch_ros.parameter_descriptions": launch_ros_parameters_module,
    }
    spec = importlib.util.spec_from_file_location("rviz_only_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    with mock.patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class RvizOnlyLaunchTest(unittest.TestCase):
    def test_integrated_rviz_enables_rgb_camera_displays(self) -> None:
        text = RVIZ_CONFIG_PATH.read_text(encoding="utf-8")
        for name, topic in (
            ("LeftWrist", "/cam_left/color/image"),
            ("RightWrist", "/cam_right/color/image"),
            ("Head", "/cam_head/color/image"),
            ("Base", "/cam_base/color/image"),
        ):
            with self.subTest(name=name):
                display = re.search(
                    rf"- Class: rviz_default_plugins/Image\n"
                    rf"\s+Enabled: (?P<enabled>\w+)\n"
                    rf"(?:(?!- Class: rviz_default_plugins/Image).)*?"
                    rf"\s+Name: {name}\n"
                    rf"(?:(?!- Class: rviz_default_plugins/Image).)*?"
                    rf"\s+Value: {re.escape(topic)}\n",
                    text,
                    re.DOTALL,
                )
                self.assertIsNotNone(display)
                assert display is not None
                self.assertEqual(display.group("enabled"), "true")

    def test_rviz_display_error_explains_missing_gui_environment(self) -> None:
        launch_file = load_launch_module()

        with mock.patch.dict(os.environ, {}, clear=True):
            error = launch_file._rviz_display_error("")

        self.assertIsNotNone(error)
        assert error is not None
        self.assertIn("DISPLAY", error)
        self.assertIn("qt_qpa_platform", error)

    def test_rviz_display_error_allows_display_wayland_or_explicit_qt_platform(self) -> None:
        launch_file = load_launch_module()

        with mock.patch.dict(os.environ, {"DISPLAY": ":0"}, clear=True):
            self.assertIsNone(launch_file._rviz_display_error(""))
        with mock.patch.dict(os.environ, {"WAYLAND_DISPLAY": "wayland-0"}, clear=True):
            self.assertIsNone(launch_file._rviz_display_error(""))
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertIsNone(launch_file._rviz_display_error("offscreen"))
        with mock.patch.dict(os.environ, {"QT_QPA_PLATFORM": "xcb"}, clear=True):
            self.assertIsNone(launch_file._rviz_display_error(""))

    def test_rviz_node_environment_propagates_explicit_qt_platform_only(self) -> None:
        launch_file = load_launch_module()

        self.assertEqual(launch_file._rviz_node_environment(""), {})
        self.assertEqual(
            launch_file._rviz_node_environment("offscreen"),
            {"QT_QPA_PLATFORM": "offscreen"},
        )


if __name__ == "__main__":
    unittest.main()
