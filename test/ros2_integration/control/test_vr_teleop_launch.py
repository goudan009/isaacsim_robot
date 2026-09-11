#!/usr/bin/env python3
"""Static contract tests for the Isaac-specific VR teleoperation launch."""

from pathlib import Path
import unittest


REPO_DIR = Path(__file__).resolve().parents[3]
PACKAGE_DIR = REPO_DIR / "ros2_pkgs" / "control" / "bringup"
LAUNCH_FILE = PACKAGE_DIR / "launch" / "vr_teleop.launch.py"


class VrTeleopLaunchTest(unittest.TestCase):
    def test_isaac_arm_adapter_is_executable(self) -> None:
        adapter = PACKAGE_DIR / "scripts" / "isaacsim_vr_arm_node.py"

        self.assertTrue(adapter.stat().st_mode & 0o111)

    def test_launch_starts_complete_vr_control_path(self) -> None:
        script = LAUNCH_FILE.read_text(encoding="utf-8")

        for package, executable in (
            ("openflex_vr_bridge", "pico_pose_bridge_node"),
            ("isaacsim_bringup", "isaacsim_vr_arm_node.py"),
            ("openarmx_head_teleop_vr_pico", "head_teleop_node"),
            ("swerve_bringup", "vr_teleop_node"),
            ("swerve_bringup", "vr_lift_control_node"),
        ):
            with self.subTest(package=package, executable=executable):
                self.assertIn(f'package="{package}"', script)
                self.assertIn(f'executable="{executable}"', script)

    def test_isaac_arm_adapter_selects_hand_tcp_without_modifying_common_vr(self) -> None:
        adapter = PACKAGE_DIR / "scripts" / "isaacsim_vr_arm_node.py"
        script = LAUNCH_FILE.read_text(encoding="utf-8")

        self.assertTrue(adapter.is_file())
        adapter_text = adapter.read_text(encoding="utf-8")
        self.assertIn('"openarmx_left_hand_tcp"', adapter_text)
        self.assertIn('"openarmx_right_hand_tcp"', adapter_text)
        self.assertIn('"urdf_path": isaac_urdf', script)

    def test_launch_uses_isaac_urdf_and_conservative_chassis_defaults(self) -> None:
        script = LAUNCH_FILE.read_text(encoding="utf-8")

        self.assertIn('"openflex_isaac_robot.urdf"', script)
        self.assertIn('DeclareLaunchArgument("max_linear_speed", default_value="0.35")', script)
        self.assertIn('DeclareLaunchArgument("boost_linear_speed", default_value="0.50")', script)
        self.assertIn('DeclareLaunchArgument("max_angular_speed", default_value="0.60")', script)
        self.assertIn('DeclareLaunchArgument("acceleration_time", default_value="2.0")', script)
        self.assertIn('DeclareLaunchArgument("listen_port", default_value="5100")', script)

    def test_launch_stops_existing_vr_stack_before_creating_nodes(self) -> None:
        script = LAUNCH_FILE.read_text(encoding="utf-8")

        self.assertIn(
            'DeclareLaunchArgument("stop_existing_vr", default_value="true")',
            script,
        )
        self.assertIn('"stop_stale_vr_processes.py"', script)
        self.assertIn("function=_start_vr_stack", script)


if __name__ == "__main__":
    unittest.main()
