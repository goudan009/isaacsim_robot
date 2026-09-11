#!/usr/bin/env python3
"""Verify the staged Isaac Sim migration boundary."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]


class StagedMigrationLayoutTest(unittest.TestCase):
    def test_robot_assets_are_in_the_core_layer(self) -> None:
        self.assertTrue((ROOT / "isaac_sim_core/assets/robots/openflex_robot.usda").is_file())
        self.assertTrue((ROOT / "isaac_sim_core/assets/environments/robot_only_stage.usda").is_file())

    def test_sensor_assets_and_configs_are_in_the_sensor_layer(self) -> None:
        self.assertTrue((ROOT / "isaac_sim_core/assets/environments/mid360_empty_stage.usda").is_file())
        self.assertTrue((ROOT / "isaac_sim_core/config/sensor_params/realsense/realsense_quad.yaml").is_file())
        self.assertTrue((ROOT / "isaac_sim_core/config/sensor_params/mid360/mid360_robot_mount.yaml").is_file())
        self.assertTrue((ROOT / "ros2_pkgs/simulation_bridge/sensor_pkg/package.xml").is_file())

    def test_robot_control_packages_are_present(self) -> None:
        for package in ("description", "contract", "bringup"):
            self.assertTrue((ROOT / "ros2_pkgs/control" / package / "package.xml").is_file())
        self.assertTrue(
            (ROOT / "ros2_pkgs/control/bringup/config/controllers.isaac.mobile_base.yaml").is_file()
        )

    def test_ros2_package_paths_do_not_use_openflex_prefix(self) -> None:
        paths = [path.relative_to(ROOT).as_posix() for path in (ROOT / "ros2_pkgs").rglob("*")]
        self.assertFalse(any("openflex" in path.lower() for path in paths))

    def test_full_body_composition_is_deferred(self) -> None:
        forbidden = (
            ROOT / "ros2_pkgs/control/bringup/launch/all_body.launch.py",
            ROOT / "ros2_pkgs/control/bringup/scripts/start_sim_with_openflex_rest.py",
        )
        self.assertTrue(all(not path.exists() for path in forbidden))


if __name__ == "__main__":
    unittest.main()
