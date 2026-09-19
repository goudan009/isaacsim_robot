#!/usr/bin/env python3
"""Verify the standalone Isaac Sim repository boundary."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
ROS_ROOT = ROOT / "ros2_pkgs" / "openflex_isaac_sim"


class StandaloneRepositoryLayoutTest(unittest.TestCase):
    def test_robot_and_sensor_assets_remain_in_the_core_layer(self) -> None:
        self.assertTrue((ROOT / "isaac_sim_core/assets/robots/openflex_robot.usda").is_file())
        self.assertTrue((ROOT / "isaac_sim_core/assets/environments/robot_only_stage.usda").is_file())
        self.assertTrue((ROOT / "isaac_sim_core/config/sensor_params/mid360/mid360_robot_mount.yaml").is_file())

    def test_openflex_isaac_packages_are_owned_by_this_repository(self) -> None:
        expected = {
            "openflex_isaac_bridge",
            "openflex_isaac_bringup",
            "openflex_isaac_contract",
            "openflex_isaac_controllers",
            "openflex_isaac_description",
            "openflex_isaac_sensors",
        }
        actual = {
            path.parent.name
            for path in ROS_ROOT.glob("*/package.xml")
        }

        self.assertEqual(actual, expected)

    def test_old_split_package_layout_is_absent(self) -> None:
        self.assertFalse(any((ROOT / "ros2_pkgs" / "control").glob("*/package.xml")))
        self.assertFalse(any((ROOT / "ros2_pkgs" / "simulation_bridge").glob("*/package.xml")))

    def test_runtime_reports_are_versioned_with_the_simulator(self) -> None:
        report_root = ROOT / "reports" / "runtime" / "full_chain"
        for name in (
            "isaac6_full_chain_20260919_8dof.json",
            "isaac6_vla_interface_final_20260919.json",
            "isaac6_vla_control_final_20260919.json",
            "isaac6_vla_rgbd_20260919.json",
        ):
            self.assertTrue((report_root / name).is_file(), name)


if __name__ == "__main__":
    unittest.main()
