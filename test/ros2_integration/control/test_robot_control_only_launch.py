#!/usr/bin/env python3
"""Static checks for the standalone Isaac Sim 6 launch contract."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[3]
BRINGUP = ROOT / "ros2_pkgs" / "openflex_isaac_sim" / "openflex_isaac_bringup"
SENSORS = ROOT / "ros2_pkgs" / "openflex_isaac_sim" / "openflex_isaac_sensors"
SIM_LAUNCH = BRINGUP / "launch" / "sim.launch.py"
START_SCRIPT = BRINGUP / "scripts" / "start_robot_control_sim.py"


class RobotControlLaunchTest(unittest.TestCase):
    def test_full_launch_exposes_all_sensor_profiles(self) -> None:
        text = SIM_LAUNCH.read_text(encoding="utf-8")

        self.assertIn('default_value="full"', text)
        self.assertIn('choices=["none", "minimal", "lidar", "full"]', text)
        self.assertIn('profile_map = {"none": "none", "minimal": "rgb_depth", "lidar": "lidar", "full": "data"}', text)
        self.assertIn('lidar_profile = "MID360_PERFORMANCE"', text)
        self.assertIn('default_value="parented"', text)

    def test_launch_keeps_rviz_separate_and_starts_vla_contract_bridge(self) -> None:
        text = SIM_LAUNCH.read_text(encoding="utf-8")

        self.assertNotIn('package="rviz2"', text)
        self.assertIn('executable="vla_contract_bridge.py"', text)
        self.assertIn('executable="camera_contract_publisher.py"', text)
        self.assertIn('executable="isaacsim_compat_bridge.py"', text)
        self.assertIn('"livox_lidar_mode": "custom"', text)
        self.assertIn('default_value="15000"', text)
        self.assertIn('"livox_max_points": ParameterValue(', text)
        self.assertIn('"pointcloud_topics": ["/livox/lidar_points"]', text)
        self.assertTrue((BRINGUP / "launch" / "rviz_only.launch.py").is_file())

    def test_isaac_startup_enables_multitick_motion_bvh(self) -> None:
        text = START_SCRIPT.read_text(encoding="utf-8")

        self.assertIn('"--/rtx/hydra/supportMultiTickRate=true"', text)
        self.assertIn('"--/renderer/raytracingMotion/enabled=true"', text)
        self.assertIn('"--/renderer/multiGpu/enabled=false"', text)

    def test_mid360_profiles_keep_dense_firing_rates(self) -> None:
        text = (SENSORS / "openflex_isaac_sensors" / "mid360.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('"omni:sensor:Core:patternFiringRateHz": 20000', text)
        self.assertIn('"omni:sensor:Core:patternFiringRateHz": 36000', text)
        self.assertIn('"omni:sensor:Core:nearRangeM": 0.1', text)
        self.assertIn('"omni:sensor:Core:farRangeM": 40.0', text)
        self.assertIn('"OPENFLEX_ISAAC_LIDAR_CONFIG", "Example_Rotary"', text)
        self.assertIn('mount_mode: str = "parented"', text)

    def test_controller_target_is_90_hz(self) -> None:
        text = (BRINGUP / "config" / "controllers.isaac.mobile_base.yaml").read_text(
            encoding="utf-8"
        )

        self.assertIn("update_rate: 90", text)


if __name__ == "__main__":
    unittest.main()
