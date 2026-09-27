#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch


SCRIPT = (
    Path(__file__).resolve().parents[3]
    / "ros2_pkgs/openflex_isaac_sim/openflex_isaac_bringup/scripts/start_robot_control_sim.py"
)
SPEC = importlib.util.spec_from_file_location("start_robot_control_sim", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class SimulationAppStartupTest(unittest.TestCase):
    def test_headless_startup_hides_ui_and_keeps_sensor_rendering_enabled(self) -> None:
        calls = []

        def simulation_app(config, *args, **kwargs):
            calls.append((config, args, kwargs))
            return "app"

        fake_isaacsim = type("IsaacSim", (), {"SimulationApp": simulation_app})
        with patch.dict(sys.modules, {"isaacsim": fake_isaacsim}):
            MODULE._patch_simulation_app_startup()
            result = fake_isaacsim.SimulationApp({
                "renderer": "RayTracedLighting",
                "headless": True,
                "hide_ui": False,
                "open_usd": "/tmp/scene.usd",
                "extra_args": ["--custom-arg=true"],
            })

        self.assertEqual(result, "app")
        config, args, kwargs = calls[0]
        self.assertEqual(args, ())
        self.assertEqual(kwargs, {})
        self.assertTrue(config["hide_ui"])
        self.assertTrue(config["disable_viewport_updates"])
        self.assertEqual(config["renderer"], "RayTracedLighting")
        self.assertEqual(config["open_usd"], "/tmp/scene.usd")
        self.assertIn("--custom-arg=true", config["extra_args"])
        self.assertIn("--/renderer/multiGpu/enabled=false", config["extra_args"])

    def test_interactive_startup_keeps_requested_ui_visibility(self) -> None:
        calls = []

        def simulation_app(config, *args, **kwargs):
            calls.append(config)
            return "app"

        fake_isaacsim = type("IsaacSim", (), {"SimulationApp": simulation_app})
        with patch.dict(sys.modules, {"isaacsim": fake_isaacsim}):
            MODULE._patch_simulation_app_startup()
            fake_isaacsim.SimulationApp({"headless": False, "hide_ui": False})

        self.assertFalse(calls[0]["hide_ui"])
        self.assertNotIn("disable_viewport_updates", calls[0])


if __name__ == "__main__":
    unittest.main()
