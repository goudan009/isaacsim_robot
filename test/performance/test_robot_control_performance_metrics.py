#!/usr/bin/env python3
"""Unit tests for the timing calculations used by the runtime benchmark."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import types
import unittest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "test" / "performance" / "robot_control_performance.py"


def _load_module():
    # Keep the pure timing tests runnable on hosts without a ROS installation.
    module_names = [
        "rclpy",
        "rclpy.node",
        "geometry_msgs",
        "geometry_msgs.msg",
        "nav_msgs",
        "nav_msgs.msg",
        "rosgraph_msgs",
        "rosgraph_msgs.msg",
        "sensor_msgs",
        "sensor_msgs.msg",
    ]
    original_modules = {name: sys.modules.get(name) for name in module_names}
    for name in (
        "rclpy",
        "geometry_msgs",
        "geometry_msgs.msg",
        "nav_msgs",
        "nav_msgs.msg",
        "rosgraph_msgs",
        "rosgraph_msgs.msg",
        "sensor_msgs",
        "sensor_msgs.msg",
    ):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules["rclpy"].Node = object
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    sys.modules["rclpy"].node = rclpy_node
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy"].spin_once = lambda *args, **kwargs: None
    sys.modules["rclpy"].init = lambda: None
    sys.modules["rclpy"].ok = lambda: False
    sys.modules["rclpy"].shutdown = lambda: None
    for module_name, class_name in (
        ("geometry_msgs.msg", "Twist"),
        ("nav_msgs.msg", "Odometry"),
        ("rosgraph_msgs.msg", "Clock"),
        ("sensor_msgs.msg", "JointState"),
    ):
        setattr(sys.modules[module_name], class_name, type(class_name, (), {}))
    spec = importlib.util.spec_from_file_location("robot_control_performance", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
    finally:
        for name, original in original_modules.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original
    return module


class TimingMetricTest(unittest.TestCase):
    def test_direct_ros_receive_frequency_is_wall_clock_frequency(self) -> None:
        module = _load_module()
        samples = module.TopicSamples()
        for index in range(5):
            samples.add(float(index) * 0.01, 1.0 + index * 0.01)
        metrics = samples.stats(physics_dt=0.01)
        self.assertAlmostEqual(metrics["wall_receive"]["hz"], 100.0)
        self.assertAlmostEqual(metrics["sim_timestamp"]["hz"], 100.0)

    def test_rtf_uses_simulation_window_not_message_frequency_ratio(self) -> None:
        module = _load_module()
        samples = module.TopicSamples()
        samples.add(10.0, 20.0)
        samples.add(10.5, 20.25)
        samples.add(11.0, 20.50)
        metrics = samples.stats(physics_dt=0.01)
        self.assertAlmostEqual(metrics["rtf"], 0.5)
        self.assertAlmostEqual(metrics["sim_wall_seconds"], 1.0)

    def test_backwards_sim_time_invalidates_measurement(self) -> None:
        module = _load_module()
        samples = module.TopicSamples()
        samples.add(0.0, 1.0)
        samples.add(0.1, 1.1)
        samples.add(0.2, 0.2)
        samples.add(0.3, 0.3)
        metrics = samples.stats(physics_dt=0.1)
        self.assertEqual(metrics["sim_backward_count"], 1)
        self.assertFalse(metrics["measurement_valid"])


if __name__ == "__main__":
    unittest.main()
