#!/usr/bin/env python3
"""Unit tests for Isaac-only sensor generation profiles."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET


REPO_DIR = Path(__file__).resolve().parents[3]
GENERATOR = (
    REPO_DIR
    / "ros2_pkgs"
    / "openflex_isaac_sim"
    / "openflex_isaac_description"
    / "scripts"
    / "generate_isaac_urdf.py"
)


def load_generator_module():
    spec = importlib.util.spec_from_file_location("generate_isaac_urdf", GENERATOR)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def sensor_root() -> ET.Element:
    root = ET.Element("robot", {"name": "openflex"})
    for name in (
        "d435_color_optical_frame",
        "d435_depth_optical_frame",
        "head_yaw_link",
        "openarmx_left_hand",
        "openarmx_right_hand",
        "livox_frame",
    ):
        ET.SubElement(root, "link", {"name": name})
    return root


class GenerateIsaacUrdfSensorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.generator = load_generator_module()

    def test_data_profile_generates_224_square_cameras_and_lidar(self) -> None:
        root = sensor_root()

        added = self.generator.add_isaac_sensors(
            root,
            enabled=True,
            image_width=224,
            image_height=224,
        )

        sensors = root.findall("./isaac/sensor")
        camera_sensors = [sensor for sensor in sensors if sensor.get("type") in ("camera", "depth_camera")]
        lidar_sensors = [sensor for sensor in sensors if sensor.get("type") == "lidar"]
        self.assertEqual(added, 10)
        self.assertEqual(len(camera_sensors), 8)
        self.assertEqual(len(lidar_sensors), 1)
        imu_sensors = [sensor for sensor in sensors if sensor.get("type") == "imu"]
        self.assertEqual(len(imu_sensors), 1)
        self.assertEqual(imu_sensors[0].get("name"), "livox_frame")
        self.assertEqual(imu_sensors[0].findtext("topic"), "/livox/imu")
        self.assertTrue(all(sensor.findtext("image/width") == "224" for sensor in camera_sensors))
        self.assertTrue(all(sensor.findtext("image/height") == "224" for sensor in camera_sensors))
        render_groups = [sensor.findtext("render_group") for sensor in camera_sensors]
        self.assertCountEqual(
            render_groups,
            ["base", "base", "head", "head", "left_hand", "left_hand", "right_hand", "right_hand"],
        )

    def test_teleop_profile_omits_all_sensor_nodes(self) -> None:
        root = sensor_root()

        added = self.generator.add_isaac_sensors(
            root,
            enabled=False,
            image_width=224,
            image_height=224,
        )

        self.assertEqual(added, 0)
        self.assertEqual(root.findall("./isaac/sensor"), [])

    def test_mid360_mount_override_updates_fixed_joint(self) -> None:
        root = ET.Element("robot", {"name": "openflex"})
        joint = ET.SubElement(root, "joint", {"name": "mid360_joint", "type": "fixed"})
        ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})

        self.assertTrue(self.generator.override_mid360_mount(root, "0.11 -0.22 0.33", "0.1 0.2 0.3"))
        origin = root.find("./joint[@name='mid360_joint']/origin")
        self.assertIsNotNone(origin)
        assert origin is not None
        self.assertEqual(origin.get("xyz"), "0.11 -0.22 0.33")
        self.assertEqual(origin.get("rpy"), "0.1 0.2 0.3")

    def test_rgb_profile_contains_only_one_rgb_sensor_per_mount(self) -> None:
        root = sensor_root()

        added = self.generator.add_isaac_sensors(
            root,
            enabled=True,
            image_width=224,
            image_height=224,
            sensor_profile="rgb",
        )

        sensors = root.findall("./isaac/sensor")
        self.assertEqual(added, 4)
        self.assertEqual([sensor.get("type") for sensor in sensors], ["camera"] * 4)
        self.assertCountEqual(
            [sensor.findtext("render_group") for sensor in sensors],
            ["base", "head", "left_hand", "right_hand"],
        )

    def test_rgb_depth_profile_pairs_each_mount(self) -> None:
        root = sensor_root()

        added = self.generator.add_isaac_sensors(
            root,
            enabled=True,
            image_width=224,
            image_height=224,
            sensor_profile="rgb_depth",
        )

        sensors = root.findall("./isaac/sensor")
        self.assertEqual(added, 8)
        self.assertEqual(len(sensors), 8)
        for group in ("base", "head", "left_hand", "right_hand"):
            grouped = [sensor for sensor in sensors if sensor.findtext("render_group") == group]
            self.assertCountEqual([sensor.get("type") for sensor in grouped], ["camera", "depth_camera"])

    def test_replaces_only_high_poly_lift_collisions_with_boxes(self) -> None:
        root = ET.Element("robot", {"name": "openflex"})
        for link_name, mesh_name in (
            ("lift_base_link", "lift_link.STL"),
            ("lift_carriage_link", "plate_link.STL"),
            ("openarmx_left_link1", "link1_symp.stl"),
        ):
            link = ET.SubElement(root, "link", {"name": link_name})
            visual = ET.SubElement(link, "visual")
            visual_geometry = ET.SubElement(visual, "geometry")
            ET.SubElement(visual_geometry, "mesh", {"filename": mesh_name})
            collision = ET.SubElement(link, "collision")
            collision_geometry = ET.SubElement(collision, "geometry")
            ET.SubElement(collision_geometry, "mesh", {"filename": mesh_name})

        changed = self.generator.replace_high_poly_lift_collisions_with_boxes(root)

        self.assertEqual(changed, 2)
        expected = {
            "lift_base_link": ("0.00675 0.0407973 0.63", "0.1835 0.1712 1.5298"),
            "lift_carriage_link": ("-0.00375 0.0215 0", "0.1925 0.347 0.23"),
        }
        for link_name, (expected_origin, expected_size) in expected.items():
            link = root.find(f"./link[@name='{link_name}']")
            self.assertIsNotNone(link)
            self.assertEqual(link.find("./collision/origin").get("xyz"), expected_origin)
            self.assertEqual(link.find("./collision/origin").get("rpy"), "0 0 0")
            self.assertEqual(link.find("./collision/geometry/box").get("size"), expected_size)
            self.assertIsNone(link.find("./collision/geometry/mesh"))
            self.assertIsNotNone(link.find("./visual/geometry/mesh"))

        arm_link = root.find("./link[@name='openarmx_left_link1']")
        self.assertIsNotNone(arm_link.find("./collision/geometry/mesh"))
        self.assertIsNone(arm_link.find("./collision/geometry/box"))

    def test_removes_inertials_only_from_massless_fixed_frames(self) -> None:
        root = ET.Element("robot", {"name": "openflex"})
        for name, has_visual in (("base_footprint", False), ("camera_frame", False), ("physical_mount", True)):
            link = ET.SubElement(root, "link", {"name": name})
            ET.SubElement(link, "inertial")
            if has_visual:
                ET.SubElement(link, "visual")
            joint = ET.SubElement(root, "joint", {"name": f"{name}_joint", "type": "fixed"})
            ET.SubElement(joint, "child", {"link": name})

        removed = self.generator.remove_massless_fixed_frame_inertials(root)

        self.assertEqual(removed, 2)
        self.assertIsNone(root.find("./link[@name='base_footprint']/inertial"))
        self.assertIsNone(root.find("./link[@name='camera_frame']/inertial"))
        self.assertIsNotNone(root.find("./link[@name='physical_mount']/inertial"))


if __name__ == "__main__":
    unittest.main()
