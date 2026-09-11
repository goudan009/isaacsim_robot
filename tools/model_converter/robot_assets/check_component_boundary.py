#!/usr/bin/env python3
"""Check that component 3 remains a pure, decoupled robot-asset repository.

This check is intentionally independent of Isaac Sim and GPU availability. It
guards repository layout and catches accidental runtime/sensor code before a
release commit is made.
"""

from __future__ import annotations

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
REQUIRED_FILES = (
    ROOT / "assets/usd/robots/openflex_robot.usda",
    ROOT / "assets/usd/scenes/robot_only_stage.usda",
    ROOT / "tools/export_robot_only_asset.py",
    ROOT / "tools/validate_asset.py",
)

FORBIDDEN_FILENAMES = re.compile(
    r"(^|/)(?:.*\.launch(?:\.py)?|package\.xml|setup\.py|CMakeLists\.txt)$",
    re.IGNORECASE,
)
FORBIDDEN_ASSET_TOKENS = (
    "realsense",
    "d435",
    "d405",
    "femto_bolt_camera",
    "openflexsensors",
    "ros2_camera",
    "lidar_ros2_graph",
    "mid360_link",
    "mid360_color",
    "topic_based_ros2_control",
    "ros2_control",
    "isaacsim.ros2.bridge",
    "actiongraph",
)
FORBIDDEN_PRIM_DEFINITIONS = re.compile(
    r"\bdef\s+(?:Camera|OmniLidar|OmniGraph)\s+\"", re.IGNORECASE
)


def _repo_files() -> list[Path]:
    return [
        path
        for path in ROOT.rglob("*")
        if path.is_file() and ".git" not in path.parts and "__pycache__" not in path.parts
    ]


def main() -> int:
    errors: list[str] = []
    for path in REQUIRED_FILES:
        if not path.is_file():
            errors.append(f"required file missing: {path.relative_to(ROOT)}")

    files = _repo_files()
    for path in files:
        relative = path.relative_to(ROOT).as_posix()
        if FORBIDDEN_FILENAMES.search(relative):
            errors.append(f"runtime/package file is not allowed: {relative}")

    asset_files = [
        path
        for path in files
        if path.is_relative_to(ROOT / "assets") and path.suffix.lower() in {".usd", ".usda", ".usdc", ".usdz"}
    ]
    for path in asset_files:
        relative = path.relative_to(ROOT).as_posix()
        if path.suffix.lower() not in {".usda", ".usd"}:
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="ignore").lower()
        except OSError as exc:
            errors.append(f"cannot read {relative}: {exc}")
            continue
        for token in FORBIDDEN_ASSET_TOKENS:
            if token in text:
                errors.append(f"forbidden asset token {token!r} in {relative}")
        if FORBIDDEN_PRIM_DEFINITIONS.search(text):
            errors.append(f"sensor/runtime prim definition found in {relative}")

    robot_asset = ROOT / "assets/usd/robots/openflex_robot.usda"
    if robot_asset.is_file():
        text = robot_asset.read_text(encoding="utf-8", errors="ignore")
        for mount in (
            "CameraMount",
            "Mid360Mount",
            "HeadCameraMount",
            "LeftWristCameraMount",
            "RightWristCameraMount",
        ):
            if f'"{mount}"' not in text:
                errors.append(f"required attachment point missing: {mount}")

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1

    print(f"PASS component=robot-assets files={len(files)} usd_assets={len(asset_files)}")
    print("PASS boundary=no-sensor-prims,no-runtime-packages,no-ros2-control")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
