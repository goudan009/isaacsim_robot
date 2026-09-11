"""Create the released RealSense and MID360 suite on a loaded robot USD."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import yaml

from .rig import RealSenseRig
from .sinks import IsaacSimRos2Bridge
from .mount import LocalPose, apply_local_pose


_LIVE_SENSOR_OBJECTS: list[object] = []


def _prepare_robot_sensor_rendering(*, srtx_enabled: bool = False) -> None:
    """Select the requested ROS2 camera transport before any graph exists."""
    try:
        import carb

        carb.settings.get_settings().set("/exts/omni.replicator.srtx/enabled", bool(srtx_enabled))
        print(
            "[isaacsim_sensors] "
            + ("Enabled SRTX native ROS2 camera transport" if srtx_enabled else
               "Disabled SRTX; using the stable Replicator ROS2 path")
            + " before robot sensor graph creation"
        )
    except Exception as exc:
        print(f"[isaacsim_sensors] Could not configure SRTX: {exc}")


def _read_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"sensor asset config does not exist: {path}")
    value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(value, dict):
        raise ValueError(f"sensor configuration root must be a mapping: {path}")
    return value


def _first_existing(root: Path, *relative_paths: str) -> Path:
    for relative_path in relative_paths:
        candidate = root / relative_path
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        "sensor asset config does not exist; checked: "
        + ", ".join(str(root / value) for value in relative_paths)
    )


def load_realsense_config(sensor_asset_dir: Path) -> dict[str, Any]:
    """Load only the RealSense contract.

    The branch-specific snapshot may place config under ``realsense/config``;
    the transition tree keeps the ROS package layout for compatibility.
    """
    path = _first_existing(
        sensor_asset_dir,
        "realsense/config/realsense_robot_mounts.yaml",
        "config/realsense_robot_mounts.yaml",
        "isaac_sim_core/config/sensor_params/realsense/realsense_robot_mounts.yaml",
        "ros2_pkgs/simulation_bridge/sensor_pkg/config/realsense_robot_mounts.yaml",
    )
    return _read_yaml(path)


def load_mid360_config(sensor_asset_dir: Path) -> dict[str, Any]:
    """Load only the MID360 contract without importing RealSense code."""
    path = _first_existing(
        sensor_asset_dir,
        "mid360/config/mid360_robot_mount.yaml",
        "isaac_sim_core/config/sensor_params/mid360/mid360_robot_mount.yaml",
        "ros2_pkgs/simulation_bridge/sensor_pkg/mid360/mid360_robot_mount.yaml",
    )
    return _read_yaml(path)


def load_robot_sensor_config(sensor_asset_dir: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """Compatibility loader for callers that intentionally request both assets."""
    camera_config = load_realsense_config(sensor_asset_dir)
    lidar_config = load_mid360_config(sensor_asset_dir)
    return camera_config, lidar_config


def resolve_robot_mount_path(configured_path: str, robot_prim_path: str) -> str:
    prefix = "/World/OpenFlex"
    if configured_path == prefix:
        return robot_prim_path
    if configured_path.startswith(prefix + "/"):
        return robot_prim_path.rstrip("/") + configured_path[len(prefix):]
    return configured_path


def _profile_flags(sensor_profile: str) -> tuple[bool, bool]:
    profile = sensor_profile.strip().lower()
    if profile not in {"none", "rgb", "rgb_depth", "lidar", "data", "teleop"}:
        raise ValueError(f"unsupported sensor profile: {sensor_profile}")
    return profile in {"rgb", "rgb_depth", "data"}, profile in {"lidar", "data"}


def create_robot_sensor_suite(
    stage: object,
    robot_prim_path: str,
    sensor_asset_dir: Path | None = None,
    *,
    realsense_asset_dir: Path | None = None,
    mid360_asset_dir: Path | None = None,
    sensor_profile: str = "data",
    lidar_profile: str = "MID360_APPROX",
    lidar_transport: str = "helper",
    lidar_mount_mode: str = "fixed_kinematic",
    lidar_object_id_map: bool = True,
    lidar_tick_rate_hz: float = 10.0,
    srtx_enabled: bool = False,
    camera_resolution: tuple[int, int] | None = None,
    publish_camera_info: bool = True,
    camera_tick_rate_hz: float | None = None,
) -> dict[str, object]:
    """Attach configured sensors while keeping the robot asset sensor-free."""
    # ``sensor_asset_dir`` is the transition-era combined repository.  The
    # component-5 entrypoint may override each asset root independently once
    # components 1 and 2 are published as separate repositories.
    combined_root = Path(sensor_asset_dir).expanduser() if sensor_asset_dir else None
    realsense_root = Path(realsense_asset_dir).expanduser() if realsense_asset_dir else combined_root
    mid360_root = Path(mid360_asset_dir).expanduser() if mid360_asset_dir else combined_root
    enable_cameras, enable_lidar = _profile_flags(sensor_profile)
    if enable_cameras and realsense_root is None:
        raise ValueError("a RealSense asset root is required for camera startup")
    if enable_lidar and mid360_root is None:
        raise ValueError("a MID360 asset root is required for LiDAR startup")
    camera_config = load_realsense_config(realsense_root) if enable_cameras else {}
    lidar_config = load_mid360_config(mid360_root) if enable_lidar else {}
    created: list[str] = []
    camera_records: list[dict[str, object]] = []
    bridge: IsaacSimRos2Bridge | None = None

    if enable_cameras or enable_lidar:
        _prepare_robot_sensor_rendering(srtx_enabled=srtx_enabled)

    if enable_cameras:
        bridge = IsaacSimRos2Bridge(f"{robot_prim_path}/OpenFlexSensorROS2")
        for name, raw_config in (camera_config.get("cameras", {}) or {}).items():
            if not isinstance(raw_config, Mapping) or not bool(raw_config.get("enabled", True)):
                continue
            config = dict(raw_config)
            if camera_resolution is not None:
                width, height = camera_resolution
                if width < 1 or height < 1:
                    raise ValueError("camera_resolution dimensions must be positive")
                config["width"] = int(width)
                config["height"] = int(height)
            if camera_tick_rate_hz is not None:
                if camera_tick_rate_hz <= 0.0:
                    raise ValueError("camera_tick_rate_hz must be positive")
                config["tick_rate_hz"] = float(camera_tick_rate_hz)
            config["mount_prim_path"] = resolve_robot_mount_path(
                str(config["mount_prim_path"]), robot_prim_path
            )
            config["parent_prim"] = resolve_robot_mount_path(
                str(config.get("parent_prim", robot_prim_path)), robot_prim_path
            )
            rig = RealSenseRig(name, config)
            rig.create(
                stage,
                str(config["parent_prim"]),
                config.get("local_pose"),
                config,
                create_render_product=True,
            )
            namespace = str(config["node_namespace"])
            camera_records.append({
                "camera_key": name,
                "render_product_path": rig.render_product_path or "",
                "camera_prim_path": rig.camera_prim_path or "",
                "frame_id": str(config["optical_frame"]),
                "node_namespace": f"openflex/sensors/{namespace}",
                "rgb_topic": "color/image",
                "depth_topic": "depth/image",
                "camera_info_topic": "color/camera_info",
                "tick_rate_hz": float(config.get("tick_rate_hz", 30.0)),
                "width": int(config.get("width", 640)),
                "height": int(config.get("height", 480)),
                "publish_camera_info": bool(publish_camera_info),
                "queue_size": int((camera_config.get("queues", {}) or {}).get("ros2_queue_size", 5)),
                "qos_profile": "Sensor Data",
            })
            created.append(rig.camera_prim_path or name)
            _LIVE_SENSOR_OBJECTS.append(rig)
        if camera_records:
            bridge.attach_cameras(camera_records)
            _LIVE_SENSOR_OBJECTS.append(bridge)

    lidar_path = ""
    if enable_lidar:
        from .mid360 import (
            create_physics_imu_graph,
            create_robot_mid360,
            robot_lidar_graph_path,
            robot_sensor_root_path,
        )

        lidar = lidar_config.get("sensor", {}) or {}
        mount_path = resolve_robot_mount_path(str(lidar["mount_prim_path"]), robot_prim_path)
        mount = stage.GetPrimAtPath(mount_path)
        if not mount or not mount.IsValid():
            # Some referenced USD compositions expose camera mounts but omit
            # the authored MID360 mount. Create an equivalent session-local
            # Xform only after the replica has collected its joint chain.
            # Keeping this fallback in the sensor contract avoids patching or
            # re-composing the robot asset just to attach a runtime sensor.
            parent_path = resolve_robot_mount_path(
                str(lidar.get("parent_prim", robot_prim_path)), robot_prim_path
            )
            parent = stage.GetPrimAtPath(parent_path)
            if not parent or not parent.IsValid():
                raise RuntimeError(
                    f"MID360 parent prim does not exist: {parent_path}; cannot create mount {mount_path}"
                )
            fallback_pose = LocalPose.from_mapping(
                lidar.get("fallback_mount_pose") or lidar.get("local_pose")
            )
            apply_local_pose(stage, mount_path, fallback_pose)
            mount = stage.GetPrimAtPath(mount_path)
            if not mount or not mount.IsValid():
                raise RuntimeError(f"failed to create MID360 runtime mount: {mount_path}")
            print(f"[isaacsim_sensors] Created missing MID360 runtime mount: {mount_path}")
        lidar_path = create_robot_mid360(
            stage,
            mount_path,
            str(lidar.get("frame_id", "livox_frame")),
            str(lidar.get("pointcloud_topic", "/openflex/livox_frame/lidar")),
            lidar_profile,
            graph_path=robot_lidar_graph_path(robot_prim_path),
            sensor_root_path=robot_sensor_root_path(robot_prim_path),
            transport=lidar_transport,
            mount_mode=lidar_mount_mode,
            object_id_map=lidar_object_id_map,
            tick_rate_hz=lidar_tick_rate_hz,
        )
        sensor_root_path = robot_sensor_root_path(robot_prim_path)
        create_physics_imu_graph(
            stage,
            sensor_root_path,
            str(lidar.get("frame_id", "livox_frame")),
            str(lidar.get("imu_topic", "/livox/imu")),
        )
        created.extend([sensor_root_path, lidar_path, sensor_root_path + "/IMU"])

    if bridge is not None and camera_records:
        # Camera helpers create their SyntheticData gates on the first rendered
        # frames. Bootstrap those frames before configuring the 30 Hz gates.
        import omni.kit.app
        import omni.timeline

        omni.timeline.get_timeline_interface().play()
        app = omni.kit.app.get_app()
        for _ in range(4):
            app.update()
        bridge.configure_camera_gates(camera_records)

    return {
        "success": True,
        "created": created,
        "camera_count": len(camera_records),
        "lidar_prim_path": lidar_path,
        "sensor_profile": sensor_profile,
        "lidar_profile": lidar_profile,
        "lidar_transport": lidar_transport,
        "lidar_mount_mode": lidar_mount_mode,
        "lidar_object_id_map": bool(lidar_object_id_map),
        "lidar_tick_rate_hz": float(lidar_tick_rate_hz),
        "srtx_enabled": bool(srtx_enabled),
        "camera_resolution": list(camera_resolution) if camera_resolution is not None else None,
        "publish_camera_info": bool(publish_camera_info),
        "camera_tick_rate_hz": float(camera_tick_rate_hz) if camera_tick_rate_hz is not None else None,
        "realsense_asset_dir": str(realsense_root) if realsense_root else "",
        "mid360_asset_dir": str(mid360_root) if mid360_root else "",
    }
