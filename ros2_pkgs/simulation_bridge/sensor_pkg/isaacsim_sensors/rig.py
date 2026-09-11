"""Isaac Sim 6.0 RealSense rig with a robot-independent mounting contract."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any, Mapping

from .diagnostics import SensorDiagnostics
from .frame_packet import FramePacket
from .mount import LocalPose, apply_local_pose, resolve_mount_prim_path


class _DirectCameraProduct:
    """Read standard camera annotators from a graph-owned RenderProduct."""

    def __init__(self, resolution: tuple[int, int]) -> None:
        self._resolution = resolution
        self._annotators: dict[str, object] = {}

    def attach(self, render_product_path: str, quality_level: str) -> None:
        import omni.replicator.core as rep

        if quality_level != "L0_aligned_fast":
            raise ValueError("graph-owned RenderProducts currently support L0_aligned_fast only")
        self._annotators = {
            "rgb": rep.AnnotatorRegistry.get_annotator("rgb", device="cuda", do_array_copy=False),
            "distance_to_image_plane": rep.AnnotatorRegistry.get_annotator(
                "distance_to_image_plane", device="cuda", do_array_copy=False
            ),
        }
        for annotator in self._annotators.values():
            annotator.attach([render_product_path])

    def get_data(self, annotator_name: str) -> object | None:
        annotator = self._annotators[annotator_name]
        data = annotator.get_data(device="cuda")
        if isinstance(data, dict):
            data = data.get("data")
        if data is None or not getattr(data, "shape", None) or data.shape[0] == 0:
            return None
        import warp as wp

        if not isinstance(data, wp.array):
            data = wp.array(data, device="cuda")
        channels = 4 if annotator_name == "rgb" else 1
        data = data.reshape((*self._resolution, channels))
        return data[:, :, :3] if annotator_name == "rgb" else data


@dataclass(slots=True)
class RigConfig:
    name: str
    model: str
    tick_rate_hz: float = 30.0
    width: int = 640
    height: int = 480
    quality_level: str = "L0_aligned_fast"
    focal_length_mm: float = 3.2
    near_m: float = 0.05
    far_m: float = 10.0
    calibration_id: str = "nominal_unvalidated"
    baseline_mm: float = 50.0
    depth_focal_length_px: float = 615.0
    depth_min_m: float = 0.10
    depth_max_m: float = 10.0
    imu_enabled: bool = False
    imu_rate_hz: float = 200.0
    optical_frame: str = "camera_color_optical_frame"
    imu_frame: str = "camera_imu_frame"

    @classmethod
    def from_mapping(cls, name: str, value: Mapping[str, Any]) -> "RigConfig":
        model = str(value.get("model", "D435"))
        imu = value.get("imu", {}) or {}
        return cls(
            name=name,
            model=model,
            tick_rate_hz=float(value.get("tick_rate_hz", 30.0)),
            width=int(value.get("width", 640)),
            height=int(value.get("height", 480)),
            quality_level=str(value.get("quality_level", "L0_aligned_fast")),
            focal_length_mm=float(value.get("focal_length_mm", 3.2)),
            near_m=float(value.get("near_m", 0.05)),
            far_m=float(value.get("far_m", 10.0)),
            calibration_id=str(value.get("calibration_id", "nominal_unvalidated")),
            baseline_mm=float(value.get("baseline_mm", 50.0)),
            depth_focal_length_px=float(value.get("depth_focal_length_px", 615.0)),
            depth_min_m=float(value.get("depth_min_m", 0.10)),
            depth_max_m=float(value.get("depth_max_m", 10.0)),
            imu_enabled=bool(imu.get("enabled", False)),
            imu_rate_hz=float(imu.get("publish_rate_hz", 200.0)),
            optical_frame=str(value.get("optical_frame", f"{name}_color_optical_frame")),
            imu_frame=str(imu.get("frame", f"{name}_imu_frame")),
        )


class RealSenseRig:
    """Create one physical RGB-D view and optionally one D435i IMU."""

    def __init__(self, name: str = "realsense", config: Mapping[str, Any] | None = None) -> None:
        self.name = name
        self.config = RigConfig.from_mapping(name, config or {})
        self._stage: object | None = None
        self._sensor: object | None = None
        self._imu_sensor: object | None = None
        self._camera_path: str | None = None
        self._mount_path: str | None = None
        self._local_pose = LocalPose()
        self._render_product_path: str | None = None
        self._period_s = 1.0 / self.config.tick_rate_hz
        self._next_due_s = 0.0
        self._frame_id = 0
        self._episode_id = 0
        self._active = False
        self._diagnostics = SensorDiagnostics(name)

    def create(
        self,
        stage: object,
        parent_prim_path: str,
        local_pose: LocalPose | Mapping[str, Any] | None,
        config: Mapping[str, Any] | None = None,
        create_render_product: bool = True,
    ) -> "RealSenseRig":
        """Create the rig relative to a generic parent, with no robot imports."""
        self._stage = stage
        if config is not None:
            self.config = RigConfig.from_mapping(self.name, config)
            self._period_s = 1.0 / self.config.tick_rate_hz
        pose = local_pose if isinstance(local_pose, LocalPose) else LocalPose.from_mapping(local_pose)
        self._local_pose = pose
        mount_path = resolve_mount_prim_path(parent_prim_path, self.name, (config or {}).get("mount_prim_path"))
        self._mount_path = mount_path
        apply_local_pose(stage, mount_path, pose)

        import numpy as np
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera, SingleViewDepthCameraSensor

        self._camera_path = f"{mount_path}/{self.name}_Camera"
        camera = RtxCamera(
            self._camera_path,
            tick_rate=self.config.tick_rate_hz,
            translations=np.array([0.0, 0.0, 0.0]),
            orientations=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        camera.camera.set_focal_lengths(self.config.focal_length_mm)
        camera.camera.set_clipping_ranges(self.config.near_m, self.config.far_m)

        # Keep the ROS optical frame explicit in the USD tree. The camera
        # itself remains the only render source; this is a zero-cost TF frame.
        stage.DefinePrim(f"{self._camera_path}/{self.config.optical_frame}", "Xform")

        resolution = (self.config.height, self.config.width)
        if not create_render_product:
            self._sensor = _DirectCameraProduct(resolution)
        elif self.config.quality_level == "L1_stereo_noise":
            self._sensor = SingleViewDepthCameraSensor(
                camera, resolution=resolution, annotators=[]
            )
            self._sensor.set_sensor_baseline(self.config.baseline_mm)
            self._sensor.set_sensor_focal_length(self.config.depth_focal_length_px)
            self._sensor.set_sensor_distance_cutoffs(self.config.depth_min_m, self.config.depth_max_m)
            self._sensor.set_enabled_post_processing(True)
        elif self.config.quality_level == "L0_aligned_fast":
            self._sensor = CameraSensor(
                camera,
                resolution=resolution,
                annotators=[],
            )
        else:
            raise ValueError(f"unsupported quality level: {self.config.quality_level}")

        if self.config.imu_enabled:
            if self.config.model != "D435i":
                raise ValueError("IMU is only valid when model is D435i")
            from isaacsim.sensors.experimental.physics import IMU, IMUSensor

            imu_path = f"{mount_path}/{self.config.imu_frame}"
            stage.DefinePrim(imu_path, "Xform")
            self._imu_sensor = IMUSensor(IMU.create(imu_path))

        try:
            self._render_product_path = str(self._sensor.render_product.GetPath())
        except Exception:
            self._render_product_path = None
        return self

    def start(self) -> None:
        if self._sensor is None:
            raise RuntimeError("create() must be called before start()")
        if isinstance(self._sensor, _DirectCameraProduct):
            if not self._render_product_path:
                raise RuntimeError("configure_render_product() must be called before start()")
            self._sensor.attach(self._render_product_path, self.config.quality_level)
        elif self.config.quality_level == "L1_stereo_noise":
            self._sensor.attach_annotators(["rgb", "depth_sensor_distance"])
        else:
            self._sensor.attach_annotators(["rgb", "distance_to_image_plane"])
        self._active = True

    def configure_render_product(self, render_product_path: str) -> None:
        if not render_product_path:
            raise ValueError("render_product_path is required")
        self._render_product_path = str(render_product_path)

    def create_python_render_product(self, *, publish_rgb: bool = True, publish_depth: bool = True) -> str:
        """Create the single product used by the output path."""
        if self._camera_path is None:
            raise RuntimeError("create() must be called before create_python_render_product()")
        if not publish_rgb and not publish_depth:
            raise ValueError("at least one of publish_rgb or publish_depth must be enabled")
        import omni.replicator.core as rep

        product = rep.create.render_product(
            self._camera_path,
            (self.config.width, self.config.height),
            name=self.name,
        )
        self._render_product_path = str(product.path)
        return self._render_product_path

    def record_physics(self, sample_sim_time: float) -> None:
        self._diagnostics.record_physics(sample_sim_time)

    def reset_measurement(self) -> None:
        """Reset timing diagnostics after RTX warm-up, without resetting the rig."""

        self._diagnostics.reset_measurement()

    def poll_due_frames(self, sample_sim_time: float) -> list[FramePacket]:
        if not self._active or self._sensor is None or sample_sim_time + 1e-12 < self._next_due_s:
            return []
        annotator = "depth_sensor_distance" if self.config.quality_level == "L1_stereo_noise" else "distance_to_image_plane"
        rgb_result = self._sensor.get_data("rgb")
        depth_result = self._sensor.get_data(annotator)
        # CameraSensor returns (data, metadata), while a graph-owned direct
        # reader returns the data object itself. Keep both ownership modes
        # behind the same FramePacket contract.
        rgb = rgb_result[0] if isinstance(rgb_result, tuple) else rgb_result
        depth = depth_result[0] if isinstance(depth_result, tuple) else depth_result
        if rgb is None or depth is None:
            return []
        capture_wall_time_ns = time.time_ns()
        imu_data = self._imu_sensor.get_data() if self._imu_sensor is not None else None
        packet = FramePacket.owned(
            episode_id=self._episode_id,
            snapshot_id=None,
            camera_name=self.name,
            frame_id=self._frame_id,
            sample_sim_time_ns=max(0, int(round(sample_sim_time * 1e9))),
            capture_wall_time_ns=capture_wall_time_ns,
            calibration_id=self.config.calibration_id,
            rgb=rgb,
            depth_m=depth,
            depth_semantics=("z_depth_stereo_noise" if self.config.quality_level == "L1_stereo_noise" else "z_depth_ideal_aligned"),
            source_state_seq=self._frame_id,
            rgb_encoding="rgb8",
            depth_encoding="32FC1_m",
            imu=imu_data,
        )
        packet.validate()
        self._diagnostics.record_frame(self._frame_id, sample_sim_time, capture_wall_time_ns)
        self._frame_id += 1
        self._next_due_s = max(self._next_due_s + self._period_s, sample_sim_time + self._period_s)
        return [packet]

    def diagnostics(self) -> dict[str, Any]:
        result = self._diagnostics.summary()
        render_product_details: dict[str, Any] = {}
        if self._stage is not None and self._render_product_path:
            try:
                prim = self._stage.GetPrimAtPath(self._render_product_path)
                render_product_details = {
                    "valid": bool(prim and prim.IsValid()),
                    "children": [
                        {
                            "path": str(child.GetPath()),
                            "type": child.GetTypeName(),
                        }
                        for child in prim.GetChildren()
                    ]
                    if prim and prim.IsValid()
                    else [],
                    "ordered_vars": [
                        str(path)
                        for path in prim.GetRelationship("orderedVars").GetTargets()
                    ]
                    if prim and prim.IsValid()
                    else [],
                }
            except Exception as exc:
                render_product_details = {"error": repr(exc)}
        result.update({
            "model": self.config.model,
            "quality_level": self.config.quality_level,
            "camera_prim": self._camera_path,
            "render_product": self._render_product_path,
            "render_product_details": render_product_details,
            "imu_enabled": self._imu_sensor is not None,
        })
        return result

    @property
    def render_product_path(self) -> str | None:
        return self._render_product_path

    @property
    def camera_prim_path(self) -> str | None:
        return self._camera_path

    @property
    def mount_prim_path(self) -> str | None:
        return self._mount_path

    @property
    def local_pose(self) -> LocalPose:
        return self._local_pose

    def reset(self, episode_id: int) -> None:
        self._episode_id = int(episode_id)
        self._frame_id = 0
        self._next_due_s = 0.0

    def close(self) -> None:
        self._active = False
        self._sensor = None
        self._imu_sensor = None
