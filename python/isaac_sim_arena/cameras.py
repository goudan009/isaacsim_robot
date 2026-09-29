"""The four-camera OpenFleX rig.

Field names here become both the scene entity names and the observation keys, so they are
kept identical to the ``node_namespace``-style names the ROS 2 side uses.
"""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.sensors.camera.camera_cfg import CameraCfg
from isaaclab.utils.configclass import configclass

from isaaclab_arena.utils.cameras import ArenaCameraCfg

from isaac_sim_arena.constants import (
    CAMERA_FOCAL_LENGTH_MM,
    CAMERA_HEIGHT,
    CAMERA_HORIZONTAL_APERTURE_MM,
    CAMERA_MOUNTS,
    CAMERA_VERTICAL_APERTURE_MM,
    CAMERA_WIDTH,
)


def _openflex_camera(name: str) -> CameraCfg:
    """Build one camera from its ``realsense_robot_mounts.yaml`` entry.

    ``convention="opengl"`` is load-bearing, not decoration: the mount config documents its
    quaternions as a *ROS-compatible upright USD camera pose*, which is exactly the USD
    camera local frame. The camera sensor converts between conventions only when the
    configured origin differs from the target, so declaring the convention it is already in
    passes the pose through untouched.
    """
    mount = CAMERA_MOUNTS[name]
    return CameraCfg(
        prim_path=f"{mount['parent_prim']}/{name}",
        height=CAMERA_HEIGHT,
        width=CAMERA_WIDTH,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=CAMERA_FOCAL_LENGTH_MM,
            horizontal_aperture=CAMERA_HORIZONTAL_APERTURE_MM,
            vertical_aperture=CAMERA_VERTICAL_APERTURE_MM,
            clipping_range=mount["clipping_range"],
            focus_distance=1.0,
        ),
        offset=CameraCfg.OffsetCfg(pos=mount["pos"], rot=mount["rot_xyzw"], convention="opengl"),
    )


@configclass
class OpenFleXCameraCfg(ArenaCameraCfg):
    """OpenFleX camera rig: one chassis camera, one pan/tilt head camera, two wrists.

    ``ArenaCameraCfg.get_cfg()`` returns a tiled copy, which is what the rest of Arena
    expects; the four field names below are also what ``make_camera_observation_cfg`` turns
    into ``observation["camera_obs"]["<name>_rgb"]`` terms.
    """

    base_camera: CameraCfg = _openflex_camera("base_camera")
    head_camera: CameraCfg = _openflex_camera("head_camera")
    left_wrist_camera: CameraCfg = _openflex_camera("left_wrist_camera")
    right_wrist_camera: CameraCfg = _openflex_camera("right_wrist_camera")
