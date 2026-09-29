"""OpenFleX geometry, joint names and contract limits.

Every number here is mirrored from this repository's own canonical configs, not invented.
Sources, all inside ``isaacsim_robot``:

  ros2_pkgs/openflex_isaac_sim/openflex_isaac_contract/config/embodiment.yaml
      the 22-dim action vector, per-field limits, observation layout
  ros2_pkgs/openflex_isaac_sim/openflex_isaac_bringup/config/controllers.isaac.mobile_base.yaml
      swerve module geometry and wheel radius
  isaac_sim_core/config/sensor_params/realsense/realsense_robot_mounts.yaml
      the four camera parent prims and local poses
  isaac_sim_core/assets/robots/openflex_robot.usda
      joint names, and the URDF-derived ``drive:angular:physics:*`` gain table

If a joint name or a limit changes upstream, change it here and nowhere else.
"""

from __future__ import annotations

from pathlib import Path

# Assets are resolved relative to this file rather than from an absolute path, so a clone
# works from any directory. This file lives at <repo>/python/isaac_sim_arena/constants.py,
# which makes parents[2] the repository root.
_REPO_ROOT = Path(__file__).resolve().parents[2]

# --------------------------------------------------------------------------------------
# Robot asset
# --------------------------------------------------------------------------------------
ROBOT_USD_PATH = str(_REPO_ROOT / "isaac_sim_core/assets/robots/openflex_robot.usda")

# --------------------------------------------------------------------------------------
# Scene assets layered on top of an Arena background
# --------------------------------------------------------------------------------------
SCENE_ASSETS_DIR = str(_REPO_ROOT / "isaac_sim_core/assets/environments")

# The hydrogen supply cabinet, brought in from
# /home/2025201288DYD/openflex_ws/4-385LV2供氢系统7.usd by a byte-for-byte copy (md5
# b7827628bb433d2101798735f5a43163). The source file's numbers are millimetre-scale
# (raw bounds (0, 0, 0.03)..(1680.0, 2466.7, 1559.95)) and it declares metersPerUnit = 0.01
# with upAxis = Y -- but this USD build does not rescale a reference across metersPerUnit,
# measured two ways, so the raw numbers survive intact and the spawn has to carry both the
# scale and the axis change itself. See HYDROGEN_SUPPLY_SYSTEM_SCALE and the pose in
# scene_assets.py.
#
# The path points at the _solid wrapper rather than at the copy itself: that import has no
# collision in it anywhere, so the raw file spawns as a cabinet the robot drives through.
# The wrapper references the copy unchanged and authors a collider on its mesh, which is why
# this is the file to spawn and hydrogen_supply_system.usd is not.
HYDROGEN_SUPPLY_SYSTEM_USD_PATH = f"{SCENE_ASSETS_DIR}/hydrogen_supply_system_solid.usda"
HYDROGEN_SUPPLY_SYSTEM_SCALE = 0.001
"""Linear factor taking that file's raw units to metres: 1.680 x 1.560 x 2.467 m."""

# The USD's default prim is /openarmx_integrated, so a spawn at {ENV_REGEX_NS}/Robot makes
# every link reachable under this prefix. All prim paths below are derived from it: if the
# robot's prim path ever changes, the cameras move with it.
ROBOT_PRIM_PATH = "{ENV_REGEX_NS}/Robot"
_GEOM = f"{ROBOT_PRIM_PATH}/Geometry"
BASE_LINK_PRIM = f"{_GEOM}/base_link"
LIFT_CARRIAGE_PRIM = f"{BASE_LINK_PRIM}/lift_carriage_link"

# Height of the base_link frame above the floor when the robot is resting on its wheels.
# The USD's default prim is at floor level, but base_link is authored 0.17 m above it, and
# base_link is the articulation root -- so this, not 0, is the z an all-zero joint pose
# rests at. A reset that spawns the root below it buries the wheels in the floor and PhysX
# ejects the robot; see scene.py's init_state and validate.py's _check_reset_pose.
BASE_LINK_REST_HEIGHT = 0.170
# The USD's bbox sits 5 mm above its origin, so spawning at rest height would put the wheels
# in contact at t=0. Kept as a deliberate settle drop instead, to avoid an initial
# interpenetration with the ground plane.
SPAWN_CLEARANCE_M = 0.005

# --------------------------------------------------------------------------------------
# Swerve base
# --------------------------------------------------------------------------------------
WHEEL_RADIUS_M = 0.075

# controllers.isaac.mobile_base.yaml: fl/fr/bl/br_pos_x/y. Cross-checked against the USD's
# fl_steering_joint physics:localPos0 = (0.21, 0.2735, -0.07).
MODULE_POSITIONS_XY = (
    (0.21, 0.2735),  # fl
    (0.21, -0.2735),  # fr
    (-0.21, 0.2735),  # bl
    (-0.21, -0.2735),  # br
)
STEERING_JOINT_NAMES = (
    "fl_steering_joint",
    "fr_steering_joint",
    "bl_steering_joint",
    "br_steering_joint",
)
WHEEL_JOINT_NAMES = (
    "fl_wheel_joint",
    "fr_wheel_joint",
    "bl_wheel_joint",
    "br_wheel_joint",
)

# embodiment.yaml: limit_source "max_wheel_speed=0.8, and angular.z=0.8/sqrt(0.21^2+0.2735^2)".
# max_wheel_speed is a *linear module* speed in m/s, so the wheel's angular limit is 0.8/R.
MAX_WHEEL_LINEAR_SPEED = 0.8
MAX_WHEEL_ANGULAR_VELOCITY = MAX_WHEEL_LINEAR_SPEED / WHEEL_RADIUS_M  # 10.6667 rad/s

MAX_LINEAR_VELOCITY = 0.8  # base_twist.linear.x / .y
MAX_ANGULAR_VELOCITY = 2.320037210795224  # base_twist.angular.z == 0.8 / 0.344817...

# controllers.isaac.mobile_base.yaml steering limits, matching the USD's +-90 deg joint range.
STEERING_LOWER_LIMIT = -1.5708
STEERING_UPPER_LIMIT = 1.5708

# --------------------------------------------------------------------------------------
# Upper body
# --------------------------------------------------------------------------------------
# Contract order for the head is [yaw, pitch]; the USD authors [pitch, yaw]. Never rely on
# the articulation's own joint order -- resolve by name (see mdp/actions).
HEAD_JOINT_NAMES = ("openarmx_head_yaw_joint", "openarmx_head_pitch_joint")
LEFT_ARM_JOINT_NAMES = tuple(f"openarmx_left_joint{i}" for i in range(1, 8))
RIGHT_ARM_JOINT_NAMES = tuple(f"openarmx_right_joint{i}" for i in range(1, 8))
LIFT_JOINT_NAME = "lift_joint"

# The gripper is a parallel two-jaw: finger_joint1 and finger_joint2 are independent
# PhysicsPrismaticJoints (no physxMimicJointAPI anywhere in the USD), but the contract
# exposes exactly one action dim per hand. ParallelGripperAction drives both jaws from it.
LEFT_FINGER_JOINT_NAMES = ("openarmx_left_finger_joint1", "openarmx_left_finger_joint2")
RIGHT_FINGER_JOINT_NAMES = ("openarmx_right_finger_joint1", "openarmx_right_finger_joint2")
FINGER_LIMITS = (0.0, 0.044)

# --------------------------------------------------------------------------------------
# Action-space limits, keyed by the exact joint each contract field commands.
# Fed to JointPositionActionCfg(clip=...), which resolves them by name.
# --------------------------------------------------------------------------------------
ACTION_JOINT_LIMITS: dict[str, tuple[float, float]] = {
    LIFT_JOINT_NAME: (-0.650, 0.300),
    "openarmx_head_yaw_joint": (-1.5708, 1.5708),
    "openarmx_head_pitch_joint": (-1.5708, 1.5708),
    "openarmx_left_joint1": (-3.344396, 0.905604),
    "openarmx_left_joint2": (-3.2707963267948967, 0.1292036732051034),
    "openarmx_left_joint3": (-1.57, 1.57),
    "openarmx_left_joint4": (0.0, 1.8),
    "openarmx_left_joint5": (-1.5, 1.5),
    "openarmx_left_joint6": (-0.75, 0.75),
    "openarmx_left_joint7": (-1.4, 1.4),
    "openarmx_right_joint1": (-1.25, 3.0),
    "openarmx_right_joint2": (-0.1292036732051034, 3.2707963267948967),
    "openarmx_right_joint3": (-1.57, 1.57),
    "openarmx_right_joint4": (0.0, 1.8),
    "openarmx_right_joint5": (-1.5, 1.5),
    "openarmx_right_joint6": (-0.75, 0.75),
    "openarmx_right_joint7": (-1.4, 1.4),
}

# --------------------------------------------------------------------------------------
# Policy state vector
# --------------------------------------------------------------------------------------
# 19 = the 22 action dims minus the 3 base_twist columns, in the same order as
# embodiment.yaml. This is what a GR00T/DreamZero ``robot_joint_pos`` term must contain,
# and it is deliberately *not* the articulation's own joint order (head would come last
# with pitch before yaw).
ROBOT_STATE_JOINT_NAMES = (
    LIFT_JOINT_NAME,
    *HEAD_JOINT_NAMES,
    *LEFT_ARM_JOINT_NAMES,
    "openarmx_left_finger_joint1",
    *RIGHT_ARM_JOINT_NAMES,
    "openarmx_right_finger_joint1",
)  # len == 19

# --------------------------------------------------------------------------------------
# Cameras -- verbatim from realsense_robot_mounts.yaml
# --------------------------------------------------------------------------------------
# The config documents local_pose as a *ROS-compatible upright USD camera pose*: local -Z
# looks along robot +X, local +Y along robot +Z, local +X toward robot -Y. That is exactly
# CameraCfg.offset with convention="opengl", so the YAML values pass through unconverted.
# Rotations are stored here as xyzw because OffsetCfg.rot is xyzw; the YAML is wxyz.
#
# The `mount_prim_path` entries are deliberately unused: local_pose already bakes in the
# full transform from the parent link, and for the base camera the mount's own orient is a
# decoy (same translation, different rotation).
CAMERA_MOUNTS: dict[str, dict] = {
    "base_camera": dict(
        parent_prim=BASE_LINK_PRIM,
        pos=(0.36, 0.0, 0.055),
        rot_xyzw=(0.5, -0.5, -0.5, 0.5),  # wxyz (0.5, 0.5, -0.5, -0.5)
        clipping_range=(0.05, 10.0),
    ),
    "head_camera": dict(
        parent_prim=f"{LIFT_CARRIAGE_PRIM}/head_pitch_link/head_yaw_link",
        pos=(0.00000032669455, 0.0665480897, 0.0535495905),
        rot_xyzw=(0.70710678, 0.0, 0.0, 0.70710678),  # wxyz (0.70710678, 0.70710678, 0, 0)
        clipping_range=(0.05, 10.0),
    ),
    "left_wrist_camera": dict(
        parent_prim=f"{LIFT_CARRIAGE_PRIM}/openarmx_left_link1/openarmx_left_link2/openarmx_left_link3"
        "/openarmx_left_link4/openarmx_left_link5/openarmx_left_link6/openarmx_left_link7",
        pos=(0.06720526, -0.01250108, 0.11842935),
        rot_xyzw=(0.0, 0.95371695, 0.0, 0.30070580),  # wxyz (0.30070580, 0, 0.95371695, 0)
        clipping_range=(0.03, 3.0),
    ),
    "right_wrist_camera": dict(
        parent_prim=f"{LIFT_CARRIAGE_PRIM}/openarmx_right_link1/openarmx_right_link2/openarmx_right_link3"
        "/openarmx_right_link4/openarmx_right_link5/openarmx_right_link6/openarmx_right_link7",
        pos=(0.06721000, -0.01250000, 0.11843000),
        rot_xyzw=(0.0, -0.95371695, 0.0, -0.30070580),  # wxyz (-0.30070580, 0, -0.95371695, 0)
        clipping_range=(0.03, 3.0),
    ),
}

# Expected position of each camera prim at the all-zero joint pose, expressed in the
# base_link frame. Used by validate.py to catch a wrong parent prim or a wrong orientation
# convention before anyone trusts the images.
#
# Deliberately relative to base_link rather than absolute world. An absolute value has to
# assume a height for the base_link frame, and the asset and the spawned sim disagree about
# it: the robot's USD origin sits at floor level with base_link authored 0.175 m above it,
# whereas the spawned articulation puts the base_link frame at the spawn height (0). An
# absolute table therefore carries a 175 mm error that has nothing to do with the cameras.
# Measuring in the base_link frame drops that convention, and drops the settle with it.
#
# Obtained by composing local_pose through the joint chain (each joint's localPos0/localRot0
# plus each link prim's own xformOp); the spawned stage reproduces these to 0.1 mm. A wrong
# parent prim moves these numbers, which is the failure this table exists to catch.
CAMERA_EXPECTED_POS_IN_BASE_LINK = {
    "base_camera": (0.36, 0.0, 0.055),
    "head_camera": (0.2327, 0.0, 1.1408),
    "left_wrist_camera": (0.2342, 0.2264, 0.2888),
    "right_wrist_camera": (0.2342, -0.2016, 0.2888),
}

# Intrinsics: calibration K has fx = fy = 415 px at 640x480, and the mount config gives
# focal_length_mm = 3.2, so the apertures follow from horizontal_aperture = width * fl / fx.
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FOCAL_LENGTH_MM = 3.2
CAMERA_HORIZONTAL_APERTURE_MM = CAMERA_WIDTH * CAMERA_FOCAL_LENGTH_MM / 415.0  # 4.9349
CAMERA_VERTICAL_APERTURE_MM = CAMERA_HEIGHT * CAMERA_FOCAL_LENGTH_MM / 415.0  # 3.7012
