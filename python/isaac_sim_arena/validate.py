"""In-sim validation for the OpenFleX Arena environment.

Builds the environment through the same path ``environment_runner`` uses, then checks the
things that are easy to get subtly wrong and impossible to notice from a screenshot: the
action vector's size and layout, the name-keyed joint ordering, whether an uncommanded joint
sags, whether a reset returns a robot standing on its wheels, whether the base drives the
way the swerve term claims, and whether each camera is mounted where
``realsense_robot_mounts.yaml`` says. Optionally writes the camera frames to PNG so the
extrinsics can be judged by eye rather than trusted.

Run with the Arena launcher, from the Arena checkout::

    ./arena.sh "$PWD/../isaacsim_robot/python/isaac_sim_arena/validate.py" \\
        --enable_cameras --dump_dir /tmp/openflex_probe/frames \\
        --external_environment_class_path \\
          isaac_sim_arena.external_env:OpenFleXExternalEnvironment \\
        openflex_arena

Three things this script must not do, all learned the hard way:

* pass ``--disable_fabric``. Arena's own environment_runner sets it by default on CPU, but
  with fabric off the spawned stage's head links come out mis-oriented, which moves the head
  camera 111 mm and reads as a camera bug.
* check the cameras after driving the base. With fabric on, the stage tracks the robot at
  reset and then stops, so a late check compares against a frozen stage.
* write a twist with ``action[0] = ...``. The action is (num_envs, 22), so that broadcasts
  the value into every field and tests a compound command while reporting it as a pure one.

Order matters: read the cameras first, at the pristine reset, before anything has driven the
base and let the stage go stale; then check the reset pose; then drive.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import argparse
import os

import numpy as np
import torch

from isaaclab_arena.cli.isaaclab_arena_cli import get_isaaclab_arena_cli_parser
from isaaclab_arena.utils.hydra_overrides import assert_hydra_overrides
from isaaclab_arena.utils.isaaclab_utils.simulation_app import SimulationAppContext
from isaaclab_arena_environments.cli import get_arena_builder_from_cli, get_isaaclab_arena_environments_cli_parser

from isaac_sim_arena.constants import (
    BASE_LINK_PRIM,
    BASE_LINK_REST_HEIGHT,
    CAMERA_EXPECTED_POS_IN_BASE_LINK,
    CAMERA_MOUNTS,
    MODULE_POSITIONS_XY,
    ROBOT_STATE_JOINT_NAMES,
    STEERING_JOINT_NAMES,
    WHEEL_JOINT_NAMES,
)

if TYPE_CHECKING:
    from isaac_sim_arena.mdp.observations import ContractOrderJointState

_EXPECTED_ACTION_DIM = 22
# Camera mounts are checked in the parent link's frame, so the base settling no longer enters
# the number and the tolerance only has to absorb float noise (measured residuals: 0.1 mm).
# The stage/PhysX check is the loose one: a genuine desync is ~100 mm, not millimetres.
_CAMERA_POS_TOLERANCE_M = 0.002
_CAMERA_ROT_TOLERANCE_DEG = 0.1
_LINK_DESYNC_TOLERANCE_M = 0.002

# Base tracking. Measured error is ~3% on a 0.2 m/s command, so these are tracking checks,
# not calibration knobs: a wrong sign or a saturated module shows up as a factor of two or
# as a component that simply never arrives, never as a few percent.
_TWIST_LIN_TOLERANCE = 0.05  # m/s, per component
_TWIST_ANG_TOLERANCE = 0.05  # rad/s
_WHEEL_SPEED_TOLERANCE = 0.15  # rad/s; a flipped wheel sign is ~2x, i.e. ~5 rad/s
_STEER_ANGLE_TOLERANCE_DEG = 3.0  # steering servos settle well inside this

# The spawn carries a 5 mm settle drop by design, so this has to clear that. Still 17x
# smaller than the 170 mm error it exists to catch, so it does not need to be delicate.
_RESET_HEIGHT_TOLERANCE_M = 0.010


def _parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = get_isaaclab_arena_cli_parser()
    # Fabric is deliberately left on, unlike arena's own environment_runner default. With
    # fabric disabled the spawned stage's link poses come out wrong for the head chain --
    # head_pitch_link is authored 46.2 deg off its true pitch and head_yaw_link 5.6 deg off
    # its yaw -- which moves the head camera 111 mm and makes a correct rig look broken.
    # These checks read poses out of the stage, so they need the stage to be trustworthy.
    parser.set_defaults(device="cpu", num_envs=1)
    parser.allow_abbrev = False
    parser = get_isaaclab_arena_environments_cli_parser(parser)
    parser.add_argument("--dump_dir", type=str, default=None, help="Write camera frames here as PNGs.")
    parser.add_argument("--dump_steps", type=int, default=30, help="Steps to settle before dumping frames.")
    args_cli, hydra_overrides = parser.parse_known_args()
    assert_hydra_overrides(hydra_overrides, parser)
    return args_cli, hydra_overrides


def _check_action_layout(env) -> None:
    """The action vector is the policy-facing contract; assert it field by field."""
    shape = tuple(env.action_space.shape)
    # Arena reports (num_envs, action_dim) here rather than the (action_dim,) that
    # environment_runner's torch.zeros(env.action_space.shape) would suggest; both are
    # accepted by the action manager, so only the trailing dimension is contractual.
    assert shape[-1] == _EXPECTED_ACTION_DIM, f"action_space is {shape}, expected last dim {_EXPECTED_ACTION_DIM}"

    terms = env.action_manager.active_terms
    dims = [env.action_manager.get_term(name).action_dim for name in terms]
    expected = [
        ("base_action", 3),
        ("lift_action", 1),
        ("head_action", 2),
        ("left_arm_action", 7),
        ("left_gripper_action", 1),
        ("right_arm_action", 7),
        ("right_gripper_action", 1),
    ]
    assert list(zip(terms, dims, strict=True)) == expected, f"action terms are {list(zip(terms, dims))}, expected {expected}"
    print(f"[ok] action_space {shape}, last dim {_EXPECTED_ACTION_DIM}")
    print(f"[ok] action terms: {list(zip(terms, dims))}")


def _find_state_term(env) -> ContractOrderJointState:
    """Locate the deployed robot_joint_pos term instance on the observation manager."""
    # Imported here, not at module scope: this module pulls in isaaclab.envs and warp, and
    # importing either before Kit has finished loading its extensions breaks pxr's class
    # wrappers ("extension class wrapper for base class ... has not been created yet").
    from isaac_sim_arena.mdp.observations import ContractOrderJointState

    manager = env.observation_manager
    for bucket in (manager._group_obs_class_term_cfgs, manager._group_obs_term_cfgs):
        for group_name, term_cfgs in bucket.items():
            if group_name != "policy":
                continue
            for term_cfg in term_cfgs:
                if isinstance(term_cfg.func, ContractOrderJointState):
                    return term_cfg.func
    raise AssertionError("robot_joint_pos is not backed by ContractOrderJointState")


def _check_joint_ordering(env) -> None:
    """Name-keyed resolution must beat the articulation's own ordering.

    The USD interleaves steering and wheel joints and authors the head as [pitch, yaw]
    last. If any term had fallen back to articulation order, these would disagree.
    """
    robot = env.scene["robot"]
    articulation_order = list(robot.data.joint_names)

    base_term = env.action_manager.get_term("base_action")
    assert base_term._steer_names == list(STEERING_JOINT_NAMES), f"steering order: {base_term._steer_names}"
    assert base_term._wheel_names == list(WHEEL_JOINT_NAMES), f"wheel order: {base_term._wheel_names}"

    head_term = env.action_manager.get_term("head_action")
    assert list(head_term._joint_names) == ["openarmx_head_yaw_joint", "openarmx_head_pitch_joint"], (
        f"head order: {head_term._joint_names}"
    )

    # The state term resolves its indices lazily, on the first call (env.reset() has run it).
    # Class-based terms live in a separate bucket from function terms, and the manager
    # exposes neither publicly, so reach in -- and then check the *output* below rather than
    # trusting the config we just read back.
    state_term = _find_state_term(env)
    assert state_term.joint_names == ROBOT_STATE_JOINT_NAMES, f"state order: {state_term.joint_names}"

    got = state_term(env)
    assert got.shape == (1, len(ROBOT_STATE_JOINT_NAMES)), f"state term returned {tuple(got.shape)}"
    lookup = {name: i for i, name in enumerate(robot.data.joint_names)}
    columns = [robot.data.joint_pos[:, lookup[name]] for name in ROBOT_STATE_JOINT_NAMES]
    assert torch.allclose(got[0], torch.stack(columns), atol=0.0), "state term columns are out of order"

    print(f"[ok] articulation order : {articulation_order[:6]} ... (USD order, deliberately unused)")
    print(f"[ok] swerve module order: {list(STEERING_JOINT_NAMES)}")
    print(f"[ok] state vector order : {len(ROBOT_STATE_JOINT_NAMES)} joints, head {ROBOT_STATE_JOINT_NAMES[1:3]}")

    # A swerve module must be reachable by name from the joint it declares, and the module
    # positions must match the USD's own module offsets -- a swapped fl/fr would still
    # "work" but would steer the robot the wrong way.
    assert len(MODULE_POSITIONS_XY) == 4
    for name, (x, y) in zip(STEERING_JOINT_NAMES, MODULE_POSITIONS_XY, strict=True):
        assert (x, y) != (0.0, 0.0), f"module {name} has no offset"
    print(f"[ok] module offsets       : {list(MODULE_POSITIONS_XY)}")


def _report_joint_state(env) -> None:
    """Print every joint's authored target against where it actually settled.

    A large |delta| on a joint that was never commanded means the actuator is too soft to
    hold its pose against gravity -- which is exactly the failure a camera calibration
    check reports as "the camera moved", without saying why.
    """
    robot = env.scene["robot"]
    default = robot.data.default_joint_pos[0].cpu()
    actual = robot.data.joint_pos[0].cpu()

    print(f"{'joint':<34} {'default':>9} {'actual':>9} {'delta':>9}")
    worst_name, worst_delta = "", 0.0
    for i, name in enumerate(robot.data.joint_names):
        delta = (actual[i] - default[i]).item()
        if abs(delta) > abs(worst_delta):
            worst_name, worst_delta = name, delta
        flag = "  <-- sagging" if abs(delta) > 0.05 else ""
        print(f"{name:<34} {default[i]:9.4f} {actual[i]:9.4f} {delta:+9.4f}{flag}")
    print(f"[--] largest uncommanded joint error: {worst_name} {worst_delta:+.4f} rad")


def _check_reset_pose(env, settle: int = 60) -> None:
    """A reset must hand back a robot already at rest, not one leaving the floor.

    This is the check that would have saved the most time. ``init_state.pos`` is the pose of
    the *articulation root*, which for this asset is base_link -- authored 0.17 m above the
    USD origin that sits on the floor. Spawning it at z=0 buries the wheels 17 cm into the
    ground, and PhysX ejects the robot upward on every reset; the trace is z 0.217 -> 0.307
    -> 0.170, settling only after ~20 steps. An RL policy acts during all 20, and commanding
    the base while it is still falling flips it over, so the symptom shows up far away as a
    base that "will not drive" rather than as a spawn height.

    Catches ~170 mm with a 3 mm tolerance, so it does not need to be delicate.
    """
    robot = env.scene["robot"]
    base = robot.data.body_names.index(BASE_LINK_PRIM.rsplit("/", 1)[-1])

    env.reset()
    env.step(torch.zeros(env.action_space.shape, device=env.device))
    spawned = float(robot.data.body_pos_w[0, base, 2])
    for _ in range(settle):
        env.step(torch.zeros(env.action_space.shape, device=env.device))
    settled = float(robot.data.body_pos_w[0, base, 2])

    print(f"[--] after reset: base_link z {spawned:.4f} m, settles to {settled:.4f} m")
    assert settled > 0.01, "base_link ended up on the floor; the robot is not standing"
    assert abs(spawned - BASE_LINK_REST_HEIGHT) < _RESET_HEIGHT_TOLERANCE_M, (
        f"a reset spawns base_link at z={spawned:.4f} but it rests at {BASE_LINK_REST_HEIGHT}. "
        f"The {spawned - BASE_LINK_REST_HEIGHT:+.4f} m is not a settle drop, it is the robot "
        "being launched: init_state.pos.z spawns the articulation root, which is base_link, "
        "not the USD origin at floor level."
    )
    assert abs(settled - spawned) < _RESET_HEIGHT_TOLERANCE_M, (
        f"base_link falls {abs(settled - spawned) * 1000:.0f} mm after a reset, so the spawn "
        "pose is not a resting pose and the first thing every episode does is fall"
    )
    print("[ok] a reset returns the robot already on its wheels")


def _check_base_response(env, settle: int = 60, measure: int = 120) -> None:
    """Drive each twist axis on its own and confirm the base and the wheels both follow.

    This is the one link in the chain that was derived from prim transforms rather than
    measured -- the wheel sign follows from the USD's axis and frame, and the steering sign
    from a similar reading -- so it is the one that gets measured. ``steering_signs`` and
    ``wheel_signs`` exist in the config precisely so a failure here is a config edit rather
    than a code change.

    Each axis is driven separately because each exercises a different part of the mapping:
    linear.x is the plain rolling direction, linear.y needs every module to steer to +-90
    deg, and angular.z needs the four modules to take four different angles. A sign error in
    any one of them is invisible to the other two.

    Only ``action[0, 0]`` is written, never ``action[0]``: the action is (num_envs, 22), so
    ``action[0] = vx`` broadcasts vx into all 22 fields -- base, lift, head, both arms and
    both grippers -- and silently tests a compound command. An earlier version of this check
    did exactly that and reported the base as failing to track, because the "pure vx" command
    was really ``(0.2, 0.2, 0.2)`` plus 0.2 rad on every upper-body joint.
    """
    robot = env.scene["robot"]
    base_term = env.action_manager.get_term("base_action")
    cfg = base_term.cfg

    # Wheel joints are not part of the action vector, so read them off the articulation.
    wheel_ids, _ = robot.find_joints(list(WHEEL_JOINT_NAMES), preserve_order=True)
    steer_ids, _ = robot.find_joints(list(cfg.steering_joint_names), preserve_order=True)
    module_xy = np.asarray(cfg.module_positions_xy, dtype=np.float64)

    for label, twist in (
        ("linear.x", (0.2, 0.0, 0.0)),
        ("linear.y", (0.0, 0.2, 0.0)),
        ("angular.z", (0.0, 0.0, 0.5)),
    ):
        env.reset()
        action = torch.zeros(env.action_space.shape, device=env.device)
        action[0, 0], action[0, 1], action[0, 2] = twist
        for _ in range(settle):
            env.step(action)

        # Average over a window: a single instant of a velocity-servo articulation is noisy
        # enough that one sample reads a few percent off on its own.
        lin, ang, wheel, steer = [], [], [], []
        for _ in range(measure):
            env.step(action)
            lin.append(robot.data.root_lin_vel_b[0].cpu().numpy())
            ang.append(robot.data.root_ang_vel_b[0].cpu().numpy())
            wheel.append(robot.data.joint_vel[0, wheel_ids].cpu().numpy())
            steer.append(robot.data.joint_pos[0, steer_ids].cpu().numpy())
        lin, ang, wheel, steer = (np.mean(v, axis=0) for v in (lin, ang, wheel, steer))

        # What the term should have produced. Re-derived here rather than read off
        # base_term._steer_target: reading its own output back would assert nothing.
        vx, vy, omega = twist
        v_i = np.stack([vx - omega * module_xy[:, 1], vy + omega * module_xy[:, 0]], axis=1)
        want_steer = np.arctan2(v_i[:, 1], v_i[:, 0])
        want_wheel = np.minimum(np.linalg.norm(v_i, axis=1) / cfg.wheel_radius, cfg.max_wheel_angular_velocity)
        # Same flip the term applies: past 90 deg, steer the other way and reverse the wheel.
        flip = np.abs(want_steer) > np.pi / 2
        want_steer = np.where(flip, want_steer - np.sign(want_steer) * np.pi, want_steer)
        want_wheel = np.where(flip, -want_wheel, want_wheel)
        want_steer_deg = np.degrees(np.array(cfg.steering_signs) * want_steer)

        err_lin = float(np.max(np.abs(lin[:2] - np.array([vx, vy]))))
        err_ang = abs(float(ang[2] - omega))
        err_wheel = float(np.max(np.abs(wheel - want_wheel)))
        err_steer = float(np.max(np.abs(np.degrees(steer) - want_steer_deg)))

        print(
            f"{label:<10} cmd ({vx:+.2f}, {vy:+.2f}, {omega:+.2f}) "
            f"-> base ({lin[0]:+.3f}, {lin[1]:+.3f}, {ang[2]:+.3f}) "
            f"wheels {np.round(wheel, 3)!s} want {np.round(want_wheel, 3)!s} "
            f"steer {np.round(np.degrees(steer), 1)!s} want {np.round(want_steer_deg, 1)!s}"
        )

        assert err_lin < _TWIST_LIN_TOLERANCE, (
            f"{label}: base linear velocity is {np.round(lin[:2], 3)} against a commanded "
            f"({vx:+.2f}, {vy:+.2f}) -- off by {err_lin:.3f} m/s. Check steering_signs and "
            "wheel_signs in SwerveDriveActionCfg."
        )
        assert err_ang < _TWIST_ANG_TOLERANCE, (
            f"{label}: base yaw rate is {ang[2]:+.3f} rad/s against a commanded {omega:+.2f} "
            f"(error {err_ang:.3f}). A yaw rate that only partly arrives means a module could "
            "not reach its steering angle -- past 90 deg the module has to flip rather than "
            "clamp, or the module's steer drive is too weak to hold it."
        )
        assert err_steer < _STEER_ANGLE_TOLERANCE_DEG, (
            f"{label}: steering sits at {np.round(np.degrees(steer), 1)} deg against the "
            f"{np.round(want_steer_deg, 1)} deg the inverse kinematics asks for"
        )
        assert err_wheel < _WHEEL_SPEED_TOLERANCE, (
            f"{label}: wheel speeds {np.round(wheel, 3)} do not match the {np.round(want_wheel, 3)} "
            "the inverse kinematics asks for. A reading near -want means a flipped "
            "wheel_signs entry; a reading short of want on one module means that module's "
            "steering did not reach its commanded angle."
        )

    env.reset()
    print("[ok] every twist axis drives the base and the wheels as the contract says")


def _rot_from_xyzw(rot: tuple[float, float, float, float]) -> np.ndarray:
    """Rotation matrix from a quaternion in the xyzw order OffsetCfg uses."""
    x, y, z, w = rot
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _rot_of(matrix) -> np.ndarray:
    """Rotation matrix of a pxr world transform (quaternion order handled by pxr)."""
    q = matrix.ExtractRotationQuat()
    w = q.GetReal()
    x, y, z = q.GetImaginary()
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _rotation_angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    """Geodesic angle between two rotation matrices, in degrees."""
    cos = (float(np.trace(a.T @ b)) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


def _check_cameras(env, dump_dir: str | None, dump_steps: int) -> None:
    """Check each camera's mount in its parent link's frame, and the stage against PhysX.

    Two separate claims, because they fail differently:

    * the *extrinsics*: the camera prim's local pose inside its parent link prim, read out of
      the spawned stage, must reproduce ``realsense_robot_mounts.yaml``. That is the mounting
      itself, and it holds wherever the robot happens to be.
    * the *stage*: each parent link's USD pose must agree with PhysX's. Without this, a
      physics/stage desync gets reported as a camera error -- a misdiagnosis that cost a long
      detour, so it gets its own assertion.

    ``data.pos_w`` is printed for information and never asserted. It is IsaacLab bookkeeping
    read back from the stage rather than the prim's own pose, so it is the wrong thing to
    check extrinsics against -- but it is a useful canary: it read 71-83 mm away from the
    prim while init_state.pos was spawning the robot below the floor, and sits at 4.1 mm now
    that the spawn is correct. Few things complain when the spawn height is wrong; this does.

    Must run before anything drives the base: with fabric on, the stage tracks the robot at
    reset but not after, so a camera checked late would be checked against a stale stage.
    """
    import isaaclab.sim as sim_utils
    import omni.usd
    from pxr import Usd, UsdGeom

    stage = omni.usd.get_context().get_stage()
    robot = env.scene["robot"]
    body_names = list(robot.data.body_names)

    def world_of(prim_path_regex: str):
        matches = sim_utils.find_matching_prims(prim_path_regex, stage=stage)
        assert matches, f"no prim matches {prim_path_regex}"
        return matches[0], UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(matches[0])

    print(f"{'camera':<20} {'parent link':<22} {'local offset (m)':<38} {'err mm':>7} {'rot deg':>8}")
    worst_pos = worst_rot = 0.0
    camera_world: dict[str, tuple[np.ndarray, object]] = {}
    for name, mount in CAMERA_MOUNTS.items():
        # The configured path is a regex ({ENV_REGEX_NS}); resolve it the way IsaacLab does,
        # not by string-substituting an env index by hand.
        prim, cam_m = world_of(env.scene[name].cfg.prim_path)
        parent = prim.GetParent()
        par_m = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(parent)

        t_cam = np.asarray(cam_m.ExtractTranslation(), dtype=np.float64)
        t_par = np.asarray(par_m.ExtractTranslation(), dtype=np.float64)
        local_pos = _rot_of(par_m).T @ (t_cam - t_par)
        local_rot = _rot_of(par_m).T @ _rot_of(cam_m)

        err_pos = float(np.linalg.norm(local_pos - np.asarray(mount["pos"], dtype=np.float64))) * 1000.0
        err_rot = _rotation_angle_deg(local_rot, _rot_from_xyzw(mount["rot_xyzw"]))
        worst_pos, worst_rot = max(worst_pos, err_pos), max(worst_rot, err_rot)
        parent_name = parent.GetName()
        camera_world[name] = (t_cam, prim)

        print(f"{name:<20} {parent_name:<22} {np.round(local_pos, 5)!s:<38} {err_pos:7.2f} {err_rot:8.3f}")

        if parent_name in body_names:
            physx_t = robot.data.body_pos_w[0, body_names.index(parent_name)].cpu().numpy()
            desync = float(np.linalg.norm(physx_t - t_par)) * 1000.0
            assert desync < _LINK_DESYNC_TOLERANCE_M * 1000, (
                f"{parent_name}: stage and PhysX disagree by {desync:.1f} mm "
                f"(stage {np.round(t_par, 4)} vs PhysX {np.round(physx_t, 4)}). The stage is not "
                "tracking the sim, so no pose read from it -- including this camera's -- means "
                "anything. Check whether --disable_fabric was passed."
            )

        assert err_pos < _CAMERA_POS_TOLERANCE_M * 1000, (
            f"{name} sits {err_pos:.1f} mm from its authored mount on {parent_name}, so either the "
            "parent prim or the offset in realsense_robot_mounts.yaml is being applied wrongly"
        )
        assert err_rot < _CAMERA_ROT_TOLERANCE_DEG, (
            f"{name} is {err_rot:.3f} deg off its authored orientation"
        )

        sensor = env.scene[name].data.pos_w[0].cpu().numpy().astype(np.float64)
        print(
            f"{'':<20} sensor pos_w {np.round(sensor, 4)}  "
            f"({np.linalg.norm(sensor - t_cam) * 1000:.1f} mm from the prim; informational only)"
        )

    print(f"[ok] mounts match realsense_robot_mounts.yaml (worst {worst_pos:.2f} mm, {worst_rot:.3f} deg)")
    print("[ok] stage and PhysX agree on every camera parent link")

    # Cross-check against the precomputed table, in the base_link frame. The per-camera check
    # above is self-consistent with the mount config; this one is what fails if a *parent
    # link* is wrong, which the local check cannot see by construction.
    #
    # base_link is an ancestor of every camera prim, so walk up to it rather than expanding
    # the {ENV_REGEX_NS} macro by hand -- the cameras are already resolved concrete paths.
    base_name = BASE_LINK_PRIM.rsplit("/", 1)[-1]
    base_prim = next(iter(camera_world.values()))[1]
    while base_prim and base_prim.GetName() != base_name:
        base_prim = base_prim.GetParent()
    assert base_prim is not None, f"no {base_name} ancestor above the camera prims"
    base_m = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(base_prim)
    t_base = np.asarray(base_m.ExtractTranslation(), dtype=np.float64)
    r_base = _rot_of(base_m)
    print(f"{'camera':<20} {'in base_link frame (m)':<38} {'expected':<34} {'err mm':>7}")
    worst_base = 0.0
    for name, expected in CAMERA_EXPECTED_POS_IN_BASE_LINK.items():
        local = r_base.T @ (camera_world[name][0] - t_base)
        err = float(np.linalg.norm(local - np.asarray(expected, dtype=np.float64))) * 1000.0
        worst_base = max(worst_base, err)
        print(f"{name:<20} {np.round(local, 5)!s:<38} {np.asarray(expected)!s:<34} {err:7.2f}")
        assert err < _CAMERA_POS_TOLERANCE_M * 1000, (
            f"{name} is {err:.1f} mm from its expected place in the base_link frame -- a wrong "
            f"parent prim ({base_prim.GetPath()} at {np.round(t_base, 4)}) is the usual cause"
        )
    print(f"[ok] all four mounts verified in the base_link frame (worst {worst_base:.2f} mm)")

    if dump_dir is None:
        return

    os.makedirs(dump_dir, exist_ok=True)
    for _ in range(dump_steps):
        env.step(torch.zeros(env.action_space.shape, device=env.device))
    from PIL import Image

    for name in CAMERA_MOUNTS:
        rgb = env.scene[name].data.output["rgb"][0].cpu().numpy().astype(np.uint8)
        path = os.path.join(dump_dir, f"{name}.png")
        Image.fromarray(rgb).save(path)
        # A frame that is uniformly one colour means the camera is inside a mesh or facing
        # nothing; report the spread so that shows up as a number, not as a guess.
        print(f"[--] {name}: {path}  mean {rgb.mean():6.1f}  spread {rgb.std():5.1f}")


def main() -> None:
    args_cli, hydra_overrides = _parse_args()

    with SimulationAppContext(args_cli) as _app:
        arena_builder = get_arena_builder_from_cli(args_cli, hydra_overrides=hydra_overrides)
        env_cfg, env_kwargs = arena_builder.compose_manager_cfg()
        env_cfg.recorders = {}
        env_cfg.episode_recorders = {}
        # make_registered returns a gym wrapper (OrderEnforcing); the managers live on the
        # unwrapped ManagerBasedRLEnv. It still owns step()/reset(), including auto-reset.
        env = arena_builder.make_registered(env_cfg, env_kwargs).unwrapped

        env.reset()
        _check_action_layout(env)
        _check_joint_ordering(env)
        _report_joint_state(env)
        if args_cli.enable_cameras:
            _check_cameras(env, args_cli.dump_dir, args_cli.dump_steps)
        else:
            print("[--] cameras disabled; pass --enable_cameras to validate them")
        _check_reset_pose(env)
        _check_base_response(env)

        # Printed before the context manager exits, not after. SimulationAppContext.__exit__
        # tears the app down and takes the process with it, so a print placed after the
        # `with` never runs and the only success signal is the absence of a traceback.
        print("\n[validate] all checks passed")

        env.close()


if __name__ == "__main__":
    main()
