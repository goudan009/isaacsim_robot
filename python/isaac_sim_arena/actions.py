"""The 22-dimensional OpenFleX action vector.

Layout, in declaration order, mirroring ``openflex_isaac_contract/config/embodiment.yaml``::

    [ 0: 3] base_twist              (linear.x, linear.y, angular.z)   swerve modules
    [ 3: 4] lift_position                                            lift_joint
    [ 4: 6] head_position           (yaw, pitch)
    [ 6:13] left_arm_position       j1..j7
    [13:14] left_gripper_position   one dim, both jaws
    [14:21] right_arm_position      j1..j7
    [21:22] right_gripper_position  one dim, both jaws

Declaration order *is* the action layout: ``ActionManager._prepare_terms`` iterates
``self.cfg.__dict__``, and ``@configclass`` is a dataclass, so fields are visited in source
order. Reordering this file reorders the policy interface.
"""

from __future__ import annotations

from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg
from isaaclab.managers import ActionTermCfg
from isaaclab.utils.configclass import configclass

from isaac_sim_arena.constants import (
    ACTION_JOINT_LIMITS,
    HEAD_JOINT_NAMES,
    LEFT_ARM_JOINT_NAMES,
    LEFT_FINGER_JOINT_NAMES,
    LIFT_JOINT_NAME,
    MODULE_POSITIONS_XY,
    RIGHT_ARM_JOINT_NAMES,
    RIGHT_FINGER_JOINT_NAMES,
    STEERING_JOINT_NAMES,
    WHEEL_JOINT_NAMES,
    WHEEL_RADIUS_M,
)
from isaac_sim_arena.mdp.actions.parallel_gripper_action_cfg import ParallelGripperActionCfg
from isaac_sim_arena.mdp.actions.swerve_drive_action_cfg import SwerveDriveActionCfg


def _limits(joint_names: tuple[str, ...]) -> dict[str, tuple[float, float]]:
    """Contract limits for a group, keyed by joint name (resolved by name, not position)."""
    return {name: ACTION_JOINT_LIMITS[name] for name in joint_names}


@configclass
class OpenFleXActionsCfg:
    """OpenFleX action specification: 3 + 1 + 2 + 7 + 1 + 7 + 1 = 22."""

    # [0:3] base_twist. Handled by the custom swerve term, since the contract hands the
    # inverse kinematics to the controller rather than exposing per-wheel actions.
    base_action: ActionTermCfg = SwerveDriveActionCfg(
        asset_name="robot",
        steering_joint_names=list(STEERING_JOINT_NAMES),
        wheel_joint_names=list(WHEEL_JOINT_NAMES),
        module_positions_xy=[list(xy) for xy in MODULE_POSITIONS_XY],
        wheel_radius=WHEEL_RADIUS_M,
    )

    # [3:4] lift_position.
    lift_action: ActionTermCfg = JointPositionActionCfg(
        asset_name="robot",
        joint_names=[LIFT_JOINT_NAME],
        preserve_order=True,
        use_default_offset=False,
        clip=_limits((LIFT_JOINT_NAME,)),
    )

    # [4:6] head_position. Declared [yaw, pitch] to match the contract; the USD authors
    # [pitch, yaw], so preserve_order is what keeps them from swapping.
    head_action: ActionTermCfg = JointPositionActionCfg(
        asset_name="robot",
        joint_names=list(HEAD_JOINT_NAMES),
        preserve_order=True,
        use_default_offset=False,
        clip=_limits(HEAD_JOINT_NAMES),
    )

    # [6:13] left_arm_position.
    left_arm_action: ActionTermCfg = JointPositionActionCfg(
        asset_name="robot",
        joint_names=list(LEFT_ARM_JOINT_NAMES),
        preserve_order=True,
        use_default_offset=False,
        clip=_limits(LEFT_ARM_JOINT_NAMES),
    )

    # [13:14] left_gripper_position: one dim spread over both jaws of the parallel gripper.
    left_gripper_action: ActionTermCfg = ParallelGripperActionCfg(
        asset_name="robot",
        joint_names=list(LEFT_FINGER_JOINT_NAMES),
    )

    # [14:21] right_arm_position.
    right_arm_action: ActionTermCfg = JointPositionActionCfg(
        asset_name="robot",
        joint_names=list(RIGHT_ARM_JOINT_NAMES),
        preserve_order=True,
        use_default_offset=False,
        clip=_limits(RIGHT_ARM_JOINT_NAMES),
    )

    # [21:22] right_gripper_position.
    right_gripper_action: ActionTermCfg = ParallelGripperActionCfg(
        asset_name="robot",
        joint_names=list(RIGHT_FINGER_JOINT_NAMES),
    )
