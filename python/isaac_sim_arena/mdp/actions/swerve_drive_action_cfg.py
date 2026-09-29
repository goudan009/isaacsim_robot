"""Configuration for the swerve base action term."""

from __future__ import annotations

from dataclasses import MISSING

from isaaclab.managers.action_manager import ActionTerm, ActionTermCfg
from isaaclab.utils.configclass import configclass

from isaac_sim_arena.constants import (
    MAX_ANGULAR_VELOCITY,
    MAX_LINEAR_VELOCITY,
    MAX_WHEEL_ANGULAR_VELOCITY,
    STEERING_LOWER_LIMIT,
    STEERING_UPPER_LIMIT,
    WHEEL_RADIUS_M,
)
from isaac_sim_arena.mdp.actions.swerve_drive_action import SwerveDriveAction


@configclass
class SwerveDriveActionCfg(ActionTermCfg):
    """Body twist -> swerve module commands.

    ``preserve_order`` is deliberately not exposed: it must always be ``True`` for this
    term, since the module order is defined by the contract and not by the articulation.
    Hard-coding it inside the term removes the opportunity to get it wrong.
    """

    class_type: type[ActionTerm] = SwerveDriveAction

    steering_joint_names: list[str] = MISSING
    wheel_joint_names: list[str] = MISSING
    module_positions_xy: list[tuple[float, float]] = MISSING
    wheel_radius: float = WHEEL_RADIUS_M

    # Actions are SI (m/s, m/s, rad/s) when normalize_actions stays False, so these are the
    # contract's own limits rather than a [-1, 1] remap.
    normalize_actions: bool = False
    max_linear_velocity: float = MAX_LINEAR_VELOCITY
    max_angular_velocity: float = MAX_ANGULAR_VELOCITY
    max_wheel_angular_velocity: float = MAX_WHEEL_ANGULAR_VELOCITY

    steering_lower_limit: float = STEERING_LOWER_LIMIT
    steering_upper_limit: float = STEERING_UPPER_LIMIT

    # Calibration escape hatches, in fl/fr/bl/br order. The wheel sign follows from the USD
    # (axis +Y in a steering frame whose localRot0 is identity, so +omega spins the base
    # along +X), but that is a derivation from prim transforms rather than a measurement --
    # hence a config field rather than a constant. Flip these if the base drives backwards.
    steering_signs: list[float] = [1.0, 1.0, 1.0, 1.0]
    wheel_signs: list[float] = [1.0, 1.0, 1.0, 1.0]
