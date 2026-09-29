"""Configuration for the parallel gripper action term."""

from __future__ import annotations

from dataclasses import MISSING

from isaaclab.managers.action_manager import ActionTerm, ActionTermCfg
from isaaclab.utils.configclass import configclass

from isaac_sim_arena.constants import FINGER_LIMITS
from isaac_sim_arena.mdp.actions.parallel_gripper_action import ParallelGripperAction


@configclass
class ParallelGripperActionCfg(ActionTermCfg):
    """One absolute finger-displacement command spread over both jaws."""

    class_type: type[ActionTerm] = ParallelGripperAction

    joint_names: list[str] = MISSING
    lower_limit: float = FINGER_LIMITS[0]
    upper_limit: float = FINGER_LIMITS[1]

    # Per-jaw sign, in declared order. The two joints' localRot0 values are anti-parallel,
    # which should make [1, 1] open the jaws symmetrically, but that is a derivation from
    # prim transforms. The rest pose is unambiguously closed (the jaw collision meshes are
    # 0.4 mm apart at joint value 0) with 44 mm of travel each, so the correct setting is
    # whichever makes that gap grow -- flip here if a jaw closes when commanded to open.
    finger_signs: list[float] = [1.0, 1.0]
