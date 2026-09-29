"""OpenFleX observation specification.

Camera terms are *not* declared here. ``EmbodimentBase.get_observation_cfg`` merges
``make_camera_observation_cfg(camera_config)`` into this config whenever cameras are
enabled, producing an ``observation["camera_obs"]`` group with one ``<name>_rgb`` term per
camera in :class:`~isaac_sim_arena.cameras.OpenFleXCameraCfg`.
"""

from __future__ import annotations

import isaaclab.envs.mdp as mdp_isaac_lab
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.utils.configclass import configclass

from isaac_sim_arena.constants import ROBOT_STATE_JOINT_NAMES
from isaac_sim_arena.mdp.observations import ContractOrderJointState


@configclass
class OpenFleXObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Policy-group observations (state only; cameras land in their own group)."""

        actions = ObsTerm(func=mdp_isaac_lab.last_action)

        # 19-dim, in embodiment.yaml order: lift(1) + head(2) + left arm(7) + left
        # gripper(1) + right arm(7) + right gripper(1). This is the key a GR00T/DreamZero
        # policy reads as ``observation["policy"]["robot_joint_pos"]`` and indexes with its
        # own state config, so the length and order are a policy-visible contract.
        robot_joint_pos = ObsTerm(func=ContractOrderJointState(joint_names=ROBOT_STATE_JOINT_NAMES))

        # Base feedback. Not consumed by any policy yet; kept because a mobile base that
        # reports no base state is very hard to debug when it drifts.
        base_lin_vel = ObsTerm(func=mdp_isaac_lab.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp_isaac_lab.base_ang_vel)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()
