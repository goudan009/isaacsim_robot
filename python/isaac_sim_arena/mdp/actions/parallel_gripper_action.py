"""One action dimension driving both jaws of a parallel gripper.

The contract gives the gripper a single dimension per hand (a finger displacement in
metres, 0 = closed), but OpenFleX's USD models the two jaws as *independent* prismatic
joints -- there is no ``physxMimicJointAPI`` anywhere in the asset. Commanding only
``finger_joint1`` would therefore move one jaw and leave the other parked at its drive
target, i.e. closed. This term keeps the contract's 1-dim interface and drives both.
"""

from __future__ import annotations

from collections.abc import Sequence

import torch

from isaaclab.assets.articulation import Articulation
from isaaclab.managers.action_manager import ActionTerm


class ParallelGripperAction(ActionTerm):
    """Absolute finger displacement (metres) for a two-jaw parallel gripper.

    Action (1): the commanded jaw opening, clipped to ``[lower_limit, upper_limit]``.
    """

    cfg: ParallelGripperActionCfg  # noqa: F821 -- resolved at runtime, avoids an import cycle
    _asset: Articulation

    def __init__(self, cfg: ParallelGripperActionCfg, env):  # noqa: F821
        super().__init__(cfg, env)
        self._joint_ids, joint_names = self._asset.find_joints(list(self.cfg.joint_names), preserve_order=True)
        assert joint_names == list(self.cfg.joint_names), (
            f"gripper joints resolved out of declared order: {joint_names}"
        )
        self._signs = torch.tensor(self.cfg.finger_signs, dtype=torch.float32, device=self.device)
        self._raw_actions = torch.zeros(self.num_envs, self.action_dim, device=self.device)
        self._processed_actions = torch.zeros(self.num_envs, len(self._joint_ids), device=self.device)

    # -- ActionTerm interface ----------------------------------------------------------
    @property
    def action_dim(self) -> int:
        return 1

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    # -- Term logic --------------------------------------------------------------------
    def process_actions(self, actions: torch.Tensor) -> None:
        command = actions[:, :1].clamp(self.cfg.lower_limit, self.cfg.upper_limit)
        self._raw_actions[:] = actions
        self._processed_actions = command * self._signs.unsqueeze(0)

    def apply_actions(self) -> None:
        self._asset.actuators.target_command.set_position_index(
            value=self._processed_actions, joint_ids=self._joint_ids
        )

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        if env_ids is None:
            env_ids = slice(None)
        self._raw_actions[env_ids] = 0.0
