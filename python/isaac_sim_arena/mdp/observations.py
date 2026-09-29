"""Observation terms specific to OpenFleX."""

from __future__ import annotations

import torch
import warp as wp

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg


class ContractOrderJointState:
    """Joint positions in an explicitly declared order.

    Droid's equivalent builds its index list by filtering ``robot.data.joint_names``, which
    yields *articulation* order. For OpenFleX that is wrong in a way that would be hard to
    notice: the USD authors the head as [pitch, yaw] and puts it last, so a filtered term
    would report head pitch before yaw and the head after the arms -- not the contract's
    [lift, yaw, pitch, arms..., grippers...]. Resolving through a name->index lookup in the
    declared order makes the term independent of the asset's internal ordering.
    """

    def __init__(
        self,
        joint_names: tuple[str, ...],
        asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    ) -> None:
        self._joint_names = tuple(joint_names)
        self._asset_cfg = asset_cfg
        self._indices: list[int] | None = None

    def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        robot = env.scene[self._asset_cfg.name]
        if self._indices is None:
            lookup = {name: i for i, name in enumerate(robot.data.joint_names)}
            missing = [name for name in self._joint_names if name not in lookup]
            assert not missing, f"joints missing from the articulation: {missing}"
            self._indices = [lookup[name] for name in self._joint_names]
        return wp.to_torch(robot.data.joint_pos)[:, self._indices]

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self._joint_names
