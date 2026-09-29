"""Body-frame twist action for a four-wheel independent-steering (swerve) base.

The contract (``openflex_isaac_contract/config/embodiment.yaml``) exposes the base as three
numbers, ``(linear.x, linear.y, angular.z)`` in the robot body frame, and states that the
swerve controller owns the inverse kinematics and the "combined twist wheel-speed clipping".
This term is that controller's sim-side equivalent.
"""

from __future__ import annotations

from collections.abc import Sequence

import torch

from isaaclab.assets.articulation import Articulation
from isaaclab.managers.action_manager import ActionTerm


class SwerveDriveAction(ActionTerm):
    """Inverse-kinematics action term for a swerve base.

    Action (3): ``(vx, vy, omega)`` in the body frame, SI units.
    Output: four steering position targets and four wheel velocity targets.

    Targets are written through the actuator command interface rather than
    ``write_joint_state_to_sim``, so the wheels stay under the velocity drive the USD
    authors (no stiffness, damping only) and the steering under its position drive.
    """

    cfg: SwerveDriveActionCfg  # noqa: F821 -- resolved at runtime, avoids an import cycle
    _asset: Articulation

    def __init__(self, cfg: SwerveDriveActionCfg, env):  # noqa: F821
        super().__init__(cfg, env)

        # Joint indices are resolved by name, preserving the order declared in the config.
        # This is not optional: the articulation interleaves steering and wheel joints
        # (fl_steering, fl_wheel, fr_steering, ...) and the module order is a property of
        # the contract, not of the USD. Indexing by articulation order would silently
        # scramble the modules.
        self._steer_ids, self._steer_names = self._asset.find_joints(
            list(self.cfg.steering_joint_names), preserve_order=True
        )
        self._wheel_ids, self._wheel_names = self._asset.find_joints(
            list(self.cfg.wheel_joint_names), preserve_order=True
        )
        assert self._steer_names == list(self.cfg.steering_joint_names), (
            f"steering joints resolved out of declared order: {self._steer_names}"
        )
        assert self._wheel_names == list(self.cfg.wheel_joint_names), (
            f"wheel joints resolved out of declared order: {self._wheel_names}"
        )

        self._module_xy = torch.tensor(self.cfg.module_positions_xy, dtype=torch.float32, device=self.device)
        self._steer_sign = torch.tensor(self.cfg.steering_signs, dtype=torch.float32, device=self.device)
        self._wheel_sign = torch.tensor(self.cfg.wheel_signs, dtype=torch.float32, device=self.device)

        self._raw_actions = torch.zeros(self.num_envs, self.action_dim, device=self.device)
        self._steer_target = torch.zeros(self.num_envs, len(self._steer_ids), device=self.device)
        self._wheel_target = torch.zeros(self.num_envs, len(self._wheel_ids), device=self.device)

    # -- ActionTerm interface ----------------------------------------------------------
    @property
    def action_dim(self) -> int:
        """Twist dimension: ``(linear.x, linear.y, angular.z)``."""
        return 3

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._raw_actions

    # -- Term logic --------------------------------------------------------------------
    def process_actions(self, actions: torch.Tensor) -> None:
        """Map a body twist onto four (steering angle, wheel speed) pairs.

        For module i at body-frame position ``r_i = (x_i, y_i)``::

            v_i   = (vx - omega * y_i, vy + omega * x_i)
            delta_i = atan2(v_i.y, v_i.x)
            omega_w,i = |v_i| / R

        A wheel drive is symmetric, so ``(delta, +w)`` and ``(delta - pi, -w)`` put the
        contact patch on exactly the same velocity. That freedom is not optional here: a
        module's steering range is +-90 deg, and for this geometry a pure yaw command asks
        the left-hand modules for +-142.5 deg, so without the flip-instead-of-clamp branch
        the base cannot rotate in place at any speed -- it saturates two modules and
        translates while it turns.

        ``atan2`` plus a non-negative speed is still the starting point, and the flip is
        applied per module, so the mapping stays memoryless: no state, no hysteresis, the
        same twist always yields the same targets.
        """
        self._raw_actions[:] = actions

        vx, vy, omega = actions[:, 0], actions[:, 1], actions[:, 2]
        # 1. Per-field twist clip (embodiment.yaml: clip_each_field_to_limits).
        vx = vx.clamp(-self.cfg.max_linear_velocity, self.cfg.max_linear_velocity)
        vy = vy.clamp(-self.cfg.max_linear_velocity, self.cfg.max_linear_velocity)
        omega = omega.clamp(-self.cfg.max_angular_velocity, self.cfg.max_angular_velocity)

        # 2. Swerve inverse kinematics: v_i = v + omega x r_i.
        x = self._module_xy[:, 0].unsqueeze(0)  # (1, 4)
        y = self._module_xy[:, 1].unsqueeze(0)
        vx_i = vx.unsqueeze(1) - omega.unsqueeze(1) * y
        vy_i = vy.unsqueeze(1) + omega.unsqueeze(1) * x

        delta = torch.atan2(vy_i, vx_i)
        wheel_speed = torch.hypot(vx_i, vy_i) / self.cfg.wheel_radius
        # 3. Combined-twist wheel-speed clip, which the contract assigns to this controller.
        # Applied to the magnitude, before the flip below, so a reversed wheel is bounded too.
        wheel_speed = wheel_speed.clamp(max=self.cfg.max_wheel_angular_velocity)

        # 4. Steering that would need more than 90 deg of travel is the same physical
        # solution as the opposite angle with the wheel reversed. Flip before clamping, or
        # the clamp would discard the commanded motion instead of re-expressing it.
        flip = delta.abs() > torch.pi / 2
        delta = torch.where(flip, delta - torch.sign(delta) * torch.pi, delta)
        wheel_speed = torch.where(flip, -wheel_speed, wheel_speed)

        self._steer_target = (self._steer_sign * delta).clamp(
            self.cfg.steering_lower_limit, self.cfg.steering_upper_limit
        )
        self._wheel_target = self._wheel_sign * wheel_speed

    def apply_actions(self) -> None:
        """Steering as position targets, wheels as velocity targets."""
        command = self._asset.actuators.target_command
        command.set_position_index(value=self._steer_target, joint_ids=self._steer_ids)
        command.set_velocity_index(value=self._wheel_target, joint_ids=self._wheel_ids)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        """Zero the command; the base then holds station instead of coasting."""
        if env_ids is None:
            env_ids = slice(None)
        self._raw_actions[env_ids] = 0.0
