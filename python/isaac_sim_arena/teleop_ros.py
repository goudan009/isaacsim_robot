"""OpenFleX teleoperation from ROS 2, as an Arena teleop device.

Why a *device* and not a separate control loop: Arena's recording entry point (Isaac Lab's
``record_demos.py``) and Arena's own evaluation (``isaaclab_arena/evaluation``) consume
nothing but the action tensor handed to ``env.step``. Anything that can produce that tensor
can be recorded and evaluated, so the ROS side belongs at the same seam ``keyboard`` and
``spacemouse`` use. The robot then keeps exactly one owner of its joints -- Arena's action
manager. Driving the articulation from a ROS OmniGraph instead would put two writers on the
same joints and make both recording and evaluation meaningless.

What is wired up
----------------
With ``enable_ros`` (the default) :meth:`RosTeleopDevice.advance` joins the ROS graph,
subscribes to the topics below, and writes all but three of the action dimensions:

===================  ==========================================  ==========================
action               topic                                       notes
===================  ==========================================  ==========================
``[0:3]``            ``/cmd_vel`` (``Twist``)                    swerve base, SI units
``[3:4]``            ``/lift_manual_position_controller/jog_command``  **integrated, see below**
``[4:6]``            ``/head_forward_position_controller/commands``  ``[yaw, pitch]``, not USD order
``[6:13]``           ``/left_forward_position_controller/commands``   first 7 of the payload
``[13:14]``          (same topic)                                element 8, only when present
``[14:21]``          ``/right_forward_position_controller/commands``  first 7
``[21:22]``          (same topic)                                element 8, only when present
===================  ==========================================  ==========================

The four controller topics carry ``std_msgs/Float64MultiArray`` and concatenate, in the
order above, to exactly ``constants.ROBOT_STATE_JOINT_NAMES`` -- which is action ``[3:22]``.
So there is no permutation table here, only slicing; the ordering work was already done by
whoever wrote those topics, and re-deriving it from the articulation would undo it. Two
places where that matters:

* the head payload is ``[yaw, pitch]``. The USD authors the opposite, and the action terms
  pass ``preserve_order=True`` for exactly this reason. Do not resolve head order by name
  lookup -- it is the one joint pair where the asset disagrees with the contract.
* the gripper element is a prismatic finger in **metres** in ``[0.0, 0.044]``, despite the
  contract YAML labelling it ``rad``. 0.0 is closed.

The arm payload length is dispatched on rather than assumed. Two builds of the arm node are
extant on this machine and they disagree: the installed one emits 7 elements (no gripper),
a newer source build emits 8. Both are accepted; with a 7-element payload the gripper
dimension simply keeps whatever ``self_test_action`` gave it.

The lift is the odd one out: it arrives as a signed *velocity* jog (``std_msgs/Float64``,
m/s), because the VR stack's lift node is a button-driven jog, and the real bringup
translates it to an absolute position by integrating at 100 Hz. That translator
(``isaacsim_compat_bridge.py``) does not run under Arena, so :meth:`_update_lift` does the
integration here.

One deliberate divergence from that translator: it re-anchors the target to the *measured*
lift position whenever the jog is idle, which is safe on hardware. Here it must not, because
``lift_joint`` is a prismatic drive carrying the entire upper body and settles under load,
so re-anchoring would integrate that settle error downward every step. The target is instead
anchored once, lazily, on the first ``advance`` of an episode.

Publishing state back out
-------------------------
The device publishes ``/joint_states``, and it is not optional. Both external VR nodes gate
on it and both fail *silently* without it: the arm node returns early from its control loop
when ``_get_current_joint_positions()`` yields ``None``, and the head node returns early
while ``joint_states_received`` is false. Neither logs anything. So if the upper body does
nothing at all, the first thing to check is this topic.

The positions come from the live articulation handed over by
:func:`isaac_sim_arena.teleop_ros.set_teleop_env`, called from a ``mode="startup"`` event
term in :mod:`isaac_sim_arena.events` -- the device itself never sees ``env``, because
``record_demos.py`` constructs it from a config plus a callback dict. Reading is
lazy-refreshed by the asset layer, so ``advance`` reading between steps is fine.

Published with the **default** QoS, deliberately. The common ROS 2 convention for
``/joint_states`` is BEST_EFFORT; both consumers here subscribe with defaults, which are
RELIABLE, and a RELIABLE subscriber will not match a BEST_EFFORT publisher -- the result is
zero data and no error on either side.

Two environmental requirements come with the ROS path:

* ``isaacsim.ros2.bridge`` must be enabled *after* the environment is built -- enabling it
  first pulls ``omni.replicator.core`` in before a stage exists and the build then dies
  with ``OmniGraphError: Failed to wrap graph in node '/Replicator/SDGPipeline'``. The
  device constructor is exactly the right place and no accident: ``record_demos.py``
  builds the environment before it builds the teleop device.
* the process must be launched through ``arena_ros.sh`` rather than ``arena.sh``, because
  the bundled Humble ``.so`` files carry no RPATH and ``import rclpy`` dies without
  ``LD_LIBRARY_PATH`` pointing at them.

Stage 1 -- the seam alone
-------------------------
Setting ``enable_ros`` false (or ``--self_test_action`` with ``enable_ros`` false) leaves
the device returning a constant action and subscribing to nothing, which exercises the
entire Arena-side seam -- device registry, the retargeter lookup, ``DevicesCfg``,
``create_teleop_device``, a per-step ``advance`` -- with every ROS variable removed.

Action layout
-------------
The 22 dimensions are OpenFleX's own, declared in :mod:`isaac_sim_arena.actions`::

    [ 0: 3] base_twist   [ 3: 4] lift   [ 4: 6] head
    [ 6:13] left arm     [13:14] left gripper
    [14:21] right arm    [21:22] right gripper

Registration
------------
Importing this module is what registers the device and its retargeter, so the environment
imports it for that side effect, the same way it imports :mod:`isaac_sim_arena.embodiment`
to put ``"openflex"`` in the asset registry. The retargeter is not optional: Arena's
``DeviceRegistry.get_teleop_device_cfg`` resolves a ``"<device>__<embodiment>"`` retargeter
unconditionally, *before* it consults the device, and asserts if the key is missing.
"""

from __future__ import annotations

import os
import time
from collections.abc import Callable
from dataclasses import field

import torch

from isaaclab.devices import DeviceBase, DeviceCfg
from isaaclab.utils.configclass import configclass

from isaaclab_arena.assets.device_library import TeleopDeviceBase
from isaaclab_arena.assets.register import register_device, register_retargeter
from isaaclab_arena.assets.retargeter_library import RetargetterBase

from isaac_sim_arena.constants import ACTION_JOINT_LIMITS, LIFT_JOINT_NAME, ROBOT_STATE_JOINT_NAMES

ACTION_DIM = 22
"""Width of the arena action vector, per :mod:`isaac_sim_arena.actions`."""

# Action slices, named so the write sites below read as the contract rather than as offsets.
BASE_TWIST_SLICE = slice(0, 3)
LIFT_INDEX = BASE_TWIST_SLICE.stop
HEAD_SLICE = slice(LIFT_INDEX + 1, LIFT_INDEX + 3)
LEFT_ARM_SLICE = slice(HEAD_SLICE.stop, HEAD_SLICE.stop + 7)
LEFT_GRIPPER_INDEX = LEFT_ARM_SLICE.stop
RIGHT_ARM_SLICE = slice(LEFT_GRIPPER_INDEX + 1, LEFT_GRIPPER_INDEX + 8)
RIGHT_GRIPPER_INDEX = RIGHT_ARM_SLICE.stop

# Element count of each controller topic's payload. The arm ones come in two lengths and are
# dispatched on; see the module docstring.
HEAD_COMMAND_SIZE = 2
ARM_COMMAND_SIZES = (7, 8)

# The jog velocity the VR lift node can emit is bounded well below the contract's travel; the
# clamp is copied from the bridge that used to do this translation, so the device and the
# bringup agree on what the hardware will accept.
LIFT_JOG_VELOCITY_LIMIT = 0.10

# Where the lift sits inside the state vector, so the one caller that wants it alone does not
# have to search the tuple each step.
LIFT_STATE_INDEX = ROBOT_STATE_JOINT_NAMES.index(LIFT_JOINT_NAME)


def _as_torch(value) -> torch.Tensor:
    """Return a tensor for an Isaac Lab asset buffer.

    The buffers became ``ProxyArray`` wrappers, which expose ``.torch``; ``warp.to_torch``
    still works on them but warns that it is deprecated, and ``mdp/observations.py`` predates
    that. Prefer the accessor and keep the old call as the fallback.
    """
    accessor = getattr(value, "torch", None)
    if accessor is not None:
        return accessor
    import warp as wp

    return wp.to_torch(value)


_TELEOP_ENV = None
"""The environment handed over by the ``mode="startup"`` event term in ``isaac_sim_arena.events``.

Module-level because there is no other channel: ``record_demos.py`` builds the device from a
config plus a callback dict, so the constructor has no ``env`` argument to thread this
through. Set once per process, before the device is built.
"""


def set_teleop_env(env) -> None:
    """Record the live environment for :class:`RosTeleopDevice` to read state from.

    Called from :func:`isaac_sim_arena.events.stash_teleop_env`, which an event term runs at
    ``mode="startup"``. Overwritten on each startup so a stale environment from an earlier
    ``SimulationApp`` in the same process can never be read.
    """
    global _TELEOP_ENV
    _TELEOP_ENV = env
    print(
        f"RosTeleopDevice: environment handed over (num_envs={getattr(env, 'num_envs', '?')},"
        f" step_dt={getattr(env, 'step_dt', '?')})",
        flush=True,
    )


def teleop_env():
    """Return the environment recorded by :func:`set_teleop_env`, or ``None`` if never set."""
    return _TELEOP_ENV


class RosTeleopDevice(DeviceBase):
    """Arena teleop device whose commands come from ROS 2.

    With ``enable_ros`` the device subscribes to the base twist topic and the four upper-body
    command topics, and publishes ``/joint_states`` back out -- see the module docstring for
    the full mapping and for why that publish is not optional. Dimensions with nothing
    cached keep whatever ``self_test_action`` gave them. With ``enable_ros`` off, ``advance``
    just repeats that constant.

    The batch axis is part of the contract, not an accident: ``record_demos.py`` calls
    ``action.repeat(env.num_envs, 1)`` on whatever this returns, so it must be 2-D of shape
    ``(1, ACTION_DIM)``. A flat ``(ACTION_DIM,)`` tensor raises there.
    """

    # Bound on the drain loop in advance(). One is the floor (a spin_once with nothing ready),
    # and the ceiling only has to clear the messages that accumulate in one step: the arm node
    # publishes at 100 Hz on two topics, the head at 50 Hz, the chassis at ~30 Hz, and a wall
    # clock step is ~40 ms at the measured real-time factor of 0.5 -- call it 10 queued
    # messages, so this is roughly 3x headroom. Raising it costs nothing when idle, because
    # the loop breaks on the first call that processes no callback.
    _MAX_CALLBACKS_PER_ADVANCE = 32

    def __init__(self, cfg: RosDeviceCfg):
        """Initialize the device from its configuration.

        Args:
            cfg: Device configuration. ``create_teleop_device`` always passes this by keyword
                and passes ``retargeters`` only for devices that declare the parameter, which
                this one deliberately does not.
        """
        super().__init__(retargeters=None)
        self._cfg = cfg
        self._additional_callbacks: dict = {}

        self._action = torch.zeros(ACTION_DIM, dtype=torch.float32, device=cfg.sim_device)
        if len(cfg.self_test_action) > 0:
            assert len(cfg.self_test_action) == ACTION_DIM, (
                f"self_test_action has {len(cfg.self_test_action)} entries, expected {ACTION_DIM}"
            )
            self._action = torch.tensor(cfg.self_test_action, dtype=torch.float32, device=cfg.sim_device)
        self._rest_action = self._action.clone()

        self._rclpy = None
        self._node = None
        self._executor = None

        # Newest command per topic, or None when the topic has not spoken yet. Kept as plain
        # Python rather than tensors so the callbacks stay trivial and never touch the action.
        self._latest_twist = None
        self._left_arm_cmd: list[float] | None = None
        self._right_arm_cmd: list[float] | None = None
        self._left_gripper_cmd: float | None = None
        self._right_gripper_cmd: float | None = None
        self._head_cmd: list[float] | None = None
        self._lift_velocity = 0.0

        # None until the first advance() of an episode, which is what makes the lift anchor to
        # the measured position without depending on when reset() is called relative to
        # env.reset(). See _update_lift.
        self._lift_target: float | None = None

        # Name -> index into the articulation's own joint order, resolved on first use.
        self._joint_indices: list[int] | None = None

        # Instrumentation. Both VR nodes fail silently, so the device is the only place that
        # can tell "nothing is arriving" apart from "plenty is arriving and the action is not
        # moving", which are very different bugs. See _report.
        self._rx_counts: dict[str, int] = {}
        self._topic_labels = {
            cfg.base_twist_topic: "cmd_vel",
            cfg.left_arm_topic: "left_arm",
            cfg.right_arm_topic: "right_arm",
            cfg.head_topic: "head",
            cfg.lift_jog_topic: "lift",
        }
        self._tx_count = 0
        self._bad_payloads: list[tuple[str, int]] = []
        self._last_report = time.monotonic()

        self._joint_pub = None
        if cfg.enable_ros:
            self._join_ros_graph()

    def __str__(self) -> str:
        return "RosTeleopDevice"

    def reset(self):
        """Drop every cached command so an episode never starts on the previous one's motion.

        The arm and gripper action terms use ``use_default_offset=False``, so an action
        dimension the device does not rewrite is whatever it already held -- restoring
        ``_rest_action`` here is what returns the arms to the reset pose instead of leaving
        them wherever the last episode pointed them.

        The lift target is dropped rather than re-anchored: it is re-read from the measured
        position on the next ``advance``, which is a point where the articulation has
        definitely been reset already.
        """
        self._latest_twist = None
        self._left_arm_cmd = None
        self._right_arm_cmd = None
        self._left_gripper_cmd = None
        self._right_gripper_cmd = None
        self._head_cmd = None
        self._lift_velocity = 0.0
        self._lift_target = None
        self._action = self._rest_action.clone()

    def add_callback(self, key, func: Callable):
        """Accept a button/gesture callback without acting on it.

        Both Arena and ``record_demos.py`` attach callbacks to whatever device they build, so
        the call has to succeed even though this device has no buttons yet.
        """
        self._additional_callbacks[key] = func

    def advance(self) -> torch.Tensor:
        """Return the pending command as a fresh ``(1, ACTION_DIM)`` tensor.

        A clone per call, so the action manager can do as it likes with the tensor it is
        handed without corrupting the next step's command.

        Note that this runs on every step even while recording is paused -- ``record_demos``
        calls it unconditionally -- which is what keeps the drain and the ``/joint_states``
        publish going. That matters more than it looks: both VR nodes latch their teleop
        anchor from ``/joint_states`` and publish nothing at all until they have one, so a
        device that went quiet while paused would come back to two nodes that had never
        started.
        """
        if self._rclpy is None:
            return self._action.unsqueeze(0).clone()

        self._drain()
        self._publish_joint_states()

        if self._latest_twist is not None:
            self._action[BASE_TWIST_SLICE] = torch.tensor(
                [
                    self._latest_twist.linear.x,
                    self._latest_twist.linear.y,
                    self._latest_twist.angular.z,
                ],
                dtype=torch.float32,
                device=self._action.device,
            )

        self._update_lift()
        if self._head_cmd is not None:
            self._action[HEAD_SLICE] = torch.tensor(
                self._head_cmd, dtype=torch.float32, device=self._action.device
            )
        for source, target in (
            (self._left_arm_cmd, LEFT_ARM_SLICE),
            (self._right_arm_cmd, RIGHT_ARM_SLICE),
        ):
            if source is not None:
                self._action[target] = torch.tensor(source, dtype=torch.float32, device=self._action.device)
        # Grippers are written only when the arm node sent the 8-element payload; a 7-element
        # one says nothing about the gripper, and leaving the dimension alone holds the rest
        # pose rather than inventing a value. See the module docstring.
        if self._left_gripper_cmd is not None:
            self._action[LEFT_GRIPPER_INDEX] = self._left_gripper_cmd
        if self._right_gripper_cmd is not None:
            self._action[RIGHT_GRIPPER_INDEX] = self._right_gripper_cmd

        self._report()
        return self._action.unsqueeze(0).clone()

    def _drain(self) -> int:
        """Run every callback that is ready right now, and return how many were processed.

        Looping rather than calling ``spin_once`` once is required, not tidiness: a single
        ``spin_once`` processes at most one ready callback, and the chassis node publishes
        its idle zero-twist stream at ~30 Hz even with no headset connected. Serving that one
        message per step would starve the four upper-body subscriptions permanently, and the
        symptom would be a base that drives perfectly while the arms never move.

        With no ROS traffic at all this costs exactly one ``spin_once`` that returns nothing,
        so it cannot regress the chassis-only path that was already verified.

        A zero timeout is deliberate: a wall-clock budget would add latency to a 40 ms step
        for no benefit, and a budget-expired exit is indistinguishable from a nothing-ready
        exit, which would make the return value useless for diagnostics.
        """
        processed = 0
        for _ in range(self._MAX_CALLBACKS_PER_ADVANCE):
            before = sum(self._rx_counts.values())
            self._executor.spin_once(timeout_sec=0.0)
            if sum(self._rx_counts.values()) == before:
                break
            processed += 1
        return processed

    def _step_dt(self) -> float:
        """Control period in seconds, from the environment when it has been handed over."""
        env = teleop_env()
        step_dt = getattr(env, "step_dt", None) if env is not None else None
        return float(step_dt) if step_dt else float(self._cfg.control_dt)

    def _update_lift(self) -> None:
        """Integrate the jog velocity into the absolute lift position the action expects.

        The action dimension is an absolute joint position in metres; the VR lift node emits
        a signed velocity that is only published when it *changes*. So the target has to be
        integrated here, in the device.

        Integration is in **simulation** time, which makes the result deterministic and
        replayable. At the measured real-time factor of ~0.5 the operator perceives the lift
        moving at about half the speed it would on hardware -- that is expected, and it is
        the same half-speed the rest of the scene runs at, not a bug in this integrator.

        The target is created by the first jog of an episode, anchored to the measured
        position rather than to zero, and holds its value once the jog returns to zero --
        which is what makes a released button park the lift instead of dropping it. Same rule
        as the other ROS-driven dimensions: the action is not touched until the topic that
        owns it has actually said something, so a ``self_test_action`` value for the lift is
        preserved until then.

        Nothing re-anchors after that, deliberately -- see the module docstring for why
        copying the real bringup's re-anchor-on-idle rule would be wrong here.
        """
        if self._lift_target is None and abs(self._lift_velocity) <= 1e-9:
            return
        lower, upper = ACTION_JOINT_LIMITS[LIFT_JOINT_NAME]
        if self._lift_target is None:
            measured = self._measured_lift()
            self._lift_target = 0.0 if measured is None else measured
        if abs(self._lift_velocity) > 1e-9:
            # Clamped here rather than only by the action term's clip, so the integrator
            # cannot wind up past the endstop and then have to unwind.
            self._lift_target = min(upper, max(lower, self._lift_target + self._lift_velocity * self._step_dt()))
        self._action[LIFT_INDEX] = self._lift_target

    def _measured_lift(self) -> float | None:
        """Current ``lift_joint`` position from the sim, or ``None`` if there is no state yet."""
        positions = self._contract_joint_positions()
        if positions is None:
            return None
        return positions[LIFT_STATE_INDEX]

    def _join_ros_graph(self):
        """Enable the ROS 2 bridge, then subscribe to every command topic and publish state.

        The order is the whole point of doing this in the constructor: the bridge has to be
        enabled after the environment is built, and ``record_demos.py`` builds the
        environment before it builds the device. Enabling it earlier drags
        ``omni.replicator.core`` in before a stage exists and the build dies.
        """
        import omni.kit.app

        app = omni.kit.app.get_app()
        try:
            from isaacsim.core.experimental.utils.impl.app import enable_extension

            enable_extension("isaacsim.ros2.bridge")
        except Exception:
            app.get_extension_manager().set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
        # The extension puts rclpy on sys.path during its own startup, so this has to be
        # pumped before the import below can succeed.
        for _ in range(10):
            app.update()

        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Float64, Float64MultiArray
        except ImportError as exc:
            raise RuntimeError(
                f"enable_ros is set but rclpy could not be imported ({exc}). The bundled ROS 2"
                " libraries have no RPATH, so this process needs LD_LIBRARY_PATH pointing at"
                " them: launch through ./arena_ros.sh instead of ./arena.sh."
            ) from exc

        rclpy.init(args=[])
        self._node = rclpy.create_node("openflex_arena_ros_device")

        # Own executor rather than rclpy.spin_once, which would add and remove the node on
        # every call. Added once, here, and only ever driven from advance().
        from rclpy.executors import SingleThreadedExecutor

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        # Depth 1 on the command topics: only the newest command is ever wanted, and a deeper
        # queue would just give the drain loop more stale messages to walk through.
        self._node.create_subscription(Twist, self._cfg.base_twist_topic, self._on_twist, 1)
        self._node.create_subscription(
            Float64MultiArray, self._cfg.left_arm_topic, self._on_left_arm, 1
        )
        self._node.create_subscription(
            Float64MultiArray, self._cfg.right_arm_topic, self._on_right_arm, 1
        )
        self._node.create_subscription(Float64MultiArray, self._cfg.head_topic, self._on_head, 1)
        self._node.create_subscription(Float64, self._cfg.lift_jog_topic, self._on_lift, 1)

        # Default QoS on purpose -- see the module docstring. A BEST_EFFORT publisher here
        # would not match the RELIABLE subscribers on the other side, silently.
        self._joint_pub = self._node.create_publisher(JointState, self._cfg.joint_states_topic, 10)
        self._rclpy = rclpy

        # print, not logger.info: the arena env cfg sets logging_level='WARNING', so an info
        # line would be invisible in exactly the runs where this matters. Which topics and
        # which rmw are live is the first thing to check when a command never arrives --
        # and the domain is on it because domain 0 carries a full MuJoCo ros2_control stack
        # driving a *different* robot, publishing to these very same controller topics.
        print(
            f"RosTeleopDevice: listening on {self._cfg.base_twist_topic},"
            f" {self._cfg.left_arm_topic}, {self._cfg.right_arm_topic}, {self._cfg.head_topic},"
            f" {self._cfg.lift_jog_topic}; publishing {self._cfg.joint_states_topic}"
            f" (rmw={rclpy.get_rmw_implementation_identifier()}, node={self._node.get_name()},"
            f" ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', 'unset (domain 0)')})",
            flush=True,
        )

    def _on_twist(self, msg):
        """Cache the newest base twist. The callback runs inside ``advance``'s drain loop."""
        self._latest_twist = msg
        self._count(self._cfg.base_twist_topic)

    def _on_left_arm(self, msg):
        """Cache the newest left arm command and its gripper element, if it carries one."""
        self._store_arm(msg, "left")

    def _on_right_arm(self, msg):
        """Cache the newest right arm command and its gripper element, if it carries one."""
        self._store_arm(msg, "right")

    def _store_arm(self, msg, side: str) -> None:
        """Split an arm payload into seven joint angles and an optional gripper position.

        The payload length is not fixed: the arm node has two extant builds, one emitting
        seven elements and one emitting eight with the gripper appended. Anything else is
        recorded and ignored rather than raised, because a raise here would surface as a
        dead control loop in the sim rather than as the operator's misconfiguration it is.
        """
        topic = self._cfg.left_arm_topic if side == "left" else self._cfg.right_arm_topic
        data = list(msg.data)
        self._count(topic)
        if len(data) not in ARM_COMMAND_SIZES:
            self._bad_payloads.append((side, len(data)))
            return
        if side == "left":
            self._left_arm_cmd = data[:7]
            if len(data) == 8:
                self._left_gripper_cmd = data[7]
        else:
            self._right_arm_cmd = data[:7]
            if len(data) == 8:
                self._right_gripper_cmd = data[7]

    def _on_head(self, msg):
        """Cache the newest head command. Order is ``[yaw, pitch]``; see the module docstring."""
        self._count(self._cfg.head_topic)
        data = list(msg.data)
        if len(data) != HEAD_COMMAND_SIZE:
            self._bad_payloads.append(("head", len(data)))
            return
        self._head_cmd = data

    def _on_lift(self, msg):
        """Cache the newest lift jog velocity. Integrated into a position in ``_update_lift``."""
        self._count(self._cfg.lift_jog_topic)
        self._lift_velocity = max(-LIFT_JOG_VELOCITY_LIMIT, min(LIFT_JOG_VELOCITY_LIMIT, float(msg.data)))

    def _count(self, topic: str) -> None:
        """Record that one message arrived on ``topic``. Read by the drain loop and the report."""
        self._rx_counts[topic] = self._rx_counts.get(topic, 0) + 1

    def _contract_joint_positions(self) -> list[float] | None:
        """Joint positions in contract order, or ``None`` when there is no state to read.

        ``None`` covers both "the environment was never handed over" and "the articulation is
        not there yet", and both are ordinary early in a run rather than errors -- the caller
        treats them as "publish nothing this step".

        The articulation's own joint order is not the contract order: the USD authors the head
        last and with pitch before yaw, so an index-free read would silently transpose the
        head. Resolved once by name, the same way ``mdp/observations.py`` does it.
        """
        env = teleop_env()
        if env is None:
            return None
        robot = env.scene["robot"]
        if self._joint_indices is None:
            lookup = {name: i for i, name in enumerate(robot.data.joint_names)}
            missing = [name for name in ROBOT_STATE_JOINT_NAMES if name not in lookup]
            assert not missing, (
                f"the articulation is missing joints the contract publishes: {missing}. The"
                " device cannot publish /joint_states without them, and both VR nodes will"
                " then refuse to publish anything at all."
            )
            self._joint_indices = [lookup[name] for name in ROBOT_STATE_JOINT_NAMES]
        joint_pos = robot.data.joint_pos
        # Read fresh each call rather than cached: the accessor hands back a zero-copy view,
        # and the asset layer can rebind the buffer it points at.
        values = _as_torch(joint_pos)[0]
        return [float(values[i]) for i in self._joint_indices]

    def _publish_joint_states(self) -> None:
        """Publish the upper body's measured state, which is what opens the VR nodes' gates.

        Only the 19 contract joints are published. The wheels and steering columns are
        deliberately absent -- nothing in the teleop chain reads them, and adding them would
        mean a second ordering to keep in step with the contract.
        """
        positions = self._contract_joint_positions()
        if positions is None:
            return
        from sensor_msgs.msg import JointState

        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = list(ROBOT_STATE_JOINT_NAMES)
        msg.position = positions
        self._joint_pub.publish(msg)
        self._tx_count += 1

    def _report(self) -> None:
        """Print one status line a second.

        Unconditional rather than on-change, because the thing worth seeing is the absence of
        traffic. The failure mode this exists for: both VR nodes gate on state they do not
        announce, so "the arms do not move" looks identical whether the commands never arrived
        or arrived perfectly and something else is wrong. Per-second counts separate those in
        one line, which is the difference between a config problem and a sim problem.
        """
        now = time.monotonic()
        if now - self._last_report < 1.0:
            return
        self._last_report = now
        # Per second, not cumulative, so a topic that has gone quiet reads as 0 rather than as
        # a plausible-looking total.
        received = ", ".join(
            f"{self._topic_labels[topic]}={self._rx_counts.get(topic, 0)}"
            for topic in self._topic_labels
        )
        lift = f"{self._lift_target:.3f}" if self._lift_target is not None else "unset"
        line = (
            f"RosTeleopDevice: rx/s[{received}] /joint_states={self._tx_count} lift={lift}"
        )
        if self._bad_payloads:
            line += f" REJECTED={self._bad_payloads}"
        print(line, flush=True)
        self._rx_counts.clear()
        self._bad_payloads.clear()


@configclass
class RosDeviceCfg(DeviceCfg):
    """Configuration for the ROS 2 teleoperation device."""

    # Overridden to None, as Se3KeyboardCfg does, to keep create_teleop_device from trying to
    # build the Isaac Lab RetargeterBase entries this config would otherwise inherit.
    retargeters: None = None
    # A string rather than the class object: this is how upstream device configs name their
    # device, and {DIR} resolves to this module's package at construction time.
    class_type: type | str = "{DIR}.teleop_ros:RosTeleopDevice"

    enable_ros: bool = True
    """Subscribe to the ROS topics and publish state. False leaves the device on the constant action."""

    base_twist_topic: str = "/cmd_vel"
    """Topic carrying the swerve base twist: ``geometry_msgs/Twist`` in SI units."""

    # The four upper-body command topics, named exactly as the ros2_control controllers and the
    # VR nodes name them. Overriding one is a knob for testing against a re-topic'd stack, not
    # a supported configuration: the payload order is the contract's, not the topic's.
    head_topic: str = "/head_forward_position_controller/commands"
    """Head command: ``std_msgs/Float64MultiArray`` of 2, ``[yaw, pitch]`` in radians."""

    left_arm_topic: str = "/left_forward_position_controller/commands"
    """Left arm command: ``std_msgs/Float64MultiArray`` of 7 (or 8 with the gripper appended)."""

    right_arm_topic: str = "/right_forward_position_controller/commands"
    """Right arm command: same shape as the left."""

    lift_jog_topic: str = "/lift_manual_position_controller/jog_command"
    """Lift command: ``std_msgs/Float64``, a signed jog **velocity** in m/s, not a position."""

    joint_states_topic: str = "/joint_states"
    """Where the measured state is published. Both VR nodes read it by this name."""

    control_dt: float = 0.02
    """Control period in seconds, used to integrate the lift jog.

    Only a fallback: once the environment has been handed over, ``env.step_dt`` is read
    instead. Arena's environments run at 50 Hz, which is where this default comes from.
    """

    # Constant action used for every dimension the ROS side does not drive yet, and for all
    # of them when enable_ros is false. Left empty it means "hold still": all zeros. A list
    # with default_factory, not a tuple, to match how DeviceCfg declares its own retargeters.
    self_test_action: list[float] = field(default_factory=list)


@register_retargeter
class RosOpenFleXRetargeter(RetargetterBase):
    """Pass-through retargeter satisfying Arena's ``("ros", "openflex")`` registry key.

    Arena looks this up unconditionally before consulting the device, so it has to exist even
    though there is nothing to retarget: this device already emits the 22-dim Arena action
    vector, and returning ``None`` here is the same answer ``FrankaKeyboardRetargeter`` gives
    for the keyboard.
    """

    device = "ros"
    embodiment = "openflex"

    def __init__(self):
        pass

    def get_pipeline_builder(self, embodiment: object) -> Callable | None:
        """Return no pipeline builder: the device's output needs no retargeting."""
        return None


@register_device
class RosTeleopCfg(TeleopDeviceBase):
    """Arena device entry named ``ros``, selected with ``--teleop_device ros``.

    The name is also half of the retargeter key above, so renaming it means renaming
    ``RosOpenFleXRetargeter.device`` in the same breath.
    """

    name = "ros"

    def __init__(self, sim_device: str | None = None):
        super().__init__(sim_device)

    def get_device_cfg(
        self, pipeline_builder: Callable | None = None, embodiment: object | None = None
    ) -> DeviceCfg:
        """Build the Isaac Lab device config for this device.

        Args:
            pipeline_builder: Ignored. It exists for devices that hand IsaacTeleop a
                retargeting pipeline, and this one produces joint-space actions itself.
            embodiment: Ignored, for the same reason.

        Returns:
            The device config Arena routes into ``env_cfg.teleop_devices``.
        """
        return RosDeviceCfg(sim_device=self.sim_device or "cpu")
