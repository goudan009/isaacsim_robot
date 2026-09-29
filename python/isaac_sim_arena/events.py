"""OpenFleX event specification."""

from __future__ import annotations

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.utils.configclass import configclass

from isaaclab_arena.terms.events import reset_all_articulation_joints


def stash_teleop_env(env, env_ids) -> None:
    """Hand the live environment to the ROS teleop device, once, after the scene exists.

    Pass this to an event term with ``mode="startup"``. That mode is the only point in
    Arena's build where the scene is on the stage *and* the environment can still be
    configured -- see :func:`deactivate_prim_subtrees`, which needs it for the same reason.

    Why the device cannot simply reach the environment itself: ``record_demos.py`` builds
    the teleop device from ``env_cfg.teleop_devices``, and the only thing it passes
    alongside the config is a dict of button callbacks. The device is therefore constructed
    with no reference to ``env`` at all, and it needs one -- see
    :func:`isaac_sim_arena.teleop_ros.set_teleop_env` for what it does with it.

    Defined above the config class rather than beside the other helpers below it, because
    the class body needs the name bound at import time; the two helpers after the class are
    only ever referenced by name from an environment file.

    Args:
        env: The environment, stashed whole rather than picked apart. The device reads
            ``env.scene["robot"]`` for joint positions and ``env.step_dt`` for the control
            period, and a later need for ``num_envs`` or the episode counter is a read
            rather than another event term.
        env_ids: Unused. ``mode="startup"`` applies once for the whole stage.
    """
    # Imported here rather than at module scope: this module is imported while Arena's
    # environments are being registered, which is before Kit starts.
    from isaac_sim_arena import teleop_ros

    teleop_ros.set_teleop_env(env)


@configclass
class OpenFleXEventCfg:
    """Reset OpenFleX to its authored initial state at the start of every episode.

    This is not optional. The arm and gripper action terms use
    ``use_default_offset=False``, so nothing in the action path restores the arm pose on
    reset -- without this term the arms keep whatever pose the previous episode ended in.

    One consequence worth stating so it is not mistaken for a bug: the reset writes
    ``default_root_state`` offset by ``env_origins``, and OpenFleX's ``init_state.pos`` is
    the origin, so a rollout that *drives* the base snaps back on reset.
    """

    reset_robot = EventTerm(func=reset_all_articulation_joints, mode="reset")

    # The ROS teleop device needs the live articulation -- see `stash_teleop_env`. It lives
    # here, on the embodiment's event config, rather than in any one environment file,
    # because `--teleop_device ros` resolves against a *global* device registry: it can be
    # passed to any environment that uses this embodiment. A handoff written into a single
    # environment would leave the device articulation-less everywhere else, and the symptom
    # of that is not an error -- it is that both VR nodes go on silently publishing nothing,
    # because neither will move without /joint_states to seed its IK.
    teleop_ros_state_handoff = EventTerm(func=stash_teleop_env, mode="startup")


def deactivate_prim_subtrees(env, env_ids, subtree_paths) -> None:
    """Deactivate parts of the spawned scene, once, after the stage exists.

    Pass this to an event term with ``mode="startup"``. That mode is the only point in
    Arena's build where the scene is on the stage *and* the environment can still be
    configured: ``env_cfg_callback`` runs before Kit has spawned anything, so it can only
    edit configs, and nothing else in the build reaches back into a spawned background.

    Deactivating rather than hiding is deliberate. The point is for the subtree to stop
    being part of the scene rather than to be drawn see-through, and ``active = false``
    takes it out of any later physics or placement query at the same time. This is also
    what Arena itself does to stage prims it needs to change after spawning -- see
    ``_spawn_from_usd_with_resettable_nested_physics`` in
    ``isaaclab_arena/assets/background.py``, which walks a freshly spawned background and
    calls ``SetInstanceable(False)`` on instance roots.

    Args:
        env: The environment, read for its scene's prim paths -- a ``{ENV_REGEX_NS}``
            template only becomes a path once those are known.
        env_ids: Unused. ``mode="startup"`` applies once for the whole stage, and the
            subtrees below are deactivated in every environment because the template is
            expanded per environment rather than indexed by these.
        subtree_paths: Prim path templates to deactivate, each optionally containing
            ``{ENV_REGEX_NS}`` where the environment namespace goes.
    """
    # Imported here rather than at module scope: this module is imported while Arena's
    # environments are being registered, which is before Kit starts.
    from isaaclab.sim.utils import get_current_stage

    stage = get_current_stage()
    for template in subtree_paths:
        for env_prim_path in env.scene.env_prim_paths:
            # A plain replace, not str.format: the rest of a prim path may hold braces of
            # its own, and the macro is a literal substring rather than a format field.
            path = template.replace("{ENV_REGEX_NS}", env_prim_path)
            prim = stage.GetPrimAtPath(path)
            assert prim.IsValid(), (
                f"no prim at {path!r}: {template!r} does not match anything under {env_prim_path!r},"
                " so the asset it names did not spawn"
            )
            prim.SetActive(False)


def bind_material_to_prims(env, env_ids, material_prim_path, target_prim_paths) -> None:
    """Put a scene prim's material onto other prims, keeping one definition of the look.

    Pass this to an event term with ``mode="startup"``, for the same reason
    :func:`deactivate_prim_subtrees` needs it: it is the first point at which the spawned
    prims exist and can still be edited.

    The material is *bound*, not copied. Every target ends up pointing at the same
    ``Material`` prim the background ships, so there is one definition of the look and it
    stays the one Arena authored -- which is what lets the hydrogen cabinet wear the shelf's
    painted-steel material, maps and all, without this repository carrying its own copy of
    the MDL and the three textures behind it.

    The consequence to know before moving the source: the material prim has to stay active.
    Deactivating the subtree that holds it takes the look away from every prim bound to it,
    targets included -- so the shelf's ``Looks`` outlives the shelf.

    Args:
        env: The environment, read for its scene's prim paths.
        env_ids: Unused, as in :func:`deactivate_prim_subtrees`.
        material_prim_path: Prim path template of the ``Material`` to bind, optionally
            containing ``{ENV_REGEX_NS}`` where the environment namespace goes.
        target_prim_paths: Prim path templates to bind it onto, same templating.
    """
    from isaaclab.sim.utils import get_current_stage
    from pxr import UsdShade

    stage = get_current_stage()
    for env_prim_path in env.scene.env_prim_paths:
        # A plain replace, not str.format, for the reason given in deactivate_prim_subtrees.
        material_path = material_prim_path.replace("{ENV_REGEX_NS}", env_prim_path)
        material_prim = stage.GetPrimAtPath(material_path)
        assert material_prim.IsValid(), (
            f"no prim at {material_path!r}: {material_prim_path!r} does not match anything"
            f" under {env_prim_path!r}, so the material it names is not in the background"
        )
        assert material_prim.IsA(UsdShade.Material), (
            f"{material_path!r} is a {material_prim.GetTypeName()}, not a Material"
        )
        material = UsdShade.Material(material_prim)
        for template in target_prim_paths:
            path = template.replace("{ENV_REGEX_NS}", env_prim_path)
            target = stage.GetPrimAtPath(path)
            assert target.IsValid(), (
                f"no prim at {path!r}: {template!r} does not match anything under"
                f" {env_prim_path!r}, so the asset it names did not spawn"
            )
            # The binding is a relationship on the target, authored in the edit target's
            # layer, so it beats a weaker binding the target inherited from its own
            # reference rather than applying alongside it. Apply is what makes Bind legal
            # on a prim whose own file did not declare MaterialBindingAPI.
            UsdShade.MaterialBindingAPI.Apply(target).Bind(material)
