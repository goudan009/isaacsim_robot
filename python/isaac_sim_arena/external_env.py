"""Task-free OpenFleX environment, loaded into IsaacLab-Arena from outside the tree.

Arena's only out-of-tree hook is ``--external_environment_class_path`` (there is no
equivalent for embodiments): the module named here is imported at CLI-parse time and the
embodiment it constructs registers by simply being instantiated. That is also why every
Arena/IsaacLab import in this file lives *inside* ``get_env`` -- the module is imported
before Kit starts, so a top-level ``import isaaclab`` would pull in pxr/warp too early.

Launch::

    ./arena.sh isaaclab_arena/scripts/environment_runner.py \\
        --viz kit --num_envs 1 --device cpu --enable_cameras \\
        --external_environment_class_path \\
          isaac_sim_arena.external_env:OpenFleXExternalEnvironment \\
        openflex_arena
"""

from __future__ import annotations

import argparse

from isaaclab_arena_environments.example_environment_base import ExampleEnvironmentBase

from isaac_sim_arena.constants import BASE_LINK_REST_HEIGHT, SPAWN_CLEARANCE_M


class OpenFleXExternalEnvironment(ExampleEnvironmentBase):
    """Ground plane, dome light and OpenFleX. No task, no objects -- just the robot."""

    name: str = "openflex_arena"

    def get_env(self, args_cli: argparse.Namespace):
        from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
        from isaaclab_arena.scene.scene import Scene
        from isaaclab_arena.tasks.no_task import NoTask
        from isaaclab_arena.utils.pose import Pose

        from isaac_sim_arena.embodiment import OpenFleXEmbodiment

        ground_plane = self.asset_registry.get_asset_by_name("ground_plane")()
        light = self.asset_registry.get_asset_by_name("light")()

        # The floor goes at env-local z = 0. GroundPlane's prim path is global and env_0's
        # origin is the world origin.
        ground_plane.set_initial_pose(Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))

        embodiment = OpenFleXEmbodiment(
            # Forwarded explicitly: the legacy get_env path does not read --enable_cameras
            # for the embodiment, and without it the camera configs and their observation
            # terms are silently dropped.
            enable_cameras=args_cli.enable_cameras,
            # z is the rest height of the articulation root, which for this asset is
            # base_link -- authored 0.17 m above the USD origin that sits on the floor.
            # Spawning the root at 0 instead buries the wheels 17 cm into the ground and
            # PhysX ejects the robot on every reset; see constants.BASE_LINK_REST_HEIGHT.
            #
            # This value, not scene.py's init_state.pos, is the one that takes effect:
            # EmbodimentBase._update_scene_cfg_with_robot_initial_pose overwrites
            # init_state.pos with whatever this Pose carries. The two are kept equal so a
            # direct OpenFleXEmbodiment() spawn lands in the same place.
            initial_pose=Pose(
                position_xyz=(0.0, 0.0, BASE_LINK_REST_HEIGHT + SPAWN_CLEARANCE_M),
                rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
            ),
        )

        scene = Scene(assets=[ground_plane, light])

        return IsaacLabArenaEnvironment(
            name=self.name,
            embodiment=embodiment,
            scene=scene,
            task=NoTask(),
        )

    @staticmethod
    def add_cli_args(parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--openflex_start_joint_pose",
            type=float,
            nargs="+",
            default=None,
            help="Optional 19-joint start pose in contract order; default is the USD rest pose.",
        )
