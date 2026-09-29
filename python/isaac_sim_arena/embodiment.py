"""OpenFleX as an IsaacLab-Arena embodiment.

The action space (22) and the policy state vector (19) follow this repository's own
``openflex_isaac_contract/config/embodiment.yaml``, so an agent trained against that
contract maps onto the sim without re-ordering or rescaling.
"""

from __future__ import annotations

from isaaclab_arena.assets.register import register_asset
from isaaclab_arena.embodiments.common.arm_mode import ArmMode
from isaaclab_arena.embodiments.embodiment_base import EmbodimentBase
from isaaclab_arena.relations.collision_mode import CollisionMode
from isaaclab_arena.utils.pose import Pose

from isaac_sim_arena.actions import OpenFleXActionsCfg
from isaac_sim_arena.cameras import OpenFleXCameraCfg
from isaac_sim_arena.events import OpenFleXEventCfg
from isaac_sim_arena.observations import OpenFleXObservationsCfg
from isaac_sim_arena.scene import OpenFleXSceneCfg


# Registered under `name` so Arena environments can select this robot by string, the same
# way they select "g1_wbc_pink" -- `asset_registry.get_asset_by_name("openflex")`. Without
# the decorator the class is only reachable by constructing it directly, which is what
# --external_environment_class_path does; that path is why the omission was not visible
# until an environment tried to look the robot up by name.
@register_asset
class OpenFleXEmbodiment(EmbodimentBase):
    """OpenFleX: a four-wheel independent-steering (swerve) base with a 19-DoF upper body.

    A free-standing mobile base, not a bolted-down arm: gravity acts on it and the root
    link is not fixed, so it can be driven around the scene by the base_twist action.
    """

    name = "openflex"
    tags = ["embodiment", "mobile_base", "dual_arm"]
    default_arm_mode = ArmMode.DUAL_ARM

    def __init__(
        self,
        enable_cameras: bool = False,
        initial_pose: Pose | None = None,
        concatenate_observation_terms: bool = False,
        arm_mode: ArmMode | None = None,
        collision_mode: CollisionMode | str | None = None,
    ):
        super().__init__(
            enable_cameras=enable_cameras,
            initial_pose=initial_pose,
            concatenate_observation_terms=concatenate_observation_terms,
            arm_mode=arm_mode,
            collision_mode=collision_mode,
        )
        self.scene_config = OpenFleXSceneCfg()
        self.action_config = OpenFleXActionsCfg()
        self.camera_config = OpenFleXCameraCfg()
        self.observation_config = OpenFleXObservationsCfg()
        self.event_config = OpenFleXEventCfg()
        self.add_camera_variations(self.camera_config)

    # -- Deliberate omissions ----------------------------------------------------------
    # reward_config / termination_cfg / command_config / curriculum_config stay None: no
    # task consumes them, and compose_manager_cfg skips None-valued configs.
    # mimic_env stays None: only the --mimic path reaches for it, and a mimic env would
    # need end-effector frames that this embodiment does not define.
    # No ee_frame FrameTransformerCfg either. get_ee_frame_name therefore returns "", which
    # is what the base class does; the honest default until wrist/finger frames are chosen.

    def get_ee_frame_name(self, arm_mode: ArmMode) -> str:
        return ""
