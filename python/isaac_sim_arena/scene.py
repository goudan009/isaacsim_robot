"""Scene contribution of the OpenFleX embodiment: the robot articulation itself."""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation.articulation_cfg import ArticulationCfg
from isaaclab.utils.configclass import configclass

from isaac_sim_arena.constants import (
    BASE_LINK_REST_HEIGHT,
    MAX_WHEEL_ANGULAR_VELOCITY,
    ROBOT_PRIM_PATH,
    ROBOT_USD_PATH,
    SPAWN_CLEARANCE_M,
)


@configclass
class OpenFleXSceneCfg:
    """Additions to the scene configuration coming from the OpenFleX embodiment.

    The actuator groups mirror the drive split the USD already authors -- the steering and
    upper-body joints have position drives, the wheels have a pure velocity drive (no
    stiffness at all). Gains are *derived* from the USD's own URDF-derived
    ``drive:angular:physics:*`` table rather than invented, but where they are too soft to
    hold a pose against gravity they are raised, and that is recorded per group below.
    """

    robot: ArticulationCfg = ArticulationCfg(
        prim_path=ROBOT_PRIM_PATH,
        spawn=sim_utils.UsdFileCfg(
            usd_path=ROBOT_USD_PATH,
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                # A free-standing mobile base, unlike Droid's Franka: gravity must act, and
                # the root link must not be fixed.
                disable_gravity=False,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                # The arms sweep close to the chassis; self-collision costs solver time and
                # buys nothing for a base that is meant to be mobile.
                enabled_self_collisions=False,
                solver_position_iteration_count=32,
                solver_velocity_iteration_count=1,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            # Every authored drive targetPosition is 0, so an all-zero joint pose reproduces
            # the geometry as modelled.
            #
            # z is the rest height, not 0. The USD's default prim sits at floor level but
            # the articulation root is base_link, which the asset authors 0.17 m above that
            # origin. Spawning the root at z=0 therefore buries the wheels 17 cm into the
            # ground: PhysX ejects the robot ~14 cm upward on *every* reset, it falls back
            # for about 20 steps, and anything commanded during that drop -- which for an RL
            # policy is every step -- lands it badly enough to flip it over. The extra 5 mm
            # is the settle drop described above.
            pos=(0.0, 0.0, BASE_LINK_REST_HEIGHT + SPAWN_CLEARANCE_M),
            rot=(0.0, 0.0, 0.0, 1.0),
            joint_pos={},
            joint_vel={},
        ),
        soft_joint_pos_limit_factor=1.0,
        actuators={
            # -- Swerve steering: USD 20.94 / 2.094 / maxForce 10 -----------------------
            # Raised to 200/20: a swerve module has no castor, so its steering drive alone
            # has to hold the module against lateral scrub torque. The USD value looks like
            # a unit artefact (2.094 is exactly 120 deg/s expressed in radians).
            "steering": ImplicitActuatorCfg(
                joint_names_expr=["[fb][lr]_steering_joint"],
                effort_limit=10.0,
                velocity_limit=10.0,
                stiffness=200.0,
                damping=20.0,
            ),
            # -- Wheels: a velocity drive (USD authors no stiffness, damping 1.396) ------
            # damping is this group's velocity-tracking gain, not a physical bearing loss,
            # so it is raised well above the USD value; effort_limit bounds traction.
            "wheels": ImplicitActuatorCfg(
                joint_names_expr=["[fb][lr]_wheel_joint"],
                effort_limit=20.0,
                velocity_limit=MAX_WHEEL_ANGULAR_VELOCITY,
                stiffness=0.0,
                damping=50.0,
            ),
            # -- Lift: USD 2500 / 300, effort 3000 but maxForce 500 ---------------------
            # maxForce is the binding limit; urdf:limit:effort (3000) is what the drive
            # would ask for and never gets.
            "lift": ImplicitActuatorCfg(
                joint_names_expr=["lift_joint"],
                effort_limit=500.0,
                velocity_limit=0.1,
                stiffness=2500.0,
                damping=300.0,
            ),
            # -- Left arm: effort from the USD table, gains scaled by group effort ------
            # j1-j2 carry the shoulder (effort 120), j3-j4 the elbow (60), j5-j7 the wrist
            # (14). The USD's 13.96 N-m/rad is far too soft to hold these against gravity;
            # the ratios follow Droid's Franka (400/80 on a 87 N-m shoulder).
            "left_arm_shoulder": ImplicitActuatorCfg(
                joint_names_expr=["openarmx_left_joint[1-2]"],
                effort_limit=120.0,
                velocity_limit=10.47,
                stiffness=400.0,
                damping=40.0,
            ),
            "left_arm_elbow": ImplicitActuatorCfg(
                joint_names_expr=["openarmx_left_joint[3-4]"],
                effort_limit=60.0,
                velocity_limit=10.47,
                stiffness=200.0,
                damping=20.0,
            ),
            "left_arm_wrist": ImplicitActuatorCfg(
                joint_names_expr=["openarmx_left_joint[5-7]"],
                effort_limit=14.0,
                velocity_limit=10.47,
                stiffness=60.0,
                damping=8.0,
            ),
            # -- Right arm: mirror of the left -----------------------------------------
            "right_arm_shoulder": ImplicitActuatorCfg(
                joint_names_expr=["openarmx_right_joint[1-2]"],
                effort_limit=120.0,
                velocity_limit=10.47,
                stiffness=400.0,
                damping=40.0,
            ),
            "right_arm_elbow": ImplicitActuatorCfg(
                joint_names_expr=["openarmx_right_joint[3-4]"],
                effort_limit=60.0,
                velocity_limit=10.47,
                stiffness=200.0,
                damping=20.0,
            ),
            "right_arm_wrist": ImplicitActuatorCfg(
                joint_names_expr=["openarmx_right_joint[5-7]"],
                effort_limit=14.0,
                velocity_limit=10.47,
                stiffness=60.0,
                damping=8.0,
            ),
            # -- Fingers: USD 200 / 20 / 333, the stiffest joints on the robot ----------
            # Both jaws of both hands; there is no mimic joint, so both are driven.
            "fingers": ImplicitActuatorCfg(
                joint_names_expr=["openarmx_(left|right)_finger_joint[12]"],
                effort_limit=333.0,
                velocity_limit=10.0,
                stiffness=200.0,
                damping=20.0,
            ),
            # -- Head: USD 6.981 / 0.6981 / 14 -----------------------------------------
            # Stiffened (20/2): it carries the head camera, and 6.98 N-m/rad sags under it.
            "head": ImplicitActuatorCfg(
                joint_names_expr=["openarmx_head_(yaw|pitch)_joint"],
                effort_limit=14.0,
                velocity_limit=33.0,
                stiffness=20.0,
                damping=2.0,
            ),
        },
    )
