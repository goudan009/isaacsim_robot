"""Scene assets this repository adds on top of an Arena background.

One of them: the hydrogen supply cabinet, scenery rather than a manipulandum, so it is an
``ObjectType.BASE`` and spawns as a static ``AssetBaseCfg`` with no articulation and no rigid
body -- which is what keeps it from competing with the background for the floor. It does
carry a collider (see ``hydrogen_supply_system_solid.usda``), so it is static in the way the
ground plane is static: the robot is blocked by it and it never moves.

It is not in isaaclab_arena's own asset library, and Arena's ``ensure_assets_registered``
only walks a hard-coded list of modules, so an environment that wants it has to import this
module for the side effect of running the ``@register_asset`` decorator -- from inside
``build()``, where an isaaclab import is safe.
"""

from __future__ import annotations

from isaaclab_arena.assets.object_library import LibraryObject
from isaaclab_arena.assets.object_type import ObjectType
from isaaclab_arena.assets.register import register_asset
from isaaclab_arena.utils.pose import Pose

from isaac_sim_arena.constants import (
    HYDROGEN_SUPPLY_SYSTEM_SCALE,
    HYDROGEN_SUPPLY_SYSTEM_USD_PATH,
)


# Identity: the cabinet spawns lying down. The source file is upAxis = "Y" and is authored
# standing in a Y-up world, but USD does not convert the axis when a layer is referenced, so
# with no rotation here the model keeps its own numbers and its +Y -- the 2.467 m height --
# lies along world +Y instead of standing up.
#
# Standing it up is +90 deg about X, (0.7071067811865476, 0.0, 0.0, 0.7071067811865476),
# which maps +Y onto +Z. That is a rotation change *and* an offset change rather than one or
# the other: the two orientations put a different extent on world y, and the offset below is
# what centres that extent on the shelf.
#
# (1.0, 0.0, 0.0, 0.0) is not a way to lie it down -- that is a 180 deg turn, and it maps +Y
# onto -Z, dropping all 2.467 m of the cabinet below the floor its origin sits on.
_LYING_FLAT: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

# Extent of each of the model's own axes after the 0.001 scale. Named by axis rather than
# width/height/depth because which one stands up depends on the rotation above: unrotated,
# these are what the pose below lays along world x, y and z. The origin therefore sits at a
# corner of the body rather than at its centre, which is why the offset below is asymmetric.
HYDROGEN_SUPPLY_SYSTEM_NATIVE_X_M = 1.680
HYDROGEN_SUPPLY_SYSTEM_NATIVE_Y_M = 2.467
HYDROGEN_SUPPLY_SYSTEM_NATIVE_Z_M = 1.560

# The shelf this cabinet replaces, measured on the spawned galileo_locomanip background in
# that background's own frame. In this scene the cabinet takes the shelf's place, so these
# two spans are what the offset below is derived from rather than the offset being picked.
#
#   x [-3.957, -3.497]   the -3.957 face is the one the robot sees; the robot is at x=-4.420
#   y [-2.204, -0.596]   the room's y centreline is -0.8895, so the shelf sits off-centre
_SHELF_X_M = (-3.957, -3.497)
_SHELF_Y_M = (-2.204, -0.596)

# Where it goes, as an origin offset in the background's own frame.
#
# x: the origin starts by taking the shelf's near face, so the cabinet grows *away* from the
# robot -- x [-3.957, -2.277]. Doing it the other way round would run 1.680 m of cabinet from
# -3.957 to -5.637, straight through the robot at x = -4.420. Standing or lying, the same
# native x extent lands on world x, so this half does not depend on the rotation above.
#
# The shift below then moves it back along -x by the amount that puts the cabinet's *axis*,
# rather than its near face, on the robot. That is 1.303 m and it is the whole of the
# calculation: the near face sits at local -3.957, which is world 0.463; half of the 1.680 m
# extent is 0.840, so the axis is at world 1.303; and the robot spawns at world x 0.0. Local
# -3.957 - 1.303 = -5.260, whose world is -0.840 -- the cabinet ends up spanning world
# x [-0.840, 0.840], symmetric about the robot, with 0.332 m to spare before the room's -x
# wall at -1.172. It is expressed as a shift rather than as a fresh origin so that the 1.303 m
# is legible as a displacement, and so the near-face derivation above stays the thing the
# number is measured from.
#
# y: the cabinet is centred on the shelf in that axis, and *that* is the half the rotation
# decides -- lying down it is the 2.467 m native y, so centring alone would put it at
# y [-2.634, -0.167] against the shelf's [-2.204, -0.596], overhanging each end by 0.43 m.
# The shift below then moves it 2 m along +y, on top of that centring.
#
# +y is the only direction with room for the shift. Centring alone leaves the -y end 0.146 m
# from the table (y <= -2.780), while +y runs 4.8 m before the room's wall. Nothing on the
# floor blocks the way either: the rack, the loose boxes and the bins all start at
# x >= -2.047 and the cabinet's far x face is at -2.277, so it passes clear of them along the
# whole run. The -y end would have been the tight side at 0.15 m.
HYDROGEN_SUPPLY_SYSTEM_Y_SHIFT_M = 2.0
HYDROGEN_SUPPLY_SYSTEM_X_SHIFT_M = -1.303
HYDROGEN_SUPPLY_SYSTEM_ORIGIN_IN_BACKGROUND_FRAME = (
    _SHELF_X_M[0] + HYDROGEN_SUPPLY_SYSTEM_X_SHIFT_M,
    0.5 * (_SHELF_Y_M[0] + _SHELF_Y_M[1])
    - 0.5 * HYDROGEN_SUPPLY_SYSTEM_NATIVE_Y_M
    + HYDROGEN_SUPPLY_SYSTEM_Y_SHIFT_M,
)


# Where the cabinet's own geometry lands once it is spawned, as a prim path template. The
# solid wrapper references the imported file's /World, and /World holds the single merged
# mesh under node_; the wrapper's own "cabinet" Xform is the segment between. Spelled out
# here because binding a material onto the cabinet means naming the mesh, and the name is a
# property of the asset rather than of any one scene.
HYDROGEN_SUPPLY_SYSTEM_MESH_PRIM_PATH = (
    "{ENV_REGEX_NS}/hydrogen_supply_system/cabinet/node_/mesh_"
)


def hydrogen_supply_system_pose(background_pose: Pose) -> Pose:
    """Return the cabinet's pose, given the pose the background itself is spawned at.

    Deriving it from the background rather than hardcoding world coordinates keeps the
    cabinet on the floor if the scene is ever posed differently -- the same reason the
    environment reads ``floor_z`` off the background instead of writing -0.795.

    The rotation is identity, so the cabinet lies down, and the origin still lands on the
    floor because the model's own z extent starts at zero: the body grows up from the origin
    rather than being centred on it, which is what lets this stay a bare translation.
    Rendered, the doors and vents come out on the -X face -- the one the robot faces -- so
    nothing needs turning to face it.

    Args:
        background_pose: The galileo background's ``initial_pose``. A rotation is assumed to
            be absent; the offset below is applied without rotating it.

    Returns:
        The cabinet's root pose: lying down, resting on the background's floor plane.
    """
    offset_x, offset_y = HYDROGEN_SUPPLY_SYSTEM_ORIGIN_IN_BACKGROUND_FRAME
    position = background_pose.position_xyz
    return Pose(
        position_xyz=(position[0] + offset_x, position[1] + offset_y, position[2]),
        rotation_xyzw=_LYING_FLAT,
    )


@register_asset
class HydrogenSupplySystem(LibraryObject):
    """The hydrogen supply cabinet imported from ``openflex_ws``.

    The source file is awkward in two ways this class exists to absorb: its numbers are
    millimetre-scale although it declares ``metersPerUnit = 0.01``, and it is authored
    ``upAxis = "Y"``, which USD does not convert for a referenced layer. The scale below
    fixes the first; the rotation in :func:`hydrogen_supply_system_pose` decides what the
    second leaves standing, and it currently leaves the cabinet lying down.

    ``usd_path`` is the ``_solid`` wrapper rather than the imported file itself, because the
    import carries no collision anywhere and would otherwise spawn a cabinet the robot drives
    through; the wrapper references it unchanged and authors a collider on its mesh. See
    ``isaac_sim_arena.constants.HYDROGEN_SUPPLY_SYSTEM_USD_PATH``.

    Like any scenery here it has no usable default pose, so construct it with one.
    """

    name = "hydrogen_supply_system"
    tags = ["object", "scenery"]
    usd_path = HYDROGEN_SUPPLY_SYSTEM_USD_PATH
    object_type = ObjectType.BASE
    scale = (HYDROGEN_SUPPLY_SYSTEM_SCALE, HYDROGEN_SUPPLY_SYSTEM_SCALE, HYDROGEN_SUPPLY_SYSTEM_SCALE)
