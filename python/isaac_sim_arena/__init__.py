"""OpenFleX as an IsaacLab-Arena embodiment (out-of-tree).

Deliberately empty of imports. ``isaac_sim_arena.external_env`` is imported by Arena at
CLI-parse time -- before Kit starts -- so anything this package pulls in at import time
must be importable with no pxr/warp/SimulationApp singletons. Importing the embodiment
here would drag in ``isaaclab.sim`` and break that; import it where it is used instead.
"""
