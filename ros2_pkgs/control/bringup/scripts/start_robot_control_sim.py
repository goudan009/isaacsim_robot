#!/usr/bin/env python3
"""Run the generic Isaac REST loop with a robot-control spawn hook.

The upstream REST server's default spawn operation always configures sensors.
This small adapter replaces only that operation with URDF import, the existing
topic-based robot controller, and the simulation clock graph. The default is
sensor-free; an explicit four-camera performance probe is the sole exception.
"""

from __future__ import annotations

import runpy
import os
import sys
from pathlib import Path


def _isaac_install_root() -> Path | None:
    """Return the Isaac Sim root from the bundled Kit Python executable."""
    configured = os.environ.get("ISAACSIM_PATH", "").strip()
    if configured:
        candidate = Path(configured).expanduser()
        if (candidate / "python.sh").is_file():
            return candidate

    executable = Path(sys.executable).resolve()
    for parent in executable.parents:
        if (parent / "python.sh").is_file() and (parent / "exts").is_dir():
            return parent
    return None


def _sensor_python_paths() -> list[str]:
    """Keep only the consolidated sensor adapter on Isaac's Python path."""
    paths: list[str] = []
    configured = os.environ.get("OPENFLEX_ISAAC_SENSOR_ASSET_DIR", "").strip()
    roots = [Path(configured).expanduser()] if configured else []

    # The launch contract passes the consolidated repository root. Keep a
    # source-tree fallback so this script also works before the sensor package
    # has been installed into the colcon workspace.
    for root in roots:
        paths.extend(
            (
                str(root / "ros2_pkgs" / "simulation_bridge" / "sensor_pkg"),
                str(root / "ros2" / "openflex_isaac_sensors"),
            )
        )

    # Prefer the installed consolidated package when available. Do not pick
    # the historical package with a similar purpose: importing both trees in
    # Isaac Sim can register duplicate protobuf/ROS bridge symbols.
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if not prefix:
            continue
        prefix_path = Path(prefix)
        for candidate in (
            prefix_path / "local" / "lib" / "python3.10" / "dist-packages",
            prefix_path / "lib" / "python3.10" / "site-packages",
        ):
            if (candidate / "isaacsim_sensors").is_dir():
                paths.append(str(candidate))

    return list(dict.fromkeys(path for path in paths if Path(path).is_dir()))


def _clean_isaac_runtime_environment() -> None:
    """Match the historical stable launcher environment inside Isaac Sim."""
    for key in ("PYTHONHOME", "PYTHONUSERBASE"):
        os.environ.pop(key, None)

    sensor_paths = _sensor_python_paths()
    if sensor_paths:
        os.environ["PYTHONPATH"] = os.pathsep.join(sensor_paths)
    else:
        os.environ.pop("PYTHONPATH", None)
    os.environ["PYTHONNOUSERSITE"] = "1"

    isaac_root = _isaac_install_root()
    if isaac_root is None:
        return
    bridge_paths = []
    exts_root = isaac_root / "exts" / "3"
    for bridge_root in exts_root.glob("isaacsim.ros2.bridge-*"):
        library_path = bridge_root / "humble" / "lib"
        if library_path.is_dir():
            bridge_paths.append(str(library_path))
    if bridge_paths:
        existing = os.environ.get("LD_LIBRARY_PATH", "").split(os.pathsep)
        os.environ["LD_LIBRARY_PATH"] = os.pathsep.join(
            list(dict.fromkeys(bridge_paths + [entry for entry in existing if entry]))
        )


def _patch_simulation_app_startup() -> None:
    """Apply the single-GPU startup contract before Kit is constructed.

    The generic REST runner creates ``SimulationApp`` itself.  Settings changed
    after construction are too late for renderer device selection, and the
    default Isaac Sim profile currently enables multi-GPU even on a one-GPU
    machine.  The validated MID360 path uses one renderer device, so inject
    those Kit command-line settings into the runner's SimulationApp config.
    """
    import isaacsim

    original = getattr(isaacsim, "SimulationApp", None)
    if original is None or getattr(original, "_isaacsim_robot_single_gpu", False):
        return

    def simulation_app_with_single_gpu(config=None, *args, **kwargs):
        if isinstance(config, dict):
            config = dict(config)
            extra_args = list(config.get("extra_args", []))
            required_args = (
                "--/renderer/multiGpu/enabled=false",
                "--/renderer/multiGpu/autoEnable=false",
                "--/renderer/multiGpu/maxGpuCount=1",
                "--/exts/omni.replicator.srtx/enabled=false",
            )
            for value in required_args:
                if value not in extra_args:
                    extra_args.append(value)
            config["extra_args"] = extra_args
        return original(config, *args, **kwargs)

    simulation_app_with_single_gpu._isaacsim_robot_single_gpu = True
    isaacsim.SimulationApp = simulation_app_with_single_gpu


def _ensure_clock_graph() -> None:
    import OmniGraphSchema
    import omni.graph.core as og
    from omni.graph.core import GraphPipelineStage
    import omni.timeline
    import omni.usd

    graph_path = "/Graph/ROS_Clock"
    stage = omni.usd.get_context().get_stage()
    existing = stage.GetPrimAtPath(graph_path) if stage is not None else None
    if existing is not None and existing.IsValid():
        if existing.IsA(OmniGraphSchema.OmniGraph):
            return
        raise RuntimeError(f"cannot create ROS clock graph; path is occupied: {graph_path}")

    omni.timeline.get_timeline_interface().stop()
    keys = og.Controller.Keys
    graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                # The control stack uses sim time. Publishing /clock from the
                # render/playback tick (60 Hz) caps use_sim_time timers and
                # therefore caps the public /joint_states broadcaster. Drive
                # the clock from every physics step so the 90 Hz controller
                # target is not artificially limited by render_hz.
                ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ],
            keys.CONNECT: [
                ("OnPhysicsStep.outputs:step", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [("ReadSimTime.inputs:resetOnStop", False)],
        },
    )
    try:
        og.Controller.evaluate_sync(graph)
    except Exception:
        pass
    print(f"[robot-control-only] created ROS clock graph at {graph_path}", flush=True)


def _installed_sensor_share() -> Path:
    """Resolve the installed RealSense config inside Isaac's Python process.

    The upstream REST ``SpawnRobotRequest`` preserves ``enable_sensors`` but
    discards extension fields.  The explicit camera probe therefore resolves
    its fixed repository component from the colcon environment rather than
    relying on custom HTTP request fields.
    """

    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            candidate = Path(prefix) / "share" / "isaacsim_sensors"
            if (candidate / "config" / "realsense_robot_mounts.yaml").is_file():
                return candidate
    raise RuntimeError("installed isaacsim_sensors config was not found in AMENT_PREFIX_PATH")


def _sensor_asset_root() -> Path:
    """Resolve the consolidated sensor-asset root inside Isaac Python.

    Component 5 passes this explicitly because Isaac Sim runs in its own
    Python environment and cannot reliably discover the colcon source tree.
    The workspace root is also accepted for the in-repository layout, where
    MID360 configuration lives under ``isaac_sim_core/config``.
    """
    configured = (
        os.environ.get("OPENFLEX_ISAAC_MID360_ASSET_DIR", "").strip()
        or os.environ.get("OPENFLEX_ISAAC_SENSOR_ASSET_DIR", "").strip()
    )
    candidates: list[Path] = []
    if configured:
        candidates.append(Path(configured).expanduser().resolve())
    for parent in Path(__file__).resolve().parents:
        candidates.extend((parent, parent / "src" / "openflex_isaac_sensor_assets"))
    for candidate in candidates:
        if (
            (candidate / "mid360" / "config" / "mid360_robot_mount.yaml").is_file()
            or (candidate / "isaac_sim_core" / "config" / "sensor_params" / "mid360" / "mid360_robot_mount.yaml").is_file()
        ):
            return candidate
    raise RuntimeError(
        "MID360 asset root was not found; set OPENFLEX_ISAAC_MID360_ASSET_DIR "
        "or OPENFLEX_ISAAC_SENSOR_ASSET_DIR"
    )


def _install_control_only_spawn(rest_api_module) -> None:
    def spawn_robot_control_only(self, params: dict) -> dict:
        import spawn

        print(
            "[robot-control-only] importing URDF without sensor setup; "
            f"spawn_module={getattr(spawn, '__file__', '<unknown>')}",
            flush=True,
        )
        import omni.usd

        try:
            print("[robot-control-only] spawn.main begin", flush=True)
            obj = spawn.main(
                urdf_path=params["urdf_path"],
                x=params["x"],
                y=params["y"],
                z=params["z"],
                roll=params["roll"],
                pitch=params["pitch"],
                yaw=params["yaw"],
                fixed=params["fixed"],
            )
            prim_path = obj.GetPath().pathString if obj else None
            print(f"[robot-control-only] spawn.main complete: {prim_path}", flush=True)
        except Exception as error:
            # Isaac Sim 6's importer may have already created the graph that
            # this older spawn helper tries to create at the end of its work.
            # Preserve the successfully imported robot and reuse that graph;
            # any other spawn failure remains fatal.
            stage = omni.usd.get_context().get_stage()
            root = stage.GetPrimAtPath("/openflex")
            action_graph = stage.GetPrimAtPath("/openflex/ActionGraph")
            print(
                "[robot-control-only] spawn.main failed: "
                f"{type(error).__name__}: {error}; "
                f"root_valid={root.IsValid()} graph_valid={action_graph.IsValid()}",
                flush=True,
            )
            if not (root.IsValid() and action_graph.IsValid()):
                raise
            print(
                "[robot-control-only] reusing imported control graph after duplicate-graph notice: "
                f"{action_graph.GetPath()} ({error})",
                flush=True,
            )
            prim_path = "/openflex"
        print("[robot-control-only] creating robot control graph", flush=True)
        import robot_controller

        robot_controller.main(urdf_path=params["urdf_path"])
        print("[robot-control-only] robot control graph ready", flush=True)
        camera_data: dict[str, object] = {"disabled": True, "created": []}
        sensor_profile = os.environ.get("OPENFLEX_ROBOT_SENSOR_PROFILE", "rgb_depth").strip().lower()
        lidar_profile = os.environ.get("OPENFLEX_MID360_LIDAR_PROFILE", "MID360_PERFORMANCE").strip()
        lidar_transport = os.environ.get("OPENFLEX_MID360_LIDAR_TRANSPORT", "helper").strip().lower()
        lidar_mount_mode = os.environ.get(
            "OPENFLEX_MID360_MOUNT_MODE", "fixed_kinematic"
        ).strip().lower()
        if bool(params.get("enable_sensors", False)) and sensor_profile != "none":
            from isaacsim_sensors.integration import create_robot_sensor_suite

            stage = omni.usd.get_context().get_stage()
            camera_enabled = sensor_profile in {"rgb", "rgb_depth", "data", "teleop"}
            lidar_enabled = sensor_profile in {"lidar", "data"}
            realsense_root = _installed_sensor_share() if camera_enabled else None
            mid360_root = _sensor_asset_root() if lidar_enabled else None
            camera_data = create_robot_sensor_suite(
                stage,
                "/openflex",
                sensor_asset_dir=mid360_root or realsense_root,
                realsense_asset_dir=realsense_root,
                mid360_asset_dir=mid360_root,
                sensor_profile=sensor_profile,
                lidar_profile=lidar_profile,
                lidar_transport=lidar_transport,
                lidar_mount_mode=lidar_mount_mode,
            )
            print(
                "[robot-control-only] sensor suite ready: "
                f"profile={sensor_profile} cameras={camera_data.get('camera_count', 0)} "
                f"lidar={camera_data.get('lidar_prim_path', '')} "
                f"transport={camera_data.get('lidar_transport', 'none')} "
                f"mount_mode={camera_data.get('lidar_mount_mode', 'none')}",
                flush=True,
            )
        print("[robot-control-only] ensuring ROS clock graph", flush=True)
        _ensure_clock_graph()
        return {
            "success": True,
            "data": {"prim_path": prim_path, "sensors": camera_data},
        }

    api_class = rest_api_module.IsaacSimRestApi
    api_class._spawn_robot = spawn_robot_control_only

    # Also create the ROS clock graph for an empty-stage benchmark. The
    # graph is harmlessly idempotent when the robot spawn path calls it later,
    # and this lets the empty scene use exactly the same Isaac timing source.
    create_server = rest_api_module.create_server

    def create_server_with_clock(*args, **kwargs):
        server = create_server(*args, **kwargs)
        _ensure_clock_graph()
        if os.environ.get("OPENFLEX_EMPTY_SCENE", "").strip().lower() in {"1", "true", "yes", "on"}:
            import omni.timeline

            omni.timeline.get_timeline_interface().play()
            print("[robot-control-only] empty-scene benchmark playback started", flush=True)
        return server

    rest_api_module.create_server = create_server_with_clock

    # Keep the control-only contract explicit even if a downstream copy of
    # the REST class dispatches through _execute_command directly.
    original_execute = api_class._execute_command

    def execute_control_only(self, command):
        command_type = getattr(command, "cmd_type", None)
        spawn_type = getattr(rest_api_module.CommandType, "SPAWN_ROBOT", None)
        if command_type == spawn_type:
            return spawn_robot_control_only(self, command.params)
        return original_execute(self, command)

    api_class._execute_command = execute_control_only
    print(
        "[robot-control-only] REST spawn hooks installed: "
        f"module={getattr(rest_api_module, '__file__', '<unknown>')} "
        f"class={api_class.__module__}.{api_class.__name__}",
        flush=True,
    )


def main() -> None:
    if len(sys.argv) < 2:
        raise SystemExit(
            "usage: start_robot_control_sim.py <isaac_ros2_scripts_share> "
            "<stage> [render_hz] [physics_hz] [real_hz] [headless] [api_port]"
        )
    scripts_share = Path(sys.argv[1]).expanduser().resolve()
    # ament_python installs this dependency's ``isaac_scripts/*.py`` files
    # directly below its package share directory (the source tree keeps the
    # extra directory). Support both layouts for source and install spaces.
    isaac_scripts = scripts_share / "isaac_scripts"
    if not (isaac_scripts / "start_sim_with_rest_api.py").is_file():
        isaac_scripts = scripts_share
    runner = isaac_scripts / "start_sim_with_rest_api.py"
    if not runner.is_file():
        raise SystemExit(f"Isaac REST runner does not exist: {runner}")

    _clean_isaac_runtime_environment()
    _patch_simulation_app_startup()
    sys.path.insert(0, str(isaac_scripts))

    import rest_api_server

    _install_control_only_spawn(rest_api_server)
    print(
        "[robot-control-only] launching REST runner with "
        f"runner={runner} rest_api={getattr(rest_api_server, '__file__', '<unknown>')}",
        flush=True,
    )
    sys.argv = [str(runner), *sys.argv[2:]]
    runpy.run_path(str(runner), run_name="__main__")


if __name__ == "__main__":
    main()
