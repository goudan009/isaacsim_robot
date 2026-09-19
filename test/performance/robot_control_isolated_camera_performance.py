#!/usr/bin/env python3
"""验证双 Isaac 进程下的完整机器人控制与四相机副本状态同步。"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_control_full_performance import (
    GROUPS,
    REQUIRED_CONTROLLERS,
    FullControlNode,
    _format_metric,
    _header_seconds,
    _stop_process,
    _wait_until_ready,
)


ROOT = Path(__file__).resolve().parents[2]
RAW_ROOT = ROOT / "reports" / "sensors" / "raw"
REPORT_PATH = ROOT / "reports" / "sensors" / "sensor_validation.md"
REPLICA_SCRIPT = ROOT / "isaac_sim_core" / "scenarios" / "robot_camera_replica.py"
LIDAR_REPLICA_SCRIPT = ROOT / "isaac_sim_core" / "scenarios" / "robot_lidar_replica.py"
RELAY_SCRIPT = ROOT / "test" / "performance" / "robot_state_snapshot_relay.py"
LIDAR_RESTAMPER_SCRIPT = ROOT / "test" / "performance" / "pointcloud_authority_restamper.py"
CAMERA_STREAMS = {
    "base_rgb": "/cam_base/color/image",
    "base_depth": "/cam_base/depth/image",
    "head_rgb": "/cam_head/color/image",
    "head_depth": "/cam_head/depth/image",
    "left_wrist_rgb": "/cam_left/color/image",
    "left_wrist_depth": "/cam_left/depth/image",
    "right_wrist_rgb": "/cam_right/color/image",
    "right_wrist_depth": "/cam_right/depth/image",
}
DEFAULT_LIDAR_TOPIC = "/openflex/livox_frame/lidar"


def _isaac_path() -> Path:
    configured = os.environ.get("ISAACSIM_PATH", "").strip()
    candidates = [Path(configured).expanduser()] if configured else []
    candidates.extend((Path.home() / "isaacsim-6.0", Path.home() / "isaacsim", Path("/isaac-sim")))
    for candidate in candidates:
        if (candidate / "python.sh").is_file():
            return candidate
    raise FileNotFoundError("Isaac Sim python.sh was not found; set ISAACSIM_PATH")


def _with_taskset(cpu_set: str, command: list[str]) -> list[str]:
    return ["taskset", "--cpu-list", cpu_set, *command] if cpu_set else command


def _read_json(path: Path) -> dict[str, Any] | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def _signed_distribution(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"samples": 0, "mean_s": None, "p99_abs_s": None, "max_abs_s": None}
    absolute = sorted(abs(value) for value in values)
    p99_index = min(len(absolute) - 1, round((len(absolute) - 1) * 0.99))
    return {
        "samples": len(values),
        "mean_s": sum(values) / len(values),
        "p99_abs_s": absolute[p99_index],
        "max_abs_s": absolute[-1],
    }


def _camera_ready(node: FullControlNode) -> bool:
    return all(sample.wall for sample in node.camera_timing.values())


def _wait_for_cameras(
    node: FullControlNode,
    replica: subprocess.Popen[str],
    replica_output: Path,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and time.monotonic() < deadline and replica.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        # Initialization can publish one or more camera frames before the
        # replica has completed its warmup.  Require the explicit marker
        # written by robot_camera_replica.py before starting measurements.
        ready = _read_json(replica_output / "ready.json")
        if ready and ready.get("status") == "ready" and _camera_ready(node):
            return
    if replica.poll() is not None:
        raise RuntimeError(f"camera replica exited before publishing images: {replica.returncode}")
    missing = [name for name, sample in node.camera_timing.items() if not sample.wall]
    raise RuntimeError("timed out waiting for replica camera topics: " + ", ".join(missing))


def _wait_for_lidar(node: FullControlNode, replica: subprocess.Popen[str], timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and time.monotonic() < deadline and replica.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.lidar_nonempty_samples > 0:
            return
    if replica.poll() is not None:
        raise RuntimeError(f"sensor replica exited before publishing lidar: {replica.returncode}")
    raise RuntimeError("timed out waiting for non-empty MID360 PointCloud2")


def _authority_command(args: argparse.Namespace) -> list[str]:
    command = [
        "ros2", "launch", "openflex_isaac_bringup", "robot_control_only.launch.py",
        "headless:=true", "camera_profile:=none",
        f"physics_hz:={args.physics_hz}", f"render_hz:={args.authority_render_hz}",
        f"api_port:={args.api_port}",
    ]
    return _with_taskset(args.authority_cpu_set, command)


def _relay_command(args: argparse.Namespace, output: Path) -> list[str]:
    command = [
        sys.executable, str(RELAY_SCRIPT), "--port", str(args.snapshot_port),
        "--output", str(output),
    ]
    if args.enable_lidar and args.separate_lidar_process:
        command.extend(("--additional-port", str(args.lidar_snapshot_port)))
    return command


def _replica_command(args: argparse.Namespace, output: Path) -> list[str]:
    command = [
        str(_isaac_path() / "python.sh"), str(REPLICA_SCRIPT),
        "--stage", str(args.replica_stage),
        "--snapshot-port", str(args.snapshot_port),
        "--physics-hz", str(args.replica_physics_hz),
        "--render-hz", str(args.replica_render_hz),
        "--warmup-steps", str(args.replica_warmup_steps),
        "--pace-spin-us", str(args.pace_spin_us),
        "--pace-sleep-guard-us", str(args.pace_sleep_guard_us),
        "--kit-threads", str(args.kit_threads),
        "--camera-width", str(args.camera_width),
        "--camera-height", str(args.camera_height),
        "--camera-tick-rate-hz", str(args.camera_tick_rate_hz),
        "--sensor-profile", "data" if args.enable_lidar and not args.separate_lidar_process else "rgb_depth",
        "--output", str(output),
    ]
    if args.enable_lidar:
        command.extend(("--lidar-profile", args.lidar_profile, "--lidar-transport", args.lidar_transport, "--lidar-tick-rate-hz", str(args.lidar_tick_rate_hz)))
        if args.no_lidar_object_id_map:
            command.append("--no-lidar-object-id-map")
    if args.srtx:
        command.append("--srtx")
    if args.publish_with_queue_thread:
        command.append("--publish-with-queue-thread")
    if args.minimal_rendering:
        command.append("--minimal-rendering")
    if args.no_per_sensor_tick_tlas:
        command.append("--no-per-sensor-tick-tlas")
    if args.disable_replica_collisions:
        command.append("--disable-replica-collisions")
    if args.hide_replica_guide_meshes:
        command.append("--hide-replica-guide-meshes")
    if args.visual_lod_proxy:
        command.append("--visual-lod-proxy")
    if args.visual_kinematic_replica:
        command.append("--visual-kinematic-replica")
    if args.kinematic_sync_hz > 0.0:
        command.extend(("--kinematic-sync-hz", str(args.kinematic_sync_hz)))
    if args.anti_aliasing is not None:
        command.extend(("--anti-aliasing", str(args.anti_aliasing)))
    if args.no_camera_info:
        command.append("--no-camera-info")
    return _with_taskset(args.replica_cpu_set, command)


def _lidar_replica_command(args: argparse.Namespace, output: Path) -> list[str]:
    command = [
        str(_isaac_path() / "python.sh"), str(LIDAR_REPLICA_SCRIPT),
        "--stage", str(args.lidar_stage),
        "--snapshot-port", str(args.lidar_snapshot_port),
        "--physics-hz", str(args.lidar_physics_hz),
        "--render-hz", str(args.lidar_render_hz),
        "--warmup-steps", str(args.replica_warmup_steps),
        "--kit-threads", str(args.kit_threads),
        "--lidar-profile", str(args.lidar_profile),
        "--lidar-transport", str(args.lidar_transport),
        "--lidar-tick-rate-hz", str(args.lidar_tick_rate_hz),
        "--lidar-topic", args.lidar_raw_topic if args.restamp_lidar_to_authority_clock else args.lidar_topic,
        "--kinematic-sync-hz", str(args.lidar_kinematic_sync_hz),
        "--dynamic-transform-mode", args.lidar_dynamic_transform_mode,
        "--output", str(output),
    ]
    if args.no_lidar_object_id_map:
        command.append("--no-lidar-object-id-map")
    if args.publish_with_queue_thread:
        command.append("--publish-with-queue-thread")
    return _with_taskset(args.lidar_cpu_set, command)


def _lidar_restamper_command(args: argparse.Namespace, output: Path) -> list[str]:
    return [
        sys.executable,
        str(LIDAR_RESTAMPER_SCRIPT),
        "--source-topic", args.lidar_raw_topic,
        "--output-topic", args.lidar_topic,
        "--output", str(output),
    ]


def _run(args: argparse.Namespace, batch_dir: Path) -> dict[str, Any]:
    authority_log = batch_dir / "authority.launch.log"
    replica_log = batch_dir / "replica.launch.log"
    lidar_replica_log = batch_dir / "lidar_replica.launch.log"
    lidar_restamper_log = batch_dir / "lidar_restamper.log"
    lidar_restamper_metrics = batch_dir / "lidar_restamper.metrics.json"
    relay_log = batch_dir / "relay.log"
    relay_metrics = batch_dir / "relay.metrics.json"
    replica_output = batch_dir / "replica"
    environment = os.environ.copy()
    environment.setdefault("ROS_DOMAIN_ID", "42")
    environment["ROS_LOCALHOST_ONLY"] = "1"
    environment["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    environment.pop("ROS_DISCOVERY_SERVER", None)
    authority = subprocess.Popen(
        _authority_command(args), stdout=authority_log.open("w", encoding="utf-8"),
        stderr=subprocess.STDOUT, start_new_session=True, text=True, env=environment,
    )
    relay: subprocess.Popen[str] | None = None
    replica: subprocess.Popen[str] | None = None
    lidar_replica: subprocess.Popen[str] | None = None
    lidar_restamper: subprocess.Popen[str] | None = None
    measurement_executor: MultiThreadedExecutor | None = None
    node = FullControlNode(
        camera_profile="quad",
        lidar_topic=args.lidar_topic if args.enable_lidar else None,
    )
    result: dict[str, Any] = {
        "status": "TEST_ERROR",
        "authority_command": _authority_command(args),
        "authority_launch_log": str(authority_log.relative_to(ROOT)),
        "replica_launch_log": str(replica_log.relative_to(ROOT)),
        "relay_log": str(relay_log.relative_to(ROOT)),
        "replica_metrics": str((replica_output / "metrics.json").relative_to(ROOT)),
        "lidar_replica_launch_log": str(lidar_replica_log.relative_to(ROOT)),
        "lidar_replica_metrics": str((batch_dir / "lidar_replica" / "metrics.json").relative_to(ROOT)),
        "lidar_restamper_log": str(lidar_restamper_log.relative_to(ROOT)),
        "lidar_restamper_metrics": str(lidar_restamper_metrics.relative_to(ROOT)),
        "relay_metrics": str(relay_metrics.relative_to(ROOT)),
        "cpu_affinity": {"authority": args.authority_cpu_set, "replica": args.replica_cpu_set},
    }
    try:
        states = _wait_until_ready(node, authority, args.startup_timeout)
        result["authority_startup_seconds"] = None
        result["controller_states"] = states
        relay = subprocess.Popen(
            _relay_command(args, relay_metrics), stdout=relay_log.open("w", encoding="utf-8"),
            stderr=subprocess.STDOUT, start_new_session=True, text=True, env=environment,
        )
        replica = subprocess.Popen(
            _replica_command(args, replica_output), stdout=replica_log.open("w", encoding="utf-8"),
            stderr=subprocess.STDOUT, start_new_session=True, text=True, env=environment,
        )
        _wait_for_cameras(node, replica, replica_output, args.camera_startup_timeout)
        if args.enable_lidar:
            if args.separate_lidar_process:
                if args.restamp_lidar_to_authority_clock:
                    lidar_restamper = subprocess.Popen(
                        _lidar_restamper_command(args, lidar_restamper_metrics),
                        stdout=lidar_restamper_log.open("w", encoding="utf-8"),
                        stderr=subprocess.STDOUT, start_new_session=True, text=True, env=environment,
                    )
                lidar_replica = subprocess.Popen(
                    _lidar_replica_command(args, batch_dir / "lidar_replica"),
                    stdout=lidar_replica_log.open("w", encoding="utf-8"),
                    stderr=subprocess.STDOUT, start_new_session=True, text=True, env=environment,
                )
                _wait_for_lidar(node, lidar_replica, args.lidar_startup_timeout)
            else:
                _wait_for_lidar(node, replica, args.lidar_startup_timeout)
        node.wait_for_command_subscriptions(timeout_sec=10.0)
        before = dict(node.positions)
        odom_before = node.odom_xy()
        node.reset_timing()
        # Eight 640x480 RGB-D streams are substantially more callback work
        # than the control topics. A single ``spin_once`` loop turns the
        # verifier itself into a subscriber bottleneck and understates camera
        # delivery. Isolate callback dispatch from command publication.
        measurement_executor = MultiThreadedExecutor(num_threads=12)
        measurement_executor.add_node(node)
        measurement_start = time.monotonic()
        command_end = measurement_start + args.command_seconds
        measurement_end = measurement_start + args.duration
        max_wheel_speed = 0.0
        while rclpy.ok() and time.monotonic() < measurement_end and authority.poll() is None:
            node.publish_commands(args.base_speed if time.monotonic() < command_end else 0.0)
            measurement_executor.spin_once(timeout_sec=0.02)
            max_wheel_speed = max(max_wheel_speed, node.max_wheel_speed())
        if authority.poll() is not None:
            raise RuntimeError(f"authority exited during measurement: {authority.returncode}")
        groups: dict[str, Any] = {}
        for group in GROUPS:
            joints = []
            for joint in group.joints:
                start, end = before.get(joint), node.positions.get(joint)
                delta = None if start is None or end is None else end - start
                joints.append({"joint": joint, "before": start, "after": end, "delta": delta})
            moved = [entry for entry in joints if entry["delta"] is not None and abs(entry["delta"]) >= args.min_joint_delta]
            groups[group.name] = {
                "moved_count": len(moved), "required_count": len(group.joints),
                "response_ok": len(moved) == len(group.joints), "joints": joints,
            }
        odom_after = node.odom_xy()
        displacement = (
            math.hypot(odom_after[0] - odom_before[0], odom_after[1] - odom_before[1])
            if odom_before is not None and odom_after is not None else None
        )
        measurement_executor.remove_node(node)
        measurement_executor.shutdown()
        measurement_executor = None
        controller_states_after = node.controller_states(timeout_sec=5.0)
        timing = {
            "clock": node.timing["clock"].metrics(),
            "sim_joint_states": node.timing["sim_joint_states"].metrics(args.physics_hz),
            "joint_states": node.timing["joint_states"].metrics(),
        }
        result.update({
            "measurement_seconds": time.monotonic() - measurement_start,
            "controller_states_after": controller_states_after,
            "groups": groups,
            "base": {
                "max_wheel_speed_rad_s": max_wheel_speed,
                "odom_displacement_m": displacement,
                "response_ok": max_wheel_speed > 0.01 and displacement is not None and displacement > 0.001,
            },
            "timing": timing,
            "camera_timing": {name: sample.metrics() for name, sample in node.camera_timing.items()},
            "lidar_timing": node.lidar_timing.metrics(),
            "lidar_authority_clock_skew": _signed_distribution(node.lidar_authority_clock_skew_s),
            "lidar_nonempty_samples": node.lidar_nonempty_samples,
            "last_lidar_point_count": node.last_lidar_point_count,
        })
        result["status"] = "COMPLETE"
        return result
    except Exception as error:  # noqa: BLE001
        result["error"] = str(error)
        return result
    finally:
        if measurement_executor is not None:
            try:
                measurement_executor.remove_node(node)
            except Exception:  # noqa: BLE001
                pass
            measurement_executor.shutdown()
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for process in (lidar_replica, lidar_restamper, replica, relay, authority):
            if process is not None:
                _stop_process(process)
        result["authority_returncode"] = authority.poll()
        result["replica_returncode"] = replica.poll() if replica else None
        result["lidar_replica_returncode"] = lidar_replica.poll() if lidar_replica else None
        result["lidar_restamper_returncode"] = lidar_restamper.poll() if lidar_restamper else None
        result["relay_returncode"] = relay.poll() if relay else None
        result["replica_internal"] = _read_json(replica_output / "metrics.json")
        result["lidar_replica_internal"] = _read_json(batch_dir / "lidar_replica" / "metrics.json") if lidar_replica else None
        result["lidar_restamper_internal"] = _read_json(lidar_restamper_metrics) if lidar_restamper else None
        result["relay_internal"] = _read_json(relay_metrics)
        if result.get("status") == "COMPLETE" and result["replica_internal"] is None:
            result["status"] = "TEST_ERROR"
            result["error"] = "camera replica did not flush metrics.json before exit"
        if result.get("status") == "COMPLETE" and result["relay_internal"] is None:
            result["status"] = "TEST_ERROR"
            result["error"] = "state relay did not flush metrics JSON before exit"


def _conclusions(result: dict[str, Any], args: argparse.Namespace) -> tuple[str, str]:
    groups = result.get("groups") or {}
    cameras = result.get("camera_timing") or {}
    replica = result.get("replica_internal") or {}
    functional = (
        result.get("status") == "COMPLETE"
        and bool(groups)
        and all(item.get("response_ok") for item in groups.values())
        and bool(result.get("base", {}).get("response_ok"))
        and len(cameras) == 8
        and all(float(item.get("wall_receive_hz") or 0.0) >= args.min_camera_hz for item in cameras.values())
        and int(replica.get("physics_steps") or 0) > 0
        and int(replica.get("snapshot_apply_updates") or 0) > 0
        and int(replica.get("snapshot_applied_dof_total") or 0) > 0
        and (
            not args.enable_lidar
            or (
                float(result.get("lidar_timing", {}).get("wall_receive_hz") or 0.0) >= args.min_lidar_hz
                and int(result.get("lidar_nonempty_samples") or 0) > 0
            )
        )
    )
    timing = result.get("timing") or {}
    realtime = (
        functional
        and float(timing.get("joint_states", {}).get("wall_receive_hz") or 0.0) >= 89.95
        and float(timing.get("sim_joint_states", {}).get("physics_step_rate_hz") or 0.0) >= args.physics_hz
        and float(timing.get("clock", {}).get("rtf") or 0.0) >= 1.0
    )
    return ("通过" if functional else "未通过", "通过" if realtime else "未通过")


def _batch_id(executor: str) -> str:
    safe = "".join(char if char.isalnum() or char in "_-" else "-" for char in executor).strip("-") or "manual"
    return f"{datetime.now().astimezone().strftime('%Y%m%d-%H%M%S')}-双进程整机四相机-{safe}"


def _markdown(batch_id: str, executed_at: str, executor: str, result: dict[str, Any], args: argparse.Namespace) -> str:
    function, realtime = _conclusions(result, args)
    timing = result.get("timing") or {}
    cameras = result.get("camera_timing") or {}
    replica = result.get("replica_internal") or {}
    relay = result.get("relay_internal") or {}
    image_range = [float(item.get("wall_receive_hz") or 0.0) for item in cameras.values()]
    return "\n".join((
        f"### 批次：{batch_id}", "",
        "| 字段 | 内容 |", "| --- | --- |",
        f"| 执行时间 | {executed_at} |", f"| 执行者 | {executor} |",
        "| 版本 | Isaac Sim 6.0；双 Isaac 进程实验入口 |",
        f"| 测试范围 | 主进程：完整机器人、物理、ROS 2 控制、无传感器；副本进程：机器人视觉副本、四台 RGB-D{'、MID360' if args.enable_lidar else ''}、只读状态同步；无 RViz |",
        f"| 配置 | 主进程 physics/render `{args.physics_hz}/{args.authority_render_hz} Hz`、CPU `{args.authority_cpu_set}`；副本 physics/render `{args.replica_physics_hz}/{args.replica_render_hz} Hz`、CPU `{args.replica_cpu_set}`；四台 `{args.camera_width}×{args.camera_height} @ {args.camera_tick_rate_hz} Hz` RGB-D；MID360=`{args.lidar_profile if args.enable_lidar else '未接入'}`、transport=`{args.lidar_transport if args.enable_lidar else '未接入'}`；副本模式=`{'纯视觉运动学' if args.visual_kinematic_replica else '完整机器人'}`；运动学状态施加=`{args.kinematic_sync_hz or '每步'}`；副本碰撞=`{'关闭' if args.disable_replica_collisions else '开启'}`；guide 碰撞网格隐藏=`{args.hide_replica_guide_meshes}`；视觉 LOD 代理=`{args.visual_lod_proxy}`；MinimalRendering=`{args.minimal_rendering}`；逐传感器 TLAS=`{not args.no_per_sensor_tick_tlas}`；AA=`{args.anti_aliasing}`；SRTX=`{args.srtx}`；发布队列线程=`{args.publish_with_queue_thread}`；逐帧 CameraInfo=`{not args.no_camera_info}`；UDP `127.0.0.1:{args.snapshot_port}` |",
        f"| 功能结论 | {function} |", f"| 实时结论 | {realtime} |", "",
        "| 指标 | 通过线 | 实测值 | 判定 |", "| --- | --- | ---: | --- |",
        f"| 主进程 `/joint_states` | 不低于 `90 Hz` | {_format_metric(timing.get('joint_states', {}).get('wall_receive_hz'))} Hz | {'通过' if float(timing.get('joint_states', {}).get('wall_receive_hz') or 0.0) >= 89.95 else '未通过'} |",
        f"| 主进程 physics step rate | 不低于 `{args.physics_hz} Hz` | {_format_metric(timing.get('sim_joint_states', {}).get('physics_step_rate_hz'))} Hz | {'通过' if float(timing.get('sim_joint_states', {}).get('physics_step_rate_hz') or 0.0) >= args.physics_hz else '未通过'} |",
        f"| 主进程 RTF | 不低于 `1.0` | {_format_metric(timing.get('clock', {}).get('rtf'))} | {'通过' if float(timing.get('clock', {}).get('rtf') or 0.0) >= 1.0 else '未通过'} |",
        f"| 机器人控制响应 | 底盘、升降、头部、双臂均响应 | 底盘={_format_metric(result.get('base', {}).get('odom_displacement_m'))} m；控制组={sum(1 for item in (result.get('groups') or {}).values() if item.get('response_ok'))}/4 | {'通过' if result.get('base', {}).get('response_ok') and all(item.get('response_ok') for item in (result.get('groups') or {}).values()) else '未通过'} |",
        f"| 副本 RenderProduct 数 | `4` | {replica.get('camera_suite', {}).get('camera_count', '未采集')} | {'通过' if replica.get('camera_suite', {}).get('camera_count') == 4 else '未通过'} |",
        f"| 副本 ROS 图像实收频率 | 每路不低于 `{args.min_camera_hz} Hz` | {_format_metric(min(image_range) if image_range else None)}–{_format_metric(max(image_range) if image_range else None)} Hz | {'通过' if image_range and min(image_range) >= args.min_camera_hz else '未通过'} |",
        *(
            (
                f"| MID360 PointCloud2 | 不低于 `{args.min_lidar_hz} Hz` 且非空 | {_format_metric(result.get('lidar_timing', {}).get('wall_receive_hz'))} Hz；非空={result.get('lidar_nonempty_samples', 0)}；末帧点数={result.get('last_lidar_point_count', 0)} | {'通过' if float(result.get('lidar_timing', {}).get('wall_receive_hz') or 0.0) >= args.min_lidar_hz and int(result.get('lidar_nonempty_samples') or 0) > 0 else '未通过'} |",
            )
            if args.enable_lidar
            else ()
        ),
        f"| UDP 状态快照 | 有接收并施加关节状态 | 收={replica.get('snapshot_messages_received', '未采集')}；施加={replica.get('snapshot_apply_updates', '未采集')}；关节={replica.get('snapshot_applied_dof_total', '未采集')}；relay={_format_metric(relay.get('snapshot_wall_hz'))} Hz | {'通过' if int(replica.get('snapshot_apply_updates') or 0) > 0 and int(replica.get('snapshot_applied_dof_total') or 0) > 0 else '未通过'} |",
        f"| 快照时延 P99 / max | 仅记录 | {_format_metric((replica.get('snapshot_age_p99_s') or 0.0) * 1e3)} / {_format_metric((replica.get('snapshot_age_max_s') or 0.0) * 1e3)} ms | 仅记录 |",
        "",
        "结论：该实验将相机渲染与物理权威进程分离，但副本只同步关节位置和里程计位姿，不同步接触、柔性、对象状态或精确跨进程仿真时间。因此即使通过，也只能作为阶段 4 前的架构验证，不是产品化传感器集成验收。",
        "",
        f"原始证据：`raw/{batch_id}/result.json`、`raw/{batch_id}/authority.launch.log`、`raw/{batch_id}/replica.launch.log`、`raw/{batch_id}/relay.metrics.json`、`raw/{batch_id}/replica/metrics.json`。", "",
    ))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--command-seconds", type=float, default=4.0)
    parser.add_argument("--startup-timeout", type=float, default=300.0)
    parser.add_argument("--camera-startup-timeout", type=float, default=180.0)
    parser.add_argument("--physics-hz", type=float, default=120.0)
    parser.add_argument("--authority-render-hz", type=float, default=30.0)
    # The replica owns neither contact nor control physics. Running its
    # visual update at camera cadence prevents three redundant articulation
    # updates for every RGB-D sample while preserving 30 Hz images.
    parser.add_argument("--replica-physics-hz", type=float, default=30.0)
    parser.add_argument("--replica-render-hz", type=float, default=30.0)
    parser.add_argument("--replica-warmup-steps", type=int, default=90)
    parser.add_argument(
        "--replica-stage",
        type=Path,
        default=None,
        help=(
            "visual replica stage; when omitted, camera-only uses robot_control_stage.usda "
            "and MID360 fusion uses the historical robot_camera_lidar_stage.usda"
        ),
    )
    parser.add_argument(
        "--separate-lidar-process",
        action="store_true",
        help="run MID360 in a dedicated Isaac Sim sensor process instead of the camera replica",
    )
    parser.add_argument(
        "--lidar-stage",
        type=Path,
        default=None,
        help="stage used by the dedicated MID360 process",
    )
    parser.add_argument("--pace-spin-us", type=float, default=750.0)
    parser.add_argument("--pace-sleep-guard-us", type=float, default=0.0)
    parser.add_argument("--kit-threads", type=int, default=16)
    parser.add_argument("--srtx", action="store_true", help="enable Isaac Sim native SRTX ROS2 camera transport in the replica")
    parser.add_argument("--publish-with-queue-thread", action="store_true", help="enable ROS2 bridge asynchronous publish queue thread in the replica")
    parser.add_argument("--minimal-rendering", action="store_true", help="use MinimalRendering in the visual-only replica")
    parser.add_argument("--no-per-sensor-tick-tlas", action="store_true", help="disable per-camera tick TLAS rebuilds in the visual-only replica")
    parser.add_argument("--disable-replica-collisions", action="store_true", help="remove collision schemas from the visual-only replica")
    parser.add_argument("--hide-replica-guide-meshes", action="store_true", help="hide guide-purpose collision meshes in the visual-only replica")
    parser.add_argument("--visual-lod-proxy", action="store_true", help="replace high-detail replica visuals with bounding-box proxies")
    parser.add_argument("--visual-kinematic-replica", action="store_true", help="run the camera replica without PhysX bodies or joints")
    parser.add_argument("--kinematic-sync-hz", type=float, default=0.0, help="cap visual-kinematic state updates; 0 applies each fresh snapshot")
    parser.add_argument("--anti-aliasing", type=int, choices=(0, 1, 2, 3, 4), default=None)
    parser.add_argument("--min-camera-hz", type=float, default=20.0, help="minimum acceptable per-stream ROS image rate")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--no-camera-info", action="store_true", help="disable per-frame CameraInfo for image-capture throughput probes")
    parser.add_argument("--camera-tick-rate-hz", type=float, default=30.0)
    parser.add_argument("--enable-lidar", action="store_true", help="add MID360 to the non-authoritative sensor replica")
    parser.add_argument("--lidar-profile", default="MID360_PERFORMANCE")
    parser.add_argument("--lidar-transport", choices=("helper", "native"), default="helper")
    parser.add_argument("--lidar-tick-rate-hz", type=float, default=10.0, help="MID360 simulation-time output tick rate")
    parser.add_argument("--no-lidar-object-id-map", action="store_true", help="disable the optional MID360 object-id map writer")
    parser.add_argument("--lidar-topic", default=DEFAULT_LIDAR_TOPIC)
    parser.add_argument("--lidar-raw-topic", default=DEFAULT_LIDAR_TOPIC + "_raw")
    parser.add_argument(
        "--restamp-lidar-to-authority-clock",
        action="store_true",
        help="republish the split MID360 stream with the authority /clock timestamp",
    )
    parser.add_argument(
        "--min-lidar-hz",
        type=float,
        default=8.0,
        help="联合整机测试的 MID360 非空 PointCloud2 最低墙钟频率（Hz）；独立雷达基线仍为 10 Hz",
    )
    parser.add_argument("--lidar-startup-timeout", type=float, default=180.0)
    # Keep the default split valid on the current 64-CPU host.  Historical
    # batches used 0-31/32-63/64-95 on a 96-CPU machine; an invalid lidar
    # affinity makes the whole fusion test fail before Isaac Sim starts.
    parser.add_argument("--authority-cpu-set", default="0-15")
    parser.add_argument("--replica-cpu-set", default="16-31")
    parser.add_argument("--lidar-cpu-set", default="32-47")
    parser.add_argument("--lidar-physics-hz", type=float, default=90.0)
    parser.add_argument("--lidar-render-hz", type=float, default=60.0)
    parser.add_argument("--lidar-kinematic-sync-hz", type=float, default=10.0)
    parser.add_argument(
        "--lidar-dynamic-transform-mode",
        choices=("sensor", "environment"),
        default="environment",
    )
    parser.add_argument("--snapshot-port", type=int, default=24101)
    parser.add_argument("--lidar-snapshot-port", type=int, default=24102)
    parser.add_argument("--api-port", type=int, default=8088)
    parser.add_argument("--base-speed", type=float, default=0.18)
    parser.add_argument("--min-joint-delta", type=float, default=0.002)
    parser.add_argument("--executor", default=os.environ.get("ISAACSIM_TEST_EXECUTOR", "manual"))
    parser.add_argument("--append-report", action="store_true")
    args = parser.parse_args()
    if args.replica_stage is None:
        args.replica_stage = ROOT / "isaac_sim_core" / "assets" / "environments" / (
            "robot_camera_lidar_stage.usda"
            if args.enable_lidar and not args.separate_lidar_process
            else "robot_control_stage.usda"
        )
    else:
        # The replica is launched as a separate Isaac Sim process.  Relative
        # stage paths make USD references resolve against that process's
        # working directory instead of this repository, which can leave the
        # robot visuals visible while composed PhysicsJoint prims disappear.
        # Always pass a canonical path so stage-relative asset references are
        # deterministic and match the historical successful runs.
        args.replica_stage = args.replica_stage.expanduser().resolve()
    if args.lidar_stage is None:
        args.lidar_stage = ROOT / "isaac_sim_core" / "assets" / "environments" / (
            "mid360_empty_stage.usda" if args.separate_lidar_process else "robot_camera_lidar_stage.usda"
        )
    else:
        args.lidar_stage = args.lidar_stage.expanduser().resolve()
    if args.duration < args.command_seconds or min(args.duration, args.physics_hz, args.replica_physics_hz, args.min_camera_hz, args.min_lidar_hz) <= 0.0:
        parser.error("duration must cover command-seconds; frequencies must be positive")
    if args.camera_width <= 0 or args.camera_height <= 0:
        parser.error("camera dimensions must be positive")
    if not 1 <= args.snapshot_port <= 65535 or not 1 <= args.lidar_snapshot_port <= 65535:
        parser.error("snapshot ports must be between 1 and 65535")
    if args.separate_lidar_process and args.snapshot_port == args.lidar_snapshot_port:
        parser.error("camera and lidar snapshot ports must differ")
    if not args.replica_stage.is_file():
        parser.error(f"replica stage does not exist: {args.replica_stage}")
    if args.separate_lidar_process and not args.lidar_stage.is_file():
        parser.error(f"lidar stage does not exist: {args.lidar_stage}")
    if args.camera_tick_rate_hz <= 0.0 or args.kinematic_sync_hz < 0.0 or args.lidar_tick_rate_hz <= 0.0:
        parser.error("camera and lidar tick rates must be positive; kinematic sync rate must be non-negative")
    if args.lidar_kinematic_sync_hz <= 0.0:
        parser.error("lidar kinematic sync rate must be positive")
    if args.restamp_lidar_to_authority_clock and not args.separate_lidar_process:
        parser.error("authority clock restamping requires --separate-lidar-process")
    if args.restamp_lidar_to_authority_clock and args.lidar_raw_topic == args.lidar_topic:
        parser.error("lidar raw and output topics must differ when restamping")
    batch_id = _batch_id(args.executor)
    batch_dir = RAW_ROOT / batch_id
    batch_dir.mkdir(parents=True, exist_ok=False)
    executed_at = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %z")
    rclpy.init()
    result = _run(args, batch_dir)
    result_path = batch_dir / "result.json"
    result_path.write_text(
        json.dumps({"batch_id": batch_id, "executed_at": executed_at, "executor": args.executor, "result": result}, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if args.append_report and result.get("status") == "COMPLETE":
        with REPORT_PATH.open("a", encoding="utf-8") as report:
            report.write("\n" + _markdown(batch_id, executed_at, args.executor, result, args))
    print(result_path)
    return 0 if result.get("status") == "COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
