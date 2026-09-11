#!/usr/bin/env python3
"""测试无传感器底盘控制在三种显示模式下的功能和实时性能。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from sensor_msgs.msg import JointState


ROOT = Path(__file__).resolve().parents[2]
REPORT_DIR = ROOT / "reports" / "performance"
REPORT_PATH = REPORT_DIR / "robot_control_performance.md"
RAW_ROOT = REPORT_DIR / "raw" / "robot_control_baseline"
REPORT_MARKER = "<!-- ROBOT_CONTROL_BASELINE_BATCHES -->"
WHEEL_JOINTS = ("fl_wheel_joint", "fr_wheel_joint", "bl_wheel_joint", "br_wheel_joint")


MODES = (
    {"name": "headless_no_rviz", "headless": "true", "rviz": "false", "qt": ""},
    {"name": "headless_rviz", "headless": "true", "rviz": "true", "qt": ""},
    {"name": "gui_rviz", "headless": "false", "rviz": "true", "qt": ""},
)


@dataclass
class TopicSamples:
    """Keep transport timing and simulation-time timing as separate evidence.

    ``wall`` contains one monotonic receive timestamp per ROS callback.  This
    is the value used for the externally visible ROS frequency.  ``segments``
    contains only strictly increasing message header/simulation timestamps;
    repeated timestamps are transport duplicates and backwards jumps are
    invalid measurement evidence (normally caused by a simulation restart).
    """

    wall: list[float] = field(default_factory=list)
    segments: list[list[tuple[float, float]]] = field(default_factory=list)
    last_sim: float | None = None
    sim_duplicate_count: int = 0
    sim_backward_count: int = 0
    sim_invalid_count: int = 0

    def add(self, wall_time: float, sim_time: float | None = None) -> None:
        self.wall.append(wall_time)
        if sim_time is None or sim_time <= 0.0:
            self.sim_invalid_count += 1
            return
        if self.last_sim is None:
            self.segments.append([(wall_time, sim_time)])
        elif sim_time < self.last_sim:
            self.sim_backward_count += 1
            # Start a new epoch, but retain the reset as invalid evidence.
            self.segments.append([(wall_time, sim_time)])
        elif sim_time == self.last_sim:
            self.sim_duplicate_count += 1
        else:
            if not self.segments:
                self.segments.append([])
            self.segments[-1].append((wall_time, sim_time))
        self.last_sim = sim_time

    def reset(self) -> None:
        self.wall.clear()
        self.segments.clear()
        self.last_sim = None
        self.sim_duplicate_count = 0
        self.sim_backward_count = 0
        self.sim_invalid_count = 0

    def stats(self, physics_dt: float | None = None) -> dict[str, Any]:
        wall_stats = _interval_stats(self.wall)
        segment = max(self.segments, key=lambda values: len(values), default=[])
        sim_values = [sim for _wall, sim in segment]
        sim_stats = _interval_stats(sim_values)
        sim_seconds = None
        sim_wall_seconds = None
        rtf = None
        estimated_steps = None
        physics_step_rate = None
        if len(segment) >= 2:
            sim_seconds = segment[-1][1] - segment[0][1]
            sim_wall_seconds = segment[-1][0] - segment[0][0]
            if sim_wall_seconds > 0.0:
                rtf = sim_seconds / sim_wall_seconds
                if physics_dt and physics_dt > 0.0:
                    estimated_steps = sum(
                        max(1, int(round((b[1] - a[1]) / physics_dt)))
                        for a, b in zip(segment, segment[1:])
                    )
                    physics_step_rate = estimated_steps / sim_wall_seconds
        return {
            "wall_receive": wall_stats,
            "sim_timestamp": sim_stats,
            "sim_time_seconds": sim_seconds,
            "sim_wall_seconds": sim_wall_seconds,
            "rtf": rtf,
            "estimated_physics_steps": estimated_steps,
            "physics_step_rate_hz": physics_step_rate,
            "sim_timestamp_segments": len(self.segments),
            "sim_duplicate_count": self.sim_duplicate_count,
            "sim_backward_count": self.sim_backward_count,
            "sim_invalid_count": self.sim_invalid_count,
            "measurement_valid": self.sim_backward_count == 0 and len(segment) >= 2,
        }


def _stamp_seconds(message: Any) -> float | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _interval_stats(values: list[float]) -> dict[str, float | int | None]:
    if len(values) < 2:
        return {"count": len(values), "hz": None, "mean_gap_ms": None, "p99_gap_ms": None, "max_gap_ms": None}
    gaps = sorted((b - a) for a, b in zip(values, values[1:]) if b >= a)
    if not gaps:
        return {"count": len(values), "hz": None, "mean_gap_ms": None, "p99_gap_ms": None, "max_gap_ms": None}
    p99_index = min(len(gaps) - 1, max(0, math.ceil(len(gaps) * 0.99) - 1))
    mean_gap = sum(gaps) / len(gaps)
    return {
        "count": len(values),
        "hz": 1.0 / mean_gap if mean_gap > 0.0 else None,
        "mean_gap_ms": mean_gap * 1000.0,
        "p99_gap_ms": gaps[p99_index] * 1000.0,
        "max_gap_ms": gaps[-1] * 1000.0,
    }


class ControlSamples(Node):
    def __init__(self) -> None:
        super().__init__("measure_robot_control_performance")
        self.samples: dict[str, TopicSamples] = {
            name: TopicSamples()
            for name in ("clock", "sim_joint_states", "joint_states", "odom")
        }
        self.latest_joint: JointState | None = None
        self.latest_odom: Odometry | None = None
        self.initial_odom_xy: tuple[float, float] | None = None
        self.clock_time: float | None = None
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(JointState, "/openflex/joint_states", self._sim_joint, 10)
        self.create_subscription(JointState, "/joint_states", self._joint, 10)
        self.create_subscription(Odometry, "/odom", self._odom, 10)

    def _on_clock(self, message: Clock) -> None:
        stamp = float(message.clock.sec) + float(message.clock.nanosec) / 1_000_000_000.0
        self.clock_time = stamp
        self.samples["clock"].add(time.monotonic(), stamp)

    def _sim_joint(self, message: JointState) -> None:
        self.samples["sim_joint_states"].add(time.monotonic(), _stamp_seconds(message))

    def _joint(self, message: JointState) -> None:
        self.latest_joint = message
        self.samples["joint_states"].add(time.monotonic(), _stamp_seconds(message))

    def _odom(self, message: Odometry) -> None:
        self.latest_odom = message
        position = message.pose.pose.position
        if self.initial_odom_xy is None:
            self.initial_odom_xy = (float(position.x), float(position.y))
        self.samples["odom"].add(time.monotonic(), _stamp_seconds(message))

    def publish_cmd(self, linear_x: float) -> None:
        command = Twist()
        command.linear.x = linear_x
        self.cmd_pub.publish(command)

    def reset_measurement(self) -> None:
        """Drop startup/warmup samples before the audited measurement window."""
        for sample in self.samples.values():
            sample.reset()

    def wheel_speed(self) -> float:
        if self.latest_joint is None:
            return 0.0
        velocities = dict(zip(self.latest_joint.name, self.latest_joint.velocity))
        return max((abs(float(velocities.get(name, 0.0))) for name in WHEEL_JOINTS), default=0.0)

    def displacement(self) -> float:
        if self.initial_odom_xy is None or self.latest_odom is None:
            return 0.0
        position = self.latest_odom.pose.pose.position
        return math.hypot(float(position.x) - self.initial_odom_xy[0], float(position.y) - self.initial_odom_xy[1])


def _launch_command(
    mode: dict[str, str], api_port: int, physics_hz: float, controller_use_sim_time: bool
) -> list[str]:
    command = [
        "ros2",
        "launch",
        "isaacsim_bringup",
        "robot_control_only.launch.py",
        f"headless:={mode['headless']}",
        f"rviz:={mode['rviz']}",
        "start_upper_body:=false",
        # A/B switch: with simulation time enabled, controller-manager's
        # wall-clock receive rate is scaled by scene RTF. Keep the default
        # launch semantics explicit for this baseline; use the command-line
        # flag below to measure a wall-clock 90 Hz controller loop.
        f"controller_use_sim_time:={'true' if controller_use_sim_time else 'false'}",
        f"physics_hz:={physics_hz}",
        f"api_port:={api_port}",
    ]
    if mode["qt"]:
        command.append(f"qt_qpa_platform:={mode['qt']}")
    return command


def _resource_snapshot(process: subprocess.Popen[str]) -> dict[str, Any]:
    result: dict[str, Any] = {"cpu_percent": None, "gpu": None, "gpu_error": None}
    try:
        import psutil

        process_info = psutil.Process(process.pid)
        processes = [process_info, *process_info.children(recursive=True)]
        cpu_times = sum((item.cpu_times().user + item.cpu_times().system) for item in processes if item.is_running())
        result["cpu_seconds"] = cpu_times
    except Exception as error:
        result["process_resource_error"] = str(error)
    try:
        gpu = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=name,utilization.gpu,memory.used,temperature.gpu",
                "--format=csv,noheader,nounits",
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
        if gpu.returncode == 0:
            result["gpu"] = [line.strip() for line in gpu.stdout.splitlines() if line.strip()]
        else:
            result["gpu_error"] = gpu.stderr.strip() or f"nvidia-smi exit {gpu.returncode}"
    except Exception as error:
        result["gpu_error"] = str(error)
    return result


def _rviz_state(process: subprocess.Popen[str], log_path: Path) -> dict[str, bool]:
    """Report whether RViz started and survived the measurement window."""
    started = False
    failed = False
    try:
        log_text = log_path.read_text(encoding="utf-8", errors="replace")
        started = bool(re.search(r"\[rviz2-[^\]]+\]: process started with pid", log_text))
        failed = bool(re.search(r"\[rviz2-[^\]]+\]: process has died", log_text))
    except OSError:
        pass
    try:
        import psutil

        root = psutil.Process(process.pid)
        alive = any(
            "rviz2" in " ".join(child.cmdline())
            for child in root.children(recursive=True)
            if child.is_running()
        )
    except Exception:
        alive = False
    return {"started": started, "failed": failed, "alive": alive, "survived": started and not failed}


def _tail(path: Path, limit: int = 80) -> str:
    try:
        return "\n".join(path.read_text(encoding="utf-8", errors="replace").splitlines()[-limit:])
    except OSError as error:
        return f"unable to read launch log: {error}"


def _stop_process(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=15.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass


def run_mode(
    mode: dict[str, str], args: argparse.Namespace, node: ControlSamples, batch_dir: Path
) -> dict[str, Any]:
    log_path = batch_dir / f"{mode['name']}.launch.log"
    environment = os.environ.copy()
    environment.setdefault("RCUTILS_COLORIZED_OUTPUT", "0")
    command = _launch_command(
        mode, args.api_port, args.physics_hz, args.controller_use_sim_time
    )
    started = time.monotonic()
    with log_path.open("w", encoding="utf-8") as log_file:
        process = subprocess.Popen(
            command,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            env=environment,
            start_new_session=True,
            text=True,
        )
        ready_deadline = started + args.startup_timeout
        while time.monotonic() < ready_deadline and process.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                node.clock_time is not None
                and node.latest_joint is not None
                and node.latest_odom is not None
            ):
                break
        ready = node.clock_time is not None and node.latest_joint is not None and node.latest_odom is not None
        result: dict[str, Any] = {
            "mode": mode["name"],
            "headless": mode["headless"] == "true",
            "rviz_requested": mode["rviz"] == "true",
            "controller_use_sim_time": args.controller_use_sim_time,
            "qt_qpa_platform": mode["qt"],
            "command": command,
            "launch_log": str(log_path),
            "startup_seconds": time.monotonic() - started,
            "ready": ready,
            "launch_returncode_before_stop": process.poll(),
        }
        if ready:
            for _ in range(max(1, int(args.warmup_seconds * 10))):
                rclpy.spin_once(node, timeout_sec=0.1)
            node.reset_measurement()
            measurement_start = time.monotonic()
            end = measurement_start + args.duration
            max_wheel_speed = 0.0
            command_end = measurement_start + args.command_seconds
            while time.monotonic() < end and process.poll() is None:
                now = time.monotonic()
                node.publish_cmd(args.command_speed if now < command_end else 0.0)
                rclpy.spin_once(node, timeout_sec=0.05)
                max_wheel_speed = max(max_wheel_speed, node.wheel_speed())
            node.publish_cmd(0.0)
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)
            rviz_state = _rviz_state(process, log_path)
            control_ok = max_wheel_speed > 0.01 and node.displacement() > 0.001
            rviz_ok = not result["rviz_requested"] or rviz_state["survived"]
            topic_metrics = {
                name: samples.stats(physics_dt=1.0 / args.physics_hz)
                for name, samples in node.samples.items()
            }
            timing_valid = all(
                topic_metrics[name]["measurement_valid"]
                for name in ("clock", "sim_joint_states", "joint_states", "odom")
            )
            if not timing_valid:
                status = "NOT_VALID_TIME_RESET"
            elif not control_ok:
                status = "FAIL_CONTROL_RESPONSE"
            elif not rviz_ok:
                status = "FAIL_RVIZ"
            else:
                status = "PASS"
            result.update(
                {
                    "status": status,
                    "duration_seconds": time.monotonic() - measurement_start,
                    "measurement": {
                        "wall_start_monotonic": measurement_start,
                        "wall_end_monotonic": time.monotonic(),
                        "configured_physics_hz": args.physics_hz,
                        "configured_physics_dt_s": 1.0 / args.physics_hz,
                        "timing_valid": timing_valid,
                    },
                    "control": {
                        "max_wheel_speed_rad_s": max_wheel_speed,
                        "odom_displacement_m": node.displacement(),
                        "response_ok": max_wheel_speed > 0.01 and node.displacement() > 0.001,
                    },
                    "topics": topic_metrics,
                    "rviz_seen": rviz_state["survived"],
                    "rviz": rviz_state,
                }
            )
        else:
            result["status"] = "NOT_RUN"
            result["failure"] = "launch exited or required control topics did not become ready"
        result["resource"] = _resource_snapshot(process)
        result["launch_returncode"] = process.poll()
        result["log_tail"] = _tail(log_path)
        _stop_process(process)
    return result


def _conclusion_for_status(status: str) -> str:
    return {
        "PASS": "通过",
        "NOT_RUN": "未运行",
        "TEST_ERROR": "测试异常",
    }.get(status, "未通过")


def _batch_id(executor: str) -> str:
    safe_executor = re.sub(r"[^A-Za-z0-9_-]+", "-", executor).strip("-") or "manual"
    return f"{datetime.now().astimezone().strftime('%Y%m%d-%H%M%S')}-底盘基线-{safe_executor}"


def _format_number(value: Any, digits: int = 2) -> str:
    return "未采集" if value is None else f"{float(value):.{digits}f}"


def _markdown(
    results: list[dict[str, Any]], batch_id: str, executed_at: str, executor: str, physics_hz: float
) -> str:
    functional = [_conclusion_for_status(str(item.get("status", "NOT_RUN"))) for item in results]
    if any(value == "测试异常" for value in functional):
        function_conclusion = "测试异常"
    elif any(value == "未运行" for value in functional):
        function_conclusion = "未运行"
    elif all(value == "通过" for value in functional):
        function_conclusion = "通过"
    else:
        function_conclusion = "未通过"

    realtime_ready = all(
        item.get("topics", {}).get("sim_joint_states", {}).get("physics_step_rate_hz") is not None
        and item.get("topics", {}).get("joint_states", {}).get("wall_receive", {}).get("hz") is not None
        and item.get("topics", {}).get("clock", {}).get("rtf") is not None
        for item in results
    )
    realtime_passed = realtime_ready and all(
        float(item["topics"]["sim_joint_states"]["physics_step_rate_hz"]) >= physics_hz
        and float(item["topics"]["joint_states"]["wall_receive"]["hz"]) >= 90.0
        and float(item["topics"]["clock"]["rtf"]) >= 1.0
        for item in results
    )
    realtime_conclusion = "通过" if realtime_passed else ("未通过" if realtime_ready else "未运行")
    raw_path = f"raw/robot_control_baseline/{batch_id}/result.json"
    lines = [
        f"### 批次：{batch_id}",
        "",
        "| 字段 | 内容 |",
        "| --- | --- |",
        f"| 执行时间 | {executed_at} |",
        f"| 执行者 | {executor} |",
        "| 版本 | 未记录 |",
        "| 测试范围 | 无传感器底盘控制；`/cmd_vel`、`/joint_states`、`/odom`；三种显示模式 |",
        f"| 配置 | physics `{physics_hz:g} Hz`；`/joint_states` 目标 `90 Hz`；不启动相机、雷达、IMU、传感器桥或 SLAM |",
        f"| 功能结论 | {function_conclusion} |",
        f"| 实时结论 | {realtime_conclusion} |",
        "",
        "| 模式 | physics step rate | `/joint_states` | RTF | 底盘响应 | 功能判定 | 实时判定 |",
        "| --- | ---: | ---: | ---: | --- | --- | --- |",
    ]
    for item in results:
        topics = item.get("topics", {})
        clock = topics.get("clock", {}).get("wall_receive", {}).get("hz")
        joints = topics.get("joint_states", {}).get("wall_receive", {}).get("hz")
        physics = topics.get("sim_joint_states", {}).get("physics_step_rate_hz")
        rtf = topics.get("clock", {}).get("rtf")
        control = item.get("control", {}).get("response_ok", False)
        functional_status = _conclusion_for_status(str(item.get("status", "NOT_RUN")))
        realtime_status = (
            "通过"
            if physics is not None and joints is not None and rtf is not None
            and float(physics) >= physics_hz and float(joints) >= 90.0 and float(rtf) >= 1.0
            else ("未运行" if physics is None or joints is None or rtf is None else "未通过")
        )
        lines.append(
            f"| `{item['mode']}` | {_format_number(physics)} Hz | {_format_number(joints)} Hz | "
            f"{_format_number(rtf, 3)} | {'通过' if control else '未通过'} | {functional_status} | {realtime_status} |"
        )
    lines.extend(
        [
            "",
            f"结论：功能以控制响应、时间有效性和请求的 RViz 存活为准；实时以 physics 不低于 `{physics_hz:g} Hz`、"
            "`/joint_states` 不低于 `90 Hz`、RTF 不低于 `1.0` 为准。详细时间间隔、重复时间戳和内部状态码见原始 JSON。",
            "",
            f"原始证据：`{raw_path}` 及同目录各模式启动日志。",
        ]
    )
    return "\n".join(lines) + "\n"


def _append_batch(markdown: str) -> None:
    report_text = REPORT_PATH.read_text(encoding="utf-8")
    if REPORT_MARKER not in report_text:
        raise RuntimeError(f"report marker missing: {REPORT_MARKER}")
    REPORT_PATH.write_text(
        report_text.replace(REPORT_MARKER, f"{REPORT_MARKER}\n\n{markdown.rstrip()}\n", 1),
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--warmup-seconds", type=float, default=3.0)
    parser.add_argument("--command-seconds", type=float, default=3.0)
    parser.add_argument("--command-speed", type=float, default=0.10)
    parser.add_argument("--startup-timeout", type=float, default=180.0)
    parser.add_argument("--api-port", type=int, default=8091)
    parser.add_argument("--physics-hz", type=float, default=120.0)
    parser.add_argument(
        "--controller-use-sim-time",
        action="store_true",
        help="Use Isaac simulation time for controller_manager (default: wall clock).",
    )
    parser.add_argument("--ros-domain-id", type=int, default=None)
    parser.add_argument("--mode", choices=[item["name"] for item in MODES], action="append")
    parser.add_argument("--executor", default=os.environ.get("ISAACSIM_TEST_EXECUTOR", "manual"))
    args = parser.parse_args()
    if args.physics_hz <= 0.0:
        parser.error("--physics-hz must be positive")
    if args.ros_domain_id is not None:
        os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    elif not os.environ.get("ROS_DOMAIN_ID"):
        # Keep this run away from unrelated ROS graphs left by interactive
        # sessions. The same environment is inherited by ros2 launch.
        os.environ["ROS_DOMAIN_ID"] = str(200 + (os.getpid() % 30))
    selected = [item for item in MODES if not args.mode or item["name"] in args.mode]

    batch_id = _batch_id(args.executor)
    batch_dir = RAW_ROOT / batch_id
    batch_dir.mkdir(parents=True, exist_ok=False)
    executed_at = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %z")
    rclpy.init()
    results: list[dict[str, Any]] = []
    try:
        for index, mode in enumerate(selected):
            node = ControlSamples()
            try:
                try:
                    results.append(run_mode(mode, args, node, batch_dir))
                except Exception as error:
                    results.append(
                        {
                            "mode": mode["name"],
                            "headless": mode["headless"] == "true",
                            "rviz_requested": mode["rviz"] == "true",
                            "status": "TEST_ERROR",
                            "ready": False,
                            "failure": f"performance test utility failed: {error}",
                        }
                    )
            finally:
                node.destroy_node()
            args.api_port += index + 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    raw_path = batch_dir / "result.json"
    raw_path.write_text(
        json.dumps(
            {"batch_id": batch_id, "executed_at": executed_at, "executor": args.executor, "results": results},
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    _append_batch(_markdown(results, batch_id, executed_at, args.executor, args.physics_hz))
    print(REPORT_PATH)
    return 0 if all(item.get("status") == "PASS" for item in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
