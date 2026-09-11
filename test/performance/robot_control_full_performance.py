#!/usr/bin/env python3
"""测试完整机器人控制，并可选测量四路相机负载。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Any

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState, PointCloud2
from std_msgs.msg import Float64MultiArray


ROOT = Path(__file__).resolve().parents[2]
REPORT_DIR = ROOT / "reports" / "performance"
REPORT_PATH = REPORT_DIR / "robot_control_performance.md"
RAW_ROOT = REPORT_DIR / "raw" / "robot_control_full"
REPORT_MARKER = "<!-- ROBOT_CONTROL_FULL_BATCHES -->"
WHEEL_JOINTS = ("fl_wheel_joint", "fr_wheel_joint", "bl_wheel_joint", "br_wheel_joint")
JOINT_STATES_TOLERANCE_HZ = 0.05
CAMERA_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)
CAMERA_STREAMS = {
    "base_rgb": "/openflex/sensors/cam_base/color/image",
    "base_depth": "/openflex/sensors/cam_base/depth/image",
    "head_rgb": "/openflex/sensors/cam_head/color/image",
    "head_depth": "/openflex/sensors/cam_head/depth/image",
    "left_wrist_rgb": "/openflex/sensors/cam_left/color/image",
    "left_wrist_depth": "/openflex/sensors/cam_left/depth/image",
    "right_wrist_rgb": "/openflex/sensors/cam_right/color/image",
    "right_wrist_depth": "/openflex/sensors/cam_right/depth/image",
}


@dataclass(frozen=True)
class Group:
    name: str
    topic: str
    joints: tuple[str, ...]
    target: tuple[float, ...]


@dataclass
class TimingSamples:
    """保存单个话题的墙钟与仿真时间，用于同一窗口内的实时测量。"""

    wall: list[float]
    sim: list[float]
    duplicate_count: int = 0
    backward_count: int = 0

    def __init__(self) -> None:
        self.wall = []
        self.sim = []
        self.duplicate_count = 0
        self.backward_count = 0

    def add(self, wall: float, sim: float | None) -> None:
        if sim is None or sim <= 0.0:
            return
        if self.sim and sim < self.sim[-1]:
            self.backward_count += 1
            return
        if self.sim and sim == self.sim[-1]:
            self.duplicate_count += 1
            return
        self.wall.append(wall)
        self.sim.append(sim)

    def reset(self) -> None:
        self.wall.clear()
        self.sim.clear()
        self.duplicate_count = 0
        self.backward_count = 0

    def metrics(self, physics_hz: float | None = None) -> dict[str, float | int | bool | None]:
        if len(self.wall) < 2 or len(self.sim) < 2:
            return {
                "wall_receive_hz": None,
                "rtf": None,
                "physics_step_rate_hz": None,
                "samples": len(self.wall),
                "duplicate_count": self.duplicate_count,
                "backward_count": self.backward_count,
                "valid": False,
            }
        wall_seconds = self.wall[-1] - self.wall[0]
        sim_seconds = self.sim[-1] - self.sim[0]
        if wall_seconds <= 0.0 or sim_seconds < 0.0:
            return {
                "wall_receive_hz": None,
                "rtf": None,
                "physics_step_rate_hz": None,
                "samples": len(self.wall),
                "duplicate_count": self.duplicate_count,
                "backward_count": self.backward_count,
                "valid": False,
            }
        step_rate = None
        if physics_hz and physics_hz > 0.0:
            step_dt = 1.0 / physics_hz
            steps = sum(
                max(1, int(round((end - begin) / step_dt)))
                for begin, end in zip(self.sim, self.sim[1:])
            )
            step_rate = steps / wall_seconds
        return {
            "wall_receive_hz": (len(self.wall) - 1) / wall_seconds,
            "rtf": sim_seconds / wall_seconds,
            "physics_step_rate_hz": step_rate,
            "samples": len(self.wall),
            "duplicate_count": self.duplicate_count,
            "backward_count": self.backward_count,
            "valid": self.backward_count == 0,
        }


GROUPS = (
    Group("lift", "/lift_position_controller/commands", ("lift_joint",), (0.12,)),
    Group(
        "head",
        "/head_forward_position_controller/commands",
        ("openarmx_head_yaw_joint", "openarmx_head_pitch_joint"),
        (0.12, -0.08),
    ),
    Group(
        "left_arm",
        "/left_forward_position_controller/commands",
        tuple(f"openarmx_left_joint{i}" for i in range(1, 8))
        + ("openarmx_left_finger_joint1",),
        (0.08, -0.06, 0.05, 0.04, 0.03, -0.02, 0.04, 0.018),
    ),
    Group(
        "right_arm",
        "/right_forward_position_controller/commands",
        tuple(f"openarmx_right_joint{i}" for i in range(1, 8))
        + ("openarmx_right_finger_joint1",),
        (-0.08, 0.06, -0.05, 0.04, -0.03, 0.02, -0.04, 0.018),
    ),
)

REQUIRED_CONTROLLERS = (
    "joint_state_broadcaster",
    "swerve_drive_controller",
    "left_forward_position_controller",
    "right_forward_position_controller",
    "head_forward_position_controller",
    "lift_position_controller",
)


class FullControlNode(Node):
    def __init__(self, camera_profile: str = "none", lidar_topic: str | None = None) -> None:
        super().__init__("robot_control_full_performance")
        self.positions: dict[str, float] = {}
        self.velocities: dict[str, float] = {}
        self.clock: Clock | None = None
        self.odom: Odometry | None = None
        self.sim_joint_state: JointState | None = None
        self.timing = {
            "clock": TimingSamples(),
            "sim_joint_states": TimingSamples(),
            "joint_states": TimingSamples(),
        }
        self.camera_timing = {name: TimingSamples() for name in CAMERA_STREAMS}
        self.lidar_timing = TimingSamples()
        self.lidar_authority_clock_skew_s: list[float] = []
        self.lidar_nonempty_samples = 0
        self.last_lidar_point_count = 0
        self.controller_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.command_publishers = {
            group.name: self.create_publisher(Float64MultiArray, group.topic, 10)
            for group in GROUPS
        }
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(JointState, "/openflex/joint_states", self._sim_joint_cb, 10)
        self.create_subscription(Clock, "/clock", self._clock_cb, 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        if camera_profile == "quad":
            for name, topic in CAMERA_STREAMS.items():
                self.create_subscription(
                    Image,
                    topic,
                    lambda message, stream=name: self._camera_cb(stream, message),
                    CAMERA_QOS,
                )
        if lidar_topic:
            self.create_subscription(
                PointCloud2,
                lidar_topic,
                self._lidar_cb,
                CAMERA_QOS,
            )

    def _joint_cb(self, message: JointState) -> None:
        self.positions.update(
            {name: float(value) for name, value in zip(message.name, message.position)}
        )
        self.velocities.update(
            {name: float(value) for name, value in zip(message.name, message.velocity)}
        )
        self.timing["joint_states"].add(time.monotonic(), _header_seconds(message))

    def _clock_cb(self, message: Clock) -> None:
        self.clock = message
        self.timing["clock"].add(time.monotonic(), _clock_seconds(message))

    def _sim_joint_cb(self, message: JointState) -> None:
        self.sim_joint_state = message
        self.timing["sim_joint_states"].add(time.monotonic(), _header_seconds(message))

    def _odom_cb(self, message: Odometry) -> None:
        self.odom = message

    def _camera_cb(self, stream: str, message: Image) -> None:
        self.camera_timing[stream].add(time.monotonic(), _header_seconds(message))

    def _lidar_cb(self, message: PointCloud2) -> None:
        stamp_s = _header_seconds(message)
        self.lidar_timing.add(time.monotonic(), stamp_s)
        if stamp_s is not None and self.clock is not None:
            self.lidar_authority_clock_skew_s.append(stamp_s - _clock_seconds(self.clock))
        points = int(message.width) * max(1, int(message.height))
        self.last_lidar_point_count = points
        if points > 0:
            self.lidar_nonempty_samples += 1

    def controller_states(self, timeout_sec: float = 2.0) -> dict[str, str]:
        if not self.controller_client.wait_for_service(timeout_sec=timeout_sec):
            return {}
        future = self.controller_client.call_async(ListControllers.Request())
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or future.exception() is not None:
            return {}
        return {item.name: item.state for item in future.result().controller}

    def ready(self) -> bool:
        required_joints = {joint for group in GROUPS for joint in group.joints}
        states = self.controller_states(timeout_sec=0.2)
        return (
            self.clock is not None
            and self.odom is not None
            and required_joints.issubset(self.positions)
            and all(states.get(name) == "active" for name in REQUIRED_CONTROLLERS)
        )

    def publish_commands(self, base_speed: float) -> None:
        twist = Twist()
        twist.linear.x = base_speed
        self.cmd_vel.publish(twist)
        for group in GROUPS:
            self.command_publishers[group.name].publish(Float64MultiArray(data=list(group.target)))

    def missing_command_subscriptions(self) -> list[str]:
        missing = []
        if self.cmd_vel.get_subscription_count() < 1:
            missing.append("/cmd_vel")
        for group in GROUPS:
            if self.command_publishers[group.name].get_subscription_count() < 1:
                missing.append(group.topic)
        return missing

    def wait_for_command_subscriptions(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            missing = self.missing_command_subscriptions()
            if not missing:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = self.missing_command_subscriptions()
        if missing:
            raise RuntimeError(
                "command publisher discovery did not complete: " + ", ".join(missing)
            )

    def stop(self) -> None:
        if not rclpy.ok():
            return
        try:
            self.cmd_vel.publish(Twist())
            for group in GROUPS:
                self.command_publishers[group.name].publish(
                    Float64MultiArray(data=[0.0] * len(group.joints))
                )
        except Exception:  # noqa: BLE001
            pass

    def reset_timing(self) -> None:
        for sample in self.timing.values():
            sample.reset()
        for sample in self.camera_timing.values():
            sample.reset()
        self.lidar_timing.reset()
        self.lidar_authority_clock_skew_s.clear()
        self.lidar_nonempty_samples = 0
        self.last_lidar_point_count = 0

    def max_wheel_speed(self) -> float:
        return max((abs(self.velocities.get(name, 0.0)) for name in WHEEL_JOINTS), default=0.0)

    def odom_xy(self) -> tuple[float, float] | None:
        if self.odom is None:
            return None
        position = self.odom.pose.pose.position
        return float(position.x), float(position.y)


def _header_seconds(message: JointState) -> float | None:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _clock_seconds(message: Clock) -> float:
    return float(message.clock.sec) + float(message.clock.nanosec) / 1_000_000_000.0


def _launch_command(
    api_port: int,
    physics_hz: float,
    render_hz: float,
    camera_profile: str,
    lidar_profile: str,
    lidar_transport: str,
    lidar_mount_mode: str,
    stage: str,
) -> list[str]:
    command = [
        "ros2",
        "launch",
        "isaacsim_bringup",
        "robot_control_only.launch.py",
        "headless:=true",
        "rviz:=false",
        f"physics_hz:={physics_hz}",
        f"render_hz:={render_hz}",
        f"camera_profile:={camera_profile}",
        f"lidar_profile:={lidar_profile}",
        f"lidar_transport:={lidar_transport}",
        f"lidar_mount_mode:={lidar_mount_mode}",
        f"api_port:={api_port}",
    ]
    if stage.strip().lower() not in {"", "auto", "default"}:
        command.append(f"stage:={stage}")
    return command


def _stop_process(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=20.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass


def _existing_isaac_processes() -> list[str]:
    """Find other Isaac processes before starting a GPU benchmark."""
    result = subprocess.run(
        ["ps", "-eo", "pid=,args="],
        check=False,
        text=True,
        capture_output=True,
    )
    current_pid = str(os.getpid())
    matches = []
    for line in result.stdout.splitlines():
        # Do not match the workspace name (``isaacsim_robot``) or the Codex
        # launcher that happens to contain that path. Match an actual Kit/
        # Isaac executable or the dedicated child launcher instead.
        lowered = line.lower()
        is_isaac_executable = any(
            marker in lowered
            for marker in (
                "/isaacsim-",
                "/isaacsim/",
                "/isaac-sim",
                "start_robot_control_sim.py",
            )
        )
        if not is_isaac_executable:
            continue
        if line.lstrip().startswith(current_pid + " "):
            continue
        matches.append(line.strip())
    return matches


def _wait_until_ready(
    node: FullControlNode,
    process: subprocess.Popen[str],
    timeout_sec: float,
) -> dict[str, str]:
    deadline = time.monotonic() + timeout_sec
    last_states: dict[str, str] = {}
    while (
        rclpy.ok()
        and time.monotonic() < deadline
        and process.poll() is None
        and (node.sim_joint_state is None or node.clock is None)
    ):
        rclpy.spin_once(node, timeout_sec=0.1)

    # Spawners are deliberately serialized by the launch file. Do not poll
    # controller_manager while that chain is performing load/configure/activate
    # service calls; on Humble/Fast DDS concurrent requests can abort the node.
    settle_until = min(deadline, time.monotonic() + 20.0)
    while rclpy.ok() and time.monotonic() < settle_until and process.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    if process.poll() is None:
        last_states = node.controller_states(timeout_sec=5.0)
        required_joints = {joint for group in GROUPS for joint in group.joints}
        if (
            node.clock is not None
            and node.odom is not None
            and required_joints.issubset(node.positions)
            and all(last_states.get(name) == "active" for name in REQUIRED_CONTROLLERS)
        ):
            return last_states
    missing = sorted({joint for group in GROUPS for joint in group.joints} - set(node.positions))
    inactive = {
        name: last_states.get(name, "missing")
        for name in REQUIRED_CONTROLLERS
        if last_states.get(name) != "active"
    }
    if process.poll() is not None:
        raise RuntimeError(
            f"launch exited with code {process.returncode} before control graph was ready; "
            f"missing_joints={missing}, inactive={inactive}"
        )
    raise RuntimeError(f"control graph not ready: missing_joints={missing}, inactive={inactive}")


def _run(args: argparse.Namespace, batch_dir: Path) -> dict[str, Any]:
    concurrent = _existing_isaac_processes()
    if concurrent:
        return {
            "status": "TEST_ERROR_CONCURRENT_ISAAC",
            "error": (
                "another Isaac Sim process is already using the benchmark GPU; "
                "the benchmark was not started"
            ),
            "concurrent_isaac_processes_before_start": concurrent,
        }
    log_path = batch_dir / "launch.log"
    launch_command = _launch_command(
        args.api_port,
        args.physics_hz,
        args.render_hz,
        args.camera_profile,
        args.lidar_profile,
        args.lidar_transport,
        args.lidar_mount_mode,
        args.stage,
    )
    process = subprocess.Popen(
        launch_command,
        stdout=log_path.open("w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )
    node = FullControlNode(
        args.camera_profile,
        args.lidar_topic if args.lidar_profile != "none" else None,
    )
    started = time.monotonic()
    result: dict[str, Any] = {
        "command": launch_command,
        "launch_log": str(log_path),
    }
    try:
        states = _wait_until_ready(node, process, args.startup_timeout)
        result["startup_seconds"] = time.monotonic() - started
        result["controller_states"] = states
        # Do not lose the first command batch while DDS endpoint discovery is
        # still in progress. The runtime verifier naturally waits for this by
        # spinning before publishing; the performance test must do the same.
        node.wait_for_command_subscriptions(timeout_sec=10.0)
        before = dict(node.positions)
        odom_before = node.odom_xy()
        node.reset_timing()
        measurement_started = time.monotonic()
        measurement_end = measurement_started + args.duration
        command_end = measurement_started + args.command_seconds
        max_wheel_speed = 0.0
        while rclpy.ok() and time.monotonic() < measurement_end and process.poll() is None:
            node.publish_commands(args.base_speed if time.monotonic() < command_end else 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            max_wheel_speed = max(max_wheel_speed, node.max_wheel_speed())

        groups: dict[str, Any] = {}
        for group in GROUPS:
            joints = []
            for joint in group.joints:
                start = before.get(joint)
                end = node.positions.get(joint)
                delta = None if start is None or end is None else end - start
                joints.append({"joint": joint, "before": start, "after": end, "delta": delta})
            moved = [item for item in joints if item["delta"] is not None and abs(item["delta"]) >= args.min_joint_delta]
            groups[group.name] = {
                "topic": group.topic,
                "joints": joints,
                "moved_count": len(moved),
                "required_count": len(group.joints),
                "response_ok": len(moved) == len(group.joints),
            }
        result["groups"] = groups
        odom_after = node.odom_xy()
        displacement = (
            math.hypot(odom_after[0] - odom_before[0], odom_after[1] - odom_before[1])
            if odom_before is not None and odom_after is not None
            else None
        )
        base_response_ok = max_wheel_speed > 0.01 and displacement is not None and displacement > 0.001
        controllers_after = node.controller_states(timeout_sec=5.0)
        controller_states_ok = all(
            controllers_after.get(name) == "active" for name in REQUIRED_CONTROLLERS
        )
        timing = {
            "clock": node.timing["clock"].metrics(),
            "sim_joint_states": node.timing["sim_joint_states"].metrics(args.physics_hz),
            "joint_states": node.timing["joint_states"].metrics(),
        }
        camera_timing = (
            {name: sample.metrics() for name, sample in node.camera_timing.items()}
            if args.camera_profile == "quad"
            else {}
        )
        result.update(
            {
                "measurement_seconds": time.monotonic() - measurement_started,
                "controller_states_after": controllers_after,
                "base": {
                    "max_wheel_speed_rad_s": max_wheel_speed,
                    "odom_displacement_m": displacement,
                    "response_ok": base_response_ok,
                },
                "timing": timing,
                "camera_timing": camera_timing,
                "lidar_timing": node.lidar_timing.metrics(),
                "lidar_nonempty_samples": node.lidar_nonempty_samples,
                "last_lidar_point_count": node.last_lidar_point_count,
            }
        )
        lidar_ok = (
            args.lidar_profile == "none"
            or (
                float(node.lidar_timing.metrics().get("wall_receive_hz") or 0.0)
                >= args.min_lidar_hz
                and node.lidar_nonempty_samples > 0
            )
        )
        result["status"] = (
            "PASS"
            if all(item["response_ok"] for item in groups.values())
            and base_response_ok
            and controller_states_ok
            and lidar_ok
            else "FAIL_CONTROL_RESPONSE"
        )
        node.stop()
    except Exception as exc:  # noqa: BLE001
        result["status"] = "TEST_ERROR"
        result["error"] = str(exc)
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        result["launch_returncode"] = process.poll()
        _stop_process(process)
        result["launch_returncode_after_stop"] = process.poll()
    return result


def _batch_id(executor: str, camera_profile: str, lidar_profile: str) -> str:
    safe_executor = "".join(
        character if character.isalnum() or character in "_-" else "-" for character in executor
    ).strip("-") or "manual"
    if camera_profile == "quad" and lidar_profile != "none":
        suffix = "四相机加MID360"
    elif camera_profile == "quad":
        suffix = "四相机"
    elif lidar_profile != "none":
        suffix = "机器人加MID360"
    else:
        suffix = "无传感器"
    return f"{datetime.now().astimezone().strftime('%Y%m%d-%H%M%S')}-{suffix}-{safe_executor}"


def _functional_conclusion(result: dict[str, Any]) -> str:
    if result.get("status") == "PASS":
        return "通过"
    if result.get("status") == "TEST_ERROR":
        return "测试异常"
    if not result.get("groups"):
        return "未运行"
    return "未通过"


def _format_metric(value: Any, digits: int = 3) -> str:
    return "未采集" if value is None else f"{float(value):.{digits}f}"


def _markdown(result: dict[str, Any], batch_id: str, executed_at: str, executor: str) -> str:
    groups = result.get("groups", {})
    function_conclusion = _functional_conclusion(result)
    lines = [
        f"### 批次：{batch_id}",
        "",
        "| 字段 | 内容 |",
        "| --- | --- |",
        f"| 执行时间 | {executed_at} |",
        f"| 执行者 | {executor} |",
        "| 版本 | 未记录 |",
        f"| 测试范围 | {'四路 RGB-D 相机；' if result.get('camera_profile') == 'quad' else ''}{'MID360；' if result.get('lidar_profile') != 'none' else '无传感器；'}底盘、升降、头部、左臂、右臂；无 RViz |",
        f"| 配置 | 完整控制；physics `{result.get('physics_hz', 120.0)} Hz`；render `{result.get('render_hz', 30.0)} Hz`；控制器目标 `90 Hz`；相机 profile `{result.get('camera_profile', 'none')}`；MID360 profile `{result.get('lidar_profile', 'none')}`；transport `{result.get('lidar_transport', 'helper')}`；挂载模式 `{result.get('lidar_mount_mode', 'fixed_kinematic')}` |",
        f"| 功能结论 | {function_conclusion} |",
        f"| 实时结论 | {result.get('realtime_conclusion', '未运行')} |",
        "",
        "| 控制组 | 命令话题 | 通过线 | 实测值 | 判定 |",
        "| --- | --- | --- | ---: | --- |",
        f"| 底盘 | `/cmd_vel` | 轮速 > 0 且里程计位移 > 0 | {_format_metric(result.get('base', {}).get('odom_displacement_m'))} m | {'通过' if result.get('base', {}).get('response_ok') else '未通过'} |",
    ]
    names = {group.name: group for group in GROUPS}
    for name, group in names.items():
        item = groups.get(name, {})
        moved = item.get("moved_count", "未采集")
        required = item.get("required_count", len(group.joints))
        passed = bool(item.get("response_ok", False))
        lines.append(
            f"| {name} | `{group.topic}` | {required}/{required} 目标关节移动 | "
            f"{moved}/{required} | {'通过' if passed else ('未运行' if moved == '未采集' else '未通过')} |"
        )
    if result.get("camera_profile") == "quad":
        lines.extend([
            "",
            "| 相机流 | 目标 | 墙钟接收频率 | 仿真时间 RTF | 判定 |",
            "| --- | ---: | ---: | ---: | --- |",
        ])
        for name, topic in CAMERA_STREAMS.items():
            metric = result.get("camera_timing", {}).get(name, {})
            samples = int(metric.get("samples", 0) or 0)
            lines.append(
                f"| `{topic}` | 30 Hz | {_format_metric(metric.get('wall_receive_hz'))} Hz | "
                f"{_format_metric(metric.get('rtf'))} | "
                f"{'通过' if metric.get('wall_receive_hz', 0.0) >= 29.5 else '未通过'} |"
            )
    if result.get("lidar_profile") != "none":
        metric = result.get("lidar_timing", {})
        lines.extend([
            "",
            "| MID360 raw PointCloud2 | 目标 | 墙钟接收频率 | 非空消息 | 末帧点数 | 判定 |",
            "| --- | ---: | ---: | ---: | ---: | --- |",
            f"| `/openflex/livox_frame/lidar` | 不低于 {result.get('min_lidar_hz', 10.0)} Hz | {_format_metric(metric.get('wall_receive_hz'))} Hz | {result.get('lidar_nonempty_samples', 0)} | {result.get('last_lidar_point_count', 0)} | {'通过' if float(metric.get('wall_receive_hz') or 0.0) >= float(result.get('min_lidar_hz', 10.0)) and int(result.get('lidar_nonempty_samples') or 0) > 0 else '未通过'} |",
        ])
    lines.extend(
        [
            "",
            "| 实时指标 | 通过线 | 实测值 | 判定 |",
            "| --- | --- | ---: | --- |",
            f"| `/joint_states` | 不低于 90 Hz | {_format_metric(result.get('timing', {}).get('joint_states', {}).get('wall_receive_hz'))} Hz | {result.get('joint_states_conclusion', '未运行')} |",
            f"| physics step rate | 不低于 {result.get('physics_hz', 120.0)} Hz | {_format_metric(result.get('timing', {}).get('sim_joint_states', {}).get('physics_step_rate_hz'))} Hz | {result.get('physics_conclusion', '未运行')} |",
            f"| RTF | 不低于 1.0 | {_format_metric(result.get('timing', {}).get('clock', {}).get('rtf'))} | {result.get('rtf_conclusion', '未运行')} |",
            "",
            "结论：功能结论依据控制器持续 active、底盘响应和各目标关节实际位移；实时结论依据同一窗口内的"
            "`/joint_states`、physics step rate 和 RTF，不能互相替代。",
            "",
            f"原始证据：`raw/robot_control_full/{batch_id}/result.json`、"
            f"`raw/robot_control_full/{batch_id}/launch.log`。",
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--startup-timeout", type=float, default=240.0)
    parser.add_argument("--command-seconds", type=float, default=4.0)
    parser.add_argument("--settle-seconds", type=float, default=1.0)
    parser.add_argument("--base-speed", type=float, default=0.10)
    parser.add_argument("--min-joint-delta", type=float, default=0.005)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--physics-hz", type=float, default=120.0)
    parser.add_argument("--render-hz", type=float, default=30.0)
    parser.add_argument("--camera-profile", choices=("none", "quad"), default="none")
    parser.add_argument(
        "--lidar-profile",
        choices=("none", "performance", "full"),
        default="none",
        help="none: no MID360; performance/full: add the robot-mounted historical MID360 chain",
    )
    parser.add_argument("--lidar-transport", choices=("helper", "native"), default="helper")
    parser.add_argument(
        "--lidar-mount-mode",
        choices=("follow", "fixed_kinematic", "fixed"),
        default="fixed_kinematic",
        help="MID360 pose mode: fixed kinematic root (safe default), fixed root, or experimental follow",
    )
    parser.add_argument("--lidar-topic", default="/openflex/livox_frame/lidar")
    parser.add_argument("--min-lidar-hz", type=float, default=10.0)
    parser.add_argument(
        "--stage",
        default="auto",
        help="Isaac stage override; use mid360_empty_stage.usda for a robot+MID360 scene with geometry",
    )
    parser.add_argument(
        "--append-report",
        action="store_true",
        help="将本次指定为有效批次并追加到统一 Markdown 报告；调试重试默认只保留原始证据。",
    )
    parser.add_argument("--api-port", type=int, default=8121)
    parser.add_argument("--executor", default=os.environ.get("ISAACSIM_TEST_EXECUTOR", "manual"))
    args = parser.parse_args()
    if args.duration < args.command_seconds or args.physics_hz <= 0.0 or args.render_hz <= 0.0:
        parser.error("--duration must cover --command-seconds; frequencies must be positive")
    if args.min_lidar_hz <= 0.0:
        parser.error("--min-lidar-hz must be positive")
    batch_id = _batch_id(args.executor, args.camera_profile, args.lidar_profile)
    batch_dir = RAW_ROOT / batch_id
    batch_dir.mkdir(parents=True, exist_ok=False)
    executed_at = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %z")
    rclpy.init()
    result = _run(args, batch_dir)
    result["physics_hz"] = args.physics_hz
    result["render_hz"] = args.render_hz
    result["camera_profile"] = args.camera_profile
    result["lidar_profile"] = args.lidar_profile
    result["lidar_transport"] = args.lidar_transport
    result["lidar_mount_mode"] = args.lidar_mount_mode
    result["min_lidar_hz"] = args.min_lidar_hz
    result["stage"] = args.stage
    timing = result.get("timing", {})
    metrics = timing.get("sim_joint_states", {})
    joint_hz = timing.get("joint_states", {}).get("wall_receive_hz")
    physics_rate = metrics.get("physics_step_rate_hz")
    rtf = timing.get("clock", {}).get("rtf")
    result["joint_states_conclusion"] = (
        "通过"
        if joint_hz is not None and joint_hz >= 90.0 - JOINT_STATES_TOLERANCE_HZ
        else "未通过"
    )
    result["physics_conclusion"] = "通过" if physics_rate is not None and physics_rate >= args.physics_hz else "未通过"
    result["rtf_conclusion"] = "通过" if rtf is not None and rtf >= 1.0 else "未通过"
    result["realtime_conclusion"] = (
        "通过"
        if all(
            value == "通过"
            for value in (
                result["joint_states_conclusion"],
                result["physics_conclusion"],
                result["rtf_conclusion"],
            )
        )
        else "未通过"
    )
    raw_path = batch_dir / "result.json"
    raw_path.write_text(
        json.dumps(
            {"batch_id": batch_id, "executed_at": executed_at, "executor": args.executor, "result": result},
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    if args.append_report:
        _append_batch(_markdown(result, batch_id, executed_at, args.executor))
    print(REPORT_PATH)
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
