"""Passive ROS-topic capture for Gate 1, with ROS-free validation helpers."""

from __future__ import annotations

import json
import math
import time
from collections import Counter
from pathlib import Path
from typing import Any


REQUIRED_TOPICS = (
    "/clock",
    "/odom",
    "/obstacles",
    "/cmd_vel_raw",
    "/cmd_vel",
    "/cmd_vel_bridge",
    "/adaptive_safety_filter/mode",
    "/adaptive_safety_filter/tracking_error",
    "/adaptive_safety_filter/risk",
    "/adaptive_safety_filter/min_margin",
    "/command_distortion/lambda_v",
    "/command_distortion/lambda_omega",
)


def json_safe(value: Any) -> Any:
    """Preserve non-finite evidence as strings instead of invalid JSON numbers."""
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def nonfinite_paths(value: Any, prefix: str = "message") -> list[str]:
    if isinstance(value, float) and not math.isfinite(value):
        return [prefix]
    if isinstance(value, dict):
        return [
            path
            for key, item in value.items()
            for path in nonfinite_paths(item, f"{prefix}.{key}")
        ]
    if isinstance(value, (list, tuple)):
        return [
            path
            for index, item in enumerate(value)
            for path in nonfinite_paths(item, f"{prefix}[{index}]")
        ]
    return []


class CaptureState:
    """Track readiness, simulation progress, freshness, and invalid messages."""

    def __init__(self) -> None:
        self.counts: Counter[str] = Counter()
        self.last_received: dict[str, float] = {}
        self.first_clock_s: float | None = None
        self.clock_s: float | None = None
        self.start_clock_s: float | None = None
        self.nonfinite_messages = 0
        self.failed_modes = 0
        self.clock_regressions = 0
        self.startup_clock_regressions = 0

    def record(self, topic: str, message: dict[str, Any], wall_s: float) -> list[str]:
        self.counts[topic] += 1
        self.last_received[topic] = wall_s
        invalid = nonfinite_paths(message)
        self.nonfinite_messages += bool(invalid)
        if topic == "/clock":
            clock = message["clock"]
            value = clock["sec"] + clock["nanosec"] * 1e-9
            if self.clock_s is not None and value < self.clock_s:
                # Gazebo can publish an initial clock and then reset it while
                # the launch graph is still becoming ready. That startup
                # transition is not a run-time regression. Once all required
                # streams have been observed and start_clock_s is latched,
                # regressions remain fatal evidence.
                if self.start_clock_s is None:
                    self.startup_clock_regressions += 1
                else:
                    self.clock_regressions += 1
            self.clock_s = value
            if self.first_clock_s is None:
                self.first_clock_s = value
        if topic == "/adaptive_safety_filter/mode" and message["data"] == "FAILED":
            self.failed_modes += 1
        if self.start_clock_s is None and self.ready:
            self.start_clock_s = self.clock_s
        return invalid

    @property
    def ready(self) -> bool:
        return self.clock_s is not None and self.clock_s > 0 and not self.missing_topics

    @property
    def missing_topics(self) -> list[str]:
        return [topic for topic in REQUIRED_TOPICS if not self.counts[topic]]

    @property
    def elapsed_s(self) -> float:
        if self.start_clock_s is None or self.clock_s is None:
            return 0.0
        return self.clock_s - self.start_clock_s

    def stale_topics(self, wall_s: float, timeout_s: float) -> list[str]:
        return [
            topic for topic in REQUIRED_TOPICS
            if topic in self.last_received
            and wall_s - self.last_received[topic] > timeout_s
        ]

    def summary(self) -> dict[str, Any]:
        return {
            "topic_counts": dict(sorted(self.counts.items())),
            "required_topics": list(REQUIRED_TOPICS),
            "missing_topics": self.missing_topics,
            "first_clock_s": self.first_clock_s,
            "start_clock_s": self.start_clock_s,
            "last_clock_s": self.clock_s,
            "elapsed_sim_s": self.elapsed_s,
            "nonfinite_message_count": self.nonfinite_messages,
            "failed_mode_count": self.failed_modes,
            "clock_regression_count": self.clock_regressions,
            "startup_clock_regression_count": self.startup_clock_regressions,
        }


class RosTelemetryRecorder:
    """Subscribe before launch and append untruncated ROS messages as JSONL.

    Imports are deferred so parsing and validation tests do not need ROS 2.
    This observer does not publish commands, change parameters, or use the ROS
    CLI daemon. BEST_EFFORT matches both reliable and best-effort publishers.
    """

    def __init__(self, path: Path, domain_id: int) -> None:
        import rclpy
        from geometry_msgs.msg import Twist, TwistStamped
        from nav_msgs.msg import Odometry
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from rosidl_runtime_py.convert import message_to_ordereddict
        from std_msgs.msg import Float32, Float32MultiArray, String

        self.state = CaptureState()
        self._context = Context()
        self._node = None
        self._executor = None
        self._stream = None
        self._rclpy = rclpy
        self._convert = message_to_ordereddict
        try:
            rclpy.init(args=[], context=self._context, domain_id=domain_id)
            self._node = rclpy.create_node("gate1_telemetry_recorder", context=self._context)
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            self._stream = path.open("x", encoding="utf-8", buffering=1)
            qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
            types = {
                "/clock": Clock,
                "/odom": Odometry,
                "/obstacles": Float32MultiArray,
                "/cmd_vel_raw": TwistStamped,
                "/cmd_vel": TwistStamped,
                "/cmd_vel_bridge": Twist,
                "/adaptive_safety_filter/mode": String,
                "/adaptive_safety_filter/tracking_error": Float32,
                "/adaptive_safety_filter/risk": Float32,
                "/adaptive_safety_filter/min_margin": Float32,
                "/command_distortion/lambda_v": Float32,
                "/command_distortion/lambda_omega": Float32,
            }
            self._subscriptions = [
                self._node.create_subscription(
                    types[topic], topic,
                    lambda message, name=topic: self._receive(name, message), qos,
                )
                for topic in REQUIRED_TOPICS
            ]
        except Exception:
            self.close()
            raise

    def _receive(self, topic: str, message: Any) -> None:
        payload = self._convert(message)
        wall_s = time.monotonic()
        invalid = self.state.record(topic, payload, wall_s)
        event = {
            "topic": topic,
            "receive_monotonic_s": wall_s,
            "clock_s": self.state.clock_s,
            "message": json_safe(payload),
            "nonfinite_fields": invalid,
        }
        assert self._stream is not None
        self._stream.write(json.dumps(event, allow_nan=False) + "\n")

    def spin_once(self, timeout_s: float = 0.1) -> None:
        assert self._executor is not None
        self._executor.spin_once(timeout_sec=timeout_s)

    def close(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        if self._context.ok():
            self._rclpy.shutdown(context=self._context)
        if self._stream is not None:
            self._stream.close()
