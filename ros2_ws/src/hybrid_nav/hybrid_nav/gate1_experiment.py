"""Fixed-seed, headless Gate 1 runner for the ROS 2 Option B experiment.

The runner starts the active ``turtlebot3_adaptive_safety_filter`` launch,
keeps the Gazebo and RViz clients disabled, captures the launch stream and the
controller's raw CSV-like log under ``Output/``, and writes machine-readable
telemetry and summary artifacts.  The controller is intentionally not modified
by this module: the harness only orchestrates the existing launch and validates
the resulting diagnostics.

The summary is a diagnostic validation record.  Passing the configured margin
checks does not establish a safety guarantee for the controller or the plant.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import signal
import shutil
import subprocess
import sys
import time
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence, cast

from hybrid_nav.gate1_recorder import RosTelemetryRecorder, nonfinite_paths


TELEMETRY_HEADER = (
    "Tick",
    "X",
    "Y",
    "Theta",
    "Error",
    "V",
    "Omega",
    "Mode",
    "Risk",
    "MinMargin",
    "SolverMs",
    "ThetaV",
    "ThetaOmega",
)
_NUMERIC_FIELDS = (
    "tick",
    "x",
    "y",
    "theta",
    "error",
    "v",
    "omega",
    "risk",
    "min_margin",
    "solver_ms",
    "theta_v",
    "theta_omega",
)
KNOWN_MODES = frozenset({"LQR", "ADAPTIVE_MPC_FILTER", "BACKUP", "FAILED"})


@dataclass(frozen=True)
class TelemetryRecord:
    """One row from ``adaptive_safety_filter_node``'s run log."""

    tick: int
    x: float
    y: float
    theta: float
    error: float
    v: float
    omega: float
    mode: str
    risk: float
    min_margin: float
    solver_ms: float
    theta_v: float
    theta_omega: float

    def as_dict(self) -> dict[str, int | float | str]:
        """Return the stable, lower-snake-case representation used by Python."""

        return asdict(self)

    def as_csv_row(self) -> list[int | float | str]:
        """Return the documented, controller-compatible column order."""

        return [
            self.tick,
            self.x,
            self.y,
            self.theta,
            self.error,
            self.v,
            self.omega,
            self.mode,
            self.risk,
            self.min_margin,
            self.solver_ms,
            self.theta_v,
            self.theta_omega,
        ]


@dataclass(frozen=True)
class Gate1Config:
    """Runtime configuration for one deterministic ROS 2 run."""

    duration_s: float = 30.0
    dt: float = 0.05
    output_dir: Path = Path("Output/Gate1")
    workspace_dir: Path = Path(".")
    min_margin_m: float = 0.0
    max_safety_violation_rate: float = 0.0
    min_rows: int = 1
    ros_domain_id: int = 42
    ros_localhost_only: bool = True
    python_hash_seed: int = 0
    seed: int = 7
    use_actuation_mismatch: bool = True
    obstacle_scenario: str = "crossing"
    obstacle_format: str = "quintuple"
    package: str = "hybrid_nav"
    launch_file: str = "turtlebot3_adaptive_safety_filter.launch.py"
    shutdown_grace_s: float = 10.0
    startup_timeout_s: float = 60.0
    wall_timeout_s: float = 180.0
    topic_timeout_s: float = 10.0
    extra_launch_args: tuple[str, ...] = ()

    def validate(self) -> None:
        """Reject invalid run and validation settings before starting ROS 2."""

        finite_values = (
            self.duration_s,
            self.dt,
            self.min_margin_m,
            self.max_safety_violation_rate,
            self.shutdown_grace_s,
            self.startup_timeout_s,
            self.wall_timeout_s,
            self.topic_timeout_s,
        )
        if not all(math.isfinite(float(value)) for value in finite_values):
            raise ValueError("Gate 1 numeric settings must be finite")
        if self.duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")
        if self.max_safety_violation_rate < 0.0 or self.max_safety_violation_rate > 1.0:
            raise ValueError("max_safety_violation_rate must be in [0, 1]")
        if self.min_rows < 1:
            raise ValueError("min_rows must be at least one")
        if self.shutdown_grace_s <= 0.0:
            raise ValueError("shutdown_grace_s must be positive")
        if min(self.startup_timeout_s, self.wall_timeout_s, self.topic_timeout_s) <= 0:
            raise ValueError("startup, wall, and topic timeouts must be positive")
        if not 0 <= self.ros_domain_id <= 101:
            raise ValueError("ros_domain_id must be in [0, 101]")
        if not 0 <= self.seed < 2**32:
            raise ValueError("seed must be an unsigned 32-bit integer")
        if self.obstacle_scenario not in {"crossing", "sinusoidal"}:
            raise ValueError("obstacle_scenario must be crossing or sinusoidal")
        if not self.package or not self.launch_file:
            raise ValueError("package and launch_file must be non-empty")
        if self.obstacle_format not in {"triplet", "quintuple"}:
            raise ValueError("obstacle_format must be triplet or quintuple")
        for item in self.extra_launch_args:
            if ":=" not in item:
                raise ValueError(f"extra launch argument must use name:=value: {item!r}")
            if item.split(":=", 1)[0] not in {"x_pose", "y_pose"}:
                raise ValueError("only x_pose and y_pose may be overridden by --launch-arg")
            if not math.isfinite(float(item.split(":=", 1)[1])):
                raise ValueError("spawn coordinates must be finite")


@dataclass(frozen=True)
class RunArtifacts:
    """Paths and process details produced by :func:`run_ros2_experiment`."""

    output_dir: Path
    launch_log: Path
    raw_controller_log: Path | None
    launch_command: tuple[str, ...]
    process_returncode: int | None
    stopped_after_duration: bool
    runtime_error: str | None
    capture: dict[str, Any]
    inputs: dict[str, Any]


def build_launch_command(config: Gate1Config) -> list[str]:
    """Build the explicit headless command used by the Option B harness."""

    config.validate()
    command = [
        "ros2",
        "launch",
        config.package,
        config.launch_file,
        "use_rviz:=false",
        "use_gz_gui:=false",
        "use_sim_time:=true",
        f"use_actuation_mismatch:={'true' if config.use_actuation_mismatch else 'false'}",
        f"obstacle_scenario:={config.obstacle_scenario}",
        f"obstacle_format:={config.obstacle_format}",
        f"experiment_seed:={config.seed}",
    ]
    command.extend(config.extra_launch_args)
    return command


def _resolved_output_dir(config: Gate1Config) -> Path:
    workspace_dir = config.workspace_dir.expanduser().resolve()
    output_dir = config.output_dir.expanduser()
    if not output_dir.is_absolute():
        output_dir = workspace_dir / output_dir
    output_dir = output_dir.resolve()
    if "Output" not in output_dir.parts:
        raise ValueError("experiment artifacts must be written below an Output/ directory")
    return output_dir


def snapshot_inputs(output_dir: Path, config: Gate1Config) -> dict[str, Any]:
    """Retain the installed launch, parameter, and world inputs for provenance."""
    from ament_index_python.packages import get_package_share_directory
    import yaml

    share = Path(get_package_share_directory(config.package))
    files = ("config/params.yaml", "worlds/hybrid_obstacle_world.sdf", f"launch/{config.launch_file}")
    result: dict[str, Any] = {}
    for name in files:
        source = share / name
        data = source.read_bytes()
        destination = output_dir / "inputs" / source.name
        destination.parent.mkdir(exist_ok=True)
        destination.write_bytes(data)
        result[name] = {"sha256": hashlib.sha256(data).hexdigest(), "snapshot": f"inputs/{source.name}"}
    params = yaml.safe_load((share / "config/params.yaml").read_text(encoding="utf-8"))
    if nonfinite_paths(params):
        raise ValueError("installed params.yaml contains non-finite values")
    controller = params["adaptive_safety_filter_node"]["ros__parameters"]
    if not math.isclose(float(controller["dt"]), config.dt, rel_tol=0, abs_tol=1e-12):
        raise ValueError("--dt must match dt in the installed params.yaml")
    if not math.isclose(1.0 / float(controller["control_rate"]), config.dt, rel_tol=0, abs_tol=1e-12):
        raise ValueError("control_rate must equal 1/dt for tick-based metrics")
    result["controller_parameters"] = controller
    return result


def _signal_process_group(process: subprocess.Popen[str], sig: signal.Signals) -> None:
    if os.name == "nt":
        process.send_signal(sig)
        return
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        return


def run_ros2_experiment(config: Gate1Config) -> RunArtifacts:
    """Run the active launch headlessly and retain raw controller telemetry.

    ``duration_s`` is simulated time after all required streams are observed.
    Independent wall-clock deadlines reject startup failure and frozen clocks.
    The private working directory prevents stale logs from passing a new run.
    """

    config.validate()
    output_dir = _resolved_output_dir(config)
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError(f"output directory is not empty; choose a new run path: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    launch_log = output_dir / "gate1_ros2_launch.log"
    raw_controller_log = output_dir / "controller_raw.log"
    log_dir = output_dir / "Output" / "Logs"
    command = build_launch_command(config)

    env = os.environ.copy()
    env["TURTLEBOT3_MODEL"] = "burger"
    env["PYTHONHASHSEED"] = str(config.python_hash_seed)
    env["ROS_DOMAIN_ID"] = str(config.ros_domain_id)
    env["RCUTILS_COLORIZED_OUTPUT"] = "0"
    env["ROS_LOCALHOST_ONLY"] = "1" if config.ros_localhost_only else "0"
    env["ROS_LOG_DIR"] = str(output_dir / "ros_logs")
    env["GZ_PARTITION"] = f"hybrid_nav_gate1_{os.getpid()}"
    env["OMP_NUM_THREADS"] = "1"
    env["OPENBLAS_NUM_THREADS"] = "1"
    env["MKL_NUM_THREADS"] = "1"

    process: subprocess.Popen[str] | None = None
    process_returncode: int | None = None
    stopped_after_duration = False
    runtime_error: str | None = None
    recorder: RosTelemetryRecorder | None = None
    capture: dict[str, Any] = {}
    inputs: dict[str, Any] = {}
    previous_env = {key: os.environ.get(key) for key in env if key in {
        "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "ROS_LOG_DIR",
    }}
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    with launch_log.open("x", encoding="utf-8") as stream:
        stream.write("# Gate 1 command: " + " ".join(command) + "\n")
        stream.flush()
        try:
            signal.signal(signal.SIGTERM, interrupt)
            if os.name == "nt":
                raise RuntimeError("run Gate 1 in the Linux Docker image or a Linux ROS 2 workspace")
            inputs = snapshot_inputs(output_dir, config)
            os.environ.update({key: env[key] for key in previous_env})
            recorder = RosTelemetryRecorder(output_dir / "ros_topics.jsonl", config.ros_domain_id)
            process = subprocess.Popen(
                command, cwd=output_dir, env=env, stdout=stream,
                stderr=subprocess.STDOUT, text=True, start_new_session=True,
            )
            started = time.monotonic()
            while True:
                if process.poll() is not None:
                    raise RuntimeError(f"ROS 2 launch exited early with code {process.returncode}")
                recorder.spin_once(timeout_s=0.1)
                now = time.monotonic()
                if now - started >= config.wall_timeout_s:
                    raise RuntimeError("wall timeout before requested simulation duration")
                if not recorder.state.ready and now - started >= config.startup_timeout_s:
                    raise RuntimeError(f"startup timeout; missing topics: {recorder.state.missing_topics}")
                if recorder.state.clock_regressions:
                    raise RuntimeError("simulation clock regressed")
                stale = recorder.state.stale_topics(now, config.topic_timeout_s)
                if stale:
                    raise RuntimeError(f"stale ROS topics: {stale}")
                if recorder.state.elapsed_s >= config.duration_s:
                    stopped_after_duration = True
                    break
        except KeyboardInterrupt:
            runtime_error = "experiment interrupted"
        except Exception as exc:
            runtime_error = f"{type(exc).__name__}: {exc}"
        finally:
            if process is not None:
                _signal_process_group(process, signal.SIGINT)
                try:
                    process.wait(timeout=config.shutdown_grace_s)
                except subprocess.TimeoutExpired:
                    runtime_error = runtime_error or "forced shutdown after SIGINT timeout"
                    _signal_process_group(process, signal.SIGTERM)
                    try:
                        process.wait(timeout=config.shutdown_grace_s)
                    except subprocess.TimeoutExpired:
                        _signal_process_group(process, cast(signal.Signals, getattr(signal, "SIGKILL")))
                        process.wait(timeout=config.shutdown_grace_s)
                process_returncode = process.returncode
                if stopped_after_duration and process_returncode not in (0, 130, -int(signal.SIGINT)):
                    runtime_error = runtime_error or f"launch shutdown exit code {process_returncode}"
            if recorder is not None:
                capture = recorder.state.summary()
                recorder.close()
            for key, value in previous_env.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value
            signal.signal(signal.SIGTERM, previous_sigterm)
            if runtime_error:
                stream.write(f"# ERROR: {runtime_error}\n")

    candidates = sorted(log_dir.glob("adaptive_safety_filter_node_*.log"))
    copied_raw_log: Path | None = None
    if len(candidates) == 1:
        try:
            shutil.copyfile(candidates[0], raw_controller_log)
            copied_raw_log = raw_controller_log
        except OSError as exc:
            runtime_error = runtime_error or f"failed to copy controller log: {exc}"
    elif len(candidates) > 1:
        runtime_error = runtime_error or "multiple controller logs in isolated run directory"

    return RunArtifacts(
        output_dir=output_dir,
        launch_log=launch_log,
        raw_controller_log=copied_raw_log,
        launch_command=tuple(command),
        process_returncode=process_returncode,
        stopped_after_duration=stopped_after_duration,
        runtime_error=runtime_error,
        capture=capture,
        inputs=inputs,
    )


def _parse_float(value: str, field: str, line_number: int) -> float:
    try:
        return float(value.strip())
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"line {line_number}: invalid {field} value {value!r}"
        ) from exc


def parse_telemetry_lines(lines: Iterable[str]) -> tuple[list[TelemetryRecord], list[str]]:
    """Parse controller log lines and retain malformed-row diagnostics.

    ROS logging prepends a timestamp and log level to each line.  The parser
    intentionally searches for the stable ``Tick,`` marker, so it accepts both
    the raw ROS log and a plain CSV fixture in unit tests.
    """

    rows: list[TelemetryRecord] = []
    errors: list[str] = []
    header_seen = False
    for line_number, line in enumerate(lines, start=1):
        marker = line.find("Tick,")
        if marker >= 0:
            payload = line[marker:].strip()
        else:
            # The controller logger writes the header once, then prefixes each
            # data row with ``<timestamp> INFO``.  Rows do not repeat ``Tick,``.
            if " INFO " in line:
                payload = line.split(" INFO ", 1)[1].strip()
            else:
                payload = line.strip()
            if not payload:
                continue
            if not header_seen and payload[0] not in "+-0123456789":
                continue
        try:
            fields = next(csv.reader([payload]))
        except csv.Error as exc:
            errors.append(f"line {line_number}: CSV parse error: {exc}")
            continue
        if tuple(field.strip() for field in fields) == TELEMETRY_HEADER:
            if header_seen:
                errors.append(f"line {line_number}: repeated header (multiple runs)")
            header_seen = True
            continue
        if len(fields) != len(TELEMETRY_HEADER):
            errors.append(
                f"line {line_number}: expected {len(TELEMETRY_HEADER)} fields, "
                f"got {len(fields)}"
            )
            continue
        try:
            tick_value = _parse_float(fields[0], "Tick", line_number)
            if not math.isfinite(tick_value) or not tick_value.is_integer() or tick_value < 0:
                raise ValueError(f"line {line_number}: Tick must be a nonnegative finite integer")
            row = TelemetryRecord(
                tick=int(tick_value),
                x=_parse_float(fields[1], "X", line_number),
                y=_parse_float(fields[2], "Y", line_number),
                theta=_parse_float(fields[3], "Theta", line_number),
                error=_parse_float(fields[4], "Error", line_number),
                v=_parse_float(fields[5], "V", line_number),
                omega=_parse_float(fields[6], "Omega", line_number),
                mode=fields[7].strip(),
                risk=_parse_float(fields[8], "Risk", line_number),
                min_margin=_parse_float(fields[9], "MinMargin", line_number),
                solver_ms=_parse_float(fields[10], "SolverMs", line_number),
                theta_v=_parse_float(fields[11], "ThetaV", line_number),
                theta_omega=_parse_float(fields[12], "ThetaOmega", line_number),
            )
        except ValueError as exc:
            errors.append(str(exc))
            continue
        rows.append(row)
    if not header_seen:
        errors.append("missing telemetry header")
    return rows, errors


def parse_telemetry_file(path: Path) -> tuple[list[TelemetryRecord], list[str]]:
    """Read and parse one controller raw log."""

    if not path.is_file():
        raise FileNotFoundError(path)
    return parse_telemetry_lines(path.read_text(encoding="utf-8", errors="replace").splitlines())


def _append_unique(items: list[str], value: str) -> None:
    if value not in items:
        items.append(value)


def _percentile(values: Sequence[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percentile
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] + weight * (ordered[upper] - ordered[lower])


def _is_record_finite(row: TelemetryRecord) -> bool:
    return all(math.isfinite(float(getattr(row, field))) for field in _NUMERIC_FIELDS)


def _command_metrics(rows: Sequence[TelemetryRecord], dt: float) -> tuple[float, float, float, int]:
    if len(rows) < 2:
        return 0.0, 0.0, 0.0, 0
    acceleration_v: list[float] = []
    acceleration_omega: list[float] = []
    intervals: list[float] = []
    smoothness: list[float] = []
    tick_gaps = 0
    for previous, current in zip(rows, rows[1:]):
        tick_delta = current.tick - previous.tick
        if tick_delta != 1:
            tick_gaps += 1
            tick_delta = max(1, tick_delta)
        interval = dt * tick_delta
        delta_v = current.v - previous.v
        delta_omega = current.omega - previous.omega
        intervals.append(interval)
        acceleration_v.append(delta_v / interval)
        acceleration_omega.append(delta_omega / interval)
        smoothness.append(delta_v**2 + delta_omega**2)
    jerk_v = [
        (acceleration_v[i] - acceleration_v[i - 1]) / ((intervals[i] + intervals[i - 1]) / 2)
        for i in range(1, len(intervals))
    ]
    jerk_omega = [
        (acceleration_omega[i] - acceleration_omega[i - 1]) / ((intervals[i] + intervals[i - 1]) / 2)
        for i in range(1, len(intervals))
    ]
    return (
        math.hypot(*jerk_v) / math.sqrt(len(jerk_v)) if jerk_v else 0.0,
        math.hypot(*jerk_omega) / math.sqrt(len(jerk_omega)) if jerk_omega else 0.0,
        sum(smoothness) / len(smoothness),
        tick_gaps,
    )


def summarize_telemetry(
    rows: Sequence[TelemetryRecord],
    *,
    dt: float = 0.05,
    min_margin_m: float = 0.0,
    max_safety_violation_rate: float = 0.0,
    min_rows: int = 1,
    parse_errors: Iterable[str] = (),
    artifact_errors: Iterable[str] = (),
) -> dict[str, object]:
    """Compute Gate 1 metrics and validation status from raw telemetry.

    A safety violation is defined by the configured ``min_margin_m`` threshold
    (the controller's raw ``MinMargin`` is already relative to ``d_safe`` and
    the robot radius).  The threshold is an experiment check, not a proof.
    """

    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt must be finite and positive")
    if not math.isfinite(min_margin_m):
        raise ValueError("min_margin_m must be finite")
    if not math.isfinite(max_safety_violation_rate) or not 0.0 <= max_safety_violation_rate <= 1.0:
        raise ValueError("max_safety_violation_rate must be in [0, 1]")
    if min_rows < 1:
        raise ValueError("min_rows must be at least one")

    rows = list(rows)
    parse_errors = list(parse_errors)
    artifact_errors = list(artifact_errors)
    reasons: list[str] = []
    for value in artifact_errors:
        _append_unique(reasons, value)
    if parse_errors:
        _append_unique(reasons, "telemetry_parse_error")
    if len(rows) < min_rows:
        _append_unique(reasons, "missing_telemetry_rows")

    nonfinite_count = sum(1 for row in rows if not _is_record_finite(row))
    if nonfinite_count:
        _append_unique(reasons, "nonfinite_telemetry")
    finite_rows = all(_is_record_finite(row) for row in rows)

    metrics: dict[str, object] = {
        "telemetry_rows": len(rows),
        "tick_first": rows[0].tick if rows else None,
        "tick_last": rows[-1].tick if rows else None,
        "telemetry_duration_s": (
            (rows[-1].tick - rows[0].tick) * dt if len(rows) >= 2 else 0.0
        ),
        "tracking_rmse_m": None,
        "min_safety_margin_m": None,
        "safety_violation_count": None,
        "safety_violation_rate": None,
        "mpc_filter_rate": None,
        "backup_rate": None,
        "solver_mean_ms": None,
        "solver_p95_ms": None,
        "linear_jerk_rms": None,
        "angular_jerk_rms": None,
        "command_smoothness_cost": None,
        "mode_counts": {},
        "nonfinite_count": nonfinite_count,
        "tick_gap_count": None,
        "configured_min_margin_m": min_margin_m,
        "configured_max_safety_violation_rate": max_safety_violation_rate,
    }

    if rows and finite_rows:
        errors = [row.error for row in rows]
        margins = [row.min_margin for row in rows]
        solver_times = [row.solver_ms for row in rows]
        modes = Counter(row.mode for row in rows)
        violation_count = sum(margin < min_margin_m for margin in margins)
        violation_rate = violation_count / len(rows)
        linear_jerk, angular_jerk, smoothness, tick_gaps = _command_metrics(rows, dt)

        metrics.update(
            {
                "tracking_rmse_m": math.sqrt(sum(value * value for value in errors) / len(errors)),
                "min_safety_margin_m": min(margins),
                "safety_violation_count": violation_count,
                "safety_violation_rate": violation_rate,
                "mpc_filter_rate": modes.get("ADAPTIVE_MPC_FILTER", 0) / len(rows),
                "backup_rate": modes.get("BACKUP", 0) / len(rows),
                "solver_mean_ms": sum(solver_times) / len(solver_times),
                "solver_p95_ms": _percentile(solver_times, 0.95),
                "linear_jerk_rms": linear_jerk,
                "angular_jerk_rms": angular_jerk,
                "command_smoothness_cost": smoothness,
                "mode_counts": dict(sorted(modes.items())),
                "tick_gap_count": tick_gaps,
            }
        )

        unknown_modes = sorted(set(modes) - KNOWN_MODES)
        if unknown_modes:
            _append_unique(reasons, "unknown_controller_mode")
        if modes.get("FAILED", 0):
            _append_unique(reasons, "controller_failure_mode")
        if violation_rate > max_safety_violation_rate:
            _append_unique(reasons, "safety_violation_rate_exceeded")
        if tick_gaps:
            _append_unique(reasons, "missing_or_non_monotonic_ticks")
        if rows[0].tick != 0:
            _append_unique(reasons, "missing_initial_ticks")

    status = "PASS" if not reasons else "FAIL"
    return {
        "schema_version": 1,
        "status": status,
        "experiment": "gate1_option_b_ros2",
        "controller": "adaptive_safety_filter",
        "headless": True,
        "dt": dt,
        "margin_semantics": "Nominal LQR predicted rollout margin, not measured collision clearance.",
        "metric_definitions": {
            "timebase": "Tick * dt, not measured callback interval",
            "jerk": "RMS second finite difference of requested (V, Omega) commands",
            "command_smoothness_cost": "Mean squared successive command difference",
            "solver_latency": "Includes zero-time LQR rows",
            "telemetry_precision": "Existing rounded controller log values",
        },
        "safety_interpretation": (
            "Configured margin validation only; this diagnostic does not establish "
            "a safety guarantee."
        ),
        **metrics,
        "parse_error_count": len(parse_errors),
        "parse_errors": parse_errors,
        "failure_reasons": reasons,
    }


_SUMMARY_CSV_FIELDS = (
    "schema_version",
    "status",
    "experiment",
    "controller",
    "headless",
    "telemetry_rows",
    "tick_first",
    "tick_last",
    "telemetry_duration_s",
    "tracking_rmse_m",
    "min_safety_margin_m",
    "safety_violation_count",
    "safety_violation_rate",
    "mpc_filter_rate",
    "backup_rate",
    "solver_mean_ms",
    "solver_p95_ms",
    "linear_jerk_rms",
    "angular_jerk_rms",
    "command_smoothness_cost",
    "nonfinite_count",
    "tick_gap_count",
    "configured_min_margin_m",
    "configured_max_safety_violation_rate",
    "parse_error_count",
    "failure_reasons",
)


def write_summary_artifacts(
    output_dir: Path,
    rows: Sequence[TelemetryRecord],
    summary: dict[str, object],
) -> tuple[Path, Path, Path]:
    """Write raw parsed telemetry, JSON summary, and one-row CSV summary."""

    output_dir.mkdir(parents=True, exist_ok=True)
    telemetry_path = output_dir / "telemetry.csv"
    summary_json_path = output_dir / "summary.json"
    summary_csv_path = output_dir / "summary.csv"

    with telemetry_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(TELEMETRY_HEADER)
        writer.writerows(row.as_csv_row() for row in rows)

    with summary_json_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, sort_keys=True, allow_nan=False)
        handle.write("\n")

    csv_row: dict[str, Any] = {}
    for field in _SUMMARY_CSV_FIELDS:
        value = summary.get(field)
        if isinstance(value, (list, dict)):
            csv_row[field] = json.dumps(value, sort_keys=True)
        elif value is None:
            csv_row[field] = ""
        else:
            csv_row[field] = value
    with summary_csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(_SUMMARY_CSV_FIELDS)
        writer.writerow([csv_row[field] for field in _SUMMARY_CSV_FIELDS])

    return telemetry_path, summary_json_path, summary_csv_path


def _artifact_reference(path: Path, workspace_dir: Path) -> str:
    try:
        return path.resolve().relative_to(workspace_dir.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def create_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run the active hybrid_nav Option B experiment headlessly, capture "
            "telemetry, and fail on missing/invalid artifacts or safety-margin violations."
        )
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Simulated seconds after all required ROS streams are ready (default: 30).",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.05,
        help="Controller sample period used for jerk and duration metrics.",
    )
    parser.add_argument(
        "--output-dir",
        "--output",
        dest="output_dir",
        type=Path,
        default=Path("Output/Gate1"),
        help="Ignored artifact directory (default: Output/Gate1).",
    )
    parser.add_argument(
        "--min-margin-m",
        type=float,
        default=0.0,
        help="Configured minimum raw MinMargin accepted by the validation gate.",
    )
    parser.add_argument(
        "--max-safety-violation-rate",
        type=float,
        default=0.0,
        help="Maximum permitted fraction of rows below --min-margin-m.",
    )
    parser.add_argument(
        "--min-rows",
        type=int,
        default=1,
        help="Minimum telemetry rows required for a passing run.",
    )
    parser.add_argument("--ros-domain-id", type=int, default=42)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--startup-timeout-s", type=float, default=60.0)
    parser.add_argument("--wall-timeout-s", type=float, default=180.0)
    parser.add_argument("--topic-timeout-s", type=float, default=10.0)
    parser.add_argument(
        "--allow-network-discovery",
        action="store_true",
        help="Do not set ROS_LOCALHOST_ONLY=1 (headless default keeps it isolated).",
    )
    parser.add_argument("--obstacle-scenario", default="crossing")
    parser.add_argument(
        "--obstacle-format",
        choices=("triplet", "quintuple"),
        default="quintuple",
    )
    parser.add_argument(
        "--no-actuation-mismatch",
        action="store_true",
        help="Disable command distortion while retaining the command relay path.",
    )
    parser.add_argument("--shutdown-grace-s", type=float, default=10.0)
    parser.add_argument(
        "--launch-arg",
        action="append",
        default=[],
        help="Spawn override x_pose:=value or y_pose:=value; may be repeated.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entry point used by ``ros2 run hybrid_nav gate1_experiment``."""

    parser = create_argument_parser()
    args = parser.parse_args(argv)
    config = Gate1Config(
        duration_s=args.duration,
        dt=args.dt,
        output_dir=args.output_dir,
        min_margin_m=args.min_margin_m,
        max_safety_violation_rate=args.max_safety_violation_rate,
        min_rows=args.min_rows,
        ros_domain_id=args.ros_domain_id,
        seed=args.seed,
        startup_timeout_s=args.startup_timeout_s,
        wall_timeout_s=args.wall_timeout_s,
        topic_timeout_s=args.topic_timeout_s,
        ros_localhost_only=not args.allow_network_discovery,
        use_actuation_mismatch=not args.no_actuation_mismatch,
        obstacle_scenario=args.obstacle_scenario,
        obstacle_format=args.obstacle_format,
        shutdown_grace_s=args.shutdown_grace_s,
        extra_launch_args=tuple(args.launch_arg),
    )

    try:
        config.validate()
        artifacts = run_ros2_experiment(config)
    except ValueError as exc:
        print(f"Gate 1 configuration error: {exc}", file=sys.stderr)
        return 2
    except OSError as exc:
        print(f"Gate 1 runtime error: {exc}", file=sys.stderr)
        return 2

    artifact_errors: list[str] = []
    if artifacts.raw_controller_log is None or not artifacts.raw_controller_log.is_file():
        artifact_errors.append("missing_controller_raw_log")
    if artifacts.runtime_error:
        artifact_errors.append("ros2_launch_runtime_error")
    if not artifacts.stopped_after_duration:
        artifact_errors.append("incomplete_simulation_duration")
    if not artifacts.capture or artifacts.capture.get("missing_topics"):
        artifact_errors.append("missing_ros_topics")
    if artifacts.capture.get("nonfinite_message_count", 0):
        artifact_errors.append("nonfinite_ros_telemetry")
    if artifacts.capture.get("failed_mode_count", 0):
        artifact_errors.append("controller_failure_mode")
    if artifacts.capture.get("clock_regression_count", 0):
        artifact_errors.append("clock_regression")
    for path in [artifacts.launch_log, artifacts.output_dir / "ros_topics.jsonl"]:
        if not path.is_file() or path.stat().st_size == 0:
            artifact_errors.append(f"missing_artifact:{path.name}")

    rows: list[TelemetryRecord] = []
    parse_errors: list[str] = []
    if artifacts.raw_controller_log is not None and artifacts.raw_controller_log.is_file():
        try:
            rows, parse_errors = parse_telemetry_file(artifacts.raw_controller_log)
        except (OSError, UnicodeError) as exc:
            parse_errors.append(f"failed to read controller log: {exc}")

    summary = summarize_telemetry(
        rows,
        dt=config.dt,
        min_margin_m=config.min_margin_m,
        max_safety_violation_rate=config.max_safety_violation_rate,
        min_rows=config.min_rows,
        parse_errors=parse_errors,
        artifact_errors=artifact_errors,
    )
    summary["deterministic_settings"] = {
        "dt": config.dt,
        "ros_domain_id": config.ros_domain_id,
        "ros_localhost_only": config.ros_localhost_only,
        "use_rviz": False,
        "use_gz_gui": False,
        "use_actuation_mismatch": config.use_actuation_mismatch,
        "obstacle_scenario": config.obstacle_scenario,
        "obstacle_format": config.obstacle_format,
        "seed": config.seed,
        "python_hash_seed": config.python_hash_seed,
        "turtlebot3_model": "burger",
        "numerical_threads": 1,
    }
    summary["reproducibility_scope"] = (
        "Fixed scenario, seeds, input snapshots and simulated duration. ROS scheduling, "
        "Gazebo physics and wall-time solver fallback are not bitwise deterministic."
    )
    summary["ros_capture"] = artifacts.capture
    summary["inputs"] = artifacts.inputs
    summary["run"] = {
        "duration_requested_s": config.duration_s,
        "process_returncode": artifacts.process_returncode,
        "stopped_after_duration": artifacts.stopped_after_duration,
        "launch_command": list(artifacts.launch_command),
        "runtime_error": artifacts.runtime_error,
        "startup_timeout_s": config.startup_timeout_s,
        "wall_timeout_s": config.wall_timeout_s,
        "topic_timeout_s": config.topic_timeout_s,
    }
    summary["artifacts"] = {
        "launch_log": _artifact_reference(artifacts.launch_log, config.workspace_dir),
        "ros_topics_jsonl": _artifact_reference(
            artifacts.output_dir / "ros_topics.jsonl", config.workspace_dir
        ),
        "controller_raw_log": (
            _artifact_reference(artifacts.raw_controller_log, config.workspace_dir)
            if artifacts.raw_controller_log is not None
            else None
        ),
        "telemetry_csv": _artifact_reference(
            artifacts.output_dir / "telemetry.csv", config.workspace_dir
        ),
        "summary_json": _artifact_reference(
            artifacts.output_dir / "summary.json", config.workspace_dir
        ),
        "summary_csv": _artifact_reference(
            artifacts.output_dir / "summary.csv", config.workspace_dir
        ),
    }

    try:
        telemetry_path, summary_json_path, summary_csv_path = write_summary_artifacts(
            artifacts.output_dir, rows, summary
        )
    except (OSError, TypeError, ValueError) as exc:
        print(f"Gate 1 artifact write error: {exc}", file=sys.stderr)
        return 2

    expected_paths = [artifacts.launch_log, telemetry_path, summary_json_path, summary_csv_path]
    if artifacts.raw_controller_log is not None:
        expected_paths.append(artifacts.raw_controller_log)
    missing_paths = [path for path in expected_paths if not path.is_file()]
    if missing_paths:
        failure_reasons = cast(list[str], summary["failure_reasons"])
        for path in missing_paths:
            _append_unique(failure_reasons, f"missing_artifact:{path.name}")
        summary["status"] = "FAIL"
        try:
            write_summary_artifacts(artifacts.output_dir, rows, summary)
        except (OSError, TypeError, ValueError) as exc:
            print(f"Gate 1 artifact verification error: {exc}", file=sys.stderr)
            return 2

    print(f"Gate 1 status: {summary['status']}")
    print(f"Summary JSON: {summary_json_path}")
    print(f"Summary CSV: {summary_csv_path}")
    failure_reasons = cast(list[str], summary["failure_reasons"])
    if failure_reasons:
        print("Failure reasons: " + ", ".join(failure_reasons), file=sys.stderr)
    return 0 if summary["status"] == "PASS" else 3


if __name__ == "__main__":
    raise SystemExit(main())
