"""Pure-Python checks for the ROS 2 Gate 1 experiment harness."""

from __future__ import annotations

import csv
import hashlib
import json
import os
import sys
import types
from dataclasses import replace
from pathlib import Path

import pytest


ROS_PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "hybrid_nav"
if str(ROS_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(ROS_PACKAGE_ROOT))

from hybrid_nav.gate1_experiment import (  # noqa: E402
    Gate1Config,
    TelemetryRecord,
    build_launch_command,
    parse_telemetry_lines,
    summarize_telemetry,
    write_summary_artifacts,
)
from hybrid_nav import gate1_experiment as harness  # noqa: E402
from hybrid_nav.gate1_recorder import (  # noqa: E402
    CaptureState, REQUIRED_TOPICS, json_safe, nonfinite_paths,
)


RAW_HEADER = "Tick,X,Y,Theta,Error,V,Omega,Mode,Risk,MinMargin,SolverMs,ThetaV,ThetaOmega"


def _raw_lines(*rows: str) -> list[str]:
    return [
        f"2026-10-23 00:00:00,000 INFO {RAW_HEADER}",
        *[f"2026-10-23 00:00:00,000 INFO {row}" for row in rows],
    ]


def test_headless_launch_command_is_explicit_and_deterministic() -> None:
    command = build_launch_command(Gate1Config())

    assert command[:4] == [
        "ros2",
        "launch",
        "hybrid_nav",
        "turtlebot3_adaptive_safety_filter.launch.py",
    ]
    assert "use_rviz:=false" in command
    assert "use_gz_gui:=false" in command
    assert "use_sim_time:=true" in command
    assert "use_actuation_mismatch:=true" in command
    assert "obstacle_format:=quintuple" in command
    assert "experiment_seed:=7" in command


def test_parse_and_summarize_option_b_telemetry() -> None:
    rows, errors = parse_telemetry_lines(
        _raw_lines(
            "0,0.0,0.0,0.0,0.20,0.10,0.00,LQR,0.10,0.12,0.0,1.0,1.0",
            "1,0.005,0.0,0.0,0.10,0.12,0.10,ADAPTIVE_MPC_FILTER,0.50,0.04,2.0,0.98,1.01",
            "2,0.011,0.001,0.01,0.05,0.08,0.00,BACKUP,1.00,-0.01,3.0,0.97,1.02",
        )
    )

    assert errors == []
    assert len(rows) == 3
    summary = summarize_telemetry(rows, dt=0.05)

    assert summary["status"] == "FAIL"
    assert summary["telemetry_rows"] == 3
    assert summary["safety_violation_count"] == 1
    assert summary["safety_violation_rate"] == 1 / 3
    assert summary["mpc_filter_rate"] == 1 / 3
    assert summary["backup_rate"] == 1 / 3
    assert summary["linear_jerk_rms"] == pytest.approx(24.0)
    assert summary["angular_jerk_rms"] == pytest.approx(80.0)
    assert summary["solver_p95_ms"] == pytest.approx(2.9)
    assert "safety_violation_rate_exceeded" in summary["failure_reasons"]
    assert "safety guarantee" in summary["safety_interpretation"]


@pytest.mark.parametrize("value", ["nan", "inf", "-inf"])
def test_nonfinite_telemetry_fails_without_serializing_json_nan(tmp_path: Path, value: str) -> None:
    rows, errors = parse_telemetry_lines(
        _raw_lines(
            f"0,0.0,0.0,0.0,{value},0.1,0.0,LQR,0.0,0.2,0.0,1.0,1.0",
        )
    )
    assert errors == []

    summary = summarize_telemetry(rows)
    assert summary["status"] == "FAIL"
    assert summary["nonfinite_count"] == 1
    assert "nonfinite_telemetry" in summary["failure_reasons"]
    _, summary_path, _ = write_summary_artifacts(tmp_path, rows, summary)
    json.loads(summary_path.read_text(), parse_constant=lambda value: pytest.fail(value))


def test_summary_artifacts_are_machine_readable(tmp_path: Path) -> None:
    rows = [
        TelemetryRecord(
            tick=0,
            x=0.0,
            y=0.0,
            theta=0.0,
            error=0.1,
            v=0.1,
            omega=0.0,
            mode="LQR",
            risk=0.0,
            min_margin=0.2,
            solver_ms=0.0,
            theta_v=1.0,
            theta_omega=1.0,
        )
    ]
    summary = summarize_telemetry(rows)
    telemetry_path, json_path, csv_path = write_summary_artifacts(tmp_path, rows, summary)

    assert telemetry_path.is_file()
    assert json_path.is_file()
    assert csv_path.is_file()
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    assert payload["status"] == "PASS"
    with csv_path.open(newline="", encoding="utf-8") as handle:
        csv_rows = list(csv.DictReader(handle))
    assert len(csv_rows) == 1
    assert csv_rows[0]["telemetry_rows"] == "1"


def test_configured_margin_boundary_and_allowed_rate() -> None:
    rows, errors = parse_telemetry_lines(_raw_lines(
        "0,0,0,0,0,0,0,LQR,0,0,0,1,1",
        "1,0,0,0,0,0,0,LQR,0,-0.01,0,1,1",
    ))
    assert not errors
    assert summarize_telemetry(rows)["status"] == "FAIL"
    assert summarize_telemetry(rows, max_safety_violation_rate=0.5)["status"] == "PASS"
    assert summarize_telemetry(rows, min_margin_m=-0.01)["safety_violation_count"] == 0


@pytest.mark.parametrize("payload", ["bad,data", "0,0", "nan,0,0,0,0,0,0,LQR,0,1,0,1,1"])
def test_malformed_rows_are_never_silently_skipped(payload: str) -> None:
    rows, errors = parse_telemetry_lines(_raw_lines(payload))
    assert errors
    assert summarize_telemetry(rows, parse_errors=errors)["status"] == "FAIL"


@pytest.mark.parametrize("ticks", [(0, 2), (0, 0), (1, 2)])
def test_missing_or_duplicate_controller_ticks_fail(ticks: tuple[int, int]) -> None:
    rows, _ = parse_telemetry_lines(_raw_lines(*[
        f"{tick},0,0,0,0,0,0,LQR,0,1,0,1,1" for tick in ticks
    ]))
    assert summarize_telemetry(rows)["status"] == "FAIL"


@pytest.mark.parametrize("changes", [
    {"duration_s": float("nan")}, {"min_margin_m": float("nan")},
    {"dt": 0}, {"min_rows": 0}, {"max_safety_violation_rate": 1.1},
    {"seed": -1}, {"ros_domain_id": 102}, {"wall_timeout_s": 0},
    {"extra_launch_args": ("use_gz_gui:=true",)},
])
def test_invalid_configuration_is_rejected(changes: dict) -> None:
    with pytest.raises(ValueError):
        replace(Gate1Config(), **changes).validate()


def _ready_capture(state: CaptureState, wall_s: float = 1.0) -> None:
    for topic in REQUIRED_TOPICS:
        payload = {"clock": {"sec": 10, "nanosec": 0}} if topic == "/clock" else {"data": "LQR"}
        state.record(topic, payload, wall_s)


def test_capture_uses_sim_time_after_readiness_and_detects_stale_streams() -> None:
    state = CaptureState()
    state.record("/clock", {"clock": {"sec": 9, "nanosec": 0}}, 0.0)
    assert not state.ready
    assert state.elapsed_s == 0
    _ready_capture(state)
    assert state.ready
    assert state.elapsed_s == 0
    state.record("/clock", {"clock": {"sec": 12, "nanosec": 0}}, 2.0)
    assert state.elapsed_s == 2
    assert "/odom" in state.stale_topics(12, 10)
    assert "/clock" not in state.stale_topics(12, 10)
    state.record("/clock", {"clock": {"sec": 0, "nanosec": 0}}, 13)
    assert state.clock_regressions == 1


def test_startup_clock_reset_is_recorded_but_not_a_runtime_regression() -> None:
    state = CaptureState()
    state.record("/clock", {"clock": {"sec": 5, "nanosec": 0}}, 0.0)
    state.record("/clock", {"clock": {"sec": 0, "nanosec": 0}}, 0.1)

    assert state.clock_regressions == 0
    assert state.startup_clock_regressions == 1
    assert state.summary()["startup_clock_regression_count"] == 1


def test_raw_ros_capture_preserves_nonfinite_evidence_as_strict_json() -> None:
    message = {"data": [1.0, float("nan"), float("inf")]}
    assert nonfinite_paths(message) == ["message.data[1]", "message.data[2]"]
    assert json.loads(json.dumps(json_safe(message), allow_nan=False))["data"] == [1.0, "nan", "inf"]
    state = CaptureState()
    state.record("/obstacles", message, 1.0)
    state.record("/adaptive_safety_filter/mode", {"data": "FAILED"}, 1.0)
    assert state.nonfinite_messages == 1
    assert state.failed_modes == 1


@pytest.fixture
def fake_ros(monkeypatch: pytest.MonkeyPatch):
    """Exercise the real orchestrator with injected process and DDS boundaries."""
    if os.name == "nt":
        pytest.skip("The experiment subprocess lifecycle targets the Linux ROS 2 image")
    state = {"early_exit": False, "write_log": True, "nan": False, "margin": 0.1}

    def snapshot(path, config):
        return {"controller_parameters": {"dt": config.dt}}

    class Recorder:
        def __init__(self, path, domain_id):
            self.path = path
            self.state = CaptureState()
            if not state.get("missing_topics"):
                _ready_capture(self.state, harness.time.monotonic())
            self.path.write_text('{"topic":"/clock"}\n')

        def spin_once(self, timeout_s):
            if state.get("interrupted"):
                raise KeyboardInterrupt
            self.state.record("/clock", {"clock": {"sec": 11, "nanosec": 0}}, harness.time.monotonic())

        def close(self):
            state["recorder_closed"] = True

    class Process:
        def __init__(self, command, **kwargs):
            state["command"] = command
            state["env"] = kwargs["env"]
            state["cwd"] = kwargs["cwd"]
            self.returncode = 0 if state["early_exit"] else None
            if state["write_log"]:
                logs = kwargs["cwd"] / "Output" / "Logs"
                logs.mkdir(parents=True)
                error = "nan" if state["nan"] else "0.1"
                lines = _raw_lines(*[
                    f"{tick},0,0,0,{error},0,0,LQR,0,{state['margin']},0,1,1" for tick in range(3)
                ])
                (logs / "adaptive_safety_filter_node_test.log").write_text("\n".join(lines))

        def poll(self):
            return self.returncode

        def wait(self, timeout):
            state["wait_count"] = state.get("wait_count", 0) + 1
            if state.get("force_stop") and state["wait_count"] == 1:
                raise harness.subprocess.TimeoutExpired("ros2", timeout)
            self.returncode = 0
            return 0

    monkeypatch.setattr(harness, "snapshot_inputs", snapshot)
    monkeypatch.setattr(harness, "RosTelemetryRecorder", Recorder)
    monkeypatch.setattr(harness.subprocess, "Popen", Process)
    monkeypatch.setattr(harness, "_signal_process_group", lambda process, sig: state.update(signalled=True))
    return state


def test_complete_harness_emits_readable_artifacts_and_restores_environment(tmp_path, fake_ros):
    output = tmp_path / "Output" / "run"
    before = os.environ.get("ROS_DOMAIN_ID")
    assert harness.main(["--output", str(output), "--duration", "1"]) == 0
    summary = json.loads((output / "summary.json").read_text())
    assert summary["status"] == "PASS"
    assert summary["ros_capture"]["elapsed_sim_s"] == 1
    assert (output / "controller_raw.log").is_file()
    assert (output / "telemetry.csv").is_file()
    assert fake_ros["recorder_closed"] and fake_ros["signalled"]
    assert fake_ros["env"]["ROS_LOG_DIR"] == str(output / "ros_logs")
    assert os.environ.get("ROS_DOMAIN_ID") == before
    assert harness.main(["--output", str(output), "--duration", "1"]) == 2


@pytest.mark.parametrize("margin,run_status,report_status", [(0.1, 0, 0), (-0.1, 3, 1)])
def test_captured_harness_bundle_generates_report_with_same_outcome(
    tmp_path, fake_ros, margin, run_status, report_status,
):
    from benchmarks.generate_ros_validation_report import main as report_main

    fake_ros["margin"] = margin
    output = tmp_path / "Output" / "experiment"
    assert harness.main(["--output", str(output), "--duration", "1"]) == run_status
    plots = output / "plots"
    assert report_main([
        "--input", str(output / "summary.json"), "--output", str(plots),
    ]) == report_status
    report = (plots / "gate1_validation_report.md").read_text(encoding="utf-8")
    assert "Human sign-off:** pending" in report
    assert "unavailable: no contact/collision telemetry" in report
    for stem in ("trajectory", "safety_margin", "controller_modes", "solver_timing"):
        for extension in ("png", "pdf"):
            assert (plots / f"{stem}.{extension}").stat().st_size > 0


@pytest.mark.parametrize("failure", ["early_exit", "missing_log", "nan", "margin"])
def test_harness_returns_nonzero_and_saves_failure_summary(tmp_path, fake_ros, failure):
    if failure == "missing_log":
        fake_ros["write_log"] = False
    elif failure == "margin":
        fake_ros["margin"] = -0.1
    else:
        fake_ros[failure] = True
    output = tmp_path / "Output" / failure
    assert harness.main(["--output", str(output), "--duration", "1"]) == 3
    summary = json.loads((output / "summary.json").read_text())
    assert summary["status"] == "FAIL"
    assert summary["failure_reasons"]
    with (output / "summary.csv").open() as stream:
        assert next(csv.DictReader(stream))["status"] == "FAIL"


@pytest.mark.parametrize("failure,arguments", [
    ("missing_topics", ["--startup-timeout-s", "0.000001"]),
    ("wall_timeout", ["--wall-timeout-s", "0.000001"]),
    ("stale_topics", ["--topic-timeout-s", "0.000001"]),
    ("interrupted", []),
    ("force_stop", []),
])
def test_runtime_failures_close_capture_and_fail(tmp_path, fake_ros, failure, arguments):
    fake_ros[failure] = True
    output = tmp_path / "Output" / failure
    assert harness.main(["--output", str(output), "--duration", "1", *arguments]) == 3
    payload = json.loads((output / "summary.json").read_text())
    assert payload["run"]["runtime_error"]
    assert fake_ros["recorder_closed"]
    assert fake_ros["signalled"]


def test_input_snapshots_match_installed_sources_and_reject_wrong_dt(tmp_path, monkeypatch):
    package = types.ModuleType("ament_index_python.packages")
    setattr(package, "get_package_share_directory", lambda name: str(ROS_PACKAGE_ROOT))
    monkeypatch.setitem(sys.modules, "ament_index_python", types.ModuleType("ament_index_python"))
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", package)
    inputs = harness.snapshot_inputs(tmp_path, Gate1Config())
    raw = (ROS_PACKAGE_ROOT / "worlds/hybrid_obstacle_world.sdf").read_bytes()
    assert (tmp_path / "inputs/hybrid_obstacle_world.sdf").read_bytes() == raw
    assert inputs["worlds/hybrid_obstacle_world.sdf"]["sha256"] == hashlib.sha256(raw).hexdigest()
    with pytest.raises(ValueError, match="dt must match"):
        harness.snapshot_inputs(tmp_path, Gate1Config(dt=0.1))
