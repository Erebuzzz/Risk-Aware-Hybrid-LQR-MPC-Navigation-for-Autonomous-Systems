"""Focused tests for the Gate 1 ROS telemetry post-processor."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from benchmarks.generate_ros_validation_report import (
    ValidationError,
    generate_report,
    load_telemetry,
    main,
    summarize_run,
)


HEADER = "Tick,X,Y,Theta,Error,V,Omega,Mode,Risk,MinMargin,SolverMs,ThetaV,ThetaOmega"


def _row(tick: int, mode: str = "LQR", margin: float = 0.25) -> dict[str, object]:
    return {
        "Tick": tick,
        "X": 0.1 * tick,
        "Y": 0.02 * tick,
        "Theta": 0.01 * tick,
        "Error": 0.2 / (tick + 1),
        "V": 0.1,
        "Omega": 0.02,
        "Mode": mode,
        "Risk": 0.1 * tick,
        "MinMargin": margin,
        "SolverMs": 0.0 if mode == "LQR" else 2.0 + tick,
        "ThetaV": 0.0,
        "ThetaOmega": 0.0,
    }


def test_csv_artifact_generates_png_pdf_and_markdown(tmp_path: Path):
    csv_path = tmp_path / "telemetry.csv"
    rows = [
        _row(0, "LQR", 0.40),
        _row(1, "ADAPTIVE_MPC_FILTER", 0.32),
        _row(2, "BACKUP", 0.28),
        _row(3, "ADAPTIVE_MPC_FILTER", 0.30),
    ]
    lines = ["# run_id=synthetic-pass", "# dt=0.1", HEADER]
    lines.extend(
        ",".join(str(row[field]) for field in HEADER.split(",")) for row in rows
    )
    csv_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    run = load_telemetry([csv_path])
    summary = generate_report(run, tmp_path / "report")

    assert summary["status"] == "PASS"
    assert summary["sample_count"] == 4
    assert summary["safety_margin_min_m"] == 0.28
    assert summary["intervention_samples"] == 3
    assert summary["solver_p95_ms"] > summary["solver_mean_ms"]
    assert run.timebase == "tick * dt (dt=0.1 s)"

    report = (tmp_path / "report" / "gate1_validation_report.md").read_text(
        encoding="utf-8"
    )
    assert "synthetic-pass" in report
    assert "ADAPTIVE_MPC_FILTER" in report
    assert "Minimum logged nominal-rollout margin" in report
    assert "unavailable: no contact/collision telemetry" in report
    assert "Human sign-off:** pending" in report
    for stem in ("trajectory", "safety_margin", "controller_modes", "solver_timing"):
        assert (tmp_path / "report" / f"{stem}.png").read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
        assert (tmp_path / "report" / f"{stem}.pdf").read_bytes().startswith(b"%PDF-")


def test_json_artifact_preserves_recorded_time_and_metadata(tmp_path: Path):
    json_path = tmp_path / "telemetry.json"
    payload = {
        "metadata": {"run_id": "synthetic-json", "scenario": "crossing"},
        "telemetry": [
            {**_row(0), "time_s": 0.0},
            {**_row(1, "ADAPTIVE_MPC_FILTER"), "time_s": 0.07},
        ],
    }
    json_path.write_text(json.dumps(payload), encoding="utf-8")

    run = load_telemetry([json_path])
    summary = summarize_run(run)

    assert run.timebase == "recorded time_s"
    assert run.metadata["run_id"] == "synthetic-json"
    assert summary["duration_s"] == 0.07
    assert summary["mode_counts"] == {"ADAPTIVE_MPC_FILTER": 1, "LQR": 1}


def test_ros_log_with_negative_margin_reports_failure(tmp_path: Path):
    log_path = tmp_path / "adaptive_safety_filter_node_20261023.log"
    lines = [
        "2026-10-23 12:00:00,000 INFO " + HEADER,
        "2026-10-23 12:00:00,000 INFO 0,0.0,0.0,0.0,0.2,0.1,0.0,LQR,0.0,0.20,2.0,0.0,0.0",
        "2026-10-23 12:00:00,050 INFO 1,0.1,0.0,0.0,0.1,0.1,0.1,BACKUP,0.4,-0.02,4.0,0.0,0.0",
        "2026-10-23 12:00:00,100 INFO 2,0.2,0.0,0.0,0.1,0.1,0.1,BACKUP,0.5,-0.03,5.0,0.0,0.0",
    ]
    log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    run = load_telemetry([log_path], dt=0.05)
    summary = summarize_run(run)

    assert len(run.rows) == 3
    assert summary["status"] == "FAIL"
    assert summary["safety_violation_samples"] == 2
    assert summary["safety_violation_events"] == 1
    assert summary["collision_events"] is None
    assert "safety-violation" in summary["reasons"][0]


def test_cli_fails_clearly_for_missing_and_nonfinite_input(tmp_path: Path, capsys):
    missing = tmp_path / "missing.csv"
    assert main(["--input", str(missing), "--output", str(tmp_path / "out")]) == 2
    assert "Missing telemetry input" in capsys.readouterr().err

    invalid = tmp_path / "invalid.csv"
    invalid.write_text(
        HEADER + "\n" + ",".join(
            ["0", "nan", "0", "0", "0", "0", "0", "LQR", "0", "1", "1", "0", "0"]
        ) + "\n",
        encoding="utf-8",
    )
    assert main(["--input", str(invalid), "--output", str(tmp_path / "out")]) == 2
    assert "non-finite" in capsys.readouterr().err


@pytest.fixture
def telemetry_json(tmp_path: Path):
    def write(rows, metadata=None):
        path = tmp_path / "telemetry.json"
        path.write_text(json.dumps({"metadata": metadata or {}, "telemetry": rows}), encoding="utf-8")
        return path
    return write


@pytest.mark.parametrize(
    ("field", "value", "error"),
    [
        ("X", "nan", "non-finite"),
        ("MinMargin", float("inf"), "non-finite"),
        ("ThetaV", "nan", "non-finite"),
        ("SolverMs", -1, "negative solver_ms"),
        ("Tick", 0.5, "ticks must be"),
        ("Tick", -1, "ticks must be"),
        ("Mode", None, "unknown or empty mode"),
        ("Mode", "UNKNOWN", "unknown or empty mode"),
        ("collision", "maybe", "invalid boolean"),
        ("collision_count", 0.5, "non-negative integer"),
        ("time_s", -0.1, "time_s must be"),
    ],
)
def test_rejects_malformed_telemetry_values(telemetry_json, field, value, error):
    path = telemetry_json([{**_row(0), field: value}])
    with pytest.raises(ValidationError, match=error):
        load_telemetry([path])


@pytest.mark.parametrize("ticks", [[0, 0], [1, 0]])
def test_rejects_duplicate_or_out_of_order_ticks(telemetry_json, ticks):
    with pytest.raises(ValidationError, match="strictly increasing"):
        load_telemetry([telemetry_json([_row(tick) for tick in ticks])])


def test_optional_collision_telemetry_is_not_silently_partial(telemetry_json):
    with pytest.raises(ValidationError, match="incomplete collision coverage"):
        load_telemetry([telemetry_json([{**_row(0), "collision": False}, _row(1)])])


def test_explicit_collisions_and_active_solver_timing(telemetry_json):
    rows = [
        {**_row(0), "collision": False},
        {**_row(1, "ADAPTIVE_MPC_FILTER"), "collision": True},
        {**_row(2, "BACKUP"), "collision": True},
        {**_row(3), "collision": False},
        {**_row(4, "ADAPTIVE_MPC_FILTER"), "collision": True},
    ]
    run = load_telemetry([telemetry_json(rows, {"collision_count": 0})])
    summary = summarize_run(run, solver_time_limit_ms=3.5)
    assert summary["status"] == "FAIL"
    assert summary["collision_events"] == 2
    assert summary["collision_samples"] == 3
    assert summary["safety_violation_samples"] == 0
    assert summary["solver_mean_ms"] == pytest.approx(13 / 5)
    assert summary["solver_active_mean_ms"] == pytest.approx(13 / 3)
    assert summary["solver_timeout_samples"] == 2
    assert summary["intervention_events"] == 2


@pytest.mark.parametrize("suffix", [".csv", ".log"])
@pytest.mark.parametrize("extra", [",unexpected", ""])
def test_truncated_and_extra_fields_fail_in_csv_and_log(tmp_path, suffix, extra):
    path = tmp_path / f"broken{suffix}"
    payload = ",".join(str(value) for value in _row(0).values())
    payload = payload + extra if extra else payload.rsplit(",", 1)[0]
    prefix = "2026-10-23 12:00:00,000 INFO " if suffix == ".log" else ""
    path.write_text(prefix + HEADER + "\n" + prefix + payload + "\n", encoding="utf-8")
    with pytest.raises(ValidationError, match="expected 13 fields"):
        load_telemetry([path])


def test_log_does_not_drop_nan_position_rows(tmp_path):
    path = tmp_path / "nan.log"
    row = {**_row(0), "X": "nan"}
    prefix = "2026-10-23 12:00:00,000 INFO "
    path.write_text(prefix + HEADER + "\n" + prefix + ",".join(map(str, row.values())) + "\n")
    with pytest.raises(ValidationError, match="non-finite x"):
        load_telemetry([path])


def test_harness_bundle_and_metadata_only_json(tmp_path, monkeypatch):
    root = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root / "ros2_ws" / "src" / "hybrid_nav"))
    from hybrid_nav.gate1_experiment import (
        TelemetryRecord,
        summarize_telemetry,
        write_summary_artifacts,
    )

    rows = [TelemetryRecord(*_row(tick).values()) for tick in range(3)]
    summary = summarize_telemetry(rows, dt=0.1)
    summary["run"] = {"duration_requested_s": 30, "process_returncode": 0}
    csv_path, summary_path, summary_csv_path = write_summary_artifacts(tmp_path, rows, summary)
    run = load_telemetry([tmp_path])
    assert len(run.rows) == 3
    assert run.timebase == "tick * dt (dt=0.1 s)"
    assert run.metadata["duration_requested_s"] == 30
    assert summarize_run(run)["status"] == "PASS"
    assert len(load_telemetry([csv_path], metadata_inputs=[summary_path]).rows) == 3
    assert len(load_telemetry([summary_csv_path]).rows) == 3


def test_harness_failure_is_preserved_with_safe_telemetry(telemetry_json):
    metadata = {"status": "FAIL", "failure_reasons": ["ros2_launch_runtime_error"], "telemetry_rows": 1}
    summary = summarize_run(load_telemetry([telemetry_json([_row(0)], metadata)]))
    assert summary["status"] == "FAIL"
    assert "Harness: ros2_launch_runtime_error" in summary["reasons"]


def test_summary_rejects_missing_referenced_log(telemetry_json, tmp_path):
    telemetry_json([_row(0)])
    path = tmp_path / "summary.json"
    path.write_text(json.dumps({"status": "PASS", "artifacts": {"controller_raw_log": "missing.log"}}))
    with pytest.raises(ValidationError, match="Missing controller_raw_log artifact"):
        load_telemetry([path])


def test_duplicate_artifact_formats_do_not_double_count(telemetry_json, tmp_path):
    path = telemetry_json([_row(0), _row(1)])
    csv_path = tmp_path / "telemetry.csv"
    csv_path.write_text(HEADER + "\n" + "\n".join(
        ",".join(map(str, _row(tick).values())) for tick in range(2)
    ))
    assert len(load_telemetry([path, csv_path]).rows) == 2
    telemetry_json([_row(0), _row(1, margin=-1)])
    with pytest.raises(ValidationError, match="Conflicting telemetry artifacts"):
        load_telemetry([path, csv_path])


def test_cli_headless_failure_still_writes_figures_and_report(telemetry_json, tmp_path):
    path = telemetry_json([_row(0, "BACKUP", margin=-0.05)])
    output = tmp_path / "report"
    env = {**os.environ, "MPLBACKEND": "TkAgg"}
    env.pop("DISPLAY", None)
    result = subprocess.run(
        [sys.executable, "-m", "benchmarks.generate_ros_validation_report", "--input", str(path), "--output", str(output)],
        cwd=Path(__file__).resolve().parents[1],
        env=env, capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 1, result.stderr
    assert "Gate 1 validation: FAIL" in result.stdout
    assert "safety-violation" in result.stderr
    assert len(list(output.glob("*.pdf"))) == 4
    assert "`FAIL`" in (output / "gate1_validation_report.md").read_text()


@pytest.mark.parametrize(
    ("text", "error"),
    [
        ("{", "Malformed JSON"),
        ("[]", "no telemetry rows"),
        ('{"telemetry": "bad"}', "must be a list"),
        ('{"telemetry": [null]}', "non-object telemetry row"),
        ('{"telemetry": [{}]}', "missing required fields"),
        ('{"telemetry": [], "telemetry": []}', "Duplicate JSON field"),
    ],
)
def test_malformed_json_never_succeeds(tmp_path, text, error):
    path = tmp_path / "telemetry.json"
    path.write_text(text)
    with pytest.raises(ValidationError, match=error):
        load_telemetry([path])


def test_copied_harness_manifest_verifies_and_deduplicates_raw_files(tmp_path):
    lines = [HEADER] + [",".join(map(str, _row(tick).values())) for tick in range(2)]
    (tmp_path / "telemetry.csv").write_text("\n".join(lines))
    (tmp_path / "controller_raw.log").write_text("\n".join(
        "2026-10-23 12:00:00,000 INFO " + line for line in lines
    ))
    (tmp_path / "gate1_ros2_launch.log").write_text("ROS launch stdout\n")
    payload = {
        "status": "PASS",
        "telemetry_rows": 2,
        "deterministic_settings": {"dt": 0.05},
        "artifacts": {
            "telemetry_csv": "/ros2_ws/Output/Gate1/run_001/telemetry.csv",
            "controller_raw_log": "/ros2_ws/Output/Gate1/run_001/controller_raw.log",
            "launch_log": "/ros2_ws/Output/Gate1/run_001/gate1_ros2_launch.log",
        },
    }
    (tmp_path / "summary.json").write_text(json.dumps(payload))
    run = load_telemetry([tmp_path])
    assert len(run.rows) == 2
    assert {path.name for path in run.sources} == {
        "summary.json", "telemetry.csv", "controller_raw.log", "gate1_ros2_launch.log"
    }
    assert summarize_run(run)["status"] == "PASS"


def test_configured_positive_margin_threshold_is_honored(telemetry_json):
    path = telemetry_json([_row(0, margin=0.25)], {"configured_min_margin_m": 0.3, "status": "PASS"})
    summary = summarize_run(load_telemetry([path]))
    assert summary["status"] == "FAIL"
    assert summary["safety_violation_samples"] == 1


def test_missing_controller_ticks_prevent_a_false_pass(telemetry_json):
    run = load_telemetry([telemetry_json([_row(0), _row(2)])])
    summary = summarize_run(run)
    assert summary["status"] == "FAIL"
    assert summary["missing_tick_count"] == 1


def test_malformed_upstream_status_fails_clearly(telemetry_json):
    run = load_telemetry([telemetry_json([_row(0)], {"status": ["PASS"]})])
    with pytest.raises(ValidationError, match="Invalid upstream validation status"):
        summarize_run(run)
