"""Tests for the frozen controlled plant-level recoverability sweep."""

from __future__ import annotations

import json
import csv
from dataclasses import replace

import pytest

from benchmarks.run_controlled_recoverability_sweep import (
    FIXED_DT,
    HELD_OUT_PARAMETER_GRID,
    PRIMARY_PARAMETER_GRID,
    evaluate_controlled_case,
    get_parameter_grid,
    run_controlled_sweep,
    write_sweep_outputs,
)
from benchmarks.validate_recoverability_gap import evaluate_case
from benchmarks.validate_recoverability_gap import PlantStoppingMeasurement


def test_primary_and_held_out_grids_are_frozen_and_disjoint():
    assert FIXED_DT == 0.05
    assert PRIMARY_PARAMETER_GRID.case_count == 768
    assert HELD_OUT_PARAMETER_GRID.case_count == 162
    assert PRIMARY_PARAMETER_GRID.v0_values == (0.25, 0.75, 1.5, 2.5)
    assert HELD_OUT_PARAMETER_GRID.v0_values == (0.4, 1.1, 2.0)
    assert set(PRIMARY_PARAMETER_GRID.tau_values).isdisjoint(
        HELD_OUT_PARAMETER_GRID.tau_values
    )
    assert get_parameter_grid("held_out") is HELD_OUT_PARAMETER_GRID


def test_held_out_preflight_keeps_primary_metrics_separate(tmp_path):
    report = run_controlled_sweep("held-out", max_cases=1)
    metadata = report["metadata"]
    tuning = metadata["tuning"]
    primary_metrics = report["primary_metrics"]
    logged_analysis = report["logged_analysis"]

    assert metadata["split"] == "held-out"
    assert metadata["held_out_frozen"] is True
    assert metadata["evidence_role"] == "preflight"
    assert metadata["complete_case_grid"] is False
    assert tuning["held_out_used_for_tuning"] is False
    assert primary_metrics["case_count"] == 1
    assert primary_metrics["plant_measurement_count"] == 1
    assert metadata["measurement_protocol"]["version"] == "sampled_peak_settled_v2"
    assert metadata["measurement_protocol"]["required_quiet_samples"] == 11
    assert metadata["plant_contract"]["obstacles"] == []
    assert metadata["proxy_contract"]["outcome_kind"] == "virtual_stopping_boundary_proxy"
    assert metadata["provenance"]["packages"]["mujoco"]
    assert metadata["provenance"]["source_sha256"]
    assert metadata["source_unchanged_during_run"] is True
    assert logged_analysis["status"] == "secondary"
    assert logged_analysis["tuning_used"] is False
    assert primary_metrics is not logged_analysis

    json_path, csv_path = write_sweep_outputs(report, tmp_path)
    assert json_path.exists()
    assert csv_path.exists()
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    assert set(("primary_metrics", "logged_analysis", "cases")) <= payload.keys()
    assert payload["primary_metrics"]["case_count"] == 1
    with csv_path.open(newline="", encoding="utf-8") as handle:
        csv_rows = list(csv.DictReader(handle))
    assert csv_rows[0]["measurement_status"] == "settled"
    assert csv_rows[0]["obstacle_contact_evidence"] == "not_measured_empty_obstacle_scene"
    assert "minimum_clearance" not in csv_rows[0]
    assert "collision" not in csv_rows[0]
    assert "plant_safe" not in csv_rows[0]
    with pytest.raises(FileExistsError):
        write_sweep_outputs(report, tmp_path)


def test_controlled_measurement_rejects_cases_outside_the_frozen_split():
    case = evaluate_case(
        v0=0.25,
        command_gap=0.0,
        obstacle_distance=0.1,
        tau_v=0.06,
        delta_u=0.05,
        dt=FIXED_DT,
    )
    with pytest.raises(ValueError, match="outside the frozen split"):
        evaluate_controlled_case(case, split="held-out")


def _measurement(status="settled"):
    return PlantStoppingMeasurement(
        status=status,
        reason=None if status == "settled" else "unsupported simulation",
        maximum_sampled_forward_travel_m=0.3,
        terminal_forward_travel_m=0.2,
        maximum_travel_time_s=0.2,
        elapsed_time_s=1.0,
        attempted_steps=20,
        completed_steps=20,
        sample_count=21,
        quiet_samples=11 if status == "settled" else 0,
        required_quiet_samples=11,
        terminal_body_linear_speed_m_s=0.0,
        terminal_body_angular_speed_rad_s=0.0,
        terminal_actuator_v_m_s=0.0,
        terminal_actuator_omega_rad_s=0.0,
    )


def test_proxy_uses_peak_travel_and_not_terminal_distance(monkeypatch):
    from benchmarks import run_controlled_recoverability_sweep as sweep

    monkeypatch.setattr(sweep, "measure_mujoco_stop", lambda *args, **kwargs: _measurement())
    case = evaluate_case(v0=0.4, command_gap=0.25, obstacle_distance=0.25, tau_v=0.08, delta_u=0.075, dt=0.05)
    row = evaluate_controlled_case(case, split="held-out")
    assert row.plant_distance == 0.3
    assert row.virtual_boundary_margin_m == pytest.approx(-0.05)
    assert row.plant_proxy_safe is False  # Terminal x=0.2 would incorrectly be safe.


@pytest.mark.parametrize("status", ["invalid", "not_settled"])
def test_unsupported_trials_keep_raw_denominator_and_unknown_proxy(monkeypatch, tmp_path, status):
    from benchmarks import run_controlled_recoverability_sweep as sweep

    measurements = iter([_measurement(), _measurement(status)])
    monkeypatch.setattr(sweep, "measure_mujoco_stop", lambda *args, **kwargs: next(measurements))
    report = run_controlled_sweep("primary", max_cases=2)
    metrics = report["primary_metrics"]
    assert metrics["case_count"] == metrics["plant_measurement_attempt_count"] == 2
    assert metrics["plant_measurement_count"] == 1
    assert metrics["boundary_comparison_denominator"] == 1
    assert metrics["unsupported_case_count"] == metrics["plant_proxy_unknown_case_count"] == 1
    assert metrics["plant_proxy_safe_case_count"] == 0
    assert metrics["plant_proxy_unsafe_case_count"] == 1
    assert report["metadata"]["run_status"] == "complete_with_unsupported_cases"
    invalid_row = report["cases"][1]
    assert invalid_row["plant_distance"] is None
    assert invalid_row["plant_proxy_safe"] is None
    assert invalid_row["virtual_boundary_margin_m"] is None
    assert invalid_row["measurement"]["maximum_sampled_forward_travel_m"] == 0.3
    json_path, _ = write_sweep_outputs(report, tmp_path)
    assert json.loads(json_path.read_text())["cases"][1]["plant_proxy_safe"] is None


def test_all_invalid_trials_have_no_distance_or_comparison_rate(monkeypatch):
    from benchmarks import run_controlled_recoverability_sweep as sweep

    monkeypatch.setattr(sweep, "measure_mujoco_stop", lambda *args, **kwargs: _measurement("invalid"))
    report = run_controlled_sweep("held-out", max_cases=1)
    metrics = report["primary_metrics"]
    assert metrics["case_count"] == metrics["invalid_case_count"] == 1
    assert metrics["plant_measurement_count"] == metrics["boundary_comparison_denominator"] == 0
    assert metrics["plant_distance_mean_m"] is None
    assert metrics["plant_distance_max_m"] is None
    assert metrics["plant_vs_discrete_safe_boundary_disagreement_rate"] is None


def test_nonfinite_settled_result_fails_closed(monkeypatch):
    from benchmarks import run_controlled_recoverability_sweep as sweep

    bad_measurement = replace(_measurement(), maximum_sampled_forward_travel_m=float("nan"))
    monkeypatch.setattr(sweep, "measure_mujoco_stop", lambda *args, **kwargs: bad_measurement)
    with pytest.raises(RuntimeError, match="finite nonnegative"):
        run_controlled_sweep("primary", max_cases=1)


def test_cli_returns_failure_and_keeps_artifact_for_unsupported_trials(monkeypatch, tmp_path):
    from benchmarks import run_controlled_recoverability_sweep as sweep

    monkeypatch.setattr(sweep, "measure_mujoco_stop", lambda *args, **kwargs: _measurement("invalid"))
    assert sweep.main(["--max-cases", "1", "--output", str(tmp_path)]) == 1
    payload = json.loads((tmp_path / "controlled_recoverability_sweep_primary.json").read_text())
    assert payload["primary_metrics"]["invalid_case_count"] == 1
