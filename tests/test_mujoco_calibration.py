"""Deterministic checks for the MuJoCo calibration diagnostics."""

from __future__ import annotations

from dataclasses import replace
from typing import Any, cast

import numpy as np
import pytest

from benchmarks.mujoco_calibration import (
    CalibrationTrace,
    calibration_preflight,
    default_probes,
    evaluate_trace,
    fit_plant_parameters,
    run_synthetic_calibration,
    simulate_probe,
    summarize_split,
    synthetic_ground_truth,
)
from benchmarks.mujoco_scene import MuJoCoSimulationError


def test_calibration_probes_have_disjoint_primary_and_held_out_splits():
    probes = default_probes()
    primary = {probe.name for probe in probes if probe.split == "primary"}
    held_out = {probe.name for probe in probes if probe.split == "held-out"}
    assert primary
    assert held_out
    assert primary.isdisjoint(held_out)
    assert all(probe.gravity_z == 0.0 for probe in probes)


def test_free_space_identification_probe_has_no_floor_contacts():
    probe = next(probe for probe in default_probes() if probe.split == "primary")
    trace = simulate_probe(probe, synthetic_ground_truth())
    assert np.all(trace.contact_counts == 0)
    assert trace.states.shape == (len(probe.commands), 5)
    assert trace.actuator_states.shape == (len(probe.commands), 2)


def test_primary_fit_is_evaluated_on_held_out_without_leakage():
    report = run_synthetic_calibration(max_nfev=20)
    metadata = cast(dict[str, Any], report["metadata"])
    assert metadata["held_out_used_for_fit"] is False
    assert metadata["safety_claim_supported"] is False

    estimate = cast(dict[str, float], report["estimated_parameters"])
    assert np.isclose(estimate["tau_v"], 0.12, atol=1e-8)
    assert np.isclose(estimate["tau_omega"], 0.09, atol=1e-8)
    assert np.isclose(estimate["force_gain"], 9.0, atol=1e-8)
    assert np.isclose(estimate["torque_gain"], 4.0, atol=1e-8)
    assert np.isclose(estimate["slip_gain"], 0.35, atol=1e-8)

    metrics = {
        row["split"]: row
        for row in cast(list[dict[str, Any]], report["split_metrics"])
    }
    assert metrics["primary"]["maximum_state_abs_error"] < 1e-8
    assert metrics["held-out"]["maximum_state_abs_error"] < 1e-8


def test_identifier_rejects_held_out_trace_as_training_data():
    held_out_probe = next(probe for probe in default_probes() if probe.split == "held-out")
    held_out_trace = simulate_probe(held_out_probe, synthetic_ground_truth())
    with pytest.raises(ValueError, match="primary traces only"):
        fit_plant_parameters([held_out_trace], template=synthetic_ground_truth())


def test_preflight_reports_physical_calibration_blocker_without_a_log():
    report = calibration_preflight()
    assert report["calibration_ready"] is False
    measured_data = cast(dict[str, Any], report["measured_data"])
    blockers = cast(list[str], report["blockers"])
    assert measured_data["present"] is False
    assert any("No measured actuator/odometry log" in blocker for blocker in blockers)
    assert report["safety_claim_supported"] is False


@pytest.fixture(scope="module")
def primary_trace():
    return simulate_probe(default_probes()[0], synthetic_ground_truth())


@pytest.mark.parametrize("consumer", ["fit", "evaluate", "summarize"])
@pytest.mark.parametrize(
    ("field", "malformation"),
    [
        ("states", "one_row"),
        ("states", "empty"),
        ("states", "nan"),
        ("actuator_states", "one_row"),
        ("actuator_states", "inf"),
        ("contact_counts", "one_row"),
        ("contact_counts", "nan"),
        ("contact_counts", "negative"),
        ("contact_counts", "fractional"),
    ],
)
def test_fit_and_evaluation_reject_malformed_traces(primary_trace, consumer, field, malformation):
    values = np.asarray(getattr(primary_trace, field), dtype=float).copy()
    if malformation == "one_row":
        values = values[:1]
    elif malformation == "empty":
        values = values[:0]
    else:
        values.flat[0] = {"nan": np.nan, "inf": np.inf, "negative": -1.0, "fractional": 0.5}[
            malformation
        ]
    trace = replace(primary_trace, **{field: values})
    with pytest.raises(ValueError, match=field):
        if consumer == "fit":
            fit_plant_parameters([trace], template=synthetic_ground_truth())
        elif consumer == "evaluate":
            evaluate_trace(trace, synthetic_ground_truth())
        else:
            summarize_split([trace], synthetic_ground_truth())


def test_summarize_split_rejects_mixed_training_and_held_out(primary_trace):
    held_out = simulate_probe(default_probes()[2], synthetic_ground_truth())
    with pytest.raises(ValueError, match="primary and held-out cannot be mixed"):
        summarize_split([primary_trace, held_out], synthetic_ground_truth())


def test_fit_and_all_error_metrics_use_equivalent_wrapped_headings():
    truth = synthetic_ground_truth()
    traces: list[CalibrationTrace] = []
    for probe in default_probes()[:2]:
        state = (*probe.initial_state[:2], np.pi - 0.05, *probe.initial_state[3:])
        trace = simulate_probe(replace(probe, initial_state=state), truth)
        assert np.any(trace.states[:, 2] < 0.0)
        assert np.any(trace.states[:, 2] > 0.0)
        # Equivalent headings across the branch cut must not add 2*pi to the
        # objective or to maximum_state_abs_error.
        trace.states[::2, 2] += 2.0 * np.pi
        trace.states[1::2, 2] -= 2.0 * np.pi
        metrics = evaluate_trace(trace, truth)
        assert metrics.heading_rmse_rad < 1.0e-12
        assert metrics.maximum_state_abs_error < 1.0e-12
        traces.append(trace)

    fit = fit_plant_parameters(traces, template=truth, max_nfev=20)
    assert fit.success
    assert fit.cost < 1.0e-14
    for name, value in fit.as_parameter_dict().items():
        assert value == pytest.approx(getattr(truth, name), abs=1.0e-7)


def test_probe_numerical_failure_is_propagated_instead_of_returning_a_trace(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    probe = replace(default_probes()[0], commands=((1.0e12, 0.0),))
    with pytest.raises(MuJoCoSimulationError, match="mjWARN_BADQACC"):
        simulate_probe(probe, synthetic_ground_truth())


def test_unconverged_identifier_is_not_reported_as_a_recovered_parameter_set():
    with pytest.raises(RuntimeError, match="identification did not converge"):
        run_synthetic_calibration(max_nfev=1)
