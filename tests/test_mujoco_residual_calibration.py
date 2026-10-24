"""Focused checks for the reduced-model residual calibration benchmark."""

from __future__ import annotations

from dataclasses import replace
from typing import Any, cast

import numpy as np
import pytest

from benchmarks.mujoco_residual_calibration import (
    ResidualTrace,
    build_residual_envelope,
    default_residual_probes,
    fit_residual_model,
    run_residual_calibration,
    simulate_residual_probe,
    summarize_split,
    validate_probe_partition,
)


@pytest.fixture(scope="module")
def residual_traces():
    return [simulate_residual_probe(probe) for probe in default_residual_probes()]


@pytest.fixture(scope="module")
def fitted_residuals(residual_traces):
    return fit_residual_model(residual_traces[:2])


def test_fixed_probe_partition_is_disjoint_and_contact_free(residual_traces):
    probes = default_residual_probes()
    validate_probe_partition(probes)
    primary = {probe.name for probe in probes if probe.split == "primary"}
    held_out = {probe.name for probe in probes if probe.split == "held-out"}
    assert primary and held_out and primary.isdisjoint(held_out)
    assert all(np.all(trace.contact_counts == 0) for trace in residual_traces)


def test_primary_fit_reports_position_yaw_velocity_and_primary_envelope(
    residual_traces, fitted_residuals
):
    primary = residual_traces[:2]
    held_out = residual_traces[2:]
    primary_metrics = summarize_split(primary, fitted_residuals)
    held_out_metrics = summarize_split(held_out, fitted_residuals)
    envelope = build_residual_envelope(primary, fitted_residuals)

    assert fitted_residuals.success
    assert fitted_residuals.fit_probe_names == tuple(trace.probe.name for trace in primary)
    assert primary_metrics.position_rmse_m >= 0.0
    assert primary_metrics.yaw_rmse_rad >= 0.0
    assert primary_metrics.linear_velocity_rmse_mps >= 0.0
    assert primary_metrics.angular_velocity_rmse_rps >= 0.0
    assert held_out_metrics.split == "held-out"
    assert envelope.source_split == "primary"
    assert envelope.source_sample_count == primary_metrics.sample_count
    assert all(value >= 0.0 for value in envelope.bounds.values())


def test_report_marks_held_out_as_evaluation_only_and_disables_safety_correction():
    report = run_residual_calibration(max_nfev=40)
    metadata = cast(dict[str, Any], report["metadata"])
    assert metadata["held_out_used_for_fit"] is False
    assert metadata["safety_correction_applied"] is False
    assert metadata["safety_claim_supported"] is False
    assert metadata["fit_probe_names"] == metadata["primary_probe_names"]
    assert set(metadata["fit_probe_names"]).isdisjoint(metadata["held_out_probe_names"])
    assert metadata["inter_sample_limitations"]
    envelope = cast(dict[str, Any], report["residual_envelope"])
    assert envelope["source_split"] == "primary"


def test_fit_and_summary_reject_mixed_or_held_out_traces(residual_traces, fitted_residuals):
    with pytest.raises(ValueError, match="primary and held-out"):
        fit_residual_model(residual_traces[:1] + residual_traces[2:3])
    with pytest.raises(ValueError, match="primary"):
        fit_residual_model(residual_traces[2:])
    with pytest.raises(ValueError, match="primary and held-out"):
        summarize_split([residual_traces[0], residual_traces[2]], fitted_residuals)


def test_duplicate_content_across_splits_is_rejected():
    primary = default_residual_probes()[0]
    duplicate = replace(primary, name="accidental_held_out_copy", split="held-out")
    with pytest.raises(ValueError, match="duplicate residual probe content"):
        validate_probe_partition([primary, duplicate])


@pytest.mark.parametrize("field, value", [
    ("states", np.zeros((1, 5))),
    ("states", np.full((len(default_residual_probes()[0].commands), 5), np.nan)),
    ("contact_counts", np.ones(len(default_residual_probes()[0].commands))),
])
def test_invalid_or_contact_traces_are_rejected(residual_traces, field, value):
    trace = residual_traces[0]
    malformed = replace(trace, **{field: value})
    with pytest.raises(ValueError, match="trace|contact"):
        summarize_split([malformed], fit_residual_model(residual_traces[:2]))


def test_nonconverged_fit_is_rejected(residual_traces):
    with pytest.raises(RuntimeError, match="did not converge"):
        fit_residual_model(residual_traces[:2], max_nfev=1)


def test_trace_arrays_are_not_mutated_by_evaluation(residual_traces, fitted_residuals):
    trace = residual_traces[0]
    states_before = trace.states.copy()
    contacts_before = trace.contact_counts.copy()
    summarize_split([trace], fitted_residuals)
    np.testing.assert_array_equal(trace.states, states_before)
    np.testing.assert_array_equal(trace.contact_counts, contacts_before)
