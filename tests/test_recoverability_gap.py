"""Tests for the deterministic actuator-lag recoverability diagnostic."""

from __future__ import annotations

import numpy as np

from benchmarks.validate_recoverability_gap import (
    continuous_minimum_travel,
    discrete_stopping_distance,
    discrete_stopping_trace,
    evaluate_case,
    run_grid,
    summarize,
)
from hybrid_controller.models.augmented_dynamics import (
    AugmentedDifferentialDriveModel,
    DynamicModelParams,
)


def test_discrete_distance_matches_augmented_model_left_endpoint_convention():
    v0 = 1.1
    u0 = 1.4
    delta_u = 0.2
    dt = 0.05
    tau_v = 0.12
    commands, velocities = discrete_stopping_trace(v0, u0, delta_u, dt, tau_v)
    model = AugmentedDifferentialDriveModel(
        DynamicModelParams(dt=dt, tau_v=tau_v, tau_omega=tau_v)
    )
    state = np.array([0.0, 0.0, 0.0, v0, 0.0])
    distance = 0.0
    for command, velocity in zip(commands, velocities):
        assert np.isclose(state[3], velocity)
        distance += dt * state[3]
        state = model.step(state, np.array([command, 0.0]))
    distance += dt * state[3] / (1.0 - np.exp(-dt / tau_v))
    assert np.isclose(distance, discrete_stopping_distance(v0, u0, delta_u, dt, tau_v))
    assert np.isclose(state[1], 0.0)


def test_discrete_stopping_distance_exposes_zero_order_hold_correction():
    continuous = continuous_minimum_travel(1.5, 1.5, 2.0, 0.10)
    discrete = discrete_stopping_distance(1.5, 1.5, 0.10, 0.05, 0.10)
    assert discrete > continuous
    assert discrete - continuous < 0.15


def test_recoverability_grid_reports_safe_boundary_disagreement():
    cases = run_grid(
        v0_values=[0.5],
        command_gap_values=[0.5],
        obstacle_distance_values=[0.7, 0.9],
        tau_values=[0.10],
        delta_u_values=[0.10],
        dt=0.05,
    )
    assert len(cases) == 2
    assert all(case.discrete_distance >= case.continuous_distance for case in cases)
    summary = summarize(cases)
    assert summary["case_count"] == 2
    assert summary["continuous_underprediction_count"] == 2


def test_evaluate_case_rejects_negative_command_gap():
    with np.testing.assert_raises(ValueError):
        evaluate_case(
            v0=0.5,
            command_gap=-0.1,
            obstacle_distance=1.0,
            tau_v=0.1,
            delta_u=0.1,
            dt=0.05,
        )
