"""Deterministic parity checks for the PyO3 numerical core."""

from __future__ import annotations

import numpy as np
from scipy.stats import norm

import ra_psf_rs
from hybrid_controller.models.augmented_dynamics import (
    AugmentedDifferentialDriveModel,
    DynamicModelParams,
)
from hybrid_controller.controllers.riccati_safety_filter import (
    RiccatiSafetyFilter,
    SafetyFilterConfig,
)


def test_rust_step_and_jacobians_match_python_reference():
    rng = np.random.default_rng(20261016)
    model = AugmentedDifferentialDriveModel(
        DynamicModelParams(dt=0.05, tau_v=0.12, tau_omega=0.08)
    )
    for _ in range(50):
        state = np.array(
            [
                rng.uniform(-2.0, 2.0),
                rng.uniform(-2.0, 2.0),
                rng.uniform(-2.5, 2.5),
                rng.uniform(-1.0, 1.5),
                rng.uniform(-1.5, 1.5),
            ]
        )
        control = rng.uniform([-1.5, -2.0], [1.5, 2.0])
        disturbance = rng.uniform([-0.5, -0.3, -0.5], [0.5, 0.3, 0.5])
        python_next = model.step(state, control, disturbance)
        rust_next = np.asarray(
            ra_psf_rs.step(
                state.tolist(),
                control.tolist(),
                disturbance.tolist(),
                0.05,
                0.12,
                0.08,
            )
        )
        assert np.max(np.abs(python_next - rust_next)) < 1e-7

        python_a, python_b = model.jacobians(state, control, disturbance)
        rust_a, rust_b = ra_psf_rs.jacobians(
            state.tolist(), disturbance.tolist(), 0.05, 0.12, 0.08
        )
        assert np.max(np.abs(python_a - np.asarray(rust_a))) < 1e-7
        assert np.max(np.abs(python_b - np.asarray(rust_b))) < 1e-7


def test_rust_rollout_matches_repeated_python_steps():
    rng = np.random.default_rng(17)
    model = AugmentedDifferentialDriveModel()
    state = rng.normal(size=5)
    controls = rng.uniform([-1.0, -1.5], [1.0, 1.5], size=(12, 2))
    disturbance = np.array([0.1, 0.04, -0.03])
    python_states = [state.copy()]
    for control in controls:
        state = model.step(state, control, disturbance)
        python_states.append(state.copy())
    rust_states = np.asarray(
        ra_psf_rs.rollout(
            python_states[0].tolist(),
            controls.tolist(),
            disturbance.tolist(),
            0.05,
            0.1,
            0.1,
        )
    )
    assert np.max(np.abs(np.asarray(python_states) - rust_states)) < 1e-7


def test_rust_cvar_multiplier_matches_scipy():
    for alpha in (0.90, 0.95, 0.99):
        expected = norm.pdf(norm.ppf(alpha)) / (1.0 - alpha)
        assert abs(ra_psf_rs.cvar_multiplier(alpha) - expected) < 1e-5


def test_rust_osqp_step_filter_respects_tangent_barrier():
    control, status, _ = ra_psf_rs.solve_step_qp(
        [0.3, 0.0],
        [2.0, 4.0],
        [0.0, 0.0],
        [-0.3, -1.0],
        [0.7, 1.0],
        [[0.0, 1.0]],
        [0.5],
    )
    assert status == "solved"
    assert control[1] >= 0.5 - 1e-4


def test_rust_horizon_filter_matches_python_reference():
    config = SafetyFilterConfig(horizon=5, max_obstacles=3, dt=0.05)
    python_filter = RiccatiSafetyFilter(config=config)
    rust_filter = RiccatiSafetyFilter(
        config=SafetyFilterConfig(horizon=5, max_obstacles=3, dt=0.05, backend="rust")
    )
    state = np.array([0.0, 0.0, 0.0, 0.4, 0.0])
    references = np.tile(state, (config.horizon + 1, 1))
    controls = np.tile(np.array([0.4, 0.0]), (config.horizon, 1))
    obstacles = [
        {"x": 0.25, "y": 0.0, "radius": 0.2, "vx": 0.0, "vy": 0.0},
    ]

    python_solution = python_filter.solve(state, references, controls, obstacles)
    rust_solution = rust_filter.solve(state, references, controls, obstacles)

    assert python_solution.feasible
    assert rust_solution.feasible
    assert np.max(
        np.abs(python_solution.optimal_control - rust_solution.optimal_control)
    ) < 1e-3
    assert np.max(
        np.abs(python_solution.predicted_states - rust_solution.predicted_states)
    ) < 1e-3
    assert np.all(
        np.abs(rust_solution.optimal_control - controls[0])
        <= np.asarray(config.max_control_delta) + 1e-3
    )
