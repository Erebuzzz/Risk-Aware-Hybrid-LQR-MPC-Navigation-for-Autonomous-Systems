"""Deterministic Gate 2 checks for the CVaR and Riccati reference filter."""

from __future__ import annotations

import numpy as np

from hybrid_controller.controllers.cvar_gate import CVaRRiskGate, RiskHysteresis
from hybrid_controller.controllers.adaptive_mpc_safety_filter import (
    AdaptiveMPCSafetyFilter,
)
from hybrid_controller.controllers.backup_safety import BackupSafetyController
from hybrid_controller.controllers.riccati_safety_filter import (
    RiccatiSafetyFilter,
    SafetyFilterConfig,
    SafetyFilterSolution,
)
from hybrid_controller.controllers.risk_predictor import LQRRiskPredictor


def test_proposition_one_convex_blend_remains_barrier_violating():
    delta = 0.7
    lqr_value = -delta
    mpc_value = 0.0
    for alpha in np.linspace(0.0, 0.99, 100):
        blended = (1.0 - alpha) * lqr_value + alpha * mpc_value
        expected = -(1.0 - alpha) * delta
        assert np.isclose(blended, expected, atol=1e-12)
        assert blended < 0.0


def test_cvar_gate_detects_penetration_with_uncertainty_padding():
    gate = CVaRRiskGate(alpha=0.95, d_safe=0.3, robot_radius=0.1, dt=0.05)
    states = np.array([[0.0, 0.0, 0.0], [0.2, 0.0, 0.0]])
    covariances = np.repeat(np.eye(5)[None, :, :] * 0.01, 2, axis=0)
    result = gate.evaluate(
        states,
        covariances,
        [{"x": 0.4, "y": 0.0, "radius": 0.1}],
    )
    assert not result.safe
    assert result.first_violation_step == 0
    assert result.maximum_risk > 0.0


def test_empty_risk_rollout_uses_finite_no_obstacle_margin_sentinel():
    predictor = LQRRiskPredictor(d_trigger=0.75)
    result = predictor.assess(
        np.zeros(3),
        np.zeros((predictor.horizon, 2)),
        np.ones(2),
        obstacles=[],
    )

    assert result.safe
    assert result.risk_score == 0.0
    assert result.min_margin == 0.75
    assert np.isfinite(result.min_margin)


def test_risk_hysteresis_has_distinct_entry_and_exit_thresholds():
    latch = RiskHysteresis(enter_threshold=0.2, exit_threshold=-0.1)
    assert not latch.update(0.19)
    assert latch.update(0.2)
    assert latch.update(0.0)
    assert not latch.update(-0.1)


def test_riccati_filter_returns_finite_control_and_reuses_workspace():
    config = SafetyFilterConfig(horizon=5, max_obstacles=3, dt=0.05)
    filter_ = RiccatiSafetyFilter(config=config)
    state = np.array([0.0, 0.0, 0.0, 0.3, 0.0])
    references = np.tile(state, (config.horizon + 1, 1))
    controls = np.tile(np.array([0.3, 0.0]), (config.horizon, 1))
    obstacles = [{"x": 1.0, "y": 0.0, "radius": 0.2}]

    first = filter_.solve(state, references, controls, obstacles)
    second = filter_.solve(state, references, controls, obstacles)
    assert first.feasible
    assert second.feasible
    assert np.all(np.isfinite(first.optimal_control))
    assert np.all(np.isfinite(second.optimal_control))
    assert second.solve_time_ms < first.solve_time_ms


def test_riccati_filter_has_finite_outputs_for_random_obstacles():
    config = SafetyFilterConfig(horizon=5, max_obstacles=4, dt=0.05)
    filter_ = RiccatiSafetyFilter(config=config)
    state = np.array([0.0, 0.0, 0.0, 0.4, 0.0])
    references = np.tile(state, (config.horizon + 1, 1))
    controls = np.tile(np.array([0.4, 0.0]), (config.horizon, 1))
    rng = np.random.default_rng(123)

    for _ in range(200):
        obstacles = [
            {
                "x": float(rng.uniform(-1.0, 1.0)),
                "y": float(rng.uniform(-1.0, 1.0)),
                "radius": float(rng.uniform(0.1, 0.25)),
                "vx": float(rng.uniform(-0.2, 0.2)),
                "vy": float(rng.uniform(-0.2, 0.2)),
            }
            for _ in range(config.max_obstacles)
        ]
        solution = filter_.solve(state, references, controls, obstacles)
        assert solution.feasible
        assert np.all(np.isfinite(solution.optimal_control))
        assert np.all(np.isfinite(solution.predicted_states))


def test_command_slew_limit_is_applied_when_filter_is_bypassed():
    filter_ = RiccatiSafetyFilter(
        config=SafetyFilterConfig(
            horizon=3,
            max_obstacles=2,
            dt=0.05,
            max_control_delta=(0.1, 0.2),
        )
    )
    filter_.set_previous_command(np.zeros(2))
    first = filter_.limit_command(np.array([1.0, 1.0]))
    second = filter_.limit_command(np.array([1.0, 1.0]))
    assert np.allclose(first, [0.1, 0.2])
    assert np.allclose(second, [0.2, 0.4])


def test_solver_feasibility_is_distinct_from_obstacle_safety():
    solution = SafetyFilterSolution(
        status="rust_solved",
        optimal_control=np.zeros(2),
        control_sequence=np.zeros((2, 2)),
        predicted_states=np.zeros((3, 5)),
        solve_time_ms=1.0,
        intervention_norm=0.0,
        slack_max=0.01,
        feasible=True,
    )
    assert solution.feasible
    assert not solution.safety_satisfied


def test_augmented_backup_uses_finite_turning_command_near_obstacle():
    backup = BackupSafetyController(v_max=2.5, omega_max=3.0)
    result = backup.compute_augmented(
        np.array([0.0, 0.0, 0.0, 1.0, 0.0]),
        [{"x": 0.35, "y": 0.0, "radius": 0.15}],
        dt=0.05,
        tau_v=0.1,
        tau_omega=0.1,
        horizon=10,
    )
    assert np.all(np.isfinite(result.control))
    assert abs(result.control[1]) > 0.0
    assert abs(result.control[0]) <= 2.5
    assert abs(result.control[1]) <= 3.0


def test_adaptive_controller_exposes_opt_in_augmented_safety_filter():
    controller = AdaptiveMPCSafetyFilter(
        dt=0.05,
        v_max=1.0,
        omega_max=2.0,
        augmented_backend="python",
        augmented_horizon=3,
        augmented_max_obstacles=2,
    )
    state = np.array([0.0, 0.0, 0.0, 0.2, 0.0])
    references = np.tile(state, (4, 1))
    controls = np.tile(np.array([0.2, 0.0]), (3, 1))
    result = controller.compute_augmented_control(
        state,
        references,
        controls,
        obstacles=[{"x": 1.0, "y": 0.5, "radius": 0.1}],
    )
    assert result.feasible
    assert np.all(np.isfinite(result.optimal_control))


def test_adaptive_controller_requires_explicit_augmented_backend():
    controller = AdaptiveMPCSafetyFilter()
    with np.testing.assert_raises(RuntimeError):
        controller.compute_augmented_control(
            np.zeros(5), np.zeros((6, 5)), np.zeros((5, 2))
        )
