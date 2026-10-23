"""Deterministic Gate 1 checks for the augmented model and observer."""

from __future__ import annotations

import numpy as np

from hybrid_controller.controllers.eso import ESOConfig, ExtendedStateObserver3D
from hybrid_controller.models.augmented_dynamics import (
    AugmentedDifferentialDriveModel,
    DynamicModelParams,
)
from hybrid_controller.trajectory.spatial_reference import SpatialFigure8Reference


def _central_jacobian(model, state, control, disturbance, step_size=1e-6):
    analytical_a, analytical_b = model.jacobians(state, control, disturbance)
    numerical_a = np.zeros_like(analytical_a)
    numerical_b = np.zeros_like(analytical_b)
    for index in range(5):
        offset = np.zeros(5)
        offset[index] = step_size
        numerical_a[:, index] = (
            model.step(state + offset, control, disturbance)
            - model.step(state - offset, control, disturbance)
        ) / (2.0 * step_size)
    for index in range(2):
        offset = np.zeros(2)
        offset[index] = step_size
        numerical_b[:, index] = (
            model.step(state, control + offset, disturbance)
            - model.step(state, control - offset, disturbance)
        ) / (2.0 * step_size)
    return analytical_a, analytical_b, numerical_a, numerical_b


def test_augmented_model_jacobians_match_finite_difference():
    rng = np.random.default_rng(20261016)
    model = AugmentedDifferentialDriveModel(
        DynamicModelParams(dt=0.05, tau_v=0.12, tau_omega=0.08)
    )
    worst = 0.0
    for _ in range(50):
        state = np.array(
            [
                rng.uniform(-2.0, 2.0),
                rng.uniform(-2.0, 2.0),
                rng.uniform(-2.5, 2.5),
                rng.uniform(0.05, 1.4),
                rng.uniform(-1.2, 1.2),
            ]
        )
        control = rng.uniform([-1.5, -2.0], [1.5, 2.0])
        disturbance = rng.uniform([-0.5, -0.3, -0.5], [0.5, 0.3, 0.5])
        analytical_a, analytical_b, numerical_a, numerical_b = _central_jacobian(
            model, state, control, disturbance
        )
        worst = max(
            worst,
            float(np.max(np.abs(analytical_a - numerical_a))),
            float(np.max(np.abs(analytical_b - numerical_b))),
        )
    assert worst < 1e-6


def test_eso_lateral_slip_converges_within_fifteen_steps():
    dt = 0.05
    v = 0.8
    omega = 0.6
    true_lateral_slip = 0.15 * v * omega
    model = AugmentedDifferentialDriveModel(
        DynamicModelParams(dt=dt, tau_v=0.1, tau_omega=0.1)
    )
    observer = ExtendedStateObserver3D(
        ESOConfig(dt=dt, tau_v=0.1, tau_omega=0.1, beta=0.35)
    )
    state = np.array([0.0, 0.0, 0.0, v, omega])
    command = np.array([v, omega])
    estimate = observer.update(state, command)
    for _ in range(15):
        state = model.step(
            state,
            command,
            np.array([0.0, true_lateral_slip, 0.0]),
        )
        estimate = observer.update(state, command)
    assert np.isclose(estimate[1], true_lateral_slip, rtol=0.10, atol=1e-4)


def test_eso_pose_adapter_returns_updated_lateral_estimate():
    observer = ExtendedStateObserver3D(ESOConfig(dt=0.05, beta=0.5))
    observer.update_from_pose_and_velocity(
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0]),
        np.array([0.4, 0.0]),
        np.array([0.4, 0.0]),
    )
    estimate = observer.update_from_pose_and_velocity(
        np.array([0.02, 0.01, 0.0]),
        np.array([0.4, 0.2]),
        np.array([0.4, 0.0]),
        np.array([0.4, 0.0]),
    )
    assert np.allclose(estimate, observer.estimate)
    assert estimate[1] > 0.0


def test_spatial_progress_does_not_run_away_when_position_is_frozen():
    reference = SpatialFigure8Reference(amplitude=2.0)
    starting_progress = 1.25
    frozen_position = reference.position(starting_progress)
    progress = starting_progress
    history = []
    for _ in range(40):
        progress, cross_track_error = reference.update_progress(
            frozen_position,
            progress,
            forward_window=0.75,
            backward_window=0.20,
        )
        history.append((progress, cross_track_error))
    unwrapped = np.unwrap(np.asarray([row[0] for row in history]))
    assert float(np.max(unwrapped) - np.min(unwrapped)) < 0.05
    assert max(row[1] for row in history) < 1e-3
