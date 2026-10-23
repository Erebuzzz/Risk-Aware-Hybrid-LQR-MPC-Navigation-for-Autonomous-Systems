"""Unit tests for decision-time benchmark telemetry helpers."""

from __future__ import annotations

import json

import numpy as np

from benchmarks.run_mujoco_monte_carlo import (
    make_lqr_rollout,
    obstacle_clearances,
    serialize_obstacle_snapshot,
)
from benchmarks.mujoco_scene import MuJoCoPlant
from hybrid_controller.controllers.riccati_safety_filter import (
    RiccatiSafetyFilter,
    SafetyFilterConfig,
)
from hybrid_controller.models.footprint import (
    DEFAULT_CONTROLLER_ROBOT_RADIUS,
    MUJOCO_ROBOT_RADIUS,
)


def test_obstacle_clearances_use_the_declared_robot_radius():
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    obstacles = [{"x": 0.5, "y": 0.0, "radius": 0.1}]
    assert np.isclose(obstacle_clearances(state, obstacles)[0], 0.5 - 0.1 - MUJOCO_ROBOT_RADIUS)


def test_obstacle_snapshot_has_stable_velocity_fields():
    snapshot = json.loads(
        serialize_obstacle_snapshot(
            [
                {"x": 0.5, "y": -0.2, "radius": 0.1},
                {"x": -0.4, "y": 0.3, "radius": 0.2, "vx": 0.1, "vy": -0.2},
            ]
        )
    )
    assert snapshot[0]["vx"] == 0.0
    assert snapshot[0]["vy"] == 0.0
    assert snapshot[1]["vx"] == 0.1
    assert snapshot[1]["vy"] == -0.2


def test_mujoco_reset_preserves_requested_body_velocity():
    plant = MuJoCoPlant([], friction=1.0)
    plant.reset(np.array([0.0, 0.0, np.pi / 2.0, 1.2, -0.3]))
    state = plant.state()
    assert np.isclose(state[3], 1.2, atol=1e-9)
    assert np.isclose(state[4], -0.3, atol=1e-9)


def test_nominal_lqr_rollout_respects_previous_command_slew_limit():
    filter_ = RiccatiSafetyFilter(
        config=SafetyFilterConfig(
            horizon=3,
            max_obstacles=2,
            dt=0.05,
            max_control_delta=(0.1, 0.2),
        )
    )
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    references = np.tile(np.array([0.0, 0.0, 0.0, 1.0, 0.0]), (4, 1))
    controls = np.tile(np.array([1.0, 0.0]), (3, 1))
    _, nominal_controls = make_lqr_rollout(
        filter_, state, references, controls, previous_command=np.zeros(2)
    )
    assert np.all(
        np.abs(nominal_controls[0])
        <= np.asarray([0.1, 0.2]) + 1e-12
    )
    assert np.all(
        np.abs(np.diff(nominal_controls, axis=0))
        <= np.asarray(filter_.config.max_control_delta) + 1e-12
    )


def test_footprint_profiles_are_explicitly_distinct():
    assert np.isclose(DEFAULT_CONTROLLER_ROBOT_RADIUS, 0.105)
    assert MUJOCO_ROBOT_RADIUS > DEFAULT_CONTROLLER_ROBOT_RADIUS
