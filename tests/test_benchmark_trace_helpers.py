"""Unit tests for decision-time benchmark telemetry helpers."""

from __future__ import annotations

import json

import numpy as np

from benchmarks.run_mujoco_monte_carlo import (
    obstacle_clearances,
    serialize_obstacle_snapshot,
)
from benchmarks.mujoco_scene import MuJoCoPlant


def test_obstacle_clearances_use_the_declared_robot_radius():
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    obstacles = [{"x": 0.5, "y": 0.0, "radius": 0.1}]
    assert np.isclose(obstacle_clearances(state, obstacles)[0], 0.2)


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
