from __future__ import annotations

import numpy as np
import pytest

from benchmarks.controlled_obstacles import (
    bounded_acceleration_obstacle,
    nonreactive_obstacle,
)


def test_nonreactive_obstacle_has_constant_velocity_and_zero_acceleration():
    obstacle = nonreactive_obstacle(1.0, -0.5, 0.2, vx=0.3, vy=-0.1)
    state = obstacle.state_at(2.0)
    assert np.isclose(state["x"], 1.6)
    assert np.isclose(state["y"], -0.7)
    assert np.isclose(state["vx"], 0.3)
    assert np.isclose(state["ay"], 0.0)
    assert np.isclose(state["acceleration_bound"], 0.0)


def test_bounded_acceleration_obstacle_rejects_out_of_bound_acceleration():
    with pytest.raises(ValueError):
        bounded_acceleration_obstacle(
            0.0,
            0.0,
            0.1,
            ax=1.0,
            acceleration_bound=0.5,
        )


def test_bounded_acceleration_obstacle_reports_reachable_state_fields():
    obstacle = bounded_acceleration_obstacle(
        0.0,
        0.0,
        0.1,
        vx=0.2,
        ax=0.4,
        acceleration_bound=0.5,
    )
    state = obstacle.state_at(0.5)
    assert np.isclose(state["x"], 0.15)
    assert np.isclose(state["vx"], 0.4)
    assert np.isclose(state["acceleration_bound"], 0.5)
