"""
Test MPCSolution.feasible field and basic MPC solve.

These tests verify the root-cause fix for "robot not moving" in MPC modes:
the MPCSolution dataclass must have a `feasible` field that is True when
the solver returns a usable control command.

Run:
    python -m pytest tests/test_mpc_solution.py -v
"""

import numpy as np
import pytest


def test_mpc_solution_has_feasible_field():
    """MPCSolution must have a 'feasible' attribute."""
    from hybrid_controller.controllers.mpc_controller import MPCSolution

    sol = MPCSolution(
        status="optimal",
        optimal_control=np.array([0.1, 0.0]),
        control_sequence=np.zeros((10, 2)),
        predicted_states=np.zeros((11, 3)),
        cost=1.0,
        solve_time_ms=5.0,
        slack_used=False,
        iterations=0,
        feasible=True,
    )
    assert hasattr(sol, 'feasible')
    assert sol.feasible is True


def test_mpc_solution_feasible_defaults_true():
    """MPCSolution.feasible should default to True (all construction sites
    in the codebase omit it, relying on the default)."""
    from hybrid_controller.controllers.mpc_controller import MPCSolution

    sol = MPCSolution(
        status="fallback",
        optimal_control=np.array([0.05, 0.1]),
        control_sequence=np.zeros((10, 2)),
        predicted_states=np.zeros((11, 3)),
        cost=float('inf'),
        solve_time_ms=2.0,
        slack_used=False,
        iterations=0,
    )
    assert sol.feasible is True


def test_mpc_solve_returns_feasible_solution():
    """A simple MPC solve on a straight-line reference must return
    feasible=True and non-zero optimal_control."""
    from hybrid_controller.controllers.mpc_controller import MPCController

    mpc = MPCController(
        horizon=5,
        dt=0.05,
        v_max=1.0,
        omega_max=1.5,
        Q_diag=[10.0, 10.0, 5.0],
        R_diag=[0.1, 0.1],
        P_diag=[20.0, 20.0, 10.0],
    )

    x0 = np.array([0.0, 0.0, 0.0])
    x_refs = np.array([[0.1 * k, 0.0, 0.0] for k in range(6)])
    u_refs = np.array([[0.1, 0.0]] * 5)

    sol = mpc.solve(x0, x_refs, u_refs, obstacles=[])

    assert sol.feasible is True
    assert sol.status == "optimal"
    assert np.linalg.norm(sol.optimal_control) > 0, \
        "MPC should produce non-zero control for straight-line tracking"


def test_mpc_fallback_is_feasible():
    """Even when MPC falls back to P-control, feasible should be True
    (the fallback still produces a usable control command)."""
    from hybrid_controller.controllers.mpc_controller import MPCController

    mpc = MPCController(horizon=5, dt=0.05, v_max=1.0, omega_max=1.5)

    x0 = np.array([0.0, 0.0, 0.0])
    x_refs = np.array([[0.1 * k, 0.0, 0.0] for k in range(6)])
    u_refs = np.array([[0.1, 0.0]] * 5)

    # Directly test the fallback path
    fallback = mpc._get_fallback_solution(x0, x_refs, u_refs, solve_time=1.0)
    assert fallback.feasible is True
    assert fallback.status == "fallback"


def test_adaptive_mpc_solution_has_feasible():
    """AdaptiveMPCController.solve_tracking must return a solution with
    feasible=True on a simple tracking problem."""
    try:
        from hybrid_controller.controllers.adaptive_mpc_controller import (
            AdaptiveMPCController,
        )
    except ImportError:
        pytest.skip("CasADi not available")

    ampc = AdaptiveMPCController(
        prediction_horizon=5,
        terminal_horizon=3,
        dt=0.05,
        v_max=1.0,
        omega_max=1.5,
    )

    x0 = np.array([0.0, 0.0, 0.0])
    x_refs = np.array([[0.1 * k, 0.0, 0.0] for k in range(9)])  # N+M+1 = 9
    u_refs = np.array([[0.1, 0.0]] * 8)  # N+M = 8

    sol = ampc.solve_tracking(x0, x_refs, u_refs, obstacles=[])
    assert hasattr(sol, 'feasible')
    assert sol.feasible is True
