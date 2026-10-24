"""Checks for Monte Carlo contact/proxy metric separation."""

from __future__ import annotations

from benchmarks.run_mujoco_monte_carlo import TrialMetrics, aggregate


def _metrics(*, collision: bool, proxy: bool, contacts: int) -> TrialMetrics:
    return TrialMetrics(
        mode="test",
        seed=1,
        friction=0.5,
        tau_v=0.1,
        tau_omega=0.1,
        collision=collision,
        minimum_clearance=-0.1 if proxy else 0.1,
        rms_cross_track_error=0.0,
        angular_jerk_rms=0.0,
        intervention_ratio=0.0,
        mean_intervention_norm=0.0,
        mean_cvar_risk=0.0,
        maximum_cvar_risk=0.0,
        maximum_slack=0.0,
        mean_latency_ms=0.0,
        p95_latency_ms=0.0,
        p99_latency_ms=0.0,
        infeasible_count=0,
        fallback_count=0,
        steps=1,
        obstacle_contact_count=contacts,
        obstacle_contact_intervals=int(contacts > 0),
        geometric_collision_proxy=proxy,
    )


def test_static_contact_rate_and_geometric_proxy_rate_are_separate():
    summary = aggregate(
        [
            _metrics(collision=True, proxy=True, contacts=2),
            _metrics(collision=False, proxy=True, contacts=0),
        ]
    )["test"]

    assert summary["collision_rate"] == 0.5
    assert summary["static_obstacle_contact_rate"] == 0.5
    assert summary["geometric_collision_proxy_rate"] == 1.0
    assert summary["obstacle_contact_count"] == 1.0
