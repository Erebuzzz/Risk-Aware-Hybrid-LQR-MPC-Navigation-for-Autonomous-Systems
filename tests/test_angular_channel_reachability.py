"""Focused tests for the diagnostic angular-channel MuJoCo benchmark."""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, replace

import numpy as np
import pytest

from benchmarks.run_angular_channel_reachability import (
    DEFAULT_SCENARIO_SEED,
    DECISION_PARAMETERS,
    evaluate_scenario,
    generate_scenarios,
    run_experiment,
    write_report,
)
from hybrid_controller.controllers.angular_channel import evaluate_angular_channel


def test_fixed_seed_primary_and_held_out_sets_are_reproducible_and_disjoint():
    primary_a = generate_scenarios("primary", seed=DEFAULT_SCENARIO_SEED, count=3)
    primary_b = generate_scenarios("primary", seed=DEFAULT_SCENARIO_SEED, count=3)
    held_out = generate_scenarios("held-out", seed=DEFAULT_SCENARIO_SEED, count=3)

    assert [asdict(item) for item in primary_a] == [asdict(item) for item in primary_b]
    assert {item.scenario_id for item in primary_a}.isdisjoint(
        item.scenario_id for item in held_out
    )
    assert all(item.obstacle.vx0 == 0.0 and item.obstacle.vy0 == 0.0 for item in primary_a)
    assert all(item.obstacle.vx0 == 0.0 and item.obstacle.vy0 == 0.0 for item in held_out)


def test_decision_margins_and_trigger_are_separate_from_two_sided_plant_outcomes():
    scenario = replace(
        generate_scenarios("primary", seed=DEFAULT_SCENARIO_SEED, count=1)[0],
        horizon_steps=3,
    )
    record = evaluate_scenario(scenario)
    expected = evaluate_angular_channel(
        np.asarray(scenario.initial_state),
        scenario.obstacle_at_reset,
        previous_omega_command=scenario.previous_omega_command,
        **DECISION_PARAMETERS,
    )

    decision = record["decision"]
    assert decision["selected_side"] == expected.selected_side
    assert decision["trigger"] is expected.trigger
    assert decision["left_margin"] == pytest.approx(expected.left_margin)
    assert decision["right_margin"] == pytest.approx(expected.right_margin)
    assert decision["selected_margin"] == pytest.approx(expected.selected_margin)
    assert decision["trigger_prediction_label"] == "controller_trigger_only"

    plant = record["plant"]
    assert set(("left", "right", "selected_side_outcome")) <= set(plant)
    assert plant["selected_side"] == expected.selected_side
    assert plant["left"]["steps"] == 3
    assert plant["right"]["steps"] == 3
    assert plant["actual_avoidance_evidence"] is False
    assert record["diagnostic_scope"]["production_safety_connected"] is False
    assert record["diagnostic_scope"]["trigger_is_not_contact"] is True


class _NoContactTelemetryPlant:
    """Deterministic fixture exposing plant state but no contact telemetry."""

    def __init__(self, obstacles, *, friction, config):
        del obstacles, friction
        self.config = config
        self._state = np.zeros(5, dtype=float)
        self._actuator = np.zeros(2, dtype=float)

    def reset(self, state):
        self._state = np.asarray(state, dtype=float).reshape(5).copy()
        self._actuator = self._state[3:5].copy()

    def step(self, command):
        command = np.asarray(command, dtype=float).reshape(2)
        self._actuator = command.copy()
        self._state[2] += self._actuator[1] * self.config.dt
        self._state[0] += self._actuator[0] * np.cos(self._state[2]) * self.config.dt
        self._state[1] += self._actuator[0] * np.sin(self._state[2]) * self.config.dt
        self._state[3:] = self._actuator
        return self._state.copy()

    def actuator_state(self):
        return self._actuator.copy()


def test_unavailable_obstacle_contact_is_reported_as_unknown_not_no_contact():
    scenario = replace(
        generate_scenarios("held-out", seed=DEFAULT_SCENARIO_SEED, count=1)[0],
        horizon_steps=2,
    )
    record = evaluate_scenario(scenario, plant_factory=_NoContactTelemetryPlant)
    selected = record["plant"]["selected_side_outcome"]

    assert selected["obstacle_contact"] is None
    assert selected["finite_no_contact_plane_crossing"] is None
    assert selected["outcome_class"] == "obstacle_contact_telemetry_unavailable"
    assert record["plant"]["actual_avoidance_evidence"] is None
    assert record["plant"]["avoidance_claim"] == "not_established"
    assert "unavailable" in record["plant"]["contact_telemetry_note"]


def test_report_keeps_primary_and_held_out_metrics_separate(tmp_path):
    report = run_experiment(
        seed=DEFAULT_SCENARIO_SEED,
        primary_count=1,
        held_out_count=1,
    )
    assert report["metadata"]["held_out_used_for_tuning"] is False
    assert report["metadata"]["production_safety_connected"] is False
    assert report["primary_metrics"]["case_count"] == 1
    assert report["held_out_metrics"]["case_count"] == 1
    assert report["primary_metrics"]["prediction_and_plant_reach_are_separate"] is True
    assert report["held_out_metrics"]["contact_is_not_inferred_from_trigger"] is True

    json_path, csv_path = write_report(report, tmp_path)
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    assert payload["metadata"]["benchmark"] == "angular_channel_reachability"
    assert len(payload["cases"]) == 2
    with csv_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert {row["split"] for row in rows} == {"primary", "held-out"}
    assert "decision_selected_side" in rows[0]
    assert "decision_left_margin" in rows[0]
    assert "plant_selected_yaw_delta_rad" in rows[0]
    with pytest.raises(FileExistsError):
        write_report(report, tmp_path)
