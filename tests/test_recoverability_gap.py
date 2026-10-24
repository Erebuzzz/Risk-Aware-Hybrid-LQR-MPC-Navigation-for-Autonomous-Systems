"""Tests for the deterministic actuator-lag recoverability diagnostic."""

from __future__ import annotations

import numpy as np
import pytest
from types import SimpleNamespace

from benchmarks.validate_recoverability_gap import (
    STOPPING_MEASUREMENT_PROTOCOL,
    compare_with_mujoco,
    continuous_minimum_travel,
    discrete_stopping_distance,
    discrete_stopping_trace,
    evaluate_case,
    measure_mujoco_stop,
    mujoco_stopping_distance,
    run_grid,
    state_only_minimum_travel,
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
        obstacle_distance_values=[0.1, 0.2],
        tau_values=[0.10],
        delta_u_values=[0.10],
        dt=0.05,
    )
    assert len(cases) == 2
    assert all(case.discrete_distance >= case.continuous_distance for case in cases)
    summary = summarize(cases)
    assert summary["case_count"] == 2
    assert summary["continuous_underprediction_count"] == 2
    assert summary["state_only_vs_continuous_disagreement_count"] == 2


def test_state_only_bound_ignores_latched_command_gap():
    assert np.isclose(state_only_minimum_travel(1.0, 0.1), 0.1)
    case = evaluate_case(
        v0=1.0,
        command_gap=1.0,
        obstacle_distance=0.2,
        tau_v=0.1,
        delta_u=0.1,
        dt=0.05,
    )
    assert case.state_only_safe
    assert not case.continuous_safe


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


@pytest.fixture
def scripted_plant(monkeypatch):
    """Replace only the physical plant to exercise the measurement protocol."""
    from benchmarks import mujoco_scene

    def install(frames):
        instances = []

        class ScriptedPlant:
            def __init__(self, obstacles, friction, config):
                assert obstacles == []
                self.config = config
                self.commands = []
                self.frame = {}
                self.data = SimpleNamespace(
                    qpos=np.zeros(7), qvel=np.zeros(6), qacc=np.zeros(6), time=0.0
                )
                instances.append(self)

            def reset(self, state):
                self.frame = {"x": state[0], "v": state[3], "actuator_v": state[3]}
                self.data.qvel[0] = state[3]

            def state(self):
                return np.array([
                    self.frame.get("x", 0.0), 0.0, 0.0,
                    self.frame.get("v", 0.0), self.frame.get("omega", 0.0),
                ])

            def actuator_state(self):
                return np.array([self.frame.get("actuator_v", 0.0), self.frame.get("actuator_omega", 0.0)])

            def step(self, command):
                index = len(self.commands)
                self.commands.append(command.copy())
                frame = frames[min(index, len(frames) - 1)]
                if isinstance(frame, Exception):
                    raise frame
                self.frame = frame
                self.data.time = frame.get("time_s", self.data.time + self.config.dt)
                self.data.qpos[0] = frame.get("x", 0.0)
                self.data.qvel[:] = [
                    frame.get("v", 0.0), frame.get("vy", 0.0), frame.get("vz", 0.0),
                    frame.get("roll_rate", 0.0), frame.get("pitch_rate", 0.0), frame.get("omega", 0.0),
                ]
                return self.state()

        monkeypatch.setattr(mujoco_scene, "MuJoCoPlant", ScriptedPlant)
        return instances

    return install


def test_stop_waits_for_hidden_actuator_even_when_body_is_temporarily_stationary(scripted_plant):
    frames = [{"x": 0.1, "actuator_v": 0.2}] * 12
    frames += [{"x": 0.6, "v": 0.2}, {"x": 0.7}]
    plants = scripted_plant(frames)
    measured = measure_mujoco_stop(0.5, 0.0, 0.1, 0.05, 0.1)
    assert measured.status == "settled"
    assert measured.maximum_sampled_forward_travel_m == 0.7
    assert measured.completed_steps == 24
    assert measured.quiet_samples == 11
    assert measured.elapsed_time_s == pytest.approx(1.2)
    assert all(command[0] == 0.0 for command in plants[0].commands)


def test_rebound_resets_consecutive_settling_window(scripted_plant):
    scripted_plant([{"x": 0.1}] * 5 + [{"x": 0.3, "v": 0.2}, {"x": 0.4}])
    measured = measure_mujoco_stop(0.5, 0.0, 0.1, 0.05, 0.1)
    assert measured.status == "settled"
    assert measured.completed_steps == 17
    assert measured.maximum_sampled_forward_travel_m == 0.4


def test_stopping_distance_is_peak_forward_travel_not_terminal_position(scripted_plant):
    scripted_plant([
        {"x": 0.3, "v": 0.2}, {"x": 0.5}, {"x": 0.4, "v": -0.2}, {"x": 0.4},
    ])
    measured = measure_mujoco_stop(0.5, 0.0, 0.1, 0.05, 0.1)
    assert measured.status == "settled"
    assert measured.maximum_sampled_forward_travel_m == 0.5
    assert measured.terminal_forward_travel_m == 0.4
    assert measured.maximum_travel_time_s == pytest.approx(0.1)
    assert mujoco_stopping_distance(0.5, 0.0, 0.1, 0.05, 0.1) == 0.5


@pytest.mark.parametrize("channel", ["v", "vy", "vz", "roll_rate", "pitch_rate", "omega", "actuator_v", "actuator_omega"])
def test_all_body_and_hidden_actuator_velocity_channels_must_settle(scripted_plant, channel):
    scripted_plant([{channel: 0.01}])
    measured = measure_mujoco_stop(0.5, 0.0, 0.1, 0.05, 0.1, max_steps=15)
    assert measured.status == "not_settled"
    assert measured.quiet_samples == 0
    assert measured.completed_steps == 15


def test_zero_command_is_required_even_if_plant_states_are_quiet(scripted_plant):
    scripted_plant([{}])
    measured = measure_mujoco_stop(0.5, 1.0, 0.1, 0.05, 0.1, max_steps=11)
    assert measured.status == "not_settled"
    assert measured.quiet_samples == 1


@pytest.mark.parametrize("bad_frame", [
    {"x": float("nan")}, {"vz": float("inf")}, {"actuator_v": float("nan")},
    {"time_s": 0.0}, RuntimeError("MuJoCo numerical invalidity at substep"),
])
def test_invalid_simulation_never_becomes_a_stopping_distance(scripted_plant, bad_frame):
    scripted_plant([{"x": 0.1}, bad_frame])
    measured = measure_mujoco_stop(0.5, 0.0, 0.1, 0.05, 0.1)
    assert measured.status == "invalid"
    assert measured.reason
    assert measured.completed_steps == 1
    assert measured.attempted_steps == 2
    assert measured.maximum_sampled_forward_travel_m == 0.1
    with pytest.raises(RuntimeError, match="Unsupported stopping measurement"):
        mujoco_stopping_distance(0.5, 0.0, 0.1, 0.05, 0.1)


def test_scalar_api_fails_closed_if_settling_window_is_incomplete(scripted_plant):
    scripted_plant([{}])
    with pytest.raises(RuntimeError, match="not_settled"):
        mujoco_stopping_distance(0.5, 0.0, 0.1, 0.05, 0.1, max_steps=10)


def test_mujoco_prefix_retains_invalid_measurement_without_distance_claim(scripted_plant):
    scripted_plant([RuntimeError("invalid qacc")])
    case = evaluate_case(v0=0.25, command_gap=0.0, obstacle_distance=0.1, tau_v=0.06, delta_u=0.05, dt=0.05)
    rows = compare_with_mujoco([case], samples=1, friction=1.0)
    assert rows[0]["measurement_status"] == "invalid"
    assert rows[0]["mujoco_distance"] is None
    assert rows[0]["mujoco_within_discrete_1cm"] is None


def test_v2_measurement_protocol_is_fixed():
    assert STOPPING_MEASUREMENT_PROTOCOL.version == "sampled_peak_settled_v2"
    assert STOPPING_MEASUREMENT_PROTOCOL.velocity_tolerance == 1e-4
    assert STOPPING_MEASUREMENT_PROTOCOL.angular_velocity_tolerance == 1e-4
    assert STOPPING_MEASUREMENT_PROTOCOL.settling_window_s == 0.5
    assert STOPPING_MEASUREMENT_PROTOCOL.max_steps == 20_000
