"""Reject reset/unstable MuJoCo trajectories instead of accepting finite resets."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from benchmarks.mujoco_scene import (
    MuJoCoPlant,
    MuJoCoPlantConfig,
    MuJoCoSimulationError,
    make_static_obstacle_regression_scene,
    run_static_obstacle_regression,
)
from benchmarks.run_mujoco_monte_carlo import obstacle_contact_telemetry


@pytest.mark.parametrize("advance_first", [False, True])
def test_real_qacc_instability_raises_even_when_auto_reset_state_is_finite(
    advance_first, monkeypatch, tmp_path
):
    # Keep MuJoCo's genuine numerical-warning log with this test's artifacts.
    monkeypatch.chdir(tmp_path)
    plant = MuJoCoPlant([], friction=0.65, config=MuJoCoPlantConfig(gravity_z=0.0))
    if advance_first:
        plant.step(np.zeros(2))

    # A finite but extreme command produces real bad-QACC detection and an
    # automatic reset inside mj_step. This is not a mocked exception.
    with pytest.raises(MuJoCoSimulationError, match="after physics substep 1.*mjWARN_BADQACC"):
        plant.step(np.array([1.0e12, 0.0]))

    assert plant.data.time == pytest.approx(plant.config.physics_dt)
    assert np.all(np.isfinite(plant.data.qpos))
    assert np.all(np.isfinite(plant.data.qvel))
    assert plant.data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number > 0
    assert np.all(plant.data.xfrc_applied == 0.0)

    with pytest.raises(MuJoCoSimulationError, match=r"call reset\(\) before reuse"):
        plant.step(np.zeros(2))
    plant.reset()
    assert np.all(np.isfinite(plant.step(np.array([0.1, 0.0]))))
    assert plant.data.time == pytest.approx(plant.config.dt)


def test_reset_inside_control_interval_is_detected_with_warning_counters_cleared(monkeypatch):
    plant = MuJoCoPlant([], friction=0.65, config=MuJoCoPlantConfig(gravity_z=0.0))
    native_step = mujoco.mj_step
    calls = 0

    def reset_on_third_substep(model, data):
        nonlocal calls
        calls += 1
        if calls == 3:
            # Exercise an actual native reset, which erases warning history.
            mujoco.mj_resetData(model, data)
        native_step(model, data)

    monkeypatch.setattr(mujoco, "mj_step", reset_on_third_substep)
    with pytest.raises(MuJoCoSimulationError, match="after physics substep 3: clock discontinuity"):
        plant.step(np.array([0.4, -0.2]))
    assert calls == 3
    assert np.all(plant.data.warning.number == 0)
    assert np.all(np.isfinite(plant.state()))
    assert np.all(plant.data.xfrc_applied == 0.0)


@pytest.mark.parametrize("field", ["qpos", "qvel", "qacc", "qacc_warmstart"])
def test_nonfinite_native_state_is_rejected_before_it_can_be_erased(field):
    plant = MuJoCoPlant([], friction=1.0)
    getattr(plant.data, field)[0] = np.nan
    plant.data.xfrc_applied[plant.robot_body_id, 0] = 1.0
    with pytest.raises(MuJoCoSimulationError, match=f"nonfinite {field}"):
        plant.step(np.zeros(2))
    assert plant.data.time == 0.0
    assert np.all(plant.data.xfrc_applied == 0.0)


def test_nonfinite_post_substep_state_is_rejected_immediately(monkeypatch):
    plant = MuJoCoPlant([], friction=0.65, config=MuJoCoPlantConfig(gravity_z=0.0))
    native_step = mujoco.mj_step
    calls = 0

    def corrupt_first_result(model, data):
        nonlocal calls
        calls += 1
        native_step(model, data)
        data.qvel[1] = np.nan

    monkeypatch.setattr(mujoco, "mj_step", corrupt_first_result)
    with pytest.raises(MuJoCoSimulationError, match="after physics substep 1.*nonfinite qvel"):
        plant.step(np.array([0.4, 0.0]))
    assert calls == 1
    assert np.all(plant.data.xfrc_applied == 0.0)


@pytest.mark.parametrize("error_type", [RuntimeError, mujoco.FatalError, mujoco.UnexpectedError])
def test_applied_force_is_cleared_if_native_step_raises(monkeypatch, error_type):
    plant = MuJoCoPlant([], friction=1.0)

    def native_failure(model, data):
        assert np.linalg.norm(data.xfrc_applied[plant.robot_body_id]) > 0.0
        raise error_type("native step failed")

    monkeypatch.setattr(mujoco, "mj_step", native_failure)
    with pytest.raises(RuntimeError, match="native step failed"):
        plant.step(np.array([0.4, 0.1]))
    assert np.all(plant.data.xfrc_applied == 0.0)


def test_total_contact_count_includes_floor_without_obstacles():
    plant = MuJoCoPlant([], friction=1.0)
    for _ in range(10):
        plant.step(np.zeros(2))
    assert plant.contact_count > 0
    assert plant.last_step_floor_contact_count > 0
    assert plant.control_interval_floor_contact_count > 0
    assert plant.obstacle_contact_count == 0
    assert plant.control_interval_obstacle_contact_count == 0
    assert plant.obstacle_contact_events == ()
    assert plant.obstacles == []
    assert plant.data.time == pytest.approx(10 * plant.config.dt)
    assert np.all(plant.data.warning.number == 0)


def test_static_obstacle_contact_records_exact_names_and_index():
    obstacles, friction = make_static_obstacle_regression_scene()
    plant = MuJoCoPlant(obstacles, friction=friction)
    for _ in range(20):
        plant.step(np.array([1.0, 0.0]))

    assert plant.contact_count > plant.last_step_obstacle_contact_count
    assert plant.last_step_obstacle_contact_count > 0
    assert plant.control_interval_obstacle_contact_count >= plant.last_step_obstacle_contact_count
    assert plant.last_step_obstacle_contact_events
    assert plant.control_interval_obstacle_contact_events
    event = plant.last_step_obstacle_contact_events[0]
    assert event.obstacle_index == 0
    assert event.geom1 == "obstacle_0"
    assert event.geom2 == "robot_body"
    assert event.body1 == "world"
    assert event.body2 == "robot"
    assert event.physics_substep == 25
    assert np.isfinite(event.distance)


def test_static_obstacle_regression_has_hit_and_clear_control_cases():
    result = run_static_obstacle_regression()
    hit = result["straight_hit"]
    clear = result["clear_control"]

    assert hit["collision"] is True
    assert hit["obstacle_contact_count"] > 0
    assert hit["obstacle_contact_events"]
    assert all(event["obstacle_index"] == 0 for event in hit["obstacle_contact_events"])
    assert clear["collision"] is False
    assert clear["obstacle_contact_count"] == 0
    assert clear["obstacle_contact_events"] == []


def test_runner_contact_telemetry_uses_obstacle_specific_samples():
    obstacles, friction = make_static_obstacle_regression_scene()
    plant = MuJoCoPlant(obstacles, friction=friction)
    for _ in range(20):
        plant.step(np.array([1.0, 0.0]))

    telemetry = obstacle_contact_telemetry(plant)
    assert telemetry["contact_count"] > 0
    assert telemetry["last_step_obstacle_contact_count"] > 0
    assert telemetry["obstacle_contact_count"] > 0
    assert telemetry["obstacle_contact_count"] == telemetry["obstacle_contact_interval_count"]
    assert telemetry["obstacle_contact_events"]
    assert telemetry["obstacle_contact_events"][0]["geom1"] == "obstacle_0"
    assert telemetry["obstacle_contact_events"][0]["body2"] == "robot"
