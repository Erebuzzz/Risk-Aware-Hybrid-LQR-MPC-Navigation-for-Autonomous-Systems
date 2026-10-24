"""Reject reset/unstable MuJoCo trajectories instead of accepting finite resets."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from benchmarks.mujoco_scene import MuJoCoPlant, MuJoCoPlantConfig, MuJoCoSimulationError


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
    assert plant.obstacles == []
    assert plant.data.time == pytest.approx(10 * plant.config.dt)
    assert np.all(plant.data.warning.number == 0)
