"""Headless MuJoCo plant for friction, actuator-lag, and contact validation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Sequence

import mujoco
import numpy as np

from hybrid_controller.models.footprint import (
    MUJOCO_ROBOT_FOOTPRINT,
    MUJOCO_ROBOT_HALF_LENGTH,
    MUJOCO_ROBOT_HALF_WIDTH,
)


@dataclass(frozen=True)
class MuJoCoPlantConfig:
    dt: float = 0.05
    physics_dt: float = 0.002
    tau_v: float = 0.10
    tau_omega: float = 0.10
    force_gain: float = 9.0
    torque_gain: float = 4.0
    robot_mass: float = 2.0
    gravity_z: float = -9.81
    robot_radius: float = MUJOCO_ROBOT_FOOTPRINT.radius
    robot_half_length: float = MUJOCO_ROBOT_HALF_LENGTH
    robot_half_width: float = MUJOCO_ROBOT_HALF_WIDTH
    slip_gain: float = 0.0

    def __post_init__(self) -> None:
        values = (
            self.dt,
            self.physics_dt,
            self.tau_v,
            self.tau_omega,
            self.force_gain,
            self.torque_gain,
            self.robot_mass,
            self.gravity_z,
            self.robot_radius,
            self.robot_half_length,
            self.robot_half_width,
            self.slip_gain,
        )
        if not all(np.isfinite(value) for value in values):
            raise ValueError("MuJoCo plant parameters must be finite")
        if self.dt <= 0.0 or self.physics_dt <= 0.0:
            raise ValueError("plant time steps must be positive")
        if self.tau_v <= 0.0 or self.tau_omega <= 0.0:
            raise ValueError("actuator time constants must be positive")
        if self.force_gain <= 0.0 or self.torque_gain <= 0.0:
            raise ValueError("force and torque gains must be positive")
        if self.robot_mass <= 0.0:
            raise ValueError("robot_mass must be positive")
        if self.robot_radius <= 0.0 or self.robot_half_length <= 0.0 or self.robot_half_width <= 0.0:
            raise ValueError("robot dimensions must be positive")
        if self.slip_gain < 0.0:
            raise ValueError("slip_gain must be nonnegative")


@dataclass(frozen=True)
class MuJoCoContactEvent:
    """One robot-to-static-obstacle contact record from MuJoCo.

    MuJoCo can expose more than one contact point for the same pair of geoms.
    Each contact point is retained as one event so the telemetry never turns
    ``mjData.ncon`` into a guessed collision episode count.  ``physics_substep``
    is one-based within the control interval and is ``None`` for callers that
    construct an event outside a running plant.
    """

    obstacle_index: int
    geom1: str
    geom2: str
    body1: str
    body2: str
    distance: float
    physics_substep: int | None = None

    def __post_init__(self) -> None:
        if self.obstacle_index < 0:
            raise ValueError("obstacle_index must be nonnegative")
        if not self.geom1 or not self.geom2 or not self.body1 or not self.body2:
            raise ValueError("contact geom and body names must not be empty")
        if not np.isfinite(self.distance):
            raise ValueError("contact distance must be finite")
        if self.physics_substep is not None and self.physics_substep <= 0:
            raise ValueError("physics_substep must be positive when provided")

    def as_dict(self) -> dict[str, Any]:
        """Return JSON-safe telemetry without changing MuJoCo's names."""

        return {
            "obstacle_index": int(self.obstacle_index),
            "geom1": self.geom1,
            "geom2": self.geom2,
            "body1": self.body1,
            "body2": self.body2,
            "distance": float(self.distance),
            "physics_substep": (
                None if self.physics_substep is None else int(self.physics_substep)
            ),
        }


def build_scene_xml(
    obstacles: Sequence[dict[str, float]],
    friction: float,
    physics_dt: float = 0.002,
    robot_half_length: float = MUJOCO_ROBOT_HALF_LENGTH,
    robot_half_width: float = MUJOCO_ROBOT_HALF_WIDTH,
    gravity_z: float = -9.81,
    robot_mass: float = 2.0,
) -> str:
    obstacle_geoms = []
    for index, obstacle in enumerate(obstacles):
        obstacle_geoms.append(
            f'<geom name="obstacle_{index}" type="cylinder" '
            f'pos="{obstacle["x"]} {obstacle["y"]} 0.25" '
            f'size="{obstacle["radius"]} 0.25" '
            f'friction="{friction} 0.01 0.001" rgba="0.5 0.1 0.1 1"/>'
        )
    return f"""
<mujoco model="ra_psf_plant">
  <option timestep="{physics_dt}" gravity="0 0 {gravity_z}" integrator="implicitfast"/>
  <default>
    <geom contype="1" conaffinity="1" friction="{friction} 0.01 0.001"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" pos="0 0 0" size="20 20 0.1" friction="{friction} 0.01 0.001"/>
    {''.join(obstacle_geoms)}
    <body name="robot" pos="0 0 0.16">
      <freejoint/>
      <geom name="robot_body" type="box" size="{robot_half_length} {robot_half_width} 0.06" mass="{robot_mass}" friction="{friction} 0.01 0.001" rgba="0.1 0.3 0.8 1"/>
    </body>
  </worldbody>
</mujoco>
"""


class MuJoCoSimulationError(RuntimeError):
    """The simulation reset, warned, or produced invalid numerical evidence."""


class MuJoCoPlant:
    """A rigid-body base with force and yaw-torque velocity control."""

    def __init__(
        self,
        obstacles: Sequence[dict[str, float]],
        friction: float,
        config: MuJoCoPlantConfig | None = None,
    ) -> None:
        self.config = config or MuJoCoPlantConfig()
        xml = build_scene_xml(
            obstacles,
            friction,
            self.config.physics_dt,
            self.config.robot_half_length,
            self.config.robot_half_width,
            self.config.gravity_z,
            self.config.robot_mass,
        )
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.model.opt.timestep = self.config.physics_dt
        self.data = mujoco.MjData(self.model)
        self.robot_body_id = self.model.body("robot").id
        self.robot_geom_ids = {
            int(geom_id)
            for geom_id in range(self.model.ngeom)
            if int(self.model.geom_bodyid[geom_id]) == int(self.robot_body_id)
        }
        self.floor_geom_id = self.model.geom("floor").id
        self.obstacle_geom_ids = {
            int(self.model.geom(f"obstacle_{index}").id): index
            for index in range(len(obstacles))
        }
        self.obstacles = list(obstacles)
        self.friction = float(friction)
        self.v_actual = 0.0
        self.omega_actual = 0.0
        self._last_step_obstacle_contact_events: tuple[MuJoCoContactEvent, ...] = ()
        self._control_interval_obstacle_contact_events: tuple[MuJoCoContactEvent, ...] = ()
        self._last_step_floor_contact_count = 0
        self._control_interval_floor_contact_count = 0
        self.reset()

    def reset(self, state: np.ndarray | None = None) -> None:
        x = np.zeros(5, dtype=float) if state is None else np.asarray(state, dtype=float).reshape(5)
        if not np.all(np.isfinite(x)):
            raise ValueError("reset state must be finite")
        mujoco.mj_resetData(self.model, self.data)
        self._expected_time = 0.0
        self._simulation_error: str | None = None
        self._clear_contact_telemetry()
        self.data.qpos[0] = x[0]
        self.data.qpos[1] = x[1]
        self.data.qpos[2] = 0.16
        self.data.qpos[3:7] = self._yaw_to_quaternion(x[2])
        self.data.qvel[:] = 0.0
        self.data.qvel[0] = x[3] * np.cos(x[2])
        self.data.qvel[1] = x[3] * np.sin(x[2])
        self.data.qvel[5] = x[4]
        self.v_actual = float(x[3])
        self.omega_actual = float(x[4])
        mujoco.mj_forward(self.model, self.data)
        self._validate_physics(self._expected_time, "after reset")

    def state(self) -> np.ndarray:
        theta = self._quaternion_to_yaw(self.data.qpos[3:7])
        c, s = np.cos(theta), np.sin(theta)
        vx_world, vy_world = self.data.qvel[0], self.data.qvel[1]
        v_body = c * vx_world + s * vy_world
        return np.array([self.data.qpos[0], self.data.qpos[1], theta, v_body, self.data.qvel[5]])

    def actuator_state(self) -> np.ndarray:
        """Return the hidden first-order actuator states used by the plant.

        These states are exposed for deterministic simulator diagnostics only;
        they are not measured robot telemetry and must not be treated as a
        physical calibration result.
        """

        return np.array([self.v_actual, self.omega_actual], dtype=float)

    @property
    def contact_count(self) -> int:
        """Return total contacts, including the floor, not obstacle collisions.

        This is ``mjData.ncon`` from the last dynamics evaluation; it is not a
        count of collision events or a maximum over the control interval.
        """

        return int(self.data.ncon)

    @property
    def last_step_obstacle_contact_events(self) -> tuple[MuJoCoContactEvent, ...]:
        """Return robot-to-obstacle contact records from the last physics step."""

        return self._last_step_obstacle_contact_events

    @property
    def last_step_obstacle_contact_count(self) -> int:
        """Return obstacle contact records from the last physics step only."""

        return len(self._last_step_obstacle_contact_events)

    @property
    def obstacle_contact_events(self) -> tuple[MuJoCoContactEvent, ...]:
        """Short alias for :attr:`last_step_obstacle_contact_events`."""

        return self.last_step_obstacle_contact_events

    @property
    def obstacle_contact_count(self) -> int:
        """Short alias for the last-physics-step obstacle contact count.

        This deliberately differs from :attr:`contact_count`, which is generic
        ``mjData.ncon`` and therefore includes floor contacts.
        """

        return self.last_step_obstacle_contact_count

    @property
    def control_interval_obstacle_contact_events(self) -> tuple[MuJoCoContactEvent, ...]:
        """Return all obstacle contact records across the last control interval.

        Records are not deduplicated: persistent contact points appear once per
        physics substep, with ``physics_substep`` identifying their sample.
        """

        return self._control_interval_obstacle_contact_events

    @property
    def control_interval_obstacle_contact_count(self) -> int:
        """Return obstacle contact records sampled across the last interval."""

        return len(self._control_interval_obstacle_contact_events)

    @property
    def obstacle_contact_interval_events(self) -> tuple[MuJoCoContactEvent, ...]:
        """Alias for :attr:`control_interval_obstacle_contact_events`."""

        return self.control_interval_obstacle_contact_events

    @property
    def obstacle_contact_interval_count(self) -> int:
        """Alias for :attr:`control_interval_obstacle_contact_count`."""

        return self.control_interval_obstacle_contact_count

    @property
    def last_step_floor_contact_count(self) -> int:
        """Return robot-floor contact points from the last physics step."""

        return int(self._last_step_floor_contact_count)

    @property
    def control_interval_floor_contact_count(self) -> int:
        """Return robot-floor contact samples across the last interval."""

        return int(self._control_interval_floor_contact_count)

    def step(self, command: np.ndarray) -> np.ndarray:
        """Advance a control interval, rejecting invalid physics immediately.

        MuJoCo can automatically reset on numerical instability and return
        finite state. Check the clock and warnings after *every* physics step,
        before another step can erase that evidence. A failed plant must be
        explicitly reset before reuse; applied forces are always cleared.
        """

        try:
            if self._simulation_error is not None:
                raise MuJoCoSimulationError(
                    "simulation is invalid; call reset() before reuse: "
                    + self._simulation_error
                )
            command = np.asarray(command, dtype=float).reshape(2)
            if not np.all(np.isfinite(command)):
                raise ValueError("command must be finite")
            self._validate_physics(self._expected_time, "before control step")
            self._last_step_obstacle_contact_events = ()
            self._control_interval_obstacle_contact_events = ()
            self._last_step_floor_contact_count = 0
            self._control_interval_floor_contact_count = 0
            self.v_actual = self._lag(
                self.v_actual, command[0], self.config.tau_v, self.config.dt
            )
            self.omega_actual = self._lag(
                self.omega_actual, command[1], self.config.tau_omega, self.config.dt
            )
            substeps = max(1, int(round(self.config.dt / self.config.physics_dt)))
            for substep in range(substeps):
                theta = self._quaternion_to_yaw(self.data.qpos[3:7])
                lateral_velocity = (
                    self.config.slip_gain
                    * max(0.0, 1.0 - self.friction)
                    * abs(self.v_actual)
                )
                target_velocity = np.array(
                    [
                        self.v_actual * np.cos(theta) - lateral_velocity * np.sin(theta),
                        self.v_actual * np.sin(theta) + lateral_velocity * np.cos(theta),
                        0.0,
                    ]
                )
                force = self.config.force_gain * (target_velocity - self.data.qvel[:3])
                torque = self.config.torque_gain * (self.omega_actual - self.data.qvel[5])
                self.data.xfrc_applied[self.robot_body_id, :3] = force
                self.data.xfrc_applied[self.robot_body_id, 3:] = [0.0, 0.0, torque]
                self._validate_physics(self._expected_time, f"before physics substep {substep + 1}")
                expected_time = self._expected_time + self.config.physics_dt
                mujoco.mj_step(self.model, self.data)
                self._validate_physics(expected_time, f"after physics substep {substep + 1}")
                self._expected_time = expected_time
                obstacle_events = self._read_obstacle_contact_events(substep + 1)
                self._last_step_obstacle_contact_events = obstacle_events
                self._control_interval_obstacle_contact_events += obstacle_events
                floor_contacts = self._count_floor_contact_points()
                self._last_step_floor_contact_count = floor_contacts
                self._control_interval_floor_contact_count += floor_contacts
            return self.state()
        except (mujoco.FatalError, mujoco.UnexpectedError) as exc:
            self._simulation_error = f"native MuJoCo step failed: {exc}"
            self._clear_contact_telemetry()
            raise MuJoCoSimulationError(self._simulation_error) from exc
        except RuntimeError as exc:
            self._simulation_error = str(exc)
            self._clear_contact_telemetry()
            raise
        finally:
            self.data.xfrc_applied[self.robot_body_id, :] = 0.0

    def _read_obstacle_contact_events(
        self, physics_substep: int
    ) -> tuple[MuJoCoContactEvent, ...]:
        """Extract robot-to-obstacle contact records from the current ``mjData``."""

        events: list[MuJoCoContactEvent] = []
        for contact_index in range(int(self.data.ncon)):
            contact = self.data.contact[contact_index]
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)
            obstacle_index: int | None = None
            if geom1 in self.obstacle_geom_ids and geom2 in self.robot_geom_ids:
                obstacle_index = self.obstacle_geom_ids[geom1]
            elif geom2 in self.obstacle_geom_ids and geom1 in self.robot_geom_ids:
                obstacle_index = self.obstacle_geom_ids[geom2]
            if obstacle_index is None:
                continue
            distance = float(contact.dist)
            if not np.isfinite(distance):
                raise MuJoCoSimulationError(
                    "nonfinite obstacle contact distance at physics substep "
                    f"{physics_substep}: contact index {contact_index}"
                )
            body1 = int(self.model.geom_bodyid[geom1])
            body2 = int(self.model.geom_bodyid[geom2])
            events.append(
                MuJoCoContactEvent(
                    obstacle_index=obstacle_index,
                    geom1=str(self.model.geom(geom1).name),
                    geom2=str(self.model.geom(geom2).name),
                    body1=str(self.model.body(body1).name),
                    body2=str(self.model.body(body2).name),
                    distance=distance,
                    physics_substep=physics_substep,
                )
            )
        return tuple(events)

    def _count_floor_contact_points(self) -> int:
        """Count robot-floor contact points without using generic ``ncon``."""

        count = 0
        for contact_index in range(int(self.data.ncon)):
            contact = self.data.contact[contact_index]
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)
            if (
                (geom1 == self.floor_geom_id and geom2 in self.robot_geom_ids)
                or (geom2 == self.floor_geom_id and geom1 in self.robot_geom_ids)
            ):
                count += 1
        return count

    def _clear_contact_telemetry(self) -> None:
        """Discard partial contact records after a failed numerical step."""

        self._last_step_obstacle_contact_events = ()
        self._control_interval_obstacle_contact_events = ()
        self._last_step_floor_contact_count = 0
        self._control_interval_floor_contact_count = 0

    def _validate_physics(self, expected_time: float, phase: str) -> None:
        """Check numerical evidence without changing MuJoCo's integration."""

        errors: list[str] = []
        observed_time = float(self.data.time)
        # The independent expected clock also catches a reset that cleared all
        # warning counters. A small absolute tolerance allows floating-point
        # addition roundoff, but never an entire missing physics step.
        clock_tolerance = min(
            self.config.physics_dt * 1.0e-6,
            max(1.0e-12, 8.0 * math.ulp(expected_time)),
        )
        if not math.isfinite(observed_time) or not math.isclose(
            observed_time, expected_time, rel_tol=0.0, abs_tol=clock_tolerance
        ):
            errors.append(
                f"clock discontinuity (expected {expected_time:.17g}s, "
                f"observed {observed_time:.17g}s; possible automatic reset)"
            )
        # Reject all mjData warnings, including bad state/acceleration/control,
        # inertia failures, and contact/constraint capacity loss. Counting only
        # at the end of a control interval would miss resets between substeps.
        for index, count in enumerate(self.data.warning.number):
            if count:
                errors.append(
                    f"{mujoco.mjtWarning(index).name} count={int(count)} "
                    f"lastinfo={int(self.data.warning.lastinfo[index])}"
                )
        for name in (
            "qpos", "qvel", "act", "qacc", "qacc_warmstart", "ctrl",
            "qfrc_applied", "xfrc_applied", "plugin_state",
        ):
            if not np.all(np.isfinite(getattr(self.data, name))):
                errors.append(f"nonfinite {name}")
        for contact_index in range(int(self.data.ncon)):
            if not math.isfinite(float(self.data.contact[contact_index].dist)):
                errors.append(f"nonfinite contact distance at index {contact_index}")
        if not math.isfinite(self.v_actual) or not math.isfinite(self.omega_actual):
            errors.append("nonfinite actuator state")
        if errors:
            message = f"invalid MuJoCo simulation {phase}: " + "; ".join(errors)
            self._simulation_error = message
            raise MuJoCoSimulationError(message)

    @staticmethod
    def _lag(value: float, command: float, tau: float, dt: float = 0.05) -> float:
        decay = np.exp(-dt / tau)
        return float(decay * value + (1.0 - decay) * command)

    @staticmethod
    def _yaw_to_quaternion(theta: float) -> np.ndarray:
        return np.array([np.cos(theta / 2.0), 0.0, 0.0, np.sin(theta / 2.0)])

    @staticmethod
    def _quaternion_to_yaw(quaternion: np.ndarray) -> float:
        w, x, y, z = quaternion
        return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def make_static_obstacle_regression_scene() -> tuple[list[dict[str, float]], float]:
    """Return the fixed scene used by the obstacle-contact regression harness.

    The scene intentionally keeps the existing plant geometry and dynamics
    unchanged.  At friction ``0.25`` a straight command from the origin reaches
    the obstacle within twenty 50 ms control intervals, while the same command
    from ``y=0.50`` is a spatially separated control case.
    """

    return ([{"x": 0.45, "y": 0.0, "radius": 0.10}], 0.25)


def run_static_obstacle_regression(*, steps: int = 20) -> dict[str, Any]:
    """Run deterministic straight-hit and spatially-clear contact cases.

    The returned event records preserve MuJoCo's exact geom/body names and the
    obstacle index.  Counts are contact-point samples, not deduplicated
    collision episodes.  This helper is a regression harness only; it does not
    tune a controller or modify the physics model.
    """

    if isinstance(steps, bool) or not isinstance(steps, int) or steps <= 0:
        raise ValueError("steps must be a positive integer")
    obstacles, friction = make_static_obstacle_regression_scene()
    command = np.array([1.0, 0.0], dtype=float)

    def rollout(initial_state: Sequence[float]) -> dict[str, Any]:
        plant = MuJoCoPlant(obstacles, friction=friction)
        plant.reset(np.asarray(initial_state, dtype=float))
        states: list[np.ndarray] = []
        for _ in range(steps):
            states.append(plant.step(command).copy())
        return {
            "states": np.asarray(states, dtype=float).tolist(),
            "collision": bool(plant.control_interval_obstacle_contact_count > 0),
            "last_step_obstacle_contact_count": int(
                plant.last_step_obstacle_contact_count
            ),
            "obstacle_contact_count": int(
                plant.control_interval_obstacle_contact_count
            ),
            "obstacle_contact_events": [
                event.as_dict() for event in plant.control_interval_obstacle_contact_events
            ],
            "contact_count": int(plant.contact_count),
        }

    return {
        "obstacles": [dict(obstacle) for obstacle in obstacles],
        "friction": float(friction),
        "steps": int(steps),
        "command": command.tolist(),
        "straight_hit": rollout((0.0, 0.0, 0.0, 0.0, 0.0)),
        "clear_control": rollout((0.0, 0.50, 0.0, 0.0, 0.0)),
    }
