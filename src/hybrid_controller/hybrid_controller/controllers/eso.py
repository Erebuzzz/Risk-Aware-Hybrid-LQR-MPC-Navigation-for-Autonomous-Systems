"""Three-state extended state observer for actuator mismatch and slip."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ESOConfig:
    """Observer configuration."""

    dt: float = 0.05
    tau_v: float = 0.10
    tau_omega: float = 0.10
    beta: float = 0.35
    max_abs_estimate: float = 20.0

    def __post_init__(self) -> None:
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")
        if self.tau_v <= 0.0 or self.tau_omega <= 0.0:
            raise ValueError("time constants must be positive")
        if not 0.0 < self.beta <= 1.0:
            raise ValueError("beta must be in (0, 1]")


class ExtendedStateObserver3D:
    """Estimate longitudinal residual, lateral slip, and yaw residual.

    Measurements are expected in the five-state format
    ``[p_x, p_y, theta, v, omega]``.  Position differences are transformed
    into the previous body frame before the lateral estimate is filtered.
    """

    def __init__(self, config: ESOConfig | None = None) -> None:
        self.config = config or ESOConfig()
        self.estimate = np.zeros(3, dtype=float)
        self._previous_state: np.ndarray | None = None
        self._previous_command: np.ndarray | None = None

    def reset(self, estimate: np.ndarray | None = None) -> None:
        self.estimate = np.zeros(3, dtype=float) if estimate is None else self._vector(estimate, 3)
        self._previous_state = None
        self._previous_command = None

    def update(self, measured_state: np.ndarray, command: np.ndarray) -> np.ndarray:
        """Update the observer and return ``[d_v, d_y, d_omega]``."""
        state = self._vector(measured_state, 5)
        u = self._vector(command, 2)
        if self._previous_state is None:
            self._previous_state = state.copy()
            self._previous_command = u.copy()
            return self.estimate.copy()

        previous = self._previous_state
        previous_command = self._previous_command
        if previous_command is None:
            previous_command = np.zeros(2, dtype=float)
        dt = self.config.dt
        yaw_rate_measured = self._angle_difference(state[2], previous[2]) / dt
        theta = float(previous[2] + 0.5 * yaw_rate_measured * dt)
        world_velocity = (state[:2] - previous[:2]) / dt
        c, s = np.cos(theta), np.sin(theta)
        body_velocity_x = c * world_velocity[0] + s * world_velocity[1]
        body_velocity_y = -s * world_velocity[0] + c * world_velocity[1]
        if abs(yaw_rate_measured) < 1e-8:
            displacement_scale = dt
        else:
            displacement_scale = 2.0 * np.sin(0.5 * yaw_rate_measured * dt) / yaw_rate_measured
        body_velocity_x /= displacement_scale / dt
        body_velocity_y /= displacement_scale / dt

        measured_acceleration = (state[3] - previous[3]) / dt
        expected_acceleration = (previous_command[0] - previous[3]) / self.config.tau_v
        d_v_raw = measured_acceleration - expected_acceleration

        d_omega_raw = yaw_rate_measured - state[4]

        raw = np.array([d_v_raw, body_velocity_y, d_omega_raw], dtype=float)
        raw = np.clip(raw, -self.config.max_abs_estimate, self.config.max_abs_estimate)
        self.estimate += self.config.beta * (raw - self.estimate)
        self.estimate = np.clip(
            self.estimate,
            -self.config.max_abs_estimate,
            self.config.max_abs_estimate,
        )
        self._previous_state = state.copy()
        self._previous_command = u.copy()
        return self.estimate.copy()

    def update_from_pose_and_velocity(
        self,
        pose: np.ndarray,
        velocity_world: np.ndarray,
        actual_velocity: np.ndarray,
        command: np.ndarray,
    ) -> np.ndarray:
        """Convenience adapter for pose and velocity sensor streams."""
        pose = self._vector(pose, 3)
        velocity_world = self._vector(velocity_world, 2)
        actual_velocity = self._vector(actual_velocity, 2)
        state = np.array(
            [pose[0], pose[1], pose[2], actual_velocity[0], actual_velocity[1]],
            dtype=float,
        )
        if self._previous_state is not None:
            previous = self._previous_state
            dt = self.config.dt
            c, s = np.cos(previous[2]), np.sin(previous[2])
            body_y = -s * velocity_world[0] + c * velocity_world[1]
            self._previous_state = np.array(
                [state[0] - velocity_world[0] * dt,
                 state[1] - velocity_world[1] * dt,
                 state[2], state[3], state[4]],
                dtype=float,
            )
            result = self.update(state, command)
            self._previous_state = state
            self.estimate[1] = (1.0 - self.config.beta) * self.estimate[1] + self.config.beta * body_y
            return self.estimate.copy()
        return self.update(state, command)

    @staticmethod
    def _angle_difference(current: float, previous: float) -> float:
        return float((current - previous + np.pi) % (2.0 * np.pi) - np.pi)

    @staticmethod
    def _vector(value: np.ndarray, size: int) -> np.ndarray:
        result = np.asarray(value, dtype=float).reshape(-1)
        if result.size != size or not np.all(np.isfinite(result)):
            raise ValueError(f"expected a finite vector of length {size}")
        return result.copy()
