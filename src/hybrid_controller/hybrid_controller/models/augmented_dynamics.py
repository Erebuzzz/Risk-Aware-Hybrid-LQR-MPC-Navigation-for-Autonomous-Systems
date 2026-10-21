"""Actuator-aware five-state differential-drive dynamics.

The model separates commanded velocities from the velocities realized by the
drivetrain.  The pose update is exact for constant body velocities over one
sample, while the actuator states use the exact solution of their first-order
lag equations.

State order:
    ``[p_x, p_y, theta, v, omega]``

Input order:
    ``[v_cmd, omega_cmd]``

Disturbance order:
    ``[d_v, d_y, d_omega]``

``d_v`` is a longitudinal acceleration residual, ``d_y`` is lateral body
velocity, and ``d_omega`` is a yaw-rate residual in the pose equation.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class DynamicModelParams:
    """Parameters for the discrete actuator-aware model."""

    dt: float = 0.05
    tau_v: float = 0.10
    tau_omega: float = 0.10

    def __post_init__(self) -> None:
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")
        if self.tau_v <= 0.0 or self.tau_omega <= 0.0:
            raise ValueError("actuator time constants must be positive")


class AugmentedDifferentialDriveModel:
    """Five-state actuator-aware differential-drive model."""

    STATE_DIM = 5
    CONTROL_DIM = 2
    DISTURBANCE_DIM = 3

    def __init__(self, params: DynamicModelParams | None = None) -> None:
        self.params = params or DynamicModelParams()

    @property
    def dt(self) -> float:
        return self.params.dt

    def step(
        self,
        state: np.ndarray,
        control: np.ndarray,
        disturbance: np.ndarray | None = None,
    ) -> np.ndarray:
        """Propagate one sample using exact constant-rate pose integration."""
        x = self._state(state)
        u = self._control(control)
        d = self._disturbance(disturbance)

        px, py, theta, v, omega = x
        d_v, d_y, d_omega = d
        dt = self.dt
        yaw_rate = omega + d_omega

        delta_x, delta_y = self._pose_increment(
            theta, v, d_y, yaw_rate, dt
        )
        v_next = self._lag_step(v, u[0], self.params.tau_v, d_v)
        omega_next = self._lag_step(omega, u[1], self.params.tau_omega, 0.0)

        return np.array(
            [px + delta_x, py + delta_y, theta + yaw_rate * dt, v_next, omega_next],
            dtype=float,
        )

    def continuous_dynamics(
        self,
        state: np.ndarray,
        control: np.ndarray,
        disturbance: np.ndarray | None = None,
    ) -> np.ndarray:
        """Return the continuous dynamics used by the discrete map."""
        x = self._state(state)
        u = self._control(control)
        d = self._disturbance(disturbance)
        _, _, theta, v, omega = x
        d_v, d_y, d_omega = d
        return np.array(
            [
                v * np.cos(theta) - d_y * np.sin(theta),
                v * np.sin(theta) + d_y * np.cos(theta),
                omega + d_omega,
                (u[0] - v) / self.params.tau_v + d_v,
                (u[1] - omega) / self.params.tau_omega,
            ],
            dtype=float,
        )

    def jacobians(
        self,
        state: np.ndarray,
        control: np.ndarray,
        disturbance: np.ndarray | None = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Return analytical discrete Jacobians ``A = df/dx`` and ``B = df/du``.

        The Jacobians correspond to the exact discrete map implemented by
        :meth:`step`.  Disturbances are treated as fixed parameters during the
        local linearization.
        """
        x = self._state(state)
        u = self._control(control)
        d = self._disturbance(disturbance)
        _, _, theta, v, omega = x
        _, d_y, d_omega = d
        dt = self.dt
        yaw_rate = omega + d_omega

        dx, dy, dtheta, dv, domega = self._pose_partials(
            theta, v, d_y, yaw_rate, dt
        )

        a_v, b_v = self._lag_coefficients(self.params.tau_v, self.dt)
        a_w, b_w = self._lag_coefficients(self.params.tau_omega, self.dt)

        A = np.zeros((self.STATE_DIM, self.STATE_DIM), dtype=float)
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[0, 2], A[0, 3], A[0, 4] = dx
        A[1, 2], A[1, 3], A[1, 4] = dy
        A[2, 2] = 1.0
        A[2, 4] = dt
        A[3, 3] = a_v
        A[4, 4] = a_w

        B = np.zeros((self.STATE_DIM, self.CONTROL_DIM), dtype=float)
        B[3, 0] = b_v
        B[4, 1] = b_w
        return A, B

    def linearize_error_dynamics(
        self,
        state: np.ndarray,
        control: np.ndarray,
        disturbance: np.ndarray | None = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Alias used by the LTV-MPC implementation."""
        return self.jacobians(state, control, disturbance)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Wrap an angle into ``[-pi, pi)``."""
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    @staticmethod
    def _lag_coefficients(tau: float, dt: float = 0.05) -> Tuple[float, float]:
        decay = float(np.exp(-dt / tau))
        return decay, 1.0 - decay

    def _lag_step(self, value: float, command: float, tau: float, residual: float) -> float:
        decay, input_gain = self._lag_coefficients(tau, self.dt)
        return float(decay * value + input_gain * command + tau * input_gain * residual)

    @staticmethod
    def _pose_increment(
        theta: float,
        v: float,
        lateral_velocity: float,
        yaw_rate: float,
        dt: float,
    ) -> Tuple[float, float]:
        theta_next = theta + yaw_rate * dt
        if abs(yaw_rate) < 1e-8:
            return (
                dt * (v * np.cos(theta) - lateral_velocity * np.sin(theta)),
                dt * (v * np.sin(theta) + lateral_velocity * np.cos(theta)),
            )

        sin_delta = np.sin(theta_next) - np.sin(theta)
        cos_delta = np.cos(theta_next) - np.cos(theta)
        dx = (v * sin_delta + lateral_velocity * cos_delta) / yaw_rate
        dy = (v * (np.cos(theta) - np.cos(theta_next)) + lateral_velocity * sin_delta) / yaw_rate
        return float(dx), float(dy)

    @staticmethod
    def _pose_partials(
        theta: float,
        v: float,
        lateral_velocity: float,
        yaw_rate: float,
        dt: float,
    ) -> Tuple[np.ndarray, np.ndarray, float, float, float]:
        theta_next = theta + yaw_rate * dt
        if abs(yaw_rate) < 1e-8:
            sin_theta = np.sin(theta)
            cos_theta = np.cos(theta)
            dx_dtheta = dt * (-v * sin_theta - lateral_velocity * cos_theta)
            dy_dtheta = dt * (v * cos_theta - lateral_velocity * sin_theta)
            dx_dv = dt * cos_theta
            dy_dv = dt * sin_theta
            dx_domega = 0.5 * dt * dt * (-v * sin_theta - lateral_velocity * cos_theta)
            dy_domega = 0.5 * dt * dt * (v * cos_theta - lateral_velocity * sin_theta)
        else:
            sin_theta = np.sin(theta)
            cos_theta = np.cos(theta)
            sin_next = np.sin(theta_next)
            cos_next = np.cos(theta_next)
            inv_rate = 1.0 / yaw_rate
            n_x = v * (sin_next - sin_theta) + lateral_velocity * (cos_next - cos_theta)
            n_y = v * (cos_theta - cos_next) + lateral_velocity * (sin_next - sin_theta)
            d_nx_drate = dt * (v * cos_next - lateral_velocity * sin_next)
            d_ny_drate = dt * (v * sin_next + lateral_velocity * cos_next)
            dx_dtheta = inv_rate * (v * (cos_next - cos_theta) + lateral_velocity * (-sin_next + sin_theta))
            dy_dtheta = inv_rate * (v * (sin_next - sin_theta) + lateral_velocity * (cos_next - cos_theta))
            dx_dv = (sin_next - sin_theta) * inv_rate
            dy_dv = (cos_theta - cos_next) * inv_rate
            dx_domega = (d_nx_drate * yaw_rate - n_x) * inv_rate * inv_rate
            dy_domega = (d_ny_drate * yaw_rate - n_y) * inv_rate * inv_rate

        return (
            np.array([dx_dtheta, dx_dv, dx_domega], dtype=float),
            np.array([dy_dtheta, dy_dv, dy_domega], dtype=float),
            1.0,
            dt,
            1.0,
        )

    @staticmethod
    def _state(value: np.ndarray) -> np.ndarray:
        result = np.asarray(value, dtype=float).reshape(-1)
        if result.size != 5 or not np.all(np.isfinite(result)):
            raise ValueError("state must be a finite vector of length 5")
        return result.copy()

    @staticmethod
    def _control(value: np.ndarray) -> np.ndarray:
        result = np.asarray(value, dtype=float).reshape(-1)
        if result.size != 2 or not np.all(np.isfinite(result)):
            raise ValueError("control must be a finite vector of length 2")
        return result.copy()

    @staticmethod
    def _disturbance(value: np.ndarray | None) -> np.ndarray:
        if value is None:
            return np.zeros(3, dtype=float)
        result = np.asarray(value, dtype=float).reshape(-1)
        if result.size != 3 or not np.all(np.isfinite(result)):
            raise ValueError("disturbance must be a finite vector of length 3")
        return result.copy()
