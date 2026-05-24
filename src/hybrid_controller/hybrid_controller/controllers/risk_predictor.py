"""
Risk prediction for LQR rollout safety filtering.

This module checks whether the nominal LQR command is safe before it is
applied. It intentionally stays sensor-agnostic: obstacles may come from a
Gazebo obstacle publisher, LiDAR clustering, a costmap, or another perception
module, as long as each obstacle exposes x, y, and radius.
"""

from dataclasses import dataclass
from typing import Any, List, Sequence, Tuple

import numpy as np


@dataclass
class RolloutRisk:
    """Safety result for a predicted nominal-controller rollout."""

    safe: bool
    risk_score: float
    min_margin: float
    nearest_obstacle_id: int
    first_violation_step: int
    predicted_states: np.ndarray


class LQRRiskPredictor:
    """
    Roll out a candidate control sequence and test obstacle clearance.

    The clearance margin is

        margin = distance(position, obstacle_center)
                 - obstacle_radius - d_safe - robot_radius

    A rollout is safe only when the margin is nonnegative for every predicted
    state and every obstacle.
    """

    def __init__(
        self,
        horizon: int = 10,
        dt: float = 0.05,
        d_safe: float = 0.3,
        robot_radius: float = 0.105,
        d_trigger: float = 0.75,
    ):
        self.horizon = int(horizon)
        self.dt = float(dt)
        self.d_safe = float(d_safe)
        self.robot_radius = float(robot_radius)
        self.d_trigger = float(max(d_trigger, d_safe + robot_radius + 1e-6))

    def assess(
        self,
        x0: np.ndarray,
        controls: np.ndarray,
        theta_hat: np.ndarray,
        obstacles: Sequence[Any] = (),
    ) -> RolloutRisk:
        """Return safety/risk information for a candidate rollout."""
        predicted_states = self.rollout(x0, controls, theta_hat)

        if not obstacles:
            return RolloutRisk(
                safe=True,
                risk_score=0.0,
                min_margin=float("inf"),
                nearest_obstacle_id=-1,
                first_violation_step=-1,
                predicted_states=predicted_states,
            )

        min_margin = float("inf")
        nearest_id = -1
        first_violation_step = -1

        for step, state in enumerate(predicted_states):
            px, py = float(state[0]), float(state[1])
            for obs_idx, obs in enumerate(obstacles):
                ox, oy, radius = self._obstacle_at_step(obs, step)
                dist = float(np.hypot(px - ox, py - oy))
                margin = dist - radius - self.d_safe - self.robot_radius
                if margin < min_margin:
                    min_margin = margin
                    nearest_id = obs_idx
                if margin < 0.0 and first_violation_step < 0:
                    first_violation_step = step

        safe = first_violation_step < 0
        risk_score = self._margin_to_risk(min_margin)

        return RolloutRisk(
            safe=safe,
            risk_score=risk_score,
            min_margin=min_margin,
            nearest_obstacle_id=nearest_id,
            first_violation_step=first_violation_step,
            predicted_states=predicted_states,
        )

    def rollout(
        self,
        x0: np.ndarray,
        controls: np.ndarray,
        theta_hat: np.ndarray,
    ) -> np.ndarray:
        """Predict future states under the adapted differential-drive model."""
        controls = self._normalize_controls(controls)
        theta_hat = np.asarray(theta_hat, dtype=float).reshape(-1)
        if theta_hat.size < 2:
            theta_hat = np.array([1.0, 1.0], dtype=float)

        x = np.asarray(x0, dtype=float).reshape(3).copy()
        states = np.zeros((self.horizon + 1, 3), dtype=float)
        states[0] = x

        for k in range(self.horizon):
            u = controls[min(k, len(controls) - 1)]
            x = self.propagate_state(x, u, theta_hat, self.dt)
            states[k + 1] = x

        return states

    @staticmethod
    def propagate_state(
        x: np.ndarray,
        u: np.ndarray,
        theta_hat: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """One Euler step of the adapted differential-drive model."""
        state = np.asarray(x, dtype=float).reshape(3).copy()
        control = np.asarray(u, dtype=float).reshape(2)
        theta = float(state[2])
        theta_hat = np.asarray(theta_hat, dtype=float).reshape(-1)
        if theta_hat.size < 2:
            theta_hat = np.array([1.0, 1.0], dtype=float)

        v = float(theta_hat[0] * control[0])
        omega = float(theta_hat[1] * control[1])

        state[0] += dt * v * np.cos(theta)
        state[1] += dt * v * np.sin(theta)
        state[2] = LQRRiskPredictor.normalize_angle(state[2] + dt * omega)
        return state

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Wrap an angle to [-pi, pi]."""
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    def _normalize_controls(self, controls: np.ndarray) -> np.ndarray:
        arr = np.asarray(controls, dtype=float)
        if arr.ndim == 1:
            arr = np.tile(arr.reshape(1, 2), (self.horizon, 1))
        if len(arr) == 0:
            arr = np.zeros((self.horizon, 2), dtype=float)
        return arr

    def _margin_to_risk(self, margin: float) -> float:
        if not np.isfinite(margin):
            return 0.0
        if margin <= 0.0:
            return 1.0
        trigger_margin = self.d_trigger - self.d_safe - self.robot_radius
        if margin >= trigger_margin:
            return 0.0
        return float(np.clip(1.0 - margin / trigger_margin, 0.0, 1.0))

    def _obstacle_at_step(self, obstacle: Any, step: int) -> Tuple[float, float, float]:
        x = self._get_obstacle_value(obstacle, "x", 0.0)
        y = self._get_obstacle_value(obstacle, "y", 0.0)
        radius = self._get_obstacle_value(obstacle, "radius", 0.0)

        # Optional constant-velocity obstacle support for future experiments.
        vx = self._get_obstacle_value(obstacle, "vx", 0.0)
        vy = self._get_obstacle_value(obstacle, "vy", 0.0)
        x += vx * self.dt * step
        y += vy * self.dt * step

        return float(x), float(y), float(radius)

    @staticmethod
    def _get_obstacle_value(obstacle: Any, key: str, default: float) -> float:
        if isinstance(obstacle, dict):
            return float(obstacle.get(key, default))
        return float(getattr(obstacle, key, default))
