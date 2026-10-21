"""
Backup safety controller for the adaptive MPC safety-filter architecture.

The backup controller is deliberately simple and deterministic. It is used
only when the predictive safety filter cannot produce a usable command within
the real-time budget.
"""

from dataclasses import dataclass
from typing import Any, Sequence, Tuple

import numpy as np

from ..models.augmented_dynamics import (
    AugmentedDifferentialDriveModel,
    DynamicModelParams,
)


@dataclass
class BackupCommand:
    """Backup controller output and diagnostic metadata."""

    control: np.ndarray
    reason: str
    nearest_obstacle_id: int
    nearest_distance: float
    heading_error: float


class BackupSafetyController:
    """
    Brake and rotate away from the nearest obstacle.

    This controller is not intended to be high performance. Its job is to avoid
    continuing forward with a stale or unsafe command when MPC is infeasible or
    too slow.
    """

    def __init__(
        self,
        v_max: float = 0.22,
        omega_max: float = 2.84,
        k_omega: float = 2.0,
        min_turn_rate: float = 0.35,
    ):
        self.v_max = float(v_max)
        self.omega_max = float(omega_max)
        self.k_omega = float(k_omega)
        self.min_turn_rate = float(min_turn_rate)

    def compute(
        self,
        state: np.ndarray,
        obstacles: Sequence[Any] = (),
        reason: str = "mpc_unavailable",
    ) -> BackupCommand:
        """Return a stop-and-turn command away from the nearest obstacle."""
        state = np.asarray(state, dtype=float).reshape(3)

        if not obstacles:
            return BackupCommand(
                control=np.array([0.0, 0.0], dtype=float),
                reason=reason,
                nearest_obstacle_id=-1,
                nearest_distance=float("inf"),
                heading_error=0.0,
            )

        obs_idx, obs_x, obs_y, dist = self._nearest_obstacle(state, obstacles)
        away_heading = float(np.arctan2(state[1] - obs_y, state[0] - obs_x))
        heading_error = self.normalize_angle(away_heading - state[2])

        omega = self.k_omega * heading_error
        if abs(omega) < self.min_turn_rate:
            omega = self.min_turn_rate * (1.0 if heading_error >= 0.0 else -1.0)
        omega = float(np.clip(omega, -self.omega_max, self.omega_max))

        return BackupCommand(
            control=np.array([0.0, omega], dtype=float),
            reason=reason,
            nearest_obstacle_id=obs_idx,
            nearest_distance=dist,
            heading_error=heading_error,
        )

    def compute_augmented(
        self,
        state: np.ndarray,
        obstacles: Sequence[Any] = (),
        *,
        dt: float = 0.05,
        tau_v: float = 0.10,
        tau_omega: float = 0.10,
        horizon: int = 10,
        robot_radius: float = 0.20,
    ) -> BackupCommand:
        """Choose an emergency command by short-horizon clearance rollout.

        The legacy 3D brake-and-turn command is retained for the original
        controller.  The actuator-aware path evaluates braking, reversing, and
        turning candidates with the same five-state model used by RA-PSF.  This
        avoids selecting a turn direction from only the current obstacle
        bearing when dynamic obstacles or actuator lag dominate.
        """
        state = np.asarray(state, dtype=float).reshape(5)
        if not np.all(np.isfinite(state)):
            raise ValueError("state must contain finite values")
        if horizon < 1:
            raise ValueError("horizon must be positive")
        if not obstacles:
            return BackupCommand(
                control=np.array([0.0, 0.0], dtype=float),
                reason="no_obstacles",
                nearest_obstacle_id=-1,
                nearest_distance=float("inf"),
                heading_error=0.0,
            )

        obs_idx, obs_x, obs_y, nearest_distance = self._nearest_obstacle(
            state[:3], obstacles
        )
        away_heading = float(np.arctan2(state[1] - obs_y, state[0] - obs_x))
        heading_error = self.normalize_angle(away_heading - state[2])
        turn_sign = 1.0 if heading_error >= 0.0 else -1.0
        turn_rates = (
            0.0,
            turn_sign * 0.5 * self.omega_max,
            turn_sign * self.omega_max,
            -turn_sign * self.omega_max,
        )
        reverse_speed = max(-self.v_max, -0.5 * self.v_max)
        speeds = (0.0, reverse_speed, 0.25 * self.v_max)
        candidates = [
            np.array([speed, turn_rate], dtype=float)
            for speed in speeds
            for turn_rate in turn_rates
        ]

        model = AugmentedDifferentialDriveModel(
            DynamicModelParams(dt=dt, tau_v=tau_v, tau_omega=tau_omega)
        )
        best_control = candidates[0]
        best_score = (-float("inf"), -float("inf"))
        for candidate in candidates:
            rollout_state = state.copy()
            minimum_clearance = float("inf")
            for step in range(1, horizon + 1):
                rollout_state = model.step(rollout_state, candidate)
                for obstacle in obstacles:
                    ox = self._get_obstacle_value(obstacle, "x", 0.0)
                    oy = self._get_obstacle_value(obstacle, "y", 0.0)
                    vx = self._get_obstacle_value(obstacle, "vx", 0.0)
                    vy = self._get_obstacle_value(obstacle, "vy", 0.0)
                    radius = self._get_obstacle_value(obstacle, "radius", 0.0)
                    distance = np.hypot(
                        rollout_state[0] - (ox + vx * step * dt),
                        rollout_state[1] - (oy + vy * step * dt),
                    )
                    minimum_clearance = min(
                        minimum_clearance,
                        float(distance - radius - robot_radius),
                    )
            turn_alignment = -abs(candidate[1] - turn_sign * self.omega_max)
            score = (minimum_clearance, turn_alignment)
            if score > best_score:
                best_score = score
                best_control = candidate

        return BackupCommand(
            control=np.array(
                [
                    np.clip(best_control[0], -self.v_max, self.v_max),
                    np.clip(best_control[1], -self.omega_max, self.omega_max),
                ],
                dtype=float,
            ),
            reason="augmented_clearance_backup",
            nearest_obstacle_id=obs_idx,
            nearest_distance=nearest_distance,
            heading_error=heading_error,
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    def _nearest_obstacle(
        self,
        state: np.ndarray,
        obstacles: Sequence[Any],
    ) -> Tuple[int, float, float, float]:
        best_idx = -1
        best_x = 0.0
        best_y = 0.0
        best_dist = float("inf")

        for idx, obstacle in enumerate(obstacles):
            ox = self._get_obstacle_value(obstacle, "x", 0.0)
            oy = self._get_obstacle_value(obstacle, "y", 0.0)
            radius = self._get_obstacle_value(obstacle, "radius", 0.0)
            dist = float(np.hypot(state[0] - ox, state[1] - oy) - radius)
            if dist < best_dist:
                best_idx = idx
                best_x = ox
                best_y = oy
                best_dist = dist

        return best_idx, best_x, best_y, best_dist

    @staticmethod
    def _get_obstacle_value(obstacle: Any, key: str, default: float) -> float:
        if isinstance(obstacle, dict):
            return float(obstacle.get(key, default))
        return float(getattr(obstacle, key, default))
