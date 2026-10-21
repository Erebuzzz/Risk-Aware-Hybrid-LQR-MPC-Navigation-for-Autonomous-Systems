"""Analytical Gaussian CVaR rollout risk evaluation."""

from __future__ import annotations

from dataclasses import dataclass
from math import exp, pi, sqrt
from typing import Any, Sequence

import numpy as np
from scipy.special import ndtri


@dataclass(frozen=True)
class ObstacleKinematics:
    """Circular obstacle with optional constant velocity."""

    x: float
    y: float
    radius: float
    vx: float = 0.0
    vy: float = 0.0

    def position_at(self, time_s: float) -> np.ndarray:
        return np.array(
            [self.x + self.vx * time_s, self.y + self.vy * time_s],
            dtype=float,
        )


@dataclass(frozen=True)
class CVaRSample:
    """Risk value for one rollout state and obstacle."""

    step: int
    obstacle_index: int
    clearance: float
    normal_sigma: float
    risk: float


@dataclass(frozen=True)
class CVaRRolloutRisk:
    """Aggregate rollout risk result."""

    maximum_risk: float
    safe: bool
    first_violation_step: int
    samples: tuple[CVaRSample, ...]


def normal_cvar_multiplier(alpha: float) -> float:
    """Return ``phi(Phi^-1(alpha)) / (1 - alpha)``."""
    if not 0.5 < alpha < 1.0:
        raise ValueError("alpha must be in (0.5, 1.0)")
    quantile = float(ndtri(alpha))
    density = exp(-0.5 * quantile * quantile) / sqrt(2.0 * pi)
    return density / (1.0 - alpha)


def eso_process_covariance(
    base_covariance: np.ndarray,
    disturbance: np.ndarray,
    kappa_slip: float = 1.0,
) -> np.ndarray:
    """Inflate a base covariance according to ESO disturbance magnitude."""
    base = np.asarray(base_covariance, dtype=float)
    if base.ndim != 2 or base.shape[0] != base.shape[1]:
        raise ValueError("base covariance must be square")
    d = np.asarray(disturbance, dtype=float).reshape(-1)
    if d.size != 3:
        raise ValueError("disturbance must have length 3")
    if kappa_slip < 0.0:
        raise ValueError("kappa_slip must be nonnegative")
    scale = 1.0 + kappa_slip * float(np.linalg.norm(d))
    return base * scale


def propagate_covariance(
    a_sequence: Sequence[np.ndarray],
    initial_covariance: np.ndarray,
    base_process_covariance: np.ndarray,
    disturbance: np.ndarray | None = None,
    kappa_slip: float = 1.0,
) -> np.ndarray:
    """Propagate covariance along an LTV rollout."""
    sigma = np.asarray(initial_covariance, dtype=float).copy()
    q = np.asarray(base_process_covariance, dtype=float)
    if sigma.shape != q.shape or sigma.shape[0] != sigma.shape[1]:
        raise ValueError("initial and process covariance shapes must match")
    q_eso = eso_process_covariance(q, np.zeros(3) if disturbance is None else disturbance, kappa_slip)
    result = [sigma.copy()]
    for matrix in a_sequence:
        a = np.asarray(matrix, dtype=float)
        if a.shape != sigma.shape:
            raise ValueError("each A matrix must match covariance dimensions")
        sigma = a @ sigma @ a.T + q_eso
        sigma = 0.5 * (sigma + sigma.T)
        result.append(sigma.copy())
    return np.asarray(result)


class CVaRRiskGate:
    """Evaluate maximum Gaussian CVaR penetration risk."""

    def __init__(
        self,
        alpha: float = 0.95,
        d_safe: float = 0.30,
        robot_radius: float = 0.105,
        dt: float = 0.05,
    ) -> None:
        self.alpha = float(alpha)
        self.c_alpha = normal_cvar_multiplier(self.alpha)
        self.d_safe = float(d_safe)
        self.robot_radius = float(robot_radius)
        self.dt = float(dt)

    def evaluate(
        self,
        rollout_states: np.ndarray,
        covariance_sequence: np.ndarray,
        obstacles: Sequence[Any],
    ) -> CVaRRolloutRisk:
        states = np.asarray(rollout_states, dtype=float)
        covariances = np.asarray(covariance_sequence, dtype=float)
        if states.ndim != 2 or states.shape[1] < 2:
            raise ValueError("rollout_states must have shape (N, >=2)")
        if covariances.ndim != 3 or len(covariances) < len(states):
            raise ValueError("covariance_sequence must have one matrix per state")

        samples: list[CVaRSample] = []
        maximum_risk = -float("inf")
        first_violation = -1
        for step, state in enumerate(states):
            position = state[:2]
            sigma_position = covariances[step][:2, :2]
            for obstacle_index, raw_obstacle in enumerate(obstacles):
                obstacle = self._obstacle(raw_obstacle)
                obstacle_position = obstacle.position_at(step * self.dt)
                offset = position - obstacle_position
                distance = float(np.linalg.norm(offset))
                if distance < 1e-10:
                    normal = np.array([1.0, 0.0], dtype=float)
                else:
                    normal = offset / distance
                clearance = distance - obstacle.radius - self.robot_radius - self.d_safe
                variance = float(normal @ sigma_position @ normal)
                normal_sigma = sqrt(max(0.0, variance))
                risk = -clearance + self.c_alpha * normal_sigma
                sample = CVaRSample(
                    step=step,
                    obstacle_index=obstacle_index,
                    clearance=clearance,
                    normal_sigma=normal_sigma,
                    risk=float(risk),
                )
                samples.append(sample)
                maximum_risk = max(maximum_risk, float(risk))
                if risk > 0.0 and first_violation < 0:
                    first_violation = step

        if not samples:
            maximum_risk = -float("inf")
        return CVaRRolloutRisk(
            maximum_risk=float(maximum_risk),
            safe=first_violation < 0,
            first_violation_step=first_violation,
            samples=tuple(samples),
        )

    @staticmethod
    def _obstacle(obstacle: Any) -> ObstacleKinematics:
        if isinstance(obstacle, ObstacleKinematics):
            return obstacle
        if isinstance(obstacle, dict):
            getter = obstacle.get
        else:
            getter = lambda key, default=0.0: getattr(obstacle, key, default)
        return ObstacleKinematics(
            x=float(getter("x", 0.0)),
            y=float(getter("y", 0.0)),
            radius=float(getter("radius", 0.0)),
            vx=float(getter("vx", 0.0)),
            vy=float(getter("vy", 0.0)),
        )


class RiskHysteresis:
    """Binary risk latch with separate entry and exit thresholds."""

    def __init__(self, enter_threshold: float = 0.0, exit_threshold: float = -0.05) -> None:
        if exit_threshold > enter_threshold:
            raise ValueError("exit threshold must not exceed entry threshold")
        self.enter_threshold = float(enter_threshold)
        self.exit_threshold = float(exit_threshold)
        self.latched = False

    def reset(self) -> None:
        self.latched = False

    def update(self, risk: float) -> bool:
        value = float(risk)
        if self.latched:
            if value <= self.exit_threshold:
                self.latched = False
        elif value >= self.enter_threshold:
            self.latched = True
        return self.latched
