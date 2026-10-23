"""Deterministic non-reactive obstacle trajectories for controlled trials."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite


@dataclass(frozen=True)
class ControlledObstacle:
    """Obstacle with constant acceleration and an optional acceleration bound."""

    x0: float
    y0: float
    vx0: float
    vy0: float
    ax: float
    ay: float
    radius: float
    acceleration_bound: float | None = None

    def __post_init__(self) -> None:
        values = (self.x0, self.y0, self.vx0, self.vy0, self.ax, self.ay, self.radius)
        if not all(isfinite(value) for value in values):
            raise ValueError("controlled obstacle parameters must be finite")
        if self.radius < 0.0:
            raise ValueError("obstacle radius must be nonnegative")
        if self.acceleration_bound is not None:
            if not isfinite(self.acceleration_bound) or self.acceleration_bound < 0.0:
                raise ValueError("acceleration_bound must be finite and nonnegative")
            if hypot(self.ax, self.ay) > self.acceleration_bound + 1e-12:
                raise ValueError("constant acceleration exceeds acceleration_bound")

    def state_at(self, time_s: float) -> dict[str, float]:
        """Return position, velocity, acceleration, and radius at ``time_s``."""
        if not isfinite(time_s) or time_s < 0.0:
            raise ValueError("time_s must be finite and nonnegative")
        return {
            "x": self.x0 + self.vx0 * time_s + 0.5 * self.ax * time_s * time_s,
            "y": self.y0 + self.vy0 * time_s + 0.5 * self.ay * time_s * time_s,
            "vx": self.vx0 + self.ax * time_s,
            "vy": self.vy0 + self.ay * time_s,
            "ax": self.ax,
            "ay": self.ay,
            "radius": self.radius,
            "acceleration_bound": (
                self.acceleration_bound
                if self.acceleration_bound is not None
                else 0.0
            ),
        }


def nonreactive_obstacle(
    x: float,
    y: float,
    radius: float,
    *,
    vx: float = 0.0,
    vy: float = 0.0,
) -> ControlledObstacle:
    """Create a non-reactive constant-velocity obstacle."""
    return ControlledObstacle(x, y, vx, vy, 0.0, 0.0, radius, 0.0)


def bounded_acceleration_obstacle(
    x: float,
    y: float,
    radius: float,
    *,
    vx: float = 0.0,
    vy: float = 0.0,
    ax: float = 0.0,
    ay: float = 0.0,
    acceleration_bound: float,
) -> ControlledObstacle:
    """Create a deterministic obstacle whose acceleration respects a bound."""
    return ControlledObstacle(
        x,
        y,
        vx,
        vy,
        ax,
        ay,
        radius,
        acceleration_bound,
    )
