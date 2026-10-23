"""Diagnostic two-sided angular-channel reachability trigger.

This module is intentionally a measurement gate. It does not alter the safety
filter or claim collision safety. The trigger checks whether the turn direction
needed to leave a forward obstacle channel is reachable after one yaw-lag and
command-slew step.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, exp, hypot, pi
from typing import Any

import numpy as np

from ..models.footprint import DEFAULT_CONTROLLER_ROBOT_RADIUS


@dataclass(frozen=True)
class AngularChannelDecision:
    """Reachability diagnostics for both turn directions."""

    bearing_error: float
    clearance: float
    forward_channel: bool
    required_turn_rate: float
    reachable_omega_min: float
    reachable_omega_max: float
    left_margin: float
    right_margin: float
    selected_margin: float
    selected_side: str
    trigger: bool


def wrap_angle(angle: float) -> float:
    """Wrap an angle to ``[-pi, pi)``."""
    return float((angle + pi) % (2.0 * pi) - pi)


def evaluate_angular_channel(
    state: np.ndarray,
    obstacle: Any,
    *,
    previous_omega_command: float,
    dt: float = 0.05,
    tau_omega: float = 0.10,
    omega_min: float = -2.0,
    omega_max: float = 2.0,
    max_delta_omega: float = 0.15,
    robot_radius: float = DEFAULT_CONTROLLER_ROBOT_RADIUS,
    d_safe: float = 0.30,
    channel_half_angle: float = pi / 4.0,
    turn_gain: float = 2.0,
    minimum_turn_rate: float = 0.35,
    trigger_clearance: float = 0.0,
    centerline_tolerance: float = 1.0e-6,
) -> AngularChannelDecision:
    """Evaluate a two-sided turn-reachability trigger.

    ``left_margin`` is positive when a left turn of the required magnitude is
    reachable after the next slew-limited command and actuator-lag update.
    ``right_margin`` has the analogous meaning for a right turn. For an
    obstacle close to the centerline, the better of the two sides is selected;
    otherwise the side away from the obstacle bearing is selected.
    """
    values = (dt, tau_omega, omega_min, omega_max, max_delta_omega, robot_radius, d_safe)
    if not all(np.isfinite(value) for value in values):
        raise ValueError("angular-channel parameters must be finite")
    if dt <= 0.0 or tau_omega <= 0.0:
        raise ValueError("dt and tau_omega must be positive")
    if omega_min >= omega_max:
        raise ValueError("omega_min must be less than omega_max")
    if max_delta_omega <= 0.0 or robot_radius < 0.0 or d_safe < 0.0:
        raise ValueError("angular limits and clearance parameters are invalid")
    if not 0.0 < channel_half_angle <= pi:
        raise ValueError("channel_half_angle must be in (0, pi]")
    if turn_gain <= 0.0 or minimum_turn_rate < 0.0:
        raise ValueError("turn_gain must be positive and minimum_turn_rate nonnegative")

    values_state = np.asarray(state, dtype=float).reshape(-1)
    if values_state.size < 3 or not np.all(np.isfinite(values_state)):
        raise ValueError("state must contain at least finite x, y, and theta")
    ox = _obstacle_value(obstacle, "x")
    oy = _obstacle_value(obstacle, "y")
    obstacle_radius = _obstacle_value(obstacle, "radius", 0.0)
    if obstacle_radius < 0.0:
        raise ValueError("obstacle radius must be nonnegative")

    dx = ox - values_state[0]
    dy = oy - values_state[1]
    distance = hypot(dx, dy)
    bearing_error = wrap_angle(atan2(dy, dx) - values_state[2])
    clearance = distance - obstacle_radius - robot_radius - d_safe
    forward_channel = abs(bearing_error) <= channel_half_angle

    required_turn_rate = max(
        minimum_turn_rate,
        turn_gain * max(0.0, channel_half_angle - abs(bearing_error)),
    )
    command_min = max(omega_min, previous_omega_command - max_delta_omega)
    command_max = min(omega_max, previous_omega_command + max_delta_omega)
    decay = exp(-dt / tau_omega)
    actual_omega = values_state[4] if values_state.size >= 5 else 0.0
    reachable_omega_min = decay * actual_omega + (1.0 - decay) * command_min
    reachable_omega_max = decay * actual_omega + (1.0 - decay) * command_max
    left_margin = reachable_omega_max - required_turn_rate
    right_margin = -required_turn_rate - reachable_omega_min

    if abs(bearing_error) <= centerline_tolerance:
        selected_side = "left" if left_margin >= right_margin else "right"
        selected_margin = max(left_margin, right_margin)
    elif bearing_error > 0.0:
        selected_side = "right"
        selected_margin = right_margin
    else:
        selected_side = "left"
        selected_margin = left_margin

    trigger = bool(
        forward_channel
        and clearance <= trigger_clearance
        and selected_margin < 0.0
    )
    return AngularChannelDecision(
        bearing_error=float(bearing_error),
        clearance=float(clearance),
        forward_channel=forward_channel,
        required_turn_rate=float(required_turn_rate),
        reachable_omega_min=float(reachable_omega_min),
        reachable_omega_max=float(reachable_omega_max),
        left_margin=float(left_margin),
        right_margin=float(right_margin),
        selected_margin=float(selected_margin),
        selected_side=selected_side,
        trigger=trigger,
    )


def _obstacle_value(obstacle: Any, key: str, default: float | None = None) -> float:
    if isinstance(obstacle, dict):
        if key not in obstacle and default is None:
            raise ValueError(f"obstacle is missing {key!r}")
        value = obstacle.get(key, default)
    else:
        if not hasattr(obstacle, key) and default is None:
            raise ValueError(f"obstacle is missing {key!r}")
        value = getattr(obstacle, key, default)
    if value is None:
        raise ValueError(f"obstacle {key!r} must not be None")
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"obstacle {key!r} must be finite")
    return result
