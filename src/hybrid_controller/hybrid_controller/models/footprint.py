"""Explicit footprint contracts used by controller and plant models.

The controller-facing ROS profile and the custom MuJoCo benchmark use different
geometric contracts. Keeping both named prevents a benchmark-only body size
from silently changing the ROS safety margin.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite


@dataclass(frozen=True)
class CircularFootprint:
    """Circular over-approximation used in clearance calculations."""

    name: str
    radius: float
    source: str

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("footprint name must not be empty")
        if not isfinite(self.radius) or self.radius <= 0.0:
            raise ValueError("footprint radius must be finite and positive")


TURTLEBOT3_BURGER_FOOTPRINT = CircularFootprint(
    name="turtlebot3_burger_controller",
    radius=0.105,
    source="ROS controller safety-margin profile",
)

MUJOCO_ROBOT_HALF_LENGTH = 0.20
MUJOCO_ROBOT_HALF_WIDTH = 0.14
MUJOCO_ROBOT_FOOTPRINT = CircularFootprint(
    name="mujoco_box_circumscribed",
    radius=hypot(MUJOCO_ROBOT_HALF_LENGTH, MUJOCO_ROBOT_HALF_WIDTH),
    source="MuJoCo robot_body box circumscribed radius",
)

DEFAULT_CONTROLLER_ROBOT_RADIUS = TURTLEBOT3_BURGER_FOOTPRINT.radius
MUJOCO_ROBOT_RADIUS = MUJOCO_ROBOT_FOOTPRINT.radius
