"""Strict adapter for the native RVO2 ORCA implementation.

The adapter deliberately raises when the binding is unavailable.  The Gate 3
benchmark must never silently replace ORCA with a different obstacle model.
"""

from __future__ import annotations

import importlib
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

from hybrid_controller.models.footprint import MUJOCO_ROBOT_RADIUS


class RVO2Unavailable(RuntimeError):
    """Raised when the real RVO2 binding cannot be loaded."""


def load_rvo2():
    python_path = os.environ.get("RVO2_PYTHON_PATH")
    if python_path:
        path = str(Path(python_path).resolve())
        if path not in sys.path:
            sys.path.insert(0, path)

    dll_path = os.environ.get("RVO2_DLL_DIR")
    if os.name == "nt":
        if not dll_path:
            raise RVO2Unavailable(
                "RVO2_DLL_DIR is required on Windows so the native RVO2 runtime "
                "can be loaded explicitly."
            )
        dll_dir = Path(dll_path)
        if not dll_dir.is_dir():
            raise RVO2Unavailable(f"RVO2_DLL_DIR does not exist: {dll_dir}")
        os.add_dll_directory(str(dll_dir))

    try:
        return importlib.import_module("rvo2")
    except ImportError as exc:
        raise RVO2Unavailable(
            "The real RVO2 Python binding is unavailable. Build Python-RVO2 "
            "and set RVO2_PYTHON_PATH and RVO2_DLL_DIR."
        ) from exc


@dataclass
class RVOAgentState:
    """Position, velocity, and preferred velocity for one ORCA agent."""

    position: np.ndarray
    velocity: np.ndarray
    preferred_velocity: np.ndarray
    radius: float


class RVO2DynamicWorld:
    """Small deterministic ORCA world used by the Monte Carlo benchmark."""

    def __init__(
        self,
        dynamic_agents: Iterable[RVOAgentState],
        dt: float = 0.05,
        robot_radius: float = MUJOCO_ROBOT_RADIUS,
        max_speed: float = 2.5,
    ) -> None:
        self._rvo2 = load_rvo2()
        self.dt = float(dt)
        self.simulator = self._rvo2.PyRVOSimulator(
            self.dt,
            5.0,
            32,
            2.0,
            2.0,
            robot_radius,
            max_speed,
        )
        self.robot_index = self.simulator.addAgent((0.0, 0.0))
        self._agents: list[RVOAgentState] = []
        self._indices: list[int] = []
        for agent in dynamic_agents:
            index = self.simulator.addAgent(
                tuple(agent.position),
                5.0,
                16,
                2.0,
                2.0,
                float(agent.radius),
                max_speed,
                tuple(agent.velocity),
            )
            self._agents.append(agent)
            self._indices.append(index)

    def step(self, robot_position: np.ndarray, robot_preferred_velocity: np.ndarray) -> list[dict[str, float]]:
        """Advance ORCA and return dynamic obstacle states."""
        robot_position = np.asarray(robot_position, dtype=float).reshape(2)
        robot_preferred_velocity = np.asarray(robot_preferred_velocity, dtype=float).reshape(2)
        self.simulator.setAgentPosition(self.robot_index, tuple(robot_position))
        self.simulator.setAgentPrefVelocity(
            self.robot_index, tuple(robot_preferred_velocity)
        )
        for state, index in zip(self._agents, self._indices):
            self.simulator.setAgentPrefVelocity(index, tuple(state.preferred_velocity))
        self.simulator.doStep()

        result: list[dict[str, float]] = []
        for state, index in zip(self._agents, self._indices):
            position = np.asarray(self.simulator.getAgentPosition(index), dtype=float)
            velocity = np.asarray(self.simulator.getAgentVelocity(index), dtype=float)
            state.position = position
            state.velocity = velocity
            result.append(
                {
                    "x": float(position[0]),
                    "y": float(position[1]),
                    "radius": float(state.radius),
                    "vx": float(velocity[0]),
                    "vy": float(velocity[1]),
                }
            )
        return result
