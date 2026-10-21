"""Headless MuJoCo plant for friction and actuator-lag validation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import mujoco
import numpy as np


@dataclass(frozen=True)
class MuJoCoPlantConfig:
    dt: float = 0.05
    physics_dt: float = 0.002
    tau_v: float = 0.10
    tau_omega: float = 0.10
    force_gain: float = 9.0
    torque_gain: float = 4.0
    robot_radius: float = 0.20
    slip_gain: float = 0.0


def build_scene_xml(
    obstacles: Sequence[dict[str, float]], friction: float, physics_dt: float = 0.002
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
  <option timestep="{physics_dt}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <default>
    <geom contype="1" conaffinity="1" friction="{friction} 0.01 0.001"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" pos="0 0 0" size="20 20 0.1" friction="{friction} 0.01 0.001"/>
    {''.join(obstacle_geoms)}
    <body name="robot" pos="0 0 0.16">
      <freejoint/>
      <geom name="robot_body" type="box" size="0.20 0.14 0.06" mass="2.0" friction="{friction} 0.01 0.001" rgba="0.1 0.3 0.8 1"/>
    </body>
  </worldbody>
</mujoco>
"""


class MuJoCoPlant:
    """A rigid-body base with force and yaw-torque velocity control."""

    def __init__(
        self,
        obstacles: Sequence[dict[str, float]],
        friction: float,
        config: MuJoCoPlantConfig | None = None,
    ) -> None:
        self.config = config or MuJoCoPlantConfig()
        xml = build_scene_xml(obstacles, friction, self.config.physics_dt)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.model.opt.timestep = self.config.physics_dt
        self.data = mujoco.MjData(self.model)
        self.robot_body_id = self.model.body("robot").id
        self.floor_geom_id = self.model.geom("floor").id
        self.obstacles = list(obstacles)
        self.friction = float(friction)
        self.v_actual = 0.0
        self.omega_actual = 0.0
        self.reset()

    def reset(self, state: np.ndarray | None = None) -> None:
        mujoco.mj_resetData(self.model, self.data)
        x = np.zeros(5, dtype=float) if state is None else np.asarray(state, dtype=float).reshape(5)
        self.data.qpos[0] = x[0]
        self.data.qpos[1] = x[1]
        self.data.qpos[2] = 0.16
        self.data.qpos[3:7] = self._yaw_to_quaternion(x[2])
        self.data.qvel[:] = 0.0
        self.v_actual = float(x[3])
        self.omega_actual = float(x[4])
        mujoco.mj_forward(self.model, self.data)

    def state(self) -> np.ndarray:
        theta = self._quaternion_to_yaw(self.data.qpos[3:7])
        c, s = np.cos(theta), np.sin(theta)
        vx_world, vy_world = self.data.qvel[0], self.data.qvel[1]
        v_body = c * vx_world + s * vy_world
        return np.array([self.data.qpos[0], self.data.qpos[1], theta, v_body, self.data.qvel[5]])

    def step(self, command: np.ndarray) -> np.ndarray:
        command = np.asarray(command, dtype=float).reshape(2)
        self.v_actual = self._lag(self.v_actual, command[0], self.config.tau_v, self.config.dt)
        self.omega_actual = self._lag(
            self.omega_actual, command[1], self.config.tau_omega, self.config.dt
        )
        substeps = max(1, int(round(self.config.dt / self.config.physics_dt)))
        for _ in range(substeps):
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
            mujoco.mj_step(self.model, self.data)
        self.data.xfrc_applied[self.robot_body_id, :] = 0.0
        return self.state()

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
