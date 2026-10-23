"""MuJoCo and RVO2 ablation runner.

The benchmark requires the native RVO2 ORCA binding.  It does not substitute a
different dynamic-obstacle model when RVO2 is unavailable.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PROJECT_ROOT / "src" / "hybrid_controller"
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from hybrid_controller.controllers.mpc_controller import Obstacle
from hybrid_controller.controllers.backup_safety import BackupSafetyController
from hybrid_controller.controllers.riccati_safety_filter import (
    RiccatiSafetyFilter,
    SafetyFilterConfig,
)
from hybrid_controller.controllers.cvar_gate import RiskHysteresis, propagate_covariance
from hybrid_controller.controllers.eso import ESOConfig, ExtendedStateObserver3D
from hybrid_controller.models.augmented_dynamics import (
    AugmentedDifferentialDriveModel,
    DynamicModelParams,
)
from hybrid_controller.models.footprint import MUJOCO_ROBOT_RADIUS
from hybrid_controller.trajectory.spatial_reference import SpatialFigure8Reference

from benchmarks.mujoco_scene import MuJoCoPlant, MuJoCoPlantConfig
from benchmarks.rvo2_adapter import RVO2DynamicWorld, RVOAgentState


MODES = (
    "1_nominal_lqr",
    "2_async_nmpc",
    "3_convex_blend",
    "4_hard_switch",
    "5_ra_psf_ours",
)

POSE_INTEGRATOR_CONVENTION = "mujoco_physics_substeps_after_exact_actuator_lag_update"
ROBOT_RADIUS = MUJOCO_ROBOT_RADIUS


@dataclass
class TrialMetrics:
    mode: str
    seed: int
    friction: float
    tau_v: float
    tau_omega: float
    collision: bool
    minimum_clearance: float
    rms_cross_track_error: float
    angular_jerk_rms: float
    intervention_ratio: float
    mean_intervention_norm: float
    mean_cvar_risk: float
    maximum_cvar_risk: float
    maximum_slack: float
    mean_latency_ms: float
    p95_latency_ms: float
    p99_latency_ms: float
    infeasible_count: int
    fallback_count: int
    steps: int


def wrap_angle(value: float) -> float:
    return float((value + np.pi) % (2.0 * np.pi) - np.pi)


def path_reference(reference: SpatialFigure8Reference, s: float, speed: float, dt: float) -> tuple[np.ndarray, np.ndarray]:
    point = reference.evaluate(s)
    next_point = reference.evaluate(s + max(0.01, speed * dt))
    omega = wrap_angle(next_point.heading - point.heading) / max(0.01, speed * dt) * speed
    state = np.array([point.position[0], point.position[1], point.heading, speed, omega])
    control = np.array([speed, omega])
    return state, control


def generate_static_obstacles(rng: np.random.Generator) -> list[dict[str, float]]:
    obstacles: list[dict[str, float]] = []
    attempts = 0
    while len(obstacles) < 3 and attempts < 200:
        candidate = {
            "x": float(rng.uniform(-1.6, 1.6)),
            "y": float(rng.uniform(-1.1, 1.1)),
            "radius": float(rng.uniform(0.12, 0.22)),
        }
        attempts += 1
        if np.hypot(candidate["x"], candidate["y"]) < 0.55:
            continue
        if any(
            np.hypot(candidate["x"] - other["x"], candidate["y"] - other["y"])
            < candidate["radius"] + other["radius"] + 0.25
            for other in obstacles
        ):
            continue
        obstacles.append(candidate)
    return obstacles


def generate_dynamic_agents(rng: np.random.Generator) -> list[RVOAgentState]:
    agents = []
    for _ in range(2):
        position = np.array(
            [rng.uniform(-1.7, 1.7), rng.uniform(-1.2, 1.2)], dtype=float
        )
        if np.linalg.norm(position) < 0.7:
            position += np.array([0.9, 0.6])
        target = -position / max(np.linalg.norm(position), 1e-6)
        velocity = 0.25 * target
        agents.append(
            RVOAgentState(
                position=position,
                velocity=velocity,
                preferred_velocity=velocity,
                radius=0.14,
            )
        )
    return agents


class AsyncNmpcBaseline:
    """Existing IPOPT adaptive NMPC with an explicit command delay queue."""

    def __init__(self, dt: float, delay_s: float = 0.18) -> None:
        from hybrid_controller.controllers.adaptive_mpc_controller import AdaptiveMPCController

        self.dt = dt
        self.delay_steps = max(1, int(round(delay_s / dt)))
        self.controller = AdaptiveMPCController(
            prediction_horizon=4,
            terminal_horizon=2,
            Q_diag=[30.0, 30.0, 8.0],
            R_diag=[0.1, 0.2],
            d_safe=0.30,
            v_max=2.5,
            omega_max=3.0,
            dt=dt,
            enable_adaptation=False,
        )
        self.queue: deque[np.ndarray] = deque(
            [np.zeros(2, dtype=float) for _ in range(self.delay_steps)],
            maxlen=self.delay_steps + 1,
        )

    def command(
        self,
        state: np.ndarray,
        references: np.ndarray,
        controls: np.ndarray,
        obstacles: Sequence[Any],
    ) -> tuple[np.ndarray, float, bool]:
        start = time.perf_counter()
        x_refs = references[:, :3]
        u_refs = controls
        try:
            solution = self.controller.solve_tracking(state[:3], x_refs, u_refs, obstacles)
            command = np.asarray(solution.optimal_control, dtype=float)
            feasible = solution.status in ("optimal", "fallback")
        except Exception:
            command = np.zeros(2, dtype=float)
            feasible = False
        self.queue.append(np.clip(command, [-2.5, -3.0], [2.5, 3.0]))
        delayed = self.queue.popleft()
        elapsed = (time.perf_counter() - start) * 1000.0
        return delayed, elapsed, feasible


def distance_risk(state: np.ndarray, obstacles: Sequence[Any], trigger: float = 1.0) -> float:
    if not obstacles:
        return 0.0
    values = []
    for obstacle in obstacles:
        x = float(obstacle["x"] if isinstance(obstacle, dict) else obstacle.x)
        y = float(obstacle["y"] if isinstance(obstacle, dict) else obstacle.y)
        radius = float(obstacle["radius"] if isinstance(obstacle, dict) else obstacle.radius)
        clearance = np.hypot(state[0] - x, state[1] - y) - radius - ROBOT_RADIUS
        values.append(np.clip(1.0 - clearance / trigger, 0.0, 1.0))
    return float(max(values))


def obstacle_clearances(
    state: np.ndarray,
    obstacles: Sequence[Any],
    robot_radius: float = ROBOT_RADIUS,
) -> list[float]:
    """Return robot-boundary clearances for an obstacle snapshot."""

    values: list[float] = []
    for obstacle in obstacles:
        ox = obstacle["x"] if isinstance(obstacle, dict) else obstacle.x
        oy = obstacle["y"] if isinstance(obstacle, dict) else obstacle.y
        radius = obstacle["radius"] if isinstance(obstacle, dict) else obstacle.radius
        values.append(
            float(np.hypot(state[0] - ox, state[1] - oy) - radius - robot_radius)
        )
    return values


def serialize_obstacle_snapshot(obstacles: Sequence[Any]) -> str:
    """Serialize obstacle positions, radii, and velocities for trace replay."""

    snapshot: list[dict[str, float]] = []
    for obstacle in obstacles:
        if isinstance(obstacle, dict):
            snapshot.append(
                {
                    "x": float(obstacle["x"]),
                    "y": float(obstacle["y"]),
                    "radius": float(obstacle["radius"]),
                    "vx": float(obstacle.get("vx", 0.0)),
                    "vy": float(obstacle.get("vy", 0.0)),
                    "ax": float(obstacle.get("ax", 0.0)),
                    "ay": float(obstacle.get("ay", 0.0)),
                    "acceleration_bound": float(obstacle.get("acceleration_bound", 0.0)),
                }
            )
        else:
            snapshot.append(
                {
                    "x": float(obstacle.x),
                    "y": float(obstacle.y),
                    "radius": float(obstacle.radius),
                    "vx": float(getattr(obstacle, "vx", 0.0)),
                    "vy": float(getattr(obstacle, "vy", 0.0)),
                    "ax": float(getattr(obstacle, "ax", 0.0)),
                    "ay": float(getattr(obstacle, "ay", 0.0)),
                    "acceleration_bound": float(
                        getattr(obstacle, "acceleration_bound", 0.0)
                    ),
                }
            )
    return json.dumps(snapshot, sort_keys=True, separators=(",", ":"))


def make_reference_window(reference: SpatialFigure8Reference, progress: float, speed: float, dt: float, horizon: int):
    states = []
    controls = []
    for index in range(horizon + 1):
        state, control = path_reference(reference, progress + speed * dt * index, speed, dt)
        states.append(state)
        if index < horizon:
            controls.append(control)
    return np.asarray(states), np.asarray(controls)


def rate_limit_command(
    command: np.ndarray,
    previous_command: np.ndarray,
    config: SafetyFilterConfig,
    multiplier: float = 1.0,
) -> np.ndarray:
    """Apply the same actuator command bounds used by the safety filter."""
    if multiplier <= 0.0:
        raise ValueError("rate-limit multiplier must be positive")
    command = np.asarray(command, dtype=float).reshape(2)
    previous = np.asarray(previous_command, dtype=float).reshape(2)
    clipped = np.clip(command, [config.v_min, config.omega_min], [config.v_max, config.omega_max])
    delta_limit = multiplier * np.asarray(config.max_control_delta, dtype=float)
    return np.clip(
        previous + np.clip(clipped - previous, -delta_limit, delta_limit),
        [config.v_min, config.omega_min],
        [config.v_max, config.omega_max],
    )


def make_lqr_rollout(
    safety_filter: RiccatiSafetyFilter,
    state: np.ndarray,
    reference_states: np.ndarray,
    reference_controls: np.ndarray,
    previous_command: np.ndarray | None = None,
    disturbance: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Build a rate-feasible nominal LQR rollout used by the safety filter."""
    d_hat = np.zeros(3, dtype=float) if disturbance is None else np.asarray(disturbance, dtype=float)
    nominal_states = np.zeros_like(reference_states)
    nominal_controls = np.zeros_like(reference_controls)
    nominal_states[0] = np.asarray(state, dtype=float)
    previous = (
        np.zeros(2, dtype=float)
        if previous_command is None
        else np.asarray(previous_command, dtype=float).reshape(2)
    )
    for index in range(len(reference_controls)):
        raw_command = safety_filter.nominal_lqr_control(
            nominal_states[index], reference_states[index], reference_controls[index]
        )
        nominal_controls[index] = rate_limit_command(
            raw_command, previous, safety_filter.config
        )
        nominal_states[index + 1] = safety_filter.model.step(
            nominal_states[index], nominal_controls[index], d_hat
        )
        previous = nominal_controls[index]
    return nominal_states, nominal_controls


def run_trial(
    mode: str,
    seed: int,
    duration: float,
    target_speed: float,
    dt: float,
    friction_override: float | None = None,
    eso_enabled: bool = True,
    horizon: int = 5,
    slack_fallback: bool = False,
    slack_fallback_threshold: float = 0.05,
    risk_horizon: int | None = None,
    max_obstacles: int = 8,
) -> tuple[TrialMetrics, list[dict[str, float | bool | str]]]:
    rng = np.random.default_rng(seed)
    friction = (
        float(friction_override)
        if friction_override is not None
        else float(rng.uniform(0.25, 1.0))
    )
    tau_v = float(rng.uniform(0.06, 0.20))
    tau_omega = float(rng.uniform(0.06, 0.20))
    static_obstacles = generate_static_obstacles(rng)
    dynamic_agents = generate_dynamic_agents(rng)
    scene_obstacle_count = len(static_obstacles) + len(dynamic_agents)
    if max_obstacles < scene_obstacle_count:
        raise ValueError(
            "max_obstacles must cover every static and dynamic obstacle in the scene "
            f"({scene_obstacle_count})"
        )
    if slack_fallback_threshold < 0.0:
        raise ValueError("slack_fallback_threshold must be nonnegative")
    rvo_world = RVO2DynamicWorld(dynamic_agents, dt=dt, max_speed=target_speed + 1.0)
    plant = MuJoCoPlant(
        static_obstacles,
        friction=friction,
        config=MuJoCoPlantConfig(
            dt=dt,
            tau_v=tau_v,
            tau_omega=tau_omega,
            slip_gain=0.35,
        ),
    )
    reference = SpatialFigure8Reference(amplitude=2.0)
    model = AugmentedDifferentialDriveModel(
        DynamicModelParams(dt=dt, tau_v=tau_v, tau_omega=tau_omega)
    )
    filter_backend = "rust" if mode == "5_ra_psf_ours" else "python"
    filter_ = RiccatiSafetyFilter(
        model=model,
        config=SafetyFilterConfig(
            horizon=horizon,
            max_obstacles=max_obstacles,
            dt=dt,
            v_max=2.5,
            omega_max=3.0,
            backend=filter_backend,
        ),
    )
    filter_horizon = filter_.N
    risk_horizon = filter_horizon if risk_horizon is None else int(risk_horizon)
    if risk_horizon < filter_horizon:
        raise ValueError("risk_horizon must be at least the QP horizon")
    backup_controller = BackupSafetyController(v_max=2.5, omega_max=3.0)
    async_nmpc = AsyncNmpcBaseline(dt=dt) if mode == "2_async_nmpc" else None
    risk_latch = RiskHysteresis(enter_threshold=0.0, exit_threshold=-0.05)
    eso = ExtendedStateObserver3D(
        ESOConfig(dt=dt, tau_v=tau_v, tau_omega=tau_omega)
    )

    state = plant.state()
    progress = 0.0
    controls_applied: list[np.ndarray] = []
    cte_values: list[float] = []
    latency_values: list[float] = []
    clearances: list[float] = []
    intervention_norms: list[float] = []
    cvar_risks: list[float] = []
    slack_values: list[float] = []
    trace: list[dict[str, float | bool | str]] = []
    interventions = 0
    infeasible = 0
    fallbacks = 0
    previous_command = np.zeros(2, dtype=float)
    fallback_active = False
    steps = max(1, int(round(duration / dt)))

    for _ in range(steps):
        pre_action_state = state.copy()
        previous_applied_command = previous_command.copy()
        fallback_active_before = fallback_active
        reference_state, reference_control = path_reference(reference, progress, target_speed, dt)
        x_refs, u_refs = make_reference_window(
            reference, progress, target_speed, dt, risk_horizon
        )
        filter_x_refs = x_refs[: filter_horizon + 1]
        filter_u_refs = u_refs[:filter_horizon]
        world_preferred = np.array(
            [reference_control[0] * np.cos(reference_state[2]), reference_control[0] * np.sin(reference_state[2])]
        )
        dynamic_obstacles = rvo_world.step(state[:2], world_preferred)
        obstacles = static_obstacles + dynamic_obstacles
        pre_step_clearances = obstacle_clearances(pre_action_state, obstacles)
        pre_step_clearance = min(pre_step_clearances) if pre_step_clearances else float("inf")
        obstacle_snapshot = serialize_obstacle_snapshot(obstacles)
        risk = distance_risk(state, obstacles)
        disturbance_estimate = eso.update(state, previous_command)
        disturbance_for_controller = (
            disturbance_estimate
            if mode == "5_ra_psf_ours" and eso_enabled
            else np.zeros(3, dtype=float)
        )
        nominal_states, nominal_controls = make_lqr_rollout(
            filter_,
            state,
            x_refs,
            u_refs,
            previous_command=previous_command,
            disturbance=disturbance_for_controller,
        )
        u_lqr = nominal_controls[0].copy()
        covariance_matrices = [
            filter_.model.jacobians(
                nominal_states[index], nominal_controls[index], disturbance_for_controller
            )[0]
            for index in range(risk_horizon)
        ]
        cvar_covariance = propagate_covariance(
            covariance_matrices,
            np.eye(5, dtype=float) * 1.0e-5,
            np.diag([1.0e-5, 1.0e-5, 1.0e-5, 2.0e-5, 2.0e-5]),
            disturbance=disturbance_for_controller,
            kappa_slip=1.0,
        )
        cvar_result = filter_.risk_gate.evaluate(
            nominal_states, cvar_covariance, obstacles
        )
        cvar_risk = cvar_result.maximum_risk
        risk_latched = risk_latch.update(cvar_risk) if mode == "5_ra_psf_ours" else True

        if mode == "5_ra_psf_ours":
            filter_.set_previous_command(previous_command)

        start = time.perf_counter()
        solution = None
        fallback_used = False
        solver_status = "unknown"
        if mode == "1_nominal_lqr":
            command = u_lqr
            feasible = True
            solver_status = "nominal_lqr"
        elif mode == "2_async_nmpc":
            assert async_nmpc is not None
            command, solver_ms, feasible = async_nmpc.command(
                state,
                filter_x_refs,
                filter_u_refs,
                [Obstacle(float(o["x"]), float(o["y"]), float(o["radius"])) for o in obstacles],
            )
            latency_values.append(solver_ms)
            solver_status = "async_nmpc_feasible" if feasible else "async_nmpc_failed"
        elif mode == "5_ra_psf_ours" and not risk_latched:
            command = u_lqr
            feasible = True
            solver_status = "risk_bypass"
            latency_values.append(0.0)
            command = filter_.limit_command(command, previous_command)
        else:
            # The safety filter is linearized around the LQR nominal rollout.
            # Use that same rollout as its state reference so the QP error
            # coordinates, tangent obstacle planes, and dynamics all share
            # one local operating point.  Passing the Figure 8 reference here
            # would mix coordinates from two different trajectories and omit
            # the affine linearization offset.
            solution = filter_.solve(
                state,
                nominal_states[: filter_horizon + 1],
                nominal_controls[:filter_horizon],
                obstacles,
                disturbance=disturbance_for_controller,
                nominal_states=nominal_states[: filter_horizon + 1],
                covariance_sequence=cvar_covariance[: filter_horizon + 1],
            )
            command_filter = solution.optimal_control
            latency_values.append(solution.solve_time_ms)
            feasible = solution.feasible
            solver_status = str(solution.status)
            if not feasible:
                fallbacks += 1
            if (
                mode == "5_ra_psf_ours"
                and slack_fallback
                and not solution.safety_satisfied
                and solution.slack_max > slack_fallback_threshold
            ):
                backup = backup_controller.compute_augmented(
                    state,
                    obstacles,
                    dt=dt,
                    tau_v=tau_v,
                    tau_omega=tau_omega,
                    horizon=max(10, filter_.N),
                    robot_radius=ROBOT_RADIUS,
                )
                fallback_multiplier = 3.0 if fallback_active else 1.0
                command = rate_limit_command(
                    backup.control,
                    previous_command,
                    filter_.config,
                    multiplier=fallback_multiplier,
                )
                feasible = False
                fallbacks += int(solution.feasible)
                fallback_used = True
            elif mode == "3_convex_blend":
                command = (1.0 - risk) * u_lqr + risk * command_filter
            elif mode == "4_hard_switch":
                command = command_filter if risk >= 0.30 else u_lqr
            else:
                command = command_filter
        fallback_active = fallback_used
        if mode in ("1_nominal_lqr",):
            latency_values.append((time.perf_counter() - start) * 1000.0)
        requested_command = np.asarray(command, dtype=float).copy()
        command = np.clip(requested_command, [-2.5, -3.0], [2.5, 3.0])
        intervention_norm = float(np.linalg.norm(command - u_lqr))
        if intervention_norm > 1e-4:
            interventions += 1
        if not feasible:
            infeasible += 1

        state = plant.step(command)
        progress, cte = reference.update_progress(state[:2], progress, forward_window=target_speed * dt * 2.0)
        cte_values.append(cte)
        controls_applied.append(command.copy())
        intervention_norms.append(intervention_norm)
        cvar_risks.append(float(cvar_risk))
        current_slack = float(solution.slack_max) if solution is not None else 0.0
        slack_values.append(current_slack if np.isfinite(current_slack) else 0.0)
        current_clearances = obstacle_clearances(state, obstacles)
        clearances.extend(current_clearances)
        post_step_clearance = min(current_clearances) if current_clearances else float("inf")
        trace.append(
            {
                "step": float(len(trace)),
                "time_s": float(len(trace) * dt),
                "dt": float(dt),
                "pose_integrator": POSE_INTEGRATOR_CONVENTION,
                "pre_x": float(pre_action_state[0]),
                "pre_y": float(pre_action_state[1]),
                "pre_theta": float(pre_action_state[2]),
                "pre_v": float(pre_action_state[3]),
                "pre_omega": float(pre_action_state[4]),
                "previous_applied_v": float(previous_applied_command[0]),
                "previous_applied_omega": float(previous_applied_command[1]),
                "x": float(state[0]),
                "y": float(state[1]),
                "theta": float(state[2]),
                "v": float(state[3]),
                "omega": float(state[4]),
                "reference_x": float(reference_state[0]),
                "reference_y": float(reference_state[1]),
                "reference_theta": float(reference_state[2]),
                "progress": float(progress),
                "cross_track_error": float(cte),
                "u_lqr_v": float(u_lqr[0]),
                "u_lqr_omega": float(u_lqr[1]),
                "requested_command_v": float(requested_command[0]),
                "requested_command_omega": float(requested_command[1]),
                "applied_command_v": float(command[0]),
                "applied_command_omega": float(command[1]),
                "command_v": float(command[0]),
                "command_omega": float(command[1]),
                "tau_v": float(tau_v),
                "tau_omega": float(tau_omega),
                "obstacle_snapshot": obstacle_snapshot,
                "risk": float(risk),
                "cvar_risk": float(cvar_risk),
                "eso_dv": float(disturbance_estimate[0]),
                "eso_dy": float(disturbance_estimate[1]),
                "eso_domega": float(disturbance_estimate[2]),
                "risk_latched": bool(risk_latched),
                "pre_step_clearance": float(pre_step_clearance),
                "post_step_clearance": float(post_step_clearance),
                "minimum_clearance": float(post_step_clearance),
                "intervention_norm": intervention_norm,
                "latency_ms": float(latency_values[-1]),
                "slack_max": current_slack,
                "solver_status": solver_status,
                "feasible": bool(feasible),
                "fallback": bool(not feasible),
                "fallback_active_before": bool(fallback_active_before),
                "fallback_used": bool(fallback_used),
                "fallback_active": bool(fallback_active),
                "mode": mode,
            }
        )
        previous_command = command

    controls_array = np.asarray(controls_applied)
    if len(controls_array) >= 3:
        angular_acceleration = np.diff(controls_array[:, 1]) / dt
        angular_jerk = np.diff(angular_acceleration) / dt
        angular_jerk_rms = float(np.sqrt(np.mean(angular_jerk**2))) if len(angular_jerk) else 0.0
    else:
        angular_jerk_rms = 0.0
    latency_array = np.asarray(latency_values, dtype=float)
    collision = bool(min(clearances) < 0.0) if clearances else False
    result = TrialMetrics(
        mode=mode,
        seed=seed,
        friction=friction,
        tau_v=tau_v,
        tau_omega=tau_omega,
        collision=collision,
        minimum_clearance=float(min(clearances)) if clearances else float("inf"),
        rms_cross_track_error=float(np.sqrt(np.mean(np.asarray(cte_values) ** 2))),
        angular_jerk_rms=angular_jerk_rms,
        intervention_ratio=float(interventions / max(steps, 1)),
        mean_intervention_norm=float(np.mean(intervention_norms))
        if intervention_norms
        else 0.0,
        mean_cvar_risk=float(np.mean(cvar_risks)) if cvar_risks else 0.0,
        maximum_cvar_risk=float(np.max(cvar_risks)) if cvar_risks else 0.0,
        maximum_slack=float(np.max(slack_values)) if slack_values else 0.0,
        mean_latency_ms=float(np.mean(latency_array)) if len(latency_array) else 0.0,
        p95_latency_ms=float(np.percentile(latency_array, 95.0)) if len(latency_array) else 0.0,
        p99_latency_ms=float(np.percentile(latency_array, 99.0)) if len(latency_array) else 0.0,
        infeasible_count=infeasible,
        fallback_count=fallbacks,
        steps=steps,
    )
    return result, trace


def aggregate(metrics: Sequence[TrialMetrics]) -> dict[str, dict[str, float]]:
    result: dict[str, dict[str, float]] = {}
    metric_names = (
        "collision_rate",
        "minimum_clearance",
        "rms_cross_track_error",
        "angular_jerk_rms",
        "intervention_ratio",
        "mean_intervention_norm",
        "mean_cvar_risk",
        "maximum_cvar_risk",
        "maximum_slack",
        "mean_latency_ms",
        "p95_latency_ms",
        "p99_latency_ms",
        "infeasible_count",
        "fallback_count",
    )
    for mode in sorted({item.mode for item in metrics}):
        group = [item for item in metrics if item.mode == mode]
        values: dict[str, np.ndarray] = {
            "collision_rate": np.asarray([item.collision for item in group], dtype=float),
            "minimum_clearance": np.asarray([item.minimum_clearance for item in group]),
            "rms_cross_track_error": np.asarray(
                [item.rms_cross_track_error for item in group]
            ),
            "angular_jerk_rms": np.asarray([item.angular_jerk_rms for item in group]),
            "intervention_ratio": np.asarray([item.intervention_ratio for item in group]),
            "mean_intervention_norm": np.asarray(
                [item.mean_intervention_norm for item in group]
            ),
            "mean_cvar_risk": np.asarray([item.mean_cvar_risk for item in group]),
            "maximum_cvar_risk": np.asarray([item.maximum_cvar_risk for item in group]),
            "maximum_slack": np.asarray([item.maximum_slack for item in group]),
            "mean_latency_ms": np.asarray([item.mean_latency_ms for item in group]),
            "p95_latency_ms": np.asarray([item.p95_latency_ms for item in group]),
            "p99_latency_ms": np.asarray([item.p99_latency_ms for item in group]),
            "infeasible_count": np.asarray([item.infeasible_count for item in group]),
            "fallback_count": np.asarray([item.fallback_count for item in group]),
        }
        aggregate_values: dict[str, float] = {}
        for name in metric_names:
            sample = values[name]
            aggregate_values[name] = float(np.mean(sample))
            aggregate_values[f"{name}_std"] = float(np.std(sample))
            aggregate_values[f"{name}_median"] = float(np.median(sample))
            aggregate_values[f"{name}_p95"] = float(np.percentile(sample, 95.0))
        result[mode] = aggregate_values
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Run MuJoCo and RVO2 RA-PSF ablations")
    parser.add_argument("--trials", type=int, default=200)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--speed", type=float, default=1.5)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument(
        "--horizon",
        type=int,
        default=5,
        help="Safety-filter horizon in control steps.",
    )
    parser.add_argument(
        "--risk-horizon",
        type=int,
        default=None,
        help="CVaR look-ahead horizon. Defaults to the safety-filter horizon.",
    )
    parser.add_argument(
        "--max-obstacles",
        type=int,
        default=8,
        help="Maximum obstacle slots allocated in the safety-filter QP; must cover the scene.",
    )
    parser.add_argument(
        "--slack-fallback",
        action="store_true",
        help="Use the experimental actuator-aware backup when RA-PSF uses obstacle slack.",
    )
    parser.add_argument(
        "--slack-fallback-threshold",
        type=float,
        default=0.05,
        help="Minimum positive QP slack required before the experimental fallback is used.",
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--trace-seed",
        type=int,
        default=42,
        help="Write per-step traces for this seed. Use -1 to disable traces.",
    )
    parser.add_argument("--output", type=Path, default=Path("Output/Plots/Paper/ablation_results"))
    parser.add_argument("--modes", nargs="+", choices=MODES, default=list(MODES))
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    records: list[TrialMetrics] = []
    for mode in args.modes:
        for trial in range(args.trials):
            seed = args.seed + trial
            metrics, trace = run_trial(
                mode,
                seed,
                args.duration,
                args.speed,
                args.dt,
                horizon=args.horizon,
                slack_fallback=args.slack_fallback,
                slack_fallback_threshold=args.slack_fallback_threshold,
                risk_horizon=args.risk_horizon,
                max_obstacles=args.max_obstacles,
            )
            records.append(metrics)
            if args.trace_seed >= 0 and seed == args.trace_seed:
                trace_dir = args.output / "traces"
                trace_dir.mkdir(parents=True, exist_ok=True)
                (trace_dir / f"{mode}_seed_{seed}.json").write_text(
                    json.dumps(trace, indent=2), encoding="utf-8"
                )
            print(
                f"{mode} trial={trial + 1}/{args.trials} collision={metrics.collision} "
                f"clearance={metrics.minimum_clearance:.3f} latency={metrics.mean_latency_ms:.3f}ms"
            )

    summary = {
        "metadata": {
            "trials": args.trials,
            "duration": args.duration,
            "target_speed": args.speed,
            "dt": args.dt,
            "horizon": args.horizon,
            "risk_horizon": (
                args.horizon if args.risk_horizon is None else args.risk_horizon
            ),
            "max_obstacles": args.max_obstacles,
            "slack_fallback": args.slack_fallback,
            "slack_fallback_threshold": args.slack_fallback_threshold,
            "seed": args.seed,
            "trace_seed": args.trace_seed,
            "rvo2_required": True,
        },
        "aggregate": aggregate(records),
        "trials": [asdict(item) for item in records],
    }
    (args.output / "mujoco_rvo2_ablation.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    with (args.output / "mujoco_rvo2_ablation.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(records[0]).keys()))
        writer.writeheader()
        writer.writerows(asdict(item) for item in records)
    print(json.dumps(summary["aggregate"], indent=2))


if __name__ == "__main__":
    main()
