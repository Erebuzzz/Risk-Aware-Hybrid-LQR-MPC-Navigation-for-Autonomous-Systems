"""Validate continuous and exact discrete stopping-distance predictions.

The diagnostic uses the same left-endpoint pose convention as
``AugmentedDifferentialDriveModel.step``: position advances with the current
actual velocity, then the actuator state is updated by the exact first-order
lag map.  The command is a nonnegative staircase that decreases by
``delta_u`` each control interval until it reaches zero.

The script intentionally separates three quantities:

* the continuous minimum-travel proposition;
* the exact zero-order-hold discrete recurrence;
* the optional MuJoCo plant measurement.

It does not claim that the continuous expression is conservative.  The output
reports any underprediction so the correction can be derived and reviewed
before it is used by a safety mechanism.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class StoppingMeasurementProtocol:
    """Sampling/termination rules fixed before the v2 sweep (2026-10-24).

    Positions are sampled at reset and after each control interval, not at
    every physics substep. A stop requires zero command and a consecutive
    0.5-second window of quiet endpoint samples, including both endpoints.
    All three translational and rotational body DOFs and both hidden actuator
    states must be quiet. This is a finite sampled measurement, not an
    infinite-horizon stopping guarantee or obstacle-contact measurement.
    """

    version: str = "sampled_peak_settled_v2"
    physics_dt: float = 0.002
    velocity_tolerance: float = 1.0e-4
    angular_velocity_tolerance: float = 1.0e-4
    settling_window_s: float = 0.5
    max_steps: int = 20_000


STOPPING_MEASUREMENT_PROTOCOL = StoppingMeasurementProtocol()


@dataclass(frozen=True)
class PlantStoppingMeasurement:
    """One stopping experiment, including unsupported/invalid observations.

    Partial distances on an unsupported run are retained for diagnosis only.
    Only ``status == 'settled'`` supports a virtual-boundary comparison.
    """

    status: str
    reason: str | None
    maximum_sampled_forward_travel_m: float | None
    terminal_forward_travel_m: float | None
    maximum_travel_time_s: float | None
    elapsed_time_s: float
    attempted_steps: int
    completed_steps: int
    sample_count: int
    quiet_samples: int
    required_quiet_samples: int
    terminal_body_linear_speed_m_s: float | None
    terminal_body_angular_speed_rad_s: float | None
    terminal_actuator_v_m_s: float | None
    terminal_actuator_omega_rad_s: float | None


@dataclass(frozen=True)
class RecoverabilityCase:
    """One point in the deterministic recoverability grid."""

    v0: float
    command_gap: float
    obstacle_distance: float
    tau_v: float
    delta_u: float
    dt: float
    rho_v: float
    u0: float
    state_only_distance: float
    continuous_distance: float
    discrete_distance: float
    state_only_minus_continuous: float
    discrete_minus_continuous: float
    state_only_safe: bool
    continuous_safe: bool
    discrete_safe: bool


def _validate_parameters(
    v0: float,
    u0: float,
    delta_u: float,
    dt: float,
    tau_v: float,
) -> None:
    values = (v0, u0, delta_u, dt, tau_v)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("recoverability parameters must be finite")
    if v0 < 0.0:
        raise ValueError("v0 must be nonnegative")
    if u0 < 0.0:
        raise ValueError("u0 must be nonnegative")
    if delta_u <= 0.0:
        raise ValueError("delta_u must be positive")
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    if tau_v <= 0.0:
        raise ValueError("tau_v must be positive")


def continuous_minimum_travel(v0: float, u0: float, rho_v: float, tau_v: float) -> float:
    """Return the continuous minimum-travel proposition.

    The result assumes nonnegative velocity and command, no reverse, no
    steering credit, and a linearly decreasing command with rate ``rho_v``.
    """

    values = (v0, u0, rho_v, tau_v)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("continuous recoverability parameters must be finite")
    if v0 < 0.0 or u0 < 0.0:
        raise ValueError("v0 and u0 must be nonnegative")
    if rho_v <= 0.0:
        raise ValueError("rho_v must be positive")
    if tau_v <= 0.0:
        raise ValueError("tau_v must be positive")
    return float(tau_v * v0 + (u0 * u0) / (2.0 * rho_v))


def state_only_minimum_travel(v0: float, tau_v: float) -> float:
    """Return the stop distance available from state velocity alone.

    This deliberately ignores the currently latched command. It is the
    state-only comparator for the controlled grid, not the rate-aware bound.
    """
    if not math.isfinite(v0) or not math.isfinite(tau_v):
        raise ValueError("state-only recoverability parameters must be finite")
    if v0 < 0.0 or tau_v <= 0.0:
        raise ValueError("v0 must be nonnegative and tau_v must be positive")
    return float(tau_v * v0)


def discrete_stopping_distance(
    v0: float,
    u0: float,
    delta_u: float,
    dt: float,
    tau_v: float,
) -> float:
    """Return exact stopping travel for the benchmark's discrete convention.

    The command at step ``k`` is ``max(0, u0 - k * delta_u)``.  Position uses
    the velocity at the beginning of each interval.  Once the command reaches
    zero, the remaining geometric tail is summed analytically, so the result
    does not depend on an arbitrary finite stopping threshold.
    """

    _validate_parameters(v0, u0, delta_u, dt, tau_v)
    decay = math.exp(-dt / tau_v)
    input_gain = 1.0 - decay
    velocity = float(v0)
    distance = 0.0
    step = 0

    while True:
        command = max(0.0, u0 - step * delta_u)
        distance += dt * velocity
        velocity = decay * velocity + input_gain * command
        if command <= 0.0:
            break
        step += 1

    distance += dt * velocity / (1.0 - decay)
    return float(distance)


def discrete_stopping_trace(
    v0: float,
    u0: float,
    delta_u: float,
    dt: float,
    tau_v: float,
) -> tuple[list[float], list[float]]:
    """Return command and beginning-of-step velocity sequences through zero."""

    _validate_parameters(v0, u0, delta_u, dt, tau_v)
    decay = math.exp(-dt / tau_v)
    input_gain = 1.0 - decay
    velocity = float(v0)
    commands: list[float] = []
    velocities: list[float] = []
    step = 0
    while True:
        command = max(0.0, u0 - step * delta_u)
        commands.append(float(command))
        velocities.append(float(velocity))
        velocity = decay * velocity + input_gain * command
        if command <= 0.0:
            break
        step += 1
    return commands, velocities


def stopping_plant_config(dt: float, tau_v: float):
    """Build the unchanged plant contract explicitly for repeatable reports."""
    from benchmarks.mujoco_scene import MuJoCoPlantConfig
    from hybrid_controller.models.footprint import (
        MUJOCO_ROBOT_HALF_LENGTH,
        MUJOCO_ROBOT_HALF_WIDTH,
        MUJOCO_ROBOT_RADIUS,
    )

    return MuJoCoPlantConfig(
        dt=dt,
        physics_dt=STOPPING_MEASUREMENT_PROTOCOL.physics_dt,
        tau_v=tau_v,
        tau_omega=tau_v,
        force_gain=9.0,
        torque_gain=4.0,
        robot_mass=2.0,
        gravity_z=-9.81,
        robot_radius=MUJOCO_ROBOT_RADIUS,
        robot_half_length=MUJOCO_ROBOT_HALF_LENGTH,
        robot_half_width=MUJOCO_ROBOT_HALF_WIDTH,
        slip_gain=0.0,
    )


def measure_mujoco_stop(
    v0: float,
    u0: float,
    delta_u: float,
    dt: float,
    tau_v: float,
    *,
    friction: float = 1.0,
    velocity_tolerance: float = STOPPING_MEASUREMENT_PROTOCOL.velocity_tolerance,
    angular_velocity_tolerance: float = STOPPING_MEASUREMENT_PROTOCOL.angular_velocity_tolerance,
    settling_window_s: float = STOPPING_MEASUREMENT_PROTOCOL.settling_window_s,
    max_steps: int = STOPPING_MEASUREMENT_PROTOCOL.max_steps,
) -> PlantStoppingMeasurement:
    """Measure maximum sampled forward travel until sustained settling.

    The command remains ``max(0, u0 - k * delta_u)``. No obstacle is placed in
    MuJoCo. Runtime failures, non-finite state, reset/time discontinuities, and
    failure to settle produce unsupported results, never a safe distance.
    MuJoCoPlant also checks numerical validity at each physics substep.
    """

    _validate_parameters(v0, u0, delta_u, dt, tau_v)
    for name, value in (
        ("friction", friction),
        ("velocity_tolerance", velocity_tolerance),
        ("angular_velocity_tolerance", angular_velocity_tolerance),
        ("settling_window_s", settling_window_s),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    if isinstance(max_steps, bool) or not isinstance(max_steps, int) or max_steps <= 0:
        raise ValueError("max_steps must be a positive integer")

    from benchmarks.mujoco_scene import MuJoCoPlant
    import numpy as np

    config = stopping_plant_config(dt, tau_v)
    substeps = max(1, int(round(config.dt / config.physics_dt)))
    sample_period = substeps * config.physics_dt
    required_quiet_samples = math.ceil(settling_window_s / sample_period) + 1
    maximum_travel = terminal_travel = maximum_time = None
    linear_speed = angular_speed = actuator_v = actuator_omega = None
    completed_steps = attempted_steps = sample_count = quiet_samples = 0
    elapsed = 0.0

    def result(status: str, reason: str | None = None) -> PlantStoppingMeasurement:
        return PlantStoppingMeasurement(
            status=status,
            reason=reason,
            maximum_sampled_forward_travel_m=maximum_travel,
            terminal_forward_travel_m=terminal_travel,
            maximum_travel_time_s=maximum_time,
            elapsed_time_s=elapsed,
            attempted_steps=attempted_steps,
            completed_steps=completed_steps,
            sample_count=sample_count,
            quiet_samples=quiet_samples,
            required_quiet_samples=required_quiet_samples,
            terminal_body_linear_speed_m_s=linear_speed,
            terminal_body_angular_speed_rad_s=angular_speed,
            terminal_actuator_v_m_s=actuator_v,
            terminal_actuator_omega_rad_s=actuator_omega,
        )

    def observe(state):
        state = np.asarray(state, dtype=float)
        actuator = np.asarray(plant.actuator_state(), dtype=float)
        qvel = np.asarray(plant.data.qvel, dtype=float)
        if state.shape != (5,) or actuator.shape != (2,) or qvel.shape != (6,):
            raise RuntimeError("unsupported plant state dimensions")
        for values in (state, actuator, plant.data.qpos, qvel, plant.data.qacc):
            if not np.all(np.isfinite(values)):
                raise RuntimeError("non-finite plant or actuator state")
        if not math.isfinite(float(plant.data.time)):
            raise RuntimeError("non-finite simulation time")
        linear = float(np.linalg.norm(qvel[:3]))
        angular = float(np.linalg.norm(qvel[3:]))
        if not math.isfinite(linear) or not math.isfinite(angular):
            raise RuntimeError("non-finite body speed norm")
        return state, actuator, linear, angular

    try:
        plant = MuJoCoPlant([], friction=friction, config=config)
        plant.reset(np.array([0.0, 0.0, 0.0, v0, 0.0], dtype=float))
        state, actuator, linear_speed, angular_speed = observe(plant.state())
        actuator_v, actuator_omega = map(float, actuator)
        initial_x = float(state[0])
        start_time = float(plant.data.time)
        maximum_travel = terminal_travel = maximum_time = 0.0
        sample_count = 1
        for step in range(max_steps):
            command = max(0.0, u0 - step * delta_u)
            attempted_steps = step + 1
            state, actuator, next_linear_speed, next_angular_speed = observe(
                plant.step(np.array([command, 0.0], dtype=float))
            )
            next_elapsed = float(plant.data.time) - start_time
            expected_elapsed = attempted_steps * sample_period
            if not math.isclose(next_elapsed, expected_elapsed, rel_tol=1.0e-9, abs_tol=1.0e-9):
                raise RuntimeError("simulation time discontinuity or reset")
            elapsed = next_elapsed
            completed_steps += 1
            sample_count += 1
            terminal_travel = float(state[0]) - initial_x
            if terminal_travel > maximum_travel:
                maximum_travel, maximum_time = terminal_travel, elapsed
            linear_speed, angular_speed = next_linear_speed, next_angular_speed
            actuator_v, actuator_omega = map(float, actuator)
            quiet = (
                command == 0.0
                and abs(actuator_v) <= velocity_tolerance
                and abs(actuator_omega) <= angular_velocity_tolerance
                and linear_speed <= velocity_tolerance
                and angular_speed <= angular_velocity_tolerance
                and abs(float(state[3])) <= velocity_tolerance
                and abs(float(state[4])) <= angular_velocity_tolerance
            )
            quiet_samples = quiet_samples + 1 if quiet else 0
            if quiet_samples >= required_quiet_samples:
                return result("settled")
    except RuntimeError as error:
        return result("invalid", f"{type(error).__name__}: {error}")
    return result(
        "not_settled",
        f"No {settling_window_s:g}s quiet sample window within {max_steps} control steps",
    )


def mujoco_stopping_distance(
    v0: float,
    u0: float,
    delta_u: float,
    dt: float,
    tau_v: float,
    *,
    friction: float = 1.0,
    velocity_tolerance: float = STOPPING_MEASUREMENT_PROTOCOL.velocity_tolerance,
    angular_velocity_tolerance: float = STOPPING_MEASUREMENT_PROTOCOL.angular_velocity_tolerance,
    settling_window_s: float = STOPPING_MEASUREMENT_PROTOCOL.settling_window_s,
    max_steps: int = STOPPING_MEASUREMENT_PROTOCOL.max_steps,
) -> float:
    """Return maximum sampled forward travel; fail closed if unsupported.

    Use :func:`measure_mujoco_stop` for raw diagnostics and status instead of
    this scalar compatibility API. Terminal position is never a stop bound.
    """
    measurement = measure_mujoco_stop(
        v0, u0, delta_u, dt, tau_v,
        friction=friction,
        velocity_tolerance=velocity_tolerance,
        angular_velocity_tolerance=angular_velocity_tolerance,
        settling_window_s=settling_window_s,
        max_steps=max_steps,
    )
    if measurement.status != "settled":
        raise RuntimeError(f"Unsupported stopping measurement ({measurement.status}): {measurement.reason}")
    assert measurement.maximum_sampled_forward_travel_m is not None
    return measurement.maximum_sampled_forward_travel_m


def evaluate_case(
    *,
    v0: float,
    command_gap: float,
    obstacle_distance: float,
    tau_v: float,
    delta_u: float,
    dt: float,
) -> RecoverabilityCase:
    """Evaluate one recoverability and safe-boundary grid point."""

    if command_gap < 0.0:
        raise ValueError("command_gap must be nonnegative")
    if obstacle_distance < 0.0:
        raise ValueError("obstacle_distance must be nonnegative")
    u0 = v0 + command_gap
    rho_v = delta_u / dt
    state_only = state_only_minimum_travel(v0, tau_v)
    continuous = continuous_minimum_travel(v0, u0, rho_v, tau_v)
    discrete = discrete_stopping_distance(v0, u0, delta_u, dt, tau_v)
    return RecoverabilityCase(
        v0=float(v0),
        command_gap=float(command_gap),
        obstacle_distance=float(obstacle_distance),
        tau_v=float(tau_v),
        delta_u=float(delta_u),
        dt=float(dt),
        rho_v=float(rho_v),
        u0=float(u0),
        state_only_distance=state_only,
        continuous_distance=continuous,
        discrete_distance=discrete,
        state_only_minus_continuous=float(state_only - continuous),
        discrete_minus_continuous=float(discrete - continuous),
        state_only_safe=bool(obstacle_distance >= state_only),
        continuous_safe=bool(obstacle_distance >= continuous),
        discrete_safe=bool(obstacle_distance >= discrete),
    )


def _parse_values(text: str) -> list[float]:
    values = [float(part.strip()) for part in text.split(",") if part.strip()]
    if not values:
        raise ValueError("value list must contain at least one number")
    return values


def run_grid(
    *,
    v0_values: Iterable[float],
    command_gap_values: Iterable[float],
    obstacle_distance_values: Iterable[float],
    tau_values: Iterable[float],
    delta_u_values: Iterable[float],
    dt: float,
) -> list[RecoverabilityCase]:
    """Run the deterministic recoverability grid in stable nested order."""

    cases: list[RecoverabilityCase] = []
    for tau_v in tau_values:
        for delta_u in delta_u_values:
            for v0 in v0_values:
                for command_gap in command_gap_values:
                    for obstacle_distance in obstacle_distance_values:
                        cases.append(
                            evaluate_case(
                                v0=float(v0),
                                command_gap=float(command_gap),
                                obstacle_distance=float(obstacle_distance),
                                tau_v=float(tau_v),
                                delta_u=float(delta_u),
                                dt=dt,
                            )
                        )
    return cases


def summarize(cases: Iterable[RecoverabilityCase]) -> dict[str, float | int]:
    """Summarize discrete correction and safe-boundary disagreement."""

    rows = list(cases)
    if not rows:
        raise ValueError("at least one recoverability case is required")
    corrections = [row.discrete_minus_continuous for row in rows]
    state_only_corrections = [row.state_only_minus_continuous for row in rows]
    return {
        "case_count": len(rows),
        "continuous_underprediction_count": sum(value > 1.0e-12 for value in corrections),
        "continuous_overprediction_count": sum(value < -1.0e-12 for value in corrections),
        "maximum_discrete_minus_continuous": float(max(corrections)),
        "minimum_discrete_minus_continuous": float(min(corrections)),
        "mean_discrete_minus_continuous": float(sum(corrections) / len(corrections)),
        "state_only_underprediction_count": sum(
            value < -1.0e-12 for value in state_only_corrections
        ),
        "maximum_continuous_minus_state_only": float(-min(state_only_corrections)),
        "state_only_vs_continuous_disagreement_count": sum(
            row.state_only_safe != row.continuous_safe for row in rows
        ),
        "state_only_vs_discrete_disagreement_count": sum(
            row.state_only_safe != row.discrete_safe for row in rows
        ),
        "safe_boundary_disagreement_count": sum(
            row.continuous_safe != row.discrete_safe for row in rows
        ),
    }


def compare_with_mujoco(
    cases: Iterable[RecoverabilityCase],
    *,
    samples: int,
    friction: float,
) -> list[dict[str, object]]:
    """Measure a prefix; unsupported trials retain status and null comparisons."""

    if samples < 0:
        raise ValueError("samples must be nonnegative")
    selected = list(cases)[:samples]
    comparisons: list[dict[str, object]] = []
    for case in selected:
        measurement = measure_mujoco_stop(
            case.v0,
            case.u0,
            case.delta_u,
            case.dt,
            case.tau_v,
            friction=friction,
        )
        measured = (
            measurement.maximum_sampled_forward_travel_m
            if measurement.status == "settled" else None
        )
        comparisons.append(
            {
                "v0": case.v0,
                "command_gap": case.command_gap,
                "tau_v": case.tau_v,
                "delta_u": case.delta_u,
                "dt": case.dt,
                "friction": float(friction),
                "continuous_distance": case.continuous_distance,
                "discrete_distance": case.discrete_distance,
                "mujoco_distance": measured,
                "mujoco_minus_continuous": (
                    measured - case.continuous_distance if measured is not None else None
                ),
                "mujoco_minus_discrete": (
                    measured - case.discrete_distance if measured is not None else None
                ),
                "mujoco_within_discrete_1cm": (
                    abs(measured - case.discrete_distance) <= 0.01 if measured is not None else None
                ),
                "plant_config_json": json.dumps(asdict(stopping_plant_config(case.dt, case.tau_v)), sort_keys=True),
                "obstacle_contact_evidence": "not_measured_empty_obstacle_scene",
                **{f"measurement_{key}": value for key, value in asdict(measurement).items()},
            }
        )
    return comparisons


def _write_outputs(
    output: Path,
    cases: list[RecoverabilityCase],
    split: str,
    dt: float,
    mujoco_comparisons: list[dict[str, object]] | None = None,
) -> None:
    output.mkdir(parents=True, exist_ok=True)
    rows = [asdict(case) for case in cases]
    summary = summarize(cases)
    payload: dict[str, object] = {
        "metadata": {"split": split, "dt": dt},
        "summary": summary,
        "cases": rows,
    }
    if mujoco_comparisons is not None:
        payload["mujoco_comparisons"] = mujoco_comparisons
        payload["mujoco_measurement_protocol"] = {
            **asdict(STOPPING_MEASUREMENT_PROTOCOL),
            "sampling": "reset and control interval endpoints",
            "distance": "maximum sampled forward displacement, not terminal x",
            "obstacles": [],
            "unsupported_measurement_count": sum(
                row["measurement_status"] != "settled" for row in mujoco_comparisons
            ),
        }
    (output / "recoverability_gap_validation.json").write_text(
        json.dumps(payload, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    with (output / "recoverability_gap_validation.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    if mujoco_comparisons:
        with (output / "recoverability_gap_mujoco_comparison.csv").open(
            "w", newline="", encoding="utf-8"
        ) as handle:
            writer = csv.DictWriter(
                handle, fieldnames=list(mujoco_comparisons[0].keys())
            )
            writer.writeheader()
            writer.writerows(mujoco_comparisons)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate continuous and discrete actuator-lag recoverability bounds"
    )
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument(
        "--split",
        choices=("primary", "held-out"),
        default="primary",
        help="Use the fixed primary or held-out parameter grid.",
    )
    parser.add_argument("--v0-values", default="0.25,0.75,1.5,2.5")
    parser.add_argument("--command-gap-values", default="0.0,0.5,1.0,1.5")
    parser.add_argument("--obstacle-distance-values", default="0.1,0.5,1.0,1.5")
    parser.add_argument("--tau-values", default="0.06,0.10,0.15,0.20")
    parser.add_argument("--delta-u-values", default="0.05,0.10,0.20")
    parser.add_argument(
        "--include-mujoco",
        action="store_true",
        help="Measure the first --mujoco-samples cases in the MuJoCo plant.",
    )
    parser.add_argument("--mujoco-samples", type=int, default=8)
    parser.add_argument("--mujoco-friction", type=float, default=1.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("Output/Plots/Paper/recoverability_gap_validation"),
    )
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    grid = {
        "primary": {
            "v0": args.v0_values,
            "gap": args.command_gap_values,
            "distance": args.obstacle_distance_values,
            "tau": args.tau_values,
            "delta": args.delta_u_values,
        },
        "held-out": {
            "v0": "0.4,1.1,2.0",
            "gap": "0.25,0.75,1.25",
            "distance": "0.25,0.75,1.25",
            "tau": "0.08,0.13,0.18",
            "delta": "0.075,0.15",
        },
    }[args.split]
    cases = run_grid(
        v0_values=_parse_values(grid["v0"]),
        command_gap_values=_parse_values(grid["gap"]),
        obstacle_distance_values=_parse_values(grid["distance"]),
        tau_values=_parse_values(grid["tau"]),
        delta_u_values=_parse_values(grid["delta"]),
        dt=args.dt,
    )
    mujoco_comparisons = (
        compare_with_mujoco(
            cases,
            samples=args.mujoco_samples,
            friction=args.mujoco_friction,
        )
        if args.include_mujoco
        else None
    )
    _write_outputs(args.output, cases, args.split, args.dt, mujoco_comparisons)
    print(json.dumps(summarize(cases), indent=2))
    if mujoco_comparisons is not None:
        print(json.dumps({"mujoco_comparisons": mujoco_comparisons}, indent=2))


if __name__ == "__main__":
    main()
