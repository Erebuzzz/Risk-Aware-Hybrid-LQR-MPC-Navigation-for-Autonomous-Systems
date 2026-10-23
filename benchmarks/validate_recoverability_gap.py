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


def mujoco_stopping_distance(
    v0: float,
    u0: float,
    delta_u: float,
    dt: float,
    tau_v: float,
    *,
    friction: float = 1.0,
    velocity_tolerance: float = 1.0e-4,
    max_steps: int = 20_000,
) -> float:
    """Measure stopping travel in the MuJoCo plant, when MuJoCo is available."""

    _validate_parameters(v0, u0, delta_u, dt, tau_v)
    if not 0.0 < friction:
        raise ValueError("friction must be positive")
    if velocity_tolerance <= 0.0:
        raise ValueError("velocity_tolerance must be positive")
    if max_steps <= 0:
        raise ValueError("max_steps must be positive")

    from benchmarks.mujoco_scene import MuJoCoPlant, MuJoCoPlantConfig
    import numpy as np

    plant = MuJoCoPlant(
        [],
        friction=friction,
        config=MuJoCoPlantConfig(dt=dt, tau_v=tau_v, tau_omega=tau_v, slip_gain=0.0),
    )
    plant.reset(np.array([0.0, 0.0, 0.0, v0, 0.0], dtype=float))
    step = 0
    zero_command_seen = False
    state = plant.state()
    while step < max_steps:
        command = max(0.0, u0 - step * delta_u)
        zero_command_seen = zero_command_seen or command <= 0.0
        state = plant.step(np.array([command, 0.0], dtype=float))
        step += 1
        if zero_command_seen and abs(float(state[3])) <= velocity_tolerance:
            break
    else:
        raise RuntimeError("MuJoCo did not reach the stopping tolerance")
    return float(state[0])


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
) -> list[dict[str, float | bool]]:
    """Measure a deterministic prefix of cases in the MuJoCo plant."""

    if samples < 0:
        raise ValueError("samples must be nonnegative")
    selected = list(cases)[:samples]
    comparisons: list[dict[str, float | bool]] = []
    for case in selected:
        measured = mujoco_stopping_distance(
            case.v0,
            case.u0,
            case.delta_u,
            case.dt,
            case.tau_v,
            friction=friction,
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
                "mujoco_distance": float(measured),
                "mujoco_minus_continuous": float(measured - case.continuous_distance),
                "mujoco_minus_discrete": float(measured - case.discrete_distance),
                "mujoco_within_discrete_1cm": bool(
                    abs(measured - case.discrete_distance) <= 0.01
                ),
            }
        )
    return comparisons


def _write_outputs(
    output: Path,
    cases: list[RecoverabilityCase],
    split: str,
    dt: float,
    mujoco_comparisons: list[dict[str, float | bool]] | None = None,
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
    (output / "recoverability_gap_validation.json").write_text(
        json.dumps(payload, indent=2),
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
