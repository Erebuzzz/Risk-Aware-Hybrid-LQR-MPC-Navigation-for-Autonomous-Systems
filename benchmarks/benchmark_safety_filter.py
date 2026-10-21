"""Measure the steady-state Python safety-filter gate.

The gate uses 20 warm-up solves followed by 500 timed solves.  The reported
latency is the OSQP solve time returned by the reusable CVXPY workspace.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PROJECT_ROOT / "src" / "hybrid_controller"
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from hybrid_controller.controllers.riccati_safety_filter import (  # noqa: E402
    RiccatiSafetyFilter,
    SafetyFilterConfig,
)
from hybrid_controller.models.augmented_dynamics import (  # noqa: E402
    AugmentedDifferentialDriveModel,
    DynamicModelParams,
)


def run_gate(warmup: int, solves: int, seed: int, backend: str) -> dict[str, object]:
    rng = np.random.default_rng(seed)
    dt = 0.05
    horizon = 5
    model = AugmentedDifferentialDriveModel(DynamicModelParams(dt=dt))
    safety_filter = RiccatiSafetyFilter(
        model=model,
        config=SafetyFilterConfig(
            horizon=horizon,
            max_obstacles=8,
            dt=dt,
            backend=backend,
        ),
    )

    state = np.array([0.0, 0.0, 0.0, 0.4, 0.0], dtype=float)
    references = np.tile(state, (horizon + 1, 1))
    controls = np.tile(np.array([0.4, 0.0], dtype=float), (horizon, 1))
    obstacles = [
        {"x": 0.8, "y": 0.2, "radius": 0.12, "vx": 0.0, "vy": 0.0},
        {"x": -0.9, "y": 0.7, "radius": 0.16, "vx": 0.0, "vy": 0.0},
    ]

    for _ in range(warmup):
        safety_filter.solve(state, references, controls, obstacles)

    latencies: list[float] = []
    infeasible = 0
    nonfinite = 0
    slack_max = 0.0
    for _ in range(solves):
        trial_state = state.copy()
        trial_state[:2] += rng.normal(0.0, 0.002, size=2)
        solution = safety_filter.solve(trial_state, references, controls, obstacles)
        latencies.append(solution.solve_time_ms)
        infeasible += int(not solution.feasible)
        slack_max = max(slack_max, float(solution.slack_max))
        nonfinite += int(
            not np.all(np.isfinite(solution.optimal_control))
            or not np.all(np.isfinite(solution.predicted_states))
        )

    values = np.asarray(latencies, dtype=float)
    mean_ms = float(np.mean(values))
    p99_ms = float(np.percentile(values, 99.0))
    passed = mean_ms < 5.0 and p99_ms < 10.0 and infeasible == 0 and nonfinite == 0
    return {
        "warmup_solves": warmup,
        "timed_solves": solves,
        "backend": backend,
        "mean_ms": mean_ms,
        "p99_ms": p99_ms,
        "min_ms": float(np.min(values)),
        "max_ms": float(np.max(values)),
        "infeasible_count": infeasible,
        "nonfinite_count": nonfinite,
        "maximum_slack": slack_max,
        "gate_mean_limit_ms": 5.0,
        "gate_p99_limit_ms": 10.0,
        "passed": passed,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the 500-solve safety-filter latency gate")
    parser.add_argument("--warmup", type=int, default=20)
    parser.add_argument("--solves", type=int, default=500)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--backend",
        choices=("python", "rust"),
        default="rust",
        help="Production Rust backend by default; use python for reference timing.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("Output/Plots/Paper/safety_filter_latency.json"),
    )
    args = parser.parse_args()

    result = run_gate(args.warmup, args.solves, args.seed, args.backend)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
