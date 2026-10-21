"""Run a measured target-speed and latency sweep for the phase diagram."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from benchmarks.run_mujoco_monte_carlo import MODES, run_trial  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the measured speed-latency sweep")
    parser.add_argument("--speeds", nargs="+", type=float, default=[0.5, 1.0, 1.5, 2.0, 2.5])
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=4200)
    parser.add_argument("--modes", nargs="+", choices=MODES, default=list(MODES))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    records: list[dict[str, float | int | str]] = []
    for speed in args.speeds:
        for mode in args.modes:
            metrics_group = []
            for trial in range(args.trials):
                seed = args.seed + int(round(speed * 100.0)) * 100 + trial
                metrics, _ = run_trial(mode, seed, args.duration, speed, args.dt)
                metrics_group.append(metrics)
                print(
                    f"speed={speed:.2f} mode={mode} trial={trial + 1}/{args.trials} "
                    f"collision={metrics.collision} latency={metrics.mean_latency_ms:.3f}ms"
                )
            records.append(
                {
                    "speed": float(speed),
                    "mode": mode,
                    "collision_rate": float(np.mean([item.collision for item in metrics_group])),
                    "latency_ms": float(np.mean([item.mean_latency_ms for item in metrics_group])),
                    "p99_latency_ms": float(np.mean([item.p99_latency_ms for item in metrics_group])),
                    "rms_cross_track_error": float(
                        np.mean([item.rms_cross_track_error for item in metrics_group])
                    ),
                    "angular_jerk_rms": float(
                        np.mean([item.angular_jerk_rms for item in metrics_group])
                    ),
                    "trials": args.trials,
                }
            )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(records, indent=2), encoding="utf-8")
    print(f"Wrote {len(records)} phase-sweep records to {args.output}")


if __name__ == "__main__":
    main()
