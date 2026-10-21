"""Run the measured friction and ESO ablation used by Figure 4."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from benchmarks.run_mujoco_monte_carlo import run_trial  # noqa: E402


VARIANTS = {
    "hard_switch_reference": ("4_hard_switch", False),
    "ra_psf_no_eso": ("5_ra_psf_ours", False),
    "ra_psf_eso": ("5_ra_psf_ours", True),
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the friction and ESO ablation")
    parser.add_argument("--frictions", nargs="+", type=float, default=[0.25, 0.4, 0.6, 0.8, 1.0])
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--speed", type=float, default=1.5)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=5100)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    records: list[dict[str, float | int | str]] = []
    for friction in args.frictions:
        for variant, (mode, eso_enabled) in VARIANTS.items():
            metrics_group = []
            for trial in range(args.trials):
                seed = args.seed + int(round(friction * 100.0)) * 100 + trial
                metrics, _ = run_trial(
                    mode,
                    seed,
                    args.duration,
                    args.speed,
                    args.dt,
                    friction_override=friction,
                    eso_enabled=eso_enabled,
                )
                metrics_group.append(metrics)
                print(
                    f"friction={friction:.2f} variant={variant} "
                    f"trial={trial + 1}/{args.trials} collision={metrics.collision}"
                )
            records.append(
                {
                    "friction": float(friction),
                    "variant": variant,
                    "collision_rate": float(np.mean([item.collision for item in metrics_group])),
                    "rms_cross_track_error": float(
                        np.mean([item.rms_cross_track_error for item in metrics_group])
                    ),
                    "angular_jerk_rms": float(
                        np.mean([item.angular_jerk_rms for item in metrics_group])
                    ),
                    "mean_latency_ms": float(
                        np.mean([item.mean_latency_ms for item in metrics_group])
                    ),
                    "trials": args.trials,
                }
            )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(records, indent=2), encoding="utf-8")
    print(f"Wrote {len(records)} slip-ablation records to {args.output}")


if __name__ == "__main__":
    main()
