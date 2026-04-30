#!/usr/bin/env python3
import subprocess
import sys
from pathlib import Path

# Define the sweep parameters
TRAJECTORIES = ["clover3", "rose4", "spiral", "random_wp", "figure8"]
SCENARIOS = ["default", "dense", "moving"]
MODE = "hybrid"

OUTPUT_DIR = Path("Output") / "Plots" / "Hybrid"


def run_output_dir(traj: str, scenario: str) -> Path:
    return OUTPUT_DIR / traj / scenario


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print(f"Starting Master Hybrid Validation Suite")
    print(
        f"Testing {len(TRAJECTORIES)} geometric paths across {len(SCENARIOS)} dynamic environment classes."
    )

    for traj in TRAJECTORIES:
        for scenario in SCENARIOS:
            out_dir = run_output_dir(traj, scenario)
            out_dir.mkdir(parents=True, exist_ok=True)
            print(f"\n===========================================================")
            print(f"Running Hybrid Navigation | Path: {traj} | Env: {scenario}")
            print(f"Output Directory: {out_dir}")
            print(f"===========================================================")

            cmd = [
                sys.executable,
                "run_simulation.py",
                "--mode",
                MODE,
                "--trajectory",
                traj,
                "--scenario",
                scenario,
            ]

            try:
                result = subprocess.run(cmd, check=True, capture_output=True, text=True)
                print(result.stdout, end="")
                if result.stderr:
                    print(result.stderr, end="")
            except subprocess.CalledProcessError as e:
                print(f"Error running {traj} in {scenario}: {e}")
                print(e.stdout)
                print(e.stderr)
                continue

            expected_file = out_dir / f"hybrid_{traj}_{scenario}_trajectory.png"
            if expected_file.exists():
                print(f"[SUCCESS] Exported plot to {expected_file}")
            else:
                print(f"[FAILURE] Expected plot {expected_file} was not generated.")


if __name__ == "__main__":
    main()
