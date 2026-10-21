"""Generate diagnostic figures from the MuJoCo and RVO2 benchmark traces.

The script never invents missing sweep data.  Figures 1 and 2 use the trace
files written by ``run_mujoco_monte_carlo.py``.  Figures 3 and 4 use optional
JSON sweep records and are marked as pending when those records are absent.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


MODE_LABELS = {
    "1_nominal_lqr": "Nominal LQR",
    "2_async_nmpc": "Async NMPC",
    "3_convex_blend": "Convex blend",
    "4_hard_switch": "Hard switch",
    "5_ra_psf_ours": "RA-PSF",
}


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def load_trace(trace_dir: Path, mode: str, seed: int) -> list[dict[str, Any]]:
    path = trace_dir / f"{mode}_seed_{seed}.json"
    if not path.exists():
        raise FileNotFoundError(f"Missing trace file: {path}")
    return load_json(path)


def values(trace: list[dict[str, Any]], key: str) -> np.ndarray:
    return np.asarray([float(row[key]) for row in trace], dtype=float)


def command_jerk(trace: list[dict[str, Any]], dt: float) -> tuple[np.ndarray, np.ndarray]:
    commands = np.column_stack(
        [values(trace, "command_v"), values(trace, "command_omega")]
    )
    if len(commands) < 3:
        return np.zeros(len(commands)), np.zeros(len(commands))
    acceleration = np.diff(commands, axis=0) / max(dt, 1.0e-9)
    jerk = np.diff(acceleration, axis=0) / max(dt, 1.0e-9)
    padded = np.concatenate([[0.0, 0.0], jerk[:, 1]])
    return np.arange(len(trace), dtype=float) * dt, padded


def save_figure(figure: plt.Figure, output: Path) -> None:
    figure.savefig(output.with_suffix(".png"), dpi=300, bbox_inches="tight")
    figure.savefig(output.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(figure)


def plot_latency(
    traces: dict[str, list[dict[str, Any]]], output: Path, configured_delay_ms: float
) -> None:
    groups: dict[str, np.ndarray] = {
        "Configured\nNMPC delay": np.full(50, configured_delay_ms, dtype=float),
    }
    nmpc = values(traces["2_async_nmpc"], "latency_ms")
    ra = values(traces["5_ra_psf_ours"], "latency_ms")
    positive_ra = ra[ra > 0.0]
    groups["Async NMPC\nmeasured"] = nmpc[nmpc > 0.0]
    groups["RA-PSF\nfirst active"] = positive_ra[:1] if len(positive_ra) else np.array([0.0])
    groups["RA-PSF\nwarm solves"] = positive_ra[1:] if len(positive_ra) > 1 else positive_ra
    groups["RA-PSF\nevent effective"] = ra

    figure, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True)
    labels = list(groups)
    data = [groups[label] for label in labels]
    axes[0].violinplot(data, showmeans=True, showextrema=True)
    axes[0].set_xticks(np.arange(1, len(labels) + 1), labels, rotation=25, ha="right")
    axes[0].set_ylabel("Latency (ms)")
    axes[0].set_title("Latency distributions")
    axes[0].set_yscale("symlog", linthresh=1.0)
    axes[0].grid(True, alpha=0.25)

    for label, sample in groups.items():
        sample = np.sort(sample[np.isfinite(sample)])
        if not len(sample):
            continue
        probability = np.arange(1, len(sample) + 1, dtype=float) / len(sample)
        axes[1].step(sample, probability, where="post", label=label.replace("\n", " "))
    axes[1].set_xlabel("Latency (ms)")
    axes[1].set_ylabel("Empirical CDF")
    axes[1].set_title("Empirical latency CDF")
    axes[1].set_xscale("symlog", linthresh=1.0)
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(fontsize=8)
    figure.suptitle("RA-PSF timing evidence for the selected trace seed")
    save_figure(figure, output)


def plot_intervention_timeline(
    traces: dict[str, list[dict[str, Any]]], output: Path, dt: float
) -> None:
    ra = traces["5_ra_psf_ours"]
    time_s = values(ra, "time_s")
    cvar = values(ra, "cvar_risk")
    clearance = values(ra, "minimum_clearance")
    intervention = values(ra, "intervention_norm")
    _, ra_jerk = command_jerk(ra, dt)
    _, blend_jerk = command_jerk(traces["3_convex_blend"], dt)
    _, switch_jerk = command_jerk(traces["4_hard_switch"], dt)

    figure, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
    axes[0].plot(time_s, cvar, label="CVaR risk", color="tab:red")
    axes[0].axhline(0.0, color="black", linewidth=0.8, label="entry")
    axes[0].axhline(-0.05, color="tab:blue", linewidth=0.8, linestyle="--", label="exit")
    axes[0].set_ylabel("Risk")
    axes[0].legend(loc="upper right", ncol=3, fontsize=8)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(time_s, clearance, color="tab:green")
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].set_ylabel("Clearance (m)")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(time_s, intervention, color="tab:purple")
    axes[2].set_ylabel("||u - uLQR||")
    axes[2].grid(True, alpha=0.25)

    axes[3].plot(time_s, ra_jerk, label="RA-PSF")
    axes[3].plot(time_s, blend_jerk, label="Convex blend", alpha=0.75)
    axes[3].plot(time_s, switch_jerk, label="Hard switch", alpha=0.75)
    axes[3].set_ylabel("Angular jerk")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="upper right", fontsize=8)
    axes[3].grid(True, alpha=0.25)
    figure.suptitle("RA-PSF intervention timeline for the selected trace seed")
    save_figure(figure, output)


def plot_phase_diagram(data: list[dict[str, Any]], output: Path) -> None:
    figure, axis = plt.subplots(figsize=(8, 5.5), constrained_layout=True)
    collisions = np.asarray([float(row["collision_rate"]) for row in data], dtype=float)
    vmax = max(1.0e-9, float(np.max(collisions)))
    markers = ("o", "s", "^", "D", "P")
    scatter = None
    for marker, mode in zip(markers, sorted({str(row.get("mode", "")) for row in data})):
        group = [row for row in data if str(row.get("mode", "")) == mode]
        speeds = np.asarray([float(row["speed"]) for row in group], dtype=float)
        latencies = np.asarray([float(row["latency_ms"]) for row in group], dtype=float)
        group_collisions = np.asarray(
            [float(row["collision_rate"]) for row in group], dtype=float
        )
        scatter = axis.scatter(
            speeds,
            latencies,
            c=group_collisions,
            cmap="magma_r",
            vmin=0.0,
            vmax=vmax,
            s=80,
            marker=marker,
            edgecolor="black",
            label=MODE_LABELS.get(mode, mode),
        )
    axis.set_xlabel("Target speed (m/s)")
    axis.set_ylabel("Measured mean latency (ms)")
    axis.set_title("Measured speed-latency phase points")
    axis.grid(True, alpha=0.25)
    if scatter is not None:
        figure.colorbar(scatter, ax=axis, label="Collision rate")
    axis.legend(fontsize=8, loc="best")
    save_figure(figure, output)


def plot_slip_ablation(data: list[dict[str, Any]], output: Path) -> None:
    variants = sorted({str(row["variant"]) for row in data})
    frictions = sorted({float(row["friction"]) for row in data})
    collision = {
        variant: [
            float(next(row["collision_rate"] for row in data if row["variant"] == variant and float(row["friction"]) == friction))
            for friction in frictions
        ]
        for variant in variants
    }
    error = {
        variant: [
            float(next(row["rms_cross_track_error"] for row in data if row["variant"] == variant and float(row["friction"]) == friction))
            for friction in frictions
        ]
        for variant in variants
    }
    figure, axes = plt.subplots(1, 2, figsize=(11, 4.5), constrained_layout=True)
    for variant in variants:
        axes[0].plot(frictions, collision[variant], marker="o", label=variant)
        axes[1].plot(frictions, error[variant], marker="o", label=variant)
    axes[0].set_xlabel("Friction coefficient")
    axes[0].set_ylabel("Collision rate")
    axes[1].set_xlabel("Friction coefficient")
    axes[1].set_ylabel("RMS cross-track error (m)")
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=8)
    figure.suptitle("Slip and friction ablation")
    save_figure(figure, output)


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate RA-PSF publication diagnostics")
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("Output/Plots/Paper"))
    parser.add_argument("--trace-seed", type=int, default=None)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--configured-delay-ms", type=float, default=180.0)
    parser.add_argument("--phase-data", type=Path)
    parser.add_argument("--slip-data", type=Path)
    args = parser.parse_args()

    summary = load_json(args.summary)
    seed = int(args.trace_seed if args.trace_seed is not None else summary["metadata"]["trace_seed"])
    trace_dir = args.summary.parent / "traces"
    modes = ("2_async_nmpc", "3_convex_blend", "4_hard_switch", "5_ra_psf_ours")
    traces = {mode: load_trace(trace_dir, mode, seed) for mode in modes}
    args.output.mkdir(parents=True, exist_ok=True)

    plot_latency(traces, args.output / "fig1_latency_cdf_violin", args.configured_delay_ms)
    plot_intervention_timeline(
        traces, args.output / "fig2_intervention_timeline", args.dt
    )

    phase_path = args.phase_data or (args.summary.parent / "phase_sweep.json")
    if phase_path.exists():
        plot_phase_diagram(load_json(phase_path), args.output / "fig3_speed_latency_phase_diagram")
    slip_path = args.slip_data or (args.summary.parent / "slip_ablation.json")
    if slip_path.exists():
        plot_slip_ablation(load_json(slip_path), args.output / "fig4_slip_robustness_ablation")

    print(f"Generated figures 1 and 2 under {args.output}")
    if not phase_path.exists():
        print(f"Figure 3 pending sweep data: {phase_path}")
    if not slip_path.exists():
        print(f"Figure 4 pending ablation data: {slip_path}")


if __name__ == "__main__":
    main()
