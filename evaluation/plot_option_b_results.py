"""
Generate Option B paper plots from adaptive safety-filter logs.

Example:
    python evaluation/plot_option_b_results.py \
        --log Output/Logs/adaptive_safety_filter_node_20260524_120000.log
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

try:
    from evaluation.safety_filter_metrics import (
        intervention_segments,
        load_adaptive_safety_log,
        summarize_log,
        write_summary,
    )
except ImportError:
    from safety_filter_metrics import (
        intervention_segments,
        load_adaptive_safety_log,
        summarize_log,
        write_summary,
    )


MODE_COLORS = {
    "LQR": "#1b9e77",
    "ADAPTIVE_MPC_FILTER": "#d95f02",
    "BACKUP": "#d62728",
    "FAILED": "#6a3d9a",
}


def plot_all(log_path: str | Path, out_dir: str | Path, dt: float = 0.05) -> None:
    rows = load_adaptive_safety_log(log_path)
    if not rows:
        raise ValueError(f"No rows found in {log_path}")

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    data = intervention_segments(rows)

    _plot_intervention_map(data, out / "intervention_map.png")
    _plot_solver_time(data, out / "solver_time.png")
    _plot_risk_margin(data, out / "risk_and_margin.png")
    _plot_commands(data, out / "commands_and_adaptation.png")

    summary = summarize_log(log_path, dt=dt)
    write_summary(out / "summary.json", summary)
    _write_markdown_summary(out / "summary.md", summary, log_path)


def _plot_intervention_map(data, out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7.0, 5.5), dpi=160)
    x = data["x"]
    y = data["y"]
    mode = data["mode"]

    for i in range(max(len(x) - 1, 0)):
        color = MODE_COLORS.get(str(mode[i]), "#555555")
        ax.plot(x[i : i + 2], y[i : i + 2], color=color, linewidth=2.0)

    for label, color in MODE_COLORS.items():
        if np.any(mode == label):
            ax.plot([], [], color=color, linewidth=3.0, label=label)

    ax.set_title("Intervention Map")
    ax.set_xlabel("x position (m)")
    ax.set_ylabel("y position (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _plot_solver_time(data, out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7.0, 4.0), dpi=160)
    tick = data["tick"]
    solver = data["solver_ms"]
    mode = data["mode"]

    ax.plot(tick, solver, color="#333333", linewidth=1.5, label="solver time")
    if np.any(mode == "ADAPTIVE_MPC_FILTER"):
        ax.scatter(
            tick[mode == "ADAPTIVE_MPC_FILTER"],
            solver[mode == "ADAPTIVE_MPC_FILTER"],
            s=12,
            color=MODE_COLORS["ADAPTIVE_MPC_FILTER"],
            label="MPC filter active",
            zorder=3,
        )
    ax.set_title("Solver Time Profile")
    ax.set_xlabel("control tick")
    ax.set_ylabel("time (ms)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _plot_risk_margin(data, out_path: Path) -> None:
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.0, 5.0), dpi=160, sharex=True)
    tick = data["tick"]

    ax1.plot(tick, data["risk"], color="#d95f02", linewidth=1.5)
    ax1.set_ylabel("risk score")
    ax1.set_ylim(-0.05, 1.05)
    ax1.grid(True, alpha=0.25)

    ax2.plot(tick, data["margin"], color="#1b9e77", linewidth=1.5)
    ax2.axhline(0.0, color="#d62728", linestyle="--", linewidth=1.0)
    ax2.set_xlabel("control tick")
    ax2.set_ylabel("safety margin (m)")
    ax2.grid(True, alpha=0.25)

    fig.suptitle("Risk Trigger and Safety Margin")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _plot_commands(data, out_path: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(7.0, 5.0), dpi=160, sharex=True)
    tick = data["tick"]

    axes[0].plot(tick, data["v"], color="#1f77b4", label="v command")
    axes[0].plot(tick, data["theta_v"], color="#4c78a8", linestyle="--", label="theta_v")
    axes[0].set_ylabel("linear / scale")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="best", fontsize=8)

    axes[1].plot(tick, data["omega"], color="#ff7f0e", label="omega command")
    axes[1].plot(
        tick,
        data["theta_omega"],
        color="#f58518",
        linestyle="--",
        label="theta_omega",
    )
    axes[1].set_xlabel("control tick")
    axes[1].set_ylabel("angular / scale")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(loc="best", fontsize=8)

    fig.suptitle("Commands and LMS Adaptation")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _write_markdown_summary(out_path: Path, summary, log_path) -> None:
    lines = [
        "# Option B Run Summary",
        "",
        f"Log: `{Path(log_path)}`",
        "",
        "| Metric | Value |",
        "|---|---:|",
    ]
    for key, value in sorted(summary.items()):
        lines.append(f"| {key} | {value:.6g} |")
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot Option B ASF results.")
    parser.add_argument("--log", required=True, help="Path to ASF run log")
    parser.add_argument(
        "--out",
        default="Output/Plots/AdaptiveSafetyFilter",
        help="Output folder for plots and summaries",
    )
    parser.add_argument("--dt", type=float, default=0.05, help="Controller dt in seconds")
    args = parser.parse_args()
    plot_all(args.log, args.out, dt=args.dt)
    print(f"Wrote Option B plots and summaries to {args.out}")


if __name__ == "__main__":
    main()
