"""
Metrics for adaptive safety-filter Gazebo runs.

The adaptive_safety_filter_node writes CSV-like rows into
Output/Logs/adaptive_safety_filter_node_*.log. This module parses those logs
and computes the core Option B paper metrics:

    - tracking RMSE
    - safety margin statistics
    - safety-filter intervention rate
    - backup usage rate
    - solver-time statistics
    - command jerk/smoothness
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, Iterable, List

import numpy as np


NUMERIC_COLUMNS = {
    "Tick",
    "X",
    "Y",
    "Theta",
    "Error",
    "V",
    "Omega",
    "Risk",
    "MinMargin",
    "SolverMs",
    "ThetaV",
    "ThetaOmega",
}


def load_adaptive_safety_log(path: str | Path) -> List[Dict[str, object]]:
    """Parse an adaptive_safety_filter_node log into row dictionaries."""
    payload_lines = []
    with Path(path).open("r", encoding="utf-8") as handle:
        for raw in handle:
            raw = raw.strip()
            if not raw:
                continue
            payload = raw.split(" INFO ", 1)[1] if " INFO " in raw else raw
            if payload.startswith("Tick,") or payload[:1].isdigit():
                payload_lines.append(payload)

    if not payload_lines:
        return []

    rows: List[Dict[str, object]] = []
    for row in csv.DictReader(payload_lines):
        parsed: Dict[str, object] = {}
        for key, value in row.items():
            if key in NUMERIC_COLUMNS:
                parsed[key] = _to_float(value)
            else:
                parsed[key] = value
        rows.append(parsed)
    return rows


def summarize_log(path: str | Path, dt: float = 0.05) -> Dict[str, float]:
    """Return scalar metrics for one adaptive safety-filter run."""
    rows = load_adaptive_safety_log(path)
    if not rows:
        raise ValueError(f"No adaptive safety-filter rows found in {path}")

    errors = _array(rows, "Error")
    risk = _array(rows, "Risk")
    margins = _array(rows, "MinMargin")
    solver_ms = _array(rows, "SolverMs")
    controls = np.column_stack([_array(rows, "V"), _array(rows, "Omega")])
    modes = [str(row.get("Mode", "")) for row in rows]
    jerk = compute_jerk_metrics(controls, dt)

    summary = {
        "samples": float(len(rows)),
        "tracking_rmse_m": float(np.sqrt(np.mean(errors**2))),
        "tracking_mean_m": float(np.mean(errors)),
        "tracking_max_m": float(np.max(errors)),
        "risk_mean": float(np.mean(risk)),
        "risk_max": float(np.max(risk)),
        "min_safety_margin_m": float(np.min(margins)),
        "safety_violation_rate": float(np.mean(margins < 0.0)),
        "lqr_rate": _mode_rate(modes, "LQR"),
        "mpc_filter_rate": _mode_rate(modes, "ADAPTIVE_MPC_FILTER"),
        "backup_rate": _mode_rate(modes, "BACKUP"),
        "failed_rate": _mode_rate(modes, "FAILED"),
        "solver_mean_ms": float(np.mean(solver_ms)),
        "solver_p95_ms": float(np.percentile(solver_ms, 95.0)),
        "solver_max_ms": float(np.max(solver_ms)),
    }
    summary.update(jerk)
    return summary


def compute_jerk_metrics(controls: np.ndarray, dt: float) -> Dict[str, float]:
    """Compute command smoothness from velocity command changes."""
    controls = np.asarray(controls, dtype=float)
    if controls.ndim != 2 or controls.shape[0] < 3:
        return {
            "linear_jerk_rms": 0.0,
            "angular_jerk_rms": 0.0,
            "command_smoothness_cost": 0.0,
        }

    dt = max(float(dt), 1e-9)
    acceleration = np.diff(controls, axis=0) / dt
    jerk = np.diff(acceleration, axis=0) / dt
    du = np.diff(controls, axis=0)
    return {
        "linear_jerk_rms": float(np.sqrt(np.mean(jerk[:, 0] ** 2))),
        "angular_jerk_rms": float(np.sqrt(np.mean(jerk[:, 1] ** 2))),
        "command_smoothness_cost": float(np.sum(du[:, 0] ** 2 + du[:, 1] ** 2)),
    }


def intervention_segments(rows: Iterable[Dict[str, object]]) -> Dict[str, np.ndarray]:
    """Return arrays used by plotting scripts."""
    rows = list(rows)
    return {
        "tick": _array(rows, "Tick"),
        "x": _array(rows, "X"),
        "y": _array(rows, "Y"),
        "error": _array(rows, "Error"),
        "v": _array(rows, "V"),
        "omega": _array(rows, "Omega"),
        "risk": _array(rows, "Risk"),
        "margin": _array(rows, "MinMargin"),
        "solver_ms": _array(rows, "SolverMs"),
        "theta_v": _array(rows, "ThetaV"),
        "theta_omega": _array(rows, "ThetaOmega"),
        "mode": np.array([str(row.get("Mode", "")) for row in rows], dtype=object),
    }


def write_summary(path: str | Path, summary: Dict[str, float]) -> None:
    """Write summary metrics as pretty JSON."""
    Path(path).write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")


def _array(rows: List[Dict[str, object]], key: str) -> np.ndarray:
    return np.array([float(row.get(key, 0.0)) for row in rows], dtype=float)


def _mode_rate(modes: List[str], mode: str) -> float:
    if not modes:
        return 0.0
    return float(sum(item == mode for item in modes) / len(modes))


def _to_float(value: object) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def main() -> None:
    parser = argparse.ArgumentParser(description="Summarize an ASF Gazebo log.")
    parser.add_argument("log", help="Path to adaptive_safety_filter_node_*.log")
    parser.add_argument("--dt", type=float, default=0.05, help="Controller dt in seconds")
    parser.add_argument("--out", help="Optional JSON output path")
    args = parser.parse_args()

    summary = summarize_log(args.log, dt=args.dt)
    text = json.dumps(summary, indent=2, sort_keys=True)
    print(text)
    if args.out:
        Path(args.out).write_text(text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
