"""Run the frozen deterministic MuJoCo plant recoverability sweep.

The analytical recoverability grid in :mod:`benchmarks.validate_recoverability_gap`
is deliberately kept as the model reference. This benchmark measures the same
command staircase in the fixed MuJoCo plant and reports the plant evidence in a
separate primary-metrics section. The held-out parameter range is immutable and
is never used to fit a correction, choose a threshold, or change the plant
configuration.

``obstacle_distance`` is a virtual forward stopping budget. The MuJoCo scene
has no obstacles, only floor contact. Comparing the budget with maximum
sampled forward travel is a stopping-boundary proxy, not a collision outcome.
The v2 measurement protocol is frozen in ``STOPPING_MEASUREMENT_PROTOCOL``:
maximum control-sampled forward travel, zero command, hidden actuator speeds
and all body velocity DOFs below their tolerances for 0.5 seconds of consecutive
quiet endpoint samples. Unsupported trials retain their raw count and reason,
with no safe/unsafe classification. No fit or tolerance depends on the split.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import platform
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from importlib.metadata import version
from itertools import islice
from pathlib import Path
from types import MappingProxyType
from typing import Any, Iterable, Mapping

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PROJECT_ROOT / "src" / "hybrid_controller"
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from benchmarks.validate_recoverability_gap import (
    STOPPING_MEASUREMENT_PROTOCOL,
    PlantStoppingMeasurement,
    RecoverabilityCase,
    evaluate_case,
    measure_mujoco_stop,
    stopping_plant_config,
)
from hybrid_controller.models.footprint import MUJOCO_ROBOT_RADIUS


FIXED_DT = 0.05
FIXED_FRICTION = 1.0
FIXED_PHYSICS_DT = STOPPING_MEASUREMENT_PROTOCOL.physics_dt
FIXED_VELOCITY_TOLERANCE = STOPPING_MEASUREMENT_PROTOCOL.velocity_tolerance
FIXED_MAX_STEPS = STOPPING_MEASUREMENT_PROTOCOL.max_steps
DISTANCE_COMPARISON_TOLERANCE = 1.0e-12


@dataclass(frozen=True)
class ParameterGrid:
    """Immutable parameter values for one evaluation split."""

    v0_values: tuple[float, ...]
    command_gap_values: tuple[float, ...]
    obstacle_distance_values: tuple[float, ...]
    tau_values: tuple[float, ...]
    delta_u_values: tuple[float, ...]

    def __post_init__(self) -> None:
        names_and_values = (
            ("v0_values", self.v0_values),
            ("command_gap_values", self.command_gap_values),
            ("obstacle_distance_values", self.obstacle_distance_values),
            ("tau_values", self.tau_values),
            ("delta_u_values", self.delta_u_values),
        )
        for name, values in names_and_values:
            if not values:
                raise ValueError(f"{name} must not be empty")
            if not isinstance(values, tuple):
                raise TypeError(f"{name} must be a tuple")
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"{name} must contain finite values")
        if any(value < 0.0 for value in self.v0_values):
            raise ValueError("v0_values must be nonnegative")
        if any(value < 0.0 for value in self.command_gap_values):
            raise ValueError("command_gap_values must be nonnegative")
        if any(value < 0.0 for value in self.obstacle_distance_values):
            raise ValueError("obstacle_distance_values must be nonnegative")
        if any(value <= 0.0 for value in self.tau_values):
            raise ValueError("tau_values must be positive")
        if any(value <= 0.0 for value in self.delta_u_values):
            raise ValueError("delta_u_values must be positive")

    @property
    def case_count(self) -> int:
        """Return the Cartesian-product size in the stable sweep order."""
        return (
            len(self.v0_values)
            * len(self.command_gap_values)
            * len(self.obstacle_distance_values)
            * len(self.tau_values)
            * len(self.delta_u_values)
        )

    def as_dict(self) -> dict[str, list[float]]:
        """Return JSON-safe copies of the frozen parameter values."""
        return {
            "v0_values": list(self.v0_values),
            "command_gap_values": list(self.command_gap_values),
            "obstacle_distance_values": list(self.obstacle_distance_values),
            "tau_values": list(self.tau_values),
            "delta_u_values": list(self.delta_u_values),
        }


# These values intentionally mirror the completed analytical validation grid.
# Keep them as tuples and expose them through a read-only mapping so a caller
# cannot accidentally tune the held-out range in-process.
PRIMARY_PARAMETER_GRID = ParameterGrid(
    v0_values=(0.25, 0.75, 1.5, 2.5),
    command_gap_values=(0.0, 0.5, 1.0, 1.5),
    obstacle_distance_values=(0.1, 0.5, 1.0, 1.5),
    tau_values=(0.06, 0.10, 0.15, 0.20),
    delta_u_values=(0.05, 0.10, 0.20),
)
HELD_OUT_PARAMETER_GRID = ParameterGrid(
    v0_values=(0.4, 1.1, 2.0),
    command_gap_values=(0.25, 0.75, 1.25),
    obstacle_distance_values=(0.25, 0.75, 1.25),
    tau_values=(0.08, 0.13, 0.18),
    delta_u_values=(0.075, 0.15),
)
FROZEN_PRIMARY_PARAMETER_GRID = PRIMARY_PARAMETER_GRID
FROZEN_HELD_OUT_PARAMETER_GRID = HELD_OUT_PARAMETER_GRID
PARAMETER_GRIDS: Mapping[str, ParameterGrid] = MappingProxyType(
    {
        "primary": PRIMARY_PARAMETER_GRID,
        "held-out": HELD_OUT_PARAMETER_GRID,
    }
)


@dataclass(frozen=True)
class ControlledSweepCase:
    """A virtual-boundary proxy and raw plant result, never contact evidence."""

    split: str
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
    plant_distance: float | None
    virtual_boundary_margin_m: float | None
    state_only_minus_plant: float | None
    continuous_minus_plant: float | None
    discrete_minus_plant: float | None
    plant_minus_continuous: float | None
    plant_minus_discrete: float | None
    state_only_safe: bool
    continuous_safe: bool
    discrete_safe: bool
    plant_proxy_safe: bool | None
    measurement: PlantStoppingMeasurement
    outcome_kind: str = "virtual_stopping_boundary_proxy"
    obstacle_contact_evidence: str = "not_measured_empty_obstacle_scene"


def _canonical_split(split: str) -> str:
    if split == "held_out":
        split = "held-out"
    if split not in PARAMETER_GRIDS:
        raise ValueError("split must be 'primary' or 'held-out'")
    return split


def get_parameter_grid(split: str) -> ParameterGrid:
    """Return the immutable grid for ``split``."""
    return PARAMETER_GRIDS[_canonical_split(split)]


def _iter_analytical_cases(split: str) -> Iterable[RecoverabilityCase]:
    """Yield analytical cases in the same stable order as ``run_grid``."""
    grid = get_parameter_grid(split)
    for tau_v in grid.tau_values:
        for delta_u in grid.delta_u_values:
            for v0 in grid.v0_values:
                for command_gap in grid.command_gap_values:
                    for obstacle_distance in grid.obstacle_distance_values:
                        yield evaluate_case(
                            v0=v0,
                            command_gap=command_gap,
                            obstacle_distance=obstacle_distance,
                            tau_v=tau_v,
                            delta_u=delta_u,
                            dt=FIXED_DT,
                        )


def evaluate_controlled_case(
    analytical_case: RecoverabilityCase,
    *,
    split: str,
) -> ControlledSweepCase:
    """Measure one case in the fixed plant without any parameter fitting."""

    canonical_split = _canonical_split(split)
    grid = get_parameter_grid(canonical_split)
    if (
        analytical_case.v0 not in grid.v0_values
        or analytical_case.command_gap not in grid.command_gap_values
        or analytical_case.obstacle_distance not in grid.obstacle_distance_values
        or analytical_case.tau_v not in grid.tau_values
        or analytical_case.delta_u not in grid.delta_u_values
    ):
        raise ValueError("analytical case is outside the frozen split parameter grid")
    if analytical_case.dt != FIXED_DT:
        raise ValueError("controlled sweep requires the fixed dt contract")
    if analytical_case != evaluate_case(
        v0=analytical_case.v0,
        command_gap=analytical_case.command_gap,
        obstacle_distance=analytical_case.obstacle_distance,
        tau_v=analytical_case.tau_v,
        delta_u=analytical_case.delta_u,
        dt=FIXED_DT,
    ):
        raise ValueError("analytical case does not match the fixed model contract")
    measurement = measure_mujoco_stop(
        analytical_case.v0,
        analytical_case.u0,
        analytical_case.delta_u,
        analytical_case.dt,
        analytical_case.tau_v,
        friction=FIXED_FRICTION,
        velocity_tolerance=FIXED_VELOCITY_TOLERANCE,
        angular_velocity_tolerance=STOPPING_MEASUREMENT_PROTOCOL.angular_velocity_tolerance,
        settling_window_s=STOPPING_MEASUREMENT_PROTOCOL.settling_window_s,
        max_steps=FIXED_MAX_STEPS,
    )
    plant_distance = (
        measurement.maximum_sampled_forward_travel_m
        if measurement.status == "settled" else None
    )
    if measurement.status == "settled" and (
        plant_distance is None or not math.isfinite(plant_distance) or plant_distance < 0.0
    ):
        raise RuntimeError("settled measurement must have finite nonnegative peak travel")
    margin = (
        analytical_case.obstacle_distance - plant_distance
        if plant_distance is not None else None
    )
    return ControlledSweepCase(
        split=canonical_split,
        v0=analytical_case.v0,
        command_gap=analytical_case.command_gap,
        obstacle_distance=analytical_case.obstacle_distance,
        tau_v=analytical_case.tau_v,
        delta_u=analytical_case.delta_u,
        dt=analytical_case.dt,
        rho_v=analytical_case.rho_v,
        u0=analytical_case.u0,
        state_only_distance=analytical_case.state_only_distance,
        continuous_distance=analytical_case.continuous_distance,
        discrete_distance=analytical_case.discrete_distance,
        plant_distance=plant_distance,
        virtual_boundary_margin_m=margin,
        state_only_minus_plant=(
            analytical_case.state_only_distance - plant_distance
            if plant_distance is not None else None
        ),
        continuous_minus_plant=(
            analytical_case.continuous_distance - plant_distance
            if plant_distance is not None else None
        ),
        discrete_minus_plant=(
            analytical_case.discrete_distance - plant_distance
            if plant_distance is not None else None
        ),
        plant_minus_continuous=(
            plant_distance - analytical_case.continuous_distance
            if plant_distance is not None else None
        ),
        plant_minus_discrete=(
            plant_distance - analytical_case.discrete_distance
            if plant_distance is not None else None
        ),
        state_only_safe=analytical_case.state_only_safe,
        continuous_safe=analytical_case.continuous_safe,
        discrete_safe=analytical_case.discrete_safe,
        plant_proxy_safe=bool(margin >= 0.0) if margin is not None else None,
        measurement=measurement,
    )


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        raise ValueError("at least one value is required")
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(ordered[lower])
    weight = position - lower
    return float(ordered[lower] * (1.0 - weight) + ordered[upper] * weight)


def summarize_primary_cases(
    cases: Iterable[ControlledSweepCase],
) -> dict[str, Any]:
    """Retain raw denominators; compare only valid settled proxy outcomes."""

    rows = list(cases)
    if not rows:
        raise ValueError("at least one controlled sweep case is required")
    valid = [row for row in rows if row.measurement.status == "settled"]
    count = len(valid)
    plant_distances = [row.plant_distance for row in valid if row.plant_distance is not None]
    plant_minus_discrete = [row.plant_minus_discrete for row in valid if row.plant_minus_discrete is not None]
    plant_minus_continuous = [row.plant_minus_continuous for row in valid if row.plant_minus_continuous is not None]
    disagreement_vs_discrete = sum(
        row.plant_proxy_safe != row.discrete_safe for row in valid
    )
    disagreement_vs_continuous = sum(
        row.plant_proxy_safe != row.continuous_safe for row in valid
    )
    return {
        "case_count": len(rows),
        "plant_measurement_attempt_count": len(rows),
        "plant_measurement_count": count,
        "invalid_case_count": sum(row.measurement.status == "invalid" for row in rows),
        "not_settled_case_count": sum(row.measurement.status == "not_settled" for row in rows),
        "unsupported_case_count": len(rows) - count,
        "valid_measurement_fraction": count / len(rows),
        "plant_proxy_safe_case_count": sum(row.plant_proxy_safe is True for row in valid),
        "plant_proxy_unsafe_case_count": sum(row.plant_proxy_safe is False for row in valid),
        "plant_proxy_unknown_case_count": len(rows) - count,
        "distance_summary_denominator": count,
        "boundary_comparison_denominator": count,
        "plant_distance_mean_m": sum(plant_distances) / count if count else None,
        "plant_distance_p95_m": _percentile(plant_distances, 95.0) if count else None,
        "plant_distance_max_m": max(plant_distances) if count else None,
        "plant_minus_discrete_mean_m": sum(plant_minus_discrete) / count if count else None,
        "plant_minus_discrete_min_m": min(plant_minus_discrete) if count else None,
        "plant_minus_discrete_max_m": max(plant_minus_discrete) if count else None,
        "discrete_underpredicts_plant_count": sum(
            value > DISTANCE_COMPARISON_TOLERANCE for value in plant_minus_discrete
        ),
        "continuous_underpredicts_plant_count": sum(
            value > DISTANCE_COMPARISON_TOLERANCE for value in plant_minus_continuous
        ),
        "plant_vs_discrete_safe_boundary_disagreement_count": disagreement_vs_discrete,
        "plant_vs_discrete_safe_boundary_disagreement_rate": (
            disagreement_vs_discrete / count if count else None
        ),
        "plant_vs_continuous_safe_boundary_disagreement_count": disagreement_vs_continuous,
        "plant_vs_continuous_safe_boundary_disagreement_rate": (
            disagreement_vs_continuous / count if count else None
        ),
        "discrete_safe_but_plant_proxy_unsafe_count": sum(
            row.discrete_safe and row.plant_proxy_safe is False for row in valid
        ),
        "continuous_safe_but_plant_proxy_unsafe_count": sum(
            row.continuous_safe and row.plant_proxy_safe is False for row in valid
        ),
    }


def summarize_logged_analysis(
    cases: Iterable[ControlledSweepCase],
) -> dict[str, object]:
    """Return secondary case-log analysis kept separate from primary metrics."""

    rows = list(cases)
    if not rows:
        raise ValueError("at least one controlled sweep case is required")
    valid = [row for row in rows if row.measurement.status == "settled"]
    state_only_corrections = [row.state_only_minus_plant for row in valid if row.state_only_minus_plant is not None]
    return {
        "status": "secondary",
        "source": "deterministic per-case sweep records",
        "tuning_used": False,
        "raw_case_count": len(rows),
        "valid_case_denominator": len(valid),
        "unsupported_case_count": len(rows) - len(valid),
        "state_only_vs_plant_proxy_boundary_disagreement_count": sum(
            row.plant_proxy_safe != row.state_only_safe for row in valid
        ),
        "state_only_minus_plant_mean_m": (
            sum(state_only_corrections) / len(valid) if valid else None
        ),
        "note": (
            "Descriptive virtual stopping-boundary analysis, not measured obstacle "
            "contacts. Unsupported cases are unknown, not safe. No held-out tuning."
        ),
    }


def _source_hashes() -> dict[str, str]:
    sources = (
        Path(__file__),
        PROJECT_ROOT / "benchmarks" / "validate_recoverability_gap.py",
        PROJECT_ROOT / "benchmarks" / "mujoco_scene.py",
        SOURCE_ROOT / "hybrid_controller" / "models" / "augmented_dynamics.py",
        SOURCE_ROOT / "hybrid_controller" / "models" / "footprint.py",
    )
    return {
        path.relative_to(PROJECT_ROOT).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sources
    }


def _json_hash(value: object) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _git_provenance(paths: Iterable[str]) -> dict[str, object]:
    try:
        revision = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=PROJECT_ROOT,
            text=True, capture_output=True, check=True, timeout=10,
        ).stdout.strip()
        status = subprocess.run(
            ["git", "status", "--porcelain", "--", *paths], cwd=PROJECT_ROOT,
            text=True, capture_output=True, check=True, timeout=10,
        ).stdout.splitlines()
        return {"revision": revision, "source_status": status}
    except (OSError, subprocess.SubprocessError) as error:
        return {"revision": None, "unavailable_reason": str(error)}


def _metadata(split: str, *, case_limit: int | None) -> dict[str, Any]:
    from benchmarks.mujoco_scene import build_scene_xml

    canonical_split = _canonical_split(split)
    complete_case_grid = case_limit is None
    grid = get_parameter_grid(canonical_split)
    configs = {str(tau): asdict(stopping_plant_config(FIXED_DT, tau)) for tau in grid.tau_values}
    scene_hashes = {}
    for tau, config in configs.items():
        xml = build_scene_xml(
            [], FIXED_FRICTION, config["physics_dt"],
            config["robot_half_length"], config["robot_half_width"],
            config["gravity_z"], config["robot_mass"],
        )
        scene_hashes[tau] = hashlib.sha256(xml.encode("utf-8")).hexdigest()
    source_hashes = _source_hashes()
    protocol = asdict(STOPPING_MEASUREMENT_PROTOCOL)
    return {
        "schema_version": 2,
        "benchmark": "controlled_recoverability_sweep",
        "evidence_role": "primary_controlled" if complete_case_grid else "preflight",
        "split": canonical_split,
        "parameter_grid": grid.as_dict(),
        "parameter_grid_sha256": _json_hash(grid.as_dict()),
        "full_case_count": grid.case_count,
        "case_limit": case_limit,
        "complete_case_grid": complete_case_grid,
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "dt": FIXED_DT,
        "plant_contract": {
            "plant": "MuJoCoPlant",
            "friction": FIXED_FRICTION,
            "config_by_tau_v": configs,
            "scene_xml_sha256_by_tau_v": scene_hashes,
            "obstacles": [],
            "floor_contact_enabled": True,
            "reset_state": "[0, 0, 0, case.v0, 0]; hidden actuator=[case.v0, 0]",
            "reset_height_m": 0.16,
            "command": "[max(0, case.u0 - k * case.delta_u), 0], k starts at 0",
            "pose_integrator": "mujoco_physics_substeps_after_exact_actuator_lag_update",
        },
        "measurement_protocol": {
            **protocol,
            "protocol_sha256": _json_hash(protocol),
            "sample_period_s": FIXED_DT,
            "velocity_tolerance_units": "m/s",
            "angular_velocity_tolerance_units": "rad/s",
            "sample_times": "reset and after each complete control interval",
            "distance": "max(0, sampled x(t) - x(0)), not terminal x or path length",
            "settling": "zero command; both hidden actuator states and norms of all body linear/angular DOFs below tolerances",
            "required_quiet_samples": math.ceil(protocol["settling_window_s"] / FIXED_DT) + 1,
            "window_rule": "consecutive quiet endpoint samples spanning at least settling_window_s",
            "invalidity_rule": "plant substep RuntimeError, non-finite states, or time discontinuity => invalid; max_steps without settling => not_settled",
            "unsupported_rule": "retain partial measurements but exclude from comparison; proxy outcome null",
            "continuous_time_bound": False,
            "post_window_tail_bound": None,
        },
        "proxy_contract": {
            "outcome_kind": "virtual_stopping_boundary_proxy",
            "obstacle_distance": "virtual forward boundary budget after footprint allowance",
            "robot_footprint": "MUJOCO_ROBOT_FOOTPRINT",
            "robot_radius_m": MUJOCO_ROBOT_RADIUS,
            "proxy_safe_rule": "settled and obstacle_distance >= maximum_sampled_forward_travel_m",
            "safe_boundary_threshold_m": 0.0,
            "distance_comparison_tolerance_m": DISTANCE_COMPARISON_TOLERANCE,
            "obstacle_contact_evidence": "not_measured_empty_obstacle_scene",
        },
        "provenance": {
            "python": sys.version,
            "python_executable": sys.executable,
            "platform": platform.platform(),
            "packages": {name: version(name) for name in ("numpy", "scipy", "mujoco")},
            "thread_environment": {name: os.environ.get(name) for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")},
            "command_line": list(sys.argv),
            "git": _git_provenance(source_hashes),
            "source_sha256": source_hashes,
        },
        "held_out_frozen": True,
        "gate0_acceptance": "not_established",
        "tuning": {
            "fit_split": None,
            "held_out_used_for_tuning": False,
            "thresholds_changed_for_split": False,
        },
    }


def run_controlled_sweep(
    split: str,
    *,
    max_cases: int | None = None,
) -> dict[str, Any]:
    """Run one frozen split and return the structured report."""

    canonical_split = _canonical_split(split)
    if max_cases is not None and (
        isinstance(max_cases, bool) or not isinstance(max_cases, int) or max_cases <= 0
    ):
        raise ValueError("max_cases must be a positive integer when provided")
    metadata = _metadata(canonical_split, case_limit=max_cases)
    analytical_cases = _iter_analytical_cases(canonical_split)
    if max_cases is not None:
        analytical_cases = islice(analytical_cases, max_cases)
    measured_cases = [
        evaluate_controlled_case(case, split=canonical_split)
        for case in analytical_cases
    ]
    if not measured_cases:
        raise ValueError("controlled sweep produced no cases")
    primary_metrics = summarize_primary_cases(measured_cases)
    metadata["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
    hashes_after = _source_hashes()
    metadata["provenance"]["source_sha256_after"] = hashes_after
    metadata["source_unchanged_during_run"] = hashes_after == metadata["provenance"]["source_sha256"]
    metadata["all_measurements_supported"] = primary_metrics["unsupported_case_count"] == 0
    metadata["run_status"] = (
        "source_changed_during_run" if not metadata["source_unchanged_during_run"]
        else "complete_with_unsupported_cases" if not metadata["all_measurements_supported"]
        else "complete"
    )
    return {
        "metadata": metadata,
        "primary_metrics": primary_metrics,
        "logged_analysis": summarize_logged_analysis(measured_cases),
        "cases": [asdict(case) for case in measured_cases],
    }


def _file_stem(split: str) -> str:
    return f"controlled_recoverability_sweep_{_canonical_split(split).replace('-', '_')}"


def write_sweep_outputs(
    report: Mapping[str, object],
    output: Path,
) -> tuple[Path, Path]:
    """Write new JSON/CSV artifacts; never overwrite an earlier experiment."""

    metadata = report.get("metadata")
    if not isinstance(metadata, Mapping):
        raise ValueError("report metadata is required")
    split = str(metadata.get("split", ""))
    stem = _file_stem(split)
    json_path = output / f"{stem}.json"
    csv_path = output / f"{stem}.csv"
    analysis_path = output / f"{stem}_logged_analysis.json"
    for path in (json_path, csv_path, analysis_path):
        if path.exists():
            raise FileExistsError(f"Experiment artifact already exists: {path}")
    raw_cases = report.get("cases")
    if not isinstance(raw_cases, list) or not raw_cases:
        raise ValueError("report cases are required")
    encoded = json.dumps(report, indent=2, allow_nan=False)
    rows = []
    for case in raw_cases:
        row = dict(case)
        measurement = row.pop("measurement")
        row.update({f"measurement_{key}": value for key, value in measurement.items()})
        rows.append(row)
    output.mkdir(parents=True, exist_ok=True)
    with json_path.open("x", encoding="utf-8") as handle:
        handle.write(encoded)
    with csv_path.open("x", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    with analysis_path.open("x", encoding="utf-8") as handle:
        json.dump(
            {
                "metadata": metadata,
                "logged_analysis": report.get("logged_analysis"),
            },
            handle,
            indent=2,
            allow_nan=False,
        )
    return json_path, csv_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the frozen controlled MuJoCo recoverability sweep"
    )
    parser.add_argument(
        "--split",
        choices=("primary", "held-out", "both"),
        default="primary",
        help="Run the primary grid, the frozen held-out grid, or both.",
    )
    parser.add_argument(
        "--max-cases",
        type=int,
        default=None,
        help="Optional deterministic prefix for a lightweight preflight; omit for evidence.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("Output/Plots/Paper/controlled_recoverability_sweep"),
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    splits = ("primary", "held-out") if args.split == "both" else (args.split,)
    exit_code = 0
    for split in splits:
        report = run_controlled_sweep(split, max_cases=args.max_cases)
        json_path, csv_path = write_sweep_outputs(report, args.output)
        if report["metadata"]["run_status"] != "complete":
            exit_code = 1
        print(
            json.dumps(
                {
                    "split": split,
                    "run_status": report["metadata"]["run_status"],
                    "primary_metrics": report["primary_metrics"],
                    "json": str(json_path),
                    "csv": str(csv_path),
                },
                indent=2,
            )
        )
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
