"""Generate Gate 1 figures and a human-validation report from ROS 2 telemetry.

The ROS 2 safety-filter node writes a CSV payload into a timestamped ``.log``
file.  The experiment runner may also emit the same rows as JSON or CSV.  This
module accepts all three forms so the post-processing step does not depend on
which artifact was copied out of a container.

The command is intentionally strict: missing files, malformed rows, non-finite
telemetry, and logged margin violations are reported clearly.  A run with
safety violations still receives plots and a Markdown report, but the command
returns a non-zero status so it can be used as a Gate 1 check.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import matplotlib

matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt
import numpy as np


class ValidationError(ValueError):
    """Raised when telemetry cannot support a trustworthy validation report."""


@dataclass(frozen=True)
class TelemetryRun:
    """Normalized telemetry and metadata for one experiment run."""

    rows: tuple[dict[str, Any], ...]
    metadata: dict[str, Any]
    sources: tuple[Path, ...]
    timebase: str


REQUIRED_FIELDS = (
    "tick",
    "x",
    "y",
    "theta",
    "error",
    "v",
    "omega",
    "mode",
    "risk",
    "min_margin",
    "solver_ms",
)

NUMERIC_FIELDS = (
    "tick",
    "time_s",
    "x",
    "y",
    "theta",
    "error",
    "v",
    "omega",
    "risk",
    "min_margin",
    "solver_ms",
    "theta_v",
    "theta_omega",
)

ROW_CONTAINER_KEYS = {
    "telemetry",
    "records",
    "samples",
    "rows",
    "data",
    "trace",
    "entries",
}

METADATA_CONTAINER_KEYS = {
    "metadata",
    "runmetadata",
    "session",
    "config",
    "parameters",
    "run",
    "summary",
    "metrics",
    "aggregate",
}


def _normalize_key(value: Any) -> str:
    return re.sub(r"[^a-z0-9]+", "", str(value).strip().lower())


FIELD_ALIASES: dict[str, tuple[str, ...]] = {
    "tick": ("tick", "step", "sample", "iteration"),
    "time_s": (
        "times",
        "time",
        "elapsed",
        "elapsed_s",
        "timestamp_s",
        "seconds",
    ),
    "x": ("x", "pos_x", "position_x", "x_m"),
    "y": ("y", "pos_y", "position_y", "y_m"),
    "theta": ("theta", "yaw", "heading", "theta_rad"),
    "error": (
        "error",
        "tracking_error",
        "tracking_error_m",
        "error_m",
        "dist_error",
    ),
    "v": ("v", "linear_velocity", "linear_x", "command_v", "velocity"),
    "omega": (
        "omega",
        "angular_velocity",
        "angular_z",
        "command_omega",
    ),
    "mode": ("mode", "controller_mode", "controller", "control_mode"),
    "risk": ("risk", "risk_score", "cvar_risk"),
    "min_margin": ("min_margin", "min_margin_m", "safety_margin", "margin"),
    "solver_ms": (
        "solver_ms",
        "solver_time_ms",
        "solver_latency_ms",
        "solve_time_ms",
        "solve_ms",
        "latency_ms",
    ),
    "theta_v": ("theta_v", "thetav", "parameter_v", "param_v"),
    "theta_omega": (
        "theta_omega",
        "thetaomega",
        "parameter_omega",
        "param_omega",
    ),
    "collision": (
        "collision",
        "collided",
        "is_collision",
        "collision_flag",
        "collision_event",
    ),
    "violation": (
        "violation",
        "safety_violation",
        "is_violation",
        "violation_flag",
    ),
    "collision_count": ("collision_count", "collisions", "collision_events"),
    "violation_count": (
        "violation_count",
        "safety_violation_count",
        "violations",
    ),
}

ALIAS_TO_FIELD = {
    _normalize_key(alias): field
    for field, aliases in FIELD_ALIASES.items()
    for alias in aliases
}


def _canonical_row(row: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(row, Mapping):
        raise ValidationError(f"Telemetry row must be an object, got {type(row).__name__}")
    normalized: dict[str, Any] = {}
    for key, value in row.items():
        canonical = ALIAS_TO_FIELD.get(_normalize_key(key), str(key).strip())
        if canonical in normalized:
            raise ValidationError(f"Duplicate telemetry field: {canonical}")
        normalized[canonical] = value
    return normalized


def _check_json_finite(value: Any, path: str = "JSON") -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise ValidationError(f"{path} contains non-finite value {value!r}")
    if isinstance(value, Mapping):
        for key, item in value.items():
            _check_json_finite(item, f"{path}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _check_json_finite(item, f"{path}[{index}]")


def _looks_like_row(value: Mapping[str, Any]) -> bool:
    fields = {_normalize_key(key) for key in value}
    required_aliases = {
        _normalize_key(alias)
        for field in REQUIRED_FIELDS
        for alias in FIELD_ALIASES[field]
    }
    return len(fields.intersection(required_aliases)) >= 3


def _metadata_from_json(payload: Any) -> dict[str, Any]:
    if not isinstance(payload, Mapping) or _looks_like_row(payload):
        return {}
    metadata: dict[str, Any] = {}
    for key, value in payload.items():
        normalized = _normalize_key(key)
        if normalized in ROW_CONTAINER_KEYS:
            continue
        if normalized in METADATA_CONTAINER_KEYS and isinstance(value, Mapping):
            metadata.update(value)
            continue
        metadata[key] = value
    return metadata


def _rows_from_json(payload: Any, source: Path) -> list[Mapping[str, Any]]:
    if isinstance(payload, list):
        rows = payload
    elif isinstance(payload, Mapping):
        containers = [key for key in payload if _normalize_key(key) in ROW_CONTAINER_KEYS]
        if len(containers) > 1:
            raise ValidationError(f"{source} has ambiguous telemetry containers: {containers}")
        rows = []
        if containers:
            value = payload[containers[0]]
            if not isinstance(value, list):
                raise ValidationError(f"{source}: {containers[0]} must be a list of telemetry rows")
            rows = value
        if not rows and _looks_like_row(payload):
            rows = [payload]
    else:
        raise ValidationError(f"JSON telemetry in {source} must be an object or array")

    if not rows:
        raise ValidationError(
            f"JSON artifact {source} contains no telemetry rows; expected a list under "
            "telemetry, records, samples, rows, data, trace, or entries"
        )
    if not all(isinstance(row, Mapping) for row in rows):
        raise ValidationError(f"JSON artifact {source} contains a non-object telemetry row")
    return rows


def _parse_metadata_comments(lines: Iterable[str]) -> dict[str, Any]:
    metadata: dict[str, Any] = {}
    for line in lines:
        stripped = line.strip()
        if not stripped.startswith("#"):
            continue
        body = stripped[1:].strip()
        if "=" in body:
            key, value = body.split("=", 1)
        elif ":" in body:
            key, value = body.split(":", 1)
        else:
            continue
        value = value.strip()
        try:
            parsed: Any = json.loads(value)
        except json.JSONDecodeError:
            parsed = value
        metadata[key.strip()] = parsed
    return metadata


_LOG_PREFIX = re.compile(
    r"^\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}[,.]\d+\s+"
    r"(?:DEBUG|INFO|WARNING|WARN|ERROR|CRITICAL)\s+"
)


def _read_text(path: Path) -> str:
    try:
        text = path.read_text(encoding="utf-8-sig")
    except (OSError, UnicodeError) as exc:
        raise ValidationError(f"Could not read artifact {path}: {exc}") from exc
    if not text.strip():
        raise ValidationError(f"Artifact {path} is empty")
    return text


def _read_table(path: Path) -> tuple[list[Mapping[str, Any]], dict[str, Any]]:
    lines = _read_text(path).splitlines()
    metadata = _parse_metadata_comments(lines)
    payloads = [
        _LOG_PREFIX.sub("", line.strip())
        for line in lines
        if line.strip() and not line.lstrip().startswith("#")
    ]
    try:
        table = list(csv.reader(payloads, strict=True))
    except csv.Error as exc:
        raise ValidationError(f"Malformed CSV/log artifact {path}: {exc}") from exc
    if len(table) < 2:
        raise ValidationError(f"{path} must contain a header and telemetry rows")
    header = [field.strip() for field in table[0]]
    if not all(header) or len(set(header)) != len(header):
        raise ValidationError(f"{path} has empty or duplicate header fields")
    rows: list[Mapping[str, Any]] = []
    for index, values in enumerate(table[1:], 2):
        if len(values) != len(header):
            raise ValidationError(
                f"{path} row {index}: expected {len(header)} fields, got {len(values)}"
            )
        rows.append(dict(zip(header, values)))
    return rows, metadata


def _unique_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValidationError(f"Duplicate JSON field: {key}")
        result[key] = value
    return result


def _read_json(path: Path) -> Any:
    try:
        payload = json.loads(_read_text(path), object_pairs_hook=_unique_json_object)
    except json.JSONDecodeError as exc:
        raise ValidationError(
            f"Malformed JSON artifact {path} at line {exc.lineno}, column {exc.colno}: {exc.msg}"
        ) from exc
    _check_json_finite(payload, str(path))
    return payload


def _expand_inputs(paths: Iterable[str | Path]) -> list[Path]:
    expanded: list[Path] = []
    for value in paths:
        path = Path(value)
        if not path.exists():
            raise ValidationError(f"Missing telemetry input: {path}")
        if path.is_dir():
            for name in ("summary.json", "telemetry.json", "telemetry.csv", "controller_raw.log"):
                candidate = path / name
                if candidate.is_file():
                    expanded.append(candidate)
                    break
            else:
                raise ValidationError(
                    f"{path} has no summary.json, telemetry.json, telemetry.csv, or controller_raw.log"
                )
        else:
            expanded.append(path)
    if not expanded:
        raise ValidationError("At least one telemetry input is required")
    return expanded


def _metadata_lookup(metadata: Mapping[str, Any], aliases: Iterable[str]) -> Any | None:
    wanted = {_normalize_key(alias) for alias in aliases}

    def walk(value: Any) -> Any | None:
        if not isinstance(value, Mapping):
            return None
        for key, item in value.items():
            if _normalize_key(key) in wanted:
                return item
        for item in value.values():
            found = walk(item)
            if found is not None:
                return found
        return None

    return walk(metadata)


def _as_float(value: Any, field: str, row_index: int, source: str) -> float:
    if value is None or (isinstance(value, str) and not value.strip()):
        raise ValidationError(f"{source} row {row_index} has an empty {field} value")
    if isinstance(value, bool):
        raise ValidationError(f"{source} row {row_index} has boolean {field}={value!r}")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValidationError(
            f"{source} row {row_index} has non-numeric {field}={value!r}"
        ) from exc
    if not math.isfinite(parsed):
        raise ValidationError(f"{source} row {row_index} has non-finite {field}={value!r}")
    return parsed


def _as_bool(value: Any, field: str, row_index: int, source: str) -> bool:
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in {"true", "1", "1.0"}:
        return True
    if normalized in {"false", "0", "0.0"}:
        return False
    raise ValidationError(f"{source} row {row_index} has invalid boolean {field}={value!r}")


def _rows_conflict(left: Mapping[str, Any], right: Mapping[str, Any]) -> bool:
    for field in set(left).intersection(right):
        if isinstance(left[field], (float, int)) and isinstance(right[field], (float, int)):
            if not math.isclose(float(left[field]), float(right[field]), rel_tol=1e-6, abs_tol=1e-8):
                return True
        elif left[field] != right[field]:
            return True
    return False


def _artifact_reference(summary_path: Path, value: Any, name: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValidationError(f"{summary_path} has a missing or invalid artifact reference: {name}")
    reference = Path(value)
    # A copied container run remains portable when its artifacts are siblings.
    candidates = (summary_path.parent / reference.name, summary_path.parent / reference, reference)
    for candidate in candidates:
        if candidate.is_file():
            if candidate.stat().st_size == 0:
                raise ValidationError(f"Artifact {candidate} is empty")
            return candidate
    raise ValidationError(f"Missing {name} artifact referenced by {summary_path}: {value}")


def _merge_metadata(metadata: dict[str, Any], additions: Mapping[str, Any], source: Path) -> None:
    for key, value in additions.items():
        if key in metadata and metadata[key] != value:
            raise ValidationError(f"Conflicting run metadata {key!r} in {source}; process one run at a time")
        metadata[key] = value


def _collect_artifacts(
    input_paths: list[Path], metadata_paths: list[Path]
) -> tuple[list[tuple[Path, list[Mapping[str, Any]]]], dict[str, Any], list[Path]]:
    pending = [(path, False) for path in input_paths] + [(path, True) for path in metadata_paths]
    tables: list[tuple[Path, list[Mapping[str, Any]]]] = []
    metadata: dict[str, Any] = {}
    sources: list[Path] = []
    visited: set[Path] = set()
    while pending:
        path, metadata_only = pending.pop(0)
        if path.resolve() in visited:
            continue
        visited.add(path.resolve())
        sources.append(path)
        if path.suffix.lower() == ".json":
            payload = _read_json(path)
            is_telemetry = isinstance(payload, list) or (
                isinstance(payload, dict)
                and (any(_normalize_key(key) in ROW_CONTAINER_KEYS for key in payload) or _looks_like_row(payload))
            )
            if is_telemetry:
                tables.append((path, _rows_from_json(payload, path)))
                _merge_metadata(metadata, _metadata_from_json(payload), path)
                continue
            if not isinstance(payload, dict) or not payload:
                raise ValidationError(f"{path} must contain a telemetry array or a non-empty summary object")
        elif path.suffix.lower() in {".csv", ".log", ".txt"}:
            rows, comments = _read_table(path)
            _merge_metadata(metadata, comments, path)
            if not metadata_only and path.name != "summary.csv":
                tables.append((path, rows))
                continue
            if len(rows) != 1:
                raise ValidationError(f"{path}: summary CSV must contain exactly one data row")
            payload = dict(rows[0])
        else:
            raise ValidationError(f"Unsupported artifact {path}; use JSON, CSV, or a ROS log")
        _merge_metadata(metadata, _metadata_from_json(payload), path)
        references = payload.get("artifacts", {})
        if not isinstance(references, dict):
            raise ValidationError(f"{path}: artifacts must be an object")
        found_telemetry = False
        for name, value in references.items():
            if name not in {
                "telemetry_csv", "telemetry_json", "controller_raw_log", "launch_log",
                "ros_topics_jsonl", "summary_json", "summary_csv",
            }:
                continue
            reference = _artifact_reference(path, value, name)
            if name not in {"telemetry_csv", "telemetry_json", "controller_raw_log"}:
                if reference.resolve() not in {source.resolve() for source in sources}:
                    sources.append(reference)
            else:
                pending.append((reference, False))
                found_telemetry = True
        if not found_telemetry and not metadata_only:
            for name in ("telemetry.csv", "telemetry.json", "controller_raw.log"):
                sibling = path.parent / name
                if sibling.is_file() and sibling.resolve() != path.resolve():
                    pending.append((sibling, False))
                    break
            else:
                raise ValidationError(
                    f"{path} contains no telemetry; supply --input telemetry.csv --metadata {path}"
                )
    if not tables:
        raise ValidationError("No telemetry rows were found in the supplied artifacts")
    _check_json_finite(metadata, "metadata")
    return tables, metadata, sources


def load_telemetry(
    inputs: Iterable[str | Path],
    *,
    metadata_inputs: Iterable[str | Path] = (),
    dt: float | None = None,
) -> TelemetryRun:
    """Load and normalize JSON, CSV, or ROS log telemetry artifacts."""

    input_paths = _expand_inputs(inputs)
    metadata_input_values = tuple(metadata_inputs)
    metadata_paths = _expand_inputs(metadata_input_values) if metadata_input_values else []
    tables, metadata, source_paths = _collect_artifacts(input_paths, metadata_paths)

    metadata_dt = _metadata_lookup(
        metadata, ("dt", "time_step", "control_dt", "sampling_period")
    )
    derived_dt = dt
    if derived_dt is None and metadata_dt is not None:
        derived_dt = _as_float(metadata_dt, "dt", 0, "metadata")
    if derived_dt is None:
        derived_dt = 0.05
    derived_dt = _as_float(derived_dt, "dt", 0, "settings")
    if derived_dt <= 0.0:
        raise ValidationError(f"dt must be a finite positive number, got {derived_dt!r}")
    by_tick: dict[int, dict[str, Any]] = {}
    recorded_time_flags: set[bool] = set()
    for path, rows in tables:
        previous_tick = -1
        previous_time = -math.inf
        for row_index, row in enumerate(rows, start=1):
            normalized = _canonical_row(row)
            missing = [field for field in REQUIRED_FIELDS if field not in normalized]
            if missing:
                raise ValidationError(f"{path} row {row_index} is missing required fields: {', '.join(missing)}")
            for field in NUMERIC_FIELDS:
                if field in normalized:
                    normalized[field] = _as_float(normalized[field], field, row_index, str(path))
            tick = normalized["tick"]
            if tick < 0 or not tick.is_integer() or tick <= previous_tick:
                raise ValidationError(f"{path} row {row_index}: ticks must be non-negative, integer, and strictly increasing")
            recorded_time_flags.add("time_s" in normalized)
            time_s = normalized.get("time_s", tick * derived_dt)
            if not math.isfinite(time_s) or time_s < 0 or time_s <= previous_time:
                raise ValidationError(f"{path} row {row_index}: time_s must be finite, non-negative, and strictly increasing")
            normalized["time_s"] = time_s
            normalized["tick"] = int(tick)
            mode = normalized["mode"]
            if not isinstance(mode, str) or mode.strip() not in {"LQR", "ADAPTIVE_MPC_FILTER", "BACKUP", "FAILED"}:
                raise ValidationError(f"{path} row {row_index} has unknown or empty mode: {mode!r}")
            normalized["mode"] = mode.strip()
            for field in ("solver_ms", "error"):
                if normalized[field] < 0:
                    raise ValidationError(f"{path} row {row_index} has negative {field}")
            for field in ("collision", "violation"):
                if field in normalized:
                    normalized[field] = _as_bool(normalized[field], field, row_index, str(path))
            for field in ("collision_count", "violation_count"):
                if field in normalized:
                    normalized[field] = _count(normalized[field], field)
            existing = by_tick.get(int(tick))
            if existing is not None and _rows_conflict(existing, normalized):
                raise ValidationError(f"Conflicting telemetry artifacts contain different values for tick {tick:g}")
            by_tick[int(tick)] = {**(existing or {}), **normalized}
            previous_tick, previous_time = int(tick), time_s
    ordered_rows = [by_tick[tick] for tick in sorted(by_tick)]
    for field in ("time_s", "theta_v", "theta_omega", "collision", "violation", "collision_count", "violation_count"):
        present = [field in row for row in ordered_rows]
        if any(present) and not all(present):
            raise ValidationError(f"Telemetry has incomplete {field} coverage")
        if field.endswith("_count") and all(present) and any(
            b[field] < a[field] for a, b in zip(ordered_rows, ordered_rows[1:])
        ):
            raise ValidationError(f"Cumulative {field} must not decrease")
    if any(b["time_s"] <= a["time_s"] for a, b in zip(ordered_rows, ordered_rows[1:])):
        raise ValidationError("Merged telemetry time_s must be strictly increasing")
    if not ordered_rows:
        raise ValidationError("No telemetry rows were found")
    declared_rows = _metadata_lookup(metadata, ("telemetry_rows", "sample_count"))
    if declared_rows is not None and _count(declared_rows, "telemetry_rows") != len(ordered_rows):
        raise ValidationError("Summary telemetry_rows disagrees with the supplied telemetry")
    timebase = "recorded time_s" if recorded_time_flags == {True} else f"tick * dt (dt={derived_dt:g} s)"
    if recorded_time_flags == {True, False}:
        timebase = f"recorded time_s and tick * dt (dt={derived_dt:g} s)"
    return TelemetryRun(tuple(ordered_rows), metadata, tuple(source_paths), timebase)


def _contiguous_events(mask: np.ndarray) -> int:
    if len(mask) == 0:
        return 0
    return int(np.count_nonzero(mask & np.concatenate(([True], ~mask[:-1]))))


def _count(value: Any, name: str) -> int:
    parsed = _as_float(value, name, 0, "count")
    if parsed < 0 or not parsed.is_integer():
        raise ValidationError(f"{name} must be a non-negative integer, got {value!r}")
    return int(parsed)


def _declared_count(metadata: Mapping[str, Any], aliases: Iterable[str]) -> int | None:
    value = _metadata_lookup(metadata, aliases)
    return None if value is None else _count(value, next(iter(aliases)))


def summarize_run(
    run: TelemetryRun,
    *,
    solver_time_limit_ms: float | None = None,
) -> dict[str, Any]:
    """Compute the metrics used in the Gate 1 Markdown report."""

    rows = run.rows
    margins = np.asarray([row["min_margin"] for row in rows], dtype=float)
    risks = np.asarray([row["risk"] for row in rows], dtype=float)
    solver_ms = np.asarray([row["solver_ms"] for row in rows], dtype=float)
    times = np.asarray([row["time_s"] for row in rows], dtype=float)
    modes = [str(row["mode"]).strip() for row in rows]
    mode_counts = {
        mode: modes.count(mode) for mode in sorted(set(modes), key=lambda item: item.lower())
    }
    intervention_mask = np.asarray([mode in {"ADAPTIVE_MPC_FILTER", "BACKUP"} for mode in modes])
    failure_samples = modes.count("FAILED")
    missing_ticks = sum(max(0, right["tick"] - left["tick"] - 1) for left, right in zip(rows, rows[1:]))
    active_solver_ms = solver_ms[intervention_mask]
    explicit_violation = np.asarray([row.get("violation", False) for row in rows], dtype=bool)
    explicit_collision = np.asarray([row.get("collision", False) for row in rows], dtype=bool)
    threshold_value = _metadata_lookup(run.metadata, ("configured_min_margin_m", "min_margin_m"))
    threshold = max(0.0, _as_float(threshold_value, "min_margin_m", 0, "metadata")) if threshold_value is not None else 0.0
    violation_mask = (margins < threshold) | explicit_violation
    violation_samples = int(np.count_nonzero(violation_mask))
    violation_events = _contiguous_events(violation_mask)
    declared_violation_count = _declared_count(
        run.metadata, ("violation_count", "safety_violation_count", "violations")
    )
    row_violation_count = max((row.get("violation_count", 0) for row in rows), default=0)
    declared_collision_count = _declared_count(
        run.metadata, ("collision_count", "collisions", "collision_events")
    )
    collision_counts: list[int] = []
    collision_sources: list[str] = []
    if declared_collision_count is not None:
        collision_counts.append(declared_collision_count)
        collision_sources.append("run metadata")
    if "collision" in rows[0]:
        collision_counts.append(_contiguous_events(explicit_collision))
        collision_sources.append("contiguous collision-flag episodes")
    if "collision_count" in rows[0]:
        collision_counts.append(max(row["collision_count"] for row in rows))
        collision_sources.append("cumulative collision_count column")
    collision_events = max(collision_counts) if collision_counts else None
    collision_source = ", ".join(collision_sources) or "unavailable: no contact/collision telemetry"

    timeout_limit = solver_time_limit_ms
    if timeout_limit is None:
        metadata_limit = _metadata_lookup(
            run.metadata,
            ("solver_time_limit_ms", "mpc_time_limit_ms", "solver_timeout_ms"),
        )
        if metadata_limit is not None:
            timeout_limit = _as_float(metadata_limit, "solver_time_limit_ms", 0, "metadata")
    if timeout_limit is not None:
        timeout_limit = _as_float(timeout_limit, "solver_time_limit_ms", 0, "settings")
        if timeout_limit <= 0.0:
            raise ValidationError(f"solver time limit must be positive, got {timeout_limit!r}")
    timeout_count = int(np.count_nonzero(solver_ms > timeout_limit)) if timeout_limit is not None else None

    reasons: list[str] = []
    if violation_samples:
        reasons.append(
            f"{violation_samples} safety-violation sample(s) in {violation_events} episode(s)"
        )
    if declared_violation_count:
        reasons.append(f"{declared_violation_count} violation(s) declared in run metadata")
    if row_violation_count:
        reasons.append(f"{row_violation_count} violation(s) declared by cumulative telemetry")
    if collision_events:
        reasons.append(f"{collision_events} collision event(s) ({collision_source})")
    if failure_samples:
        reasons.append(f"{failure_samples} controller failure mode sample(s)")
    if missing_ticks:
        reasons.append(f"{missing_ticks} missing controller tick(s) between recorded samples")
    upstream_status = _metadata_lookup(run.metadata, ("status",))
    if upstream_status is not None:
        if not isinstance(upstream_status, str) or upstream_status not in {"PASS", "FAIL"}:
            raise ValidationError(f"Invalid upstream validation status: {upstream_status!r}")
        if upstream_status == "FAIL":
            reasons.append("experiment harness reported FAIL")
    upstream_reasons = _metadata_lookup(run.metadata, ("failure_reasons",))
    if isinstance(upstream_reasons, str):
        try:
            upstream_reasons = json.loads(upstream_reasons)
        except json.JSONDecodeError as exc:
            raise ValidationError("failure_reasons must be a JSON array") from exc
    if upstream_reasons is not None:
        if not isinstance(upstream_reasons, list) or not all(isinstance(item, str) for item in upstream_reasons):
            raise ValidationError("failure_reasons must be an array of strings")
        reasons.extend(f"Harness: {reason}" for reason in upstream_reasons)
    for name in ("parse_error_count", "nonfinite_count"):
        count = _declared_count(run.metadata, (name,))
        if count:
            reasons.append(f"Harness {name}: {count}")

    return {
        "status": "PASS" if not reasons else "FAIL",
        "sample_count": len(rows),
        "missing_tick_count": missing_ticks,
        "duration_s": float(np.max(times) - np.min(times)) if len(times) else 0.0,
        "timebase": run.timebase,
        "safety_margin_min_m": float(np.min(margins)),
        "safety_margin_mean_m": float(np.mean(margins)),
        "safety_margin_p05_m": float(np.percentile(margins, 5)),
        "safety_margin_threshold_m": threshold,
        "risk_max": float(np.max(risks)),
        "safety_violation_samples": violation_samples,
        "safety_violation_events": violation_events,
        "declared_violation_count": declared_violation_count,
        "collision_events": collision_events,
        "collision_samples": int(np.count_nonzero(explicit_collision)) if "collision" in rows[0] else None,
        "collision_source": collision_source,
        "mode_counts": mode_counts,
        "intervention_samples": int(np.count_nonzero(intervention_mask)),
        "intervention_events": _contiguous_events(intervention_mask),
        "intervention_rate": float(np.mean(intervention_mask)),
        "mode_transitions": sum(a != b for a, b in zip(modes, modes[1:])),
        "controller_failure_samples": failure_samples,
        "mpc_filter_rate": modes.count("ADAPTIVE_MPC_FILTER") / len(rows),
        "backup_rate": modes.count("BACKUP") / len(rows),
        "tracking_rmse_m": math.sqrt(float(np.mean([row["error"] ** 2 for row in rows]))),
        "solver_mean_ms": float(np.mean(solver_ms)),
        "solver_p95_ms": float(np.percentile(solver_ms, 95)),
        "solver_max_ms": float(np.max(solver_ms)),
        "solver_active_samples": len(active_solver_ms),
        "solver_active_mean_ms": float(np.mean(active_solver_ms)) if len(active_solver_ms) else None,
        "solver_active_p95_ms": float(np.percentile(active_solver_ms, 95)) if len(active_solver_ms) else None,
        "solver_active_max_ms": float(np.max(active_solver_ms)) if len(active_solver_ms) else None,
        "solver_timeout_limit_ms": timeout_limit,
        "solver_timeout_samples": timeout_count,
        "reasons": reasons,
    }


def _save_figure(figure: plt.Figure, output_stem: Path) -> tuple[Path, Path]:
    png_path = output_stem.with_suffix(".png")
    pdf_path = output_stem.with_suffix(".pdf")
    try:
        figure.savefig(png_path, dpi=180, bbox_inches="tight")
        figure.savefig(pdf_path, bbox_inches="tight")
    except OSError as exc:
        raise ValidationError(f"Could not write plot {output_stem}: {exc}") from exc
    finally:
        plt.close(figure)
    return png_path, pdf_path


def _plot_trajectory(run: TelemetryRun, output_dir: Path) -> tuple[Path, Path]:
    x = np.asarray([row["x"] for row in run.rows], dtype=float)
    y = np.asarray([row["y"] for row in run.rows], dtype=float)
    modes = [str(row["mode"]) for row in run.rows]
    figure, axis = plt.subplots(figsize=(7.5, 5.5), constrained_layout=True)
    colors = plt.get_cmap("tab10")
    axis.plot(x, y, color="0.75", linewidth=1.2, zorder=1)
    for mode_index, mode in enumerate(sorted(set(modes), key=lambda item: item.lower())):
        mask = np.asarray([item == mode for item in modes], dtype=bool)
        axis.scatter(x[mask], y[mask], s=12, label=mode, color=colors(mode_index % 10), zorder=2)
    axis.scatter(x[0], y[0], marker="o", color="black", zorder=5, label="start")
    axis.scatter(x[-1], y[-1], marker="x", color="black", zorder=5, label="end")
    axis.set_xlabel("X (m)")
    axis.set_ylabel("Y (m)")
    axis.set_title("Gate 1 ROS trajectory by controller mode")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.25)
    axis.legend(fontsize=8, loc="best")
    return _save_figure(figure, output_dir / "trajectory")


def _plot_safety(
    run: TelemetryRun, summary: Mapping[str, Any], output_dir: Path
) -> tuple[Path, Path]:
    time_s = np.asarray([row["time_s"] for row in run.rows], dtype=float)
    risk = np.asarray([row["risk"] for row in run.rows], dtype=float)
    margin = np.asarray([row["min_margin"] for row in run.rows], dtype=float)
    figure, axes = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True, constrained_layout=True)
    axes[0].plot(time_s, risk, color="tab:red", linewidth=1.5, marker=".", markersize=3)
    axes[0].set_ylabel("Risk")
    axes[0].set_title("Nominal LQR rollout risk and safety margin")
    axes[0].grid(True, alpha=0.25)
    axes[1].plot(time_s, margin, color="tab:green", linewidth=1.5, marker=".", markersize=3, label="logged MinMargin")
    threshold = summary["safety_margin_threshold_m"]
    axes[1].axhline(threshold, color="black", linewidth=0.9, linestyle="--", label="gate threshold")
    violation = (margin < threshold) | np.asarray([row.get("violation", False) for row in run.rows])
    if np.any(violation):
        axes[1].scatter(time_s[violation], margin[violation], color="tab:red", s=16, label="violation")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Margin (m)")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(fontsize=8, loc="best")
    return _save_figure(figure, output_dir / "safety_margin")


def _plot_modes(run: TelemetryRun, output_dir: Path) -> tuple[Path, Path]:
    time_s = np.asarray([row["time_s"] for row in run.rows], dtype=float)
    error = np.asarray([row["error"] for row in run.rows], dtype=float)
    mode_labels = sorted({str(row["mode"]) for row in run.rows}, key=lambda item: item.lower())
    mode_codes = {mode: index for index, mode in enumerate(mode_labels)}
    mode_values = np.asarray([mode_codes[str(row["mode"])] for row in run.rows], dtype=float)
    figure, axes = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True, constrained_layout=True)
    axes[0].step(time_s, mode_values, where="post", color="tab:purple", linewidth=1.6, marker=".", markersize=3)
    axes[0].set_yticks(list(mode_codes.values()), mode_labels)
    axes[0].set_ylabel("Controller mode")
    axes[0].set_title("Controller mode and intervention timeline")
    axes[0].grid(True, alpha=0.25)
    axes[1].plot(time_s, error, color="tab:blue", linewidth=1.5, marker=".", markersize=3)
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Tracking error (m)")
    axes[1].grid(True, alpha=0.25)
    return _save_figure(figure, output_dir / "controller_modes")


def _plot_solver(run: TelemetryRun, summary: Mapping[str, Any], output_dir: Path) -> tuple[Path, Path]:
    time_s = np.asarray([row["time_s"] for row in run.rows], dtype=float)
    solver_ms = np.asarray([row["solver_ms"] for row in run.rows], dtype=float)
    figure, axes = plt.subplots(1, 2, figsize=(10, 4.2), constrained_layout=True)
    axes[0].plot(time_s, solver_ms, color="tab:orange", linewidth=1.4, marker=".", markersize=3, label="solver time")
    axes[0].axhline(
        summary["solver_p95_ms"],
        color="tab:blue",
        linestyle="--",
        linewidth=1.0,
        label="p95",
    )
    limit = summary.get("solver_timeout_limit_ms")
    if limit is not None:
        axes[0].axhline(limit, color="tab:red", linestyle=":", linewidth=1.0, label="limit")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Solver time (ms)")
    axes[0].set_title("Solver timing")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(fontsize=8, loc="best")
    axes[1].hist(solver_ms, bins=min(12, max(3, len(solver_ms))), color="tab:orange", alpha=0.8)
    axes[1].set_xlabel("Solver time (ms)")
    axes[1].set_ylabel("Samples")
    axes[1].set_title("Solver-time distribution")
    axes[1].grid(True, alpha=0.25)
    return _save_figure(figure, output_dir / "solver_timing")


def _format_value(value: Any) -> str:
    if value is None:
        return "not recorded"
    if isinstance(value, float):
        return f"{value:.6g}"
    if isinstance(value, (dict, list, tuple)):
        value = json.dumps(value, sort_keys=True)
    return str(value).replace("|", "\\|").replace("\n", " ").replace("`", "'")


def _render_report(
    run: TelemetryRun,
    summary: Mapping[str, Any],
    plot_paths: Sequence[Path],
    report_path: Path,
) -> None:
    status = str(summary["status"])
    lines = [
        "# Gate 1 human-validation report",
        "",
        f"**Automatic Gate 1 status:** `{status}`  ",
        "**Human sign-off:** pending  ",
        f"**Samples:** `{summary['sample_count']}`  ",
        f"**Missing intermediate ticks:** `{summary['missing_tick_count']}`  ",
        f"**Duration:** `{summary['duration_s']:.3f} s`  ",
        f"**Timebase:** `{run.timebase}`",
        "",
        "## Run metadata",
        "",
        "| Field | Value |",
        "| --- | --- |",
    ]
    metadata_fields = {
        "run_id", "experiment", "controller", "schema_version", "scenario", "seed",
        "headless", "dt", "duration_requested_s", "process_returncode", "stopped_after_duration",
        "runtime_error", "timestamp", "started_at", "git_commit", "deterministic_settings",
        "launch_command", "obstacle_scenario", "robot_radius", "d_safe", "mpc_time_limit_ms",
    }
    metadata_items = sorted(
        ((key, value) for key, value in run.metadata.items() if key in metadata_fields),
        key=lambda item: str(item[0]).lower(),
    )
    if metadata_items:
        lines.extend(f"| {_format_value(key)} | {_format_value(value)} |" for key, value in metadata_items)
    else:
        lines.append("| (none supplied) | (derived from telemetry) |")
    lines.extend(
        [
            "",
            "## Safety metrics",
            "",
            "| Metric | Value |",
            "| --- | --- |",
            f"| Minimum logged nominal-rollout margin | `{summary['safety_margin_min_m']:.4f} m` |",
            f"| Mean safety margin | `{summary['safety_margin_mean_m']:.4f} m` |",
            f"| 5th percentile safety margin | `{summary['safety_margin_p05_m']:.4f} m` |",
            f"| Maximum risk score | `{summary['risk_max']:.4f}` |",
            f"| Gate margin threshold | `{summary['safety_margin_threshold_m']:.4f} m` |",
            f"| Safety-violation samples | `{summary['safety_violation_samples']}` |",
            f"| Safety-violation episodes | `{summary['safety_violation_events']}` |",
            f"| Metadata-declared violations | `{_format_value(summary['declared_violation_count'])}` |",
            f"| Collision events | `{_format_value(summary['collision_events'])}` ({summary['collision_source']}) |",
            f"| Collision-flag samples | `{_format_value(summary['collision_samples'])}` |",
            f"| Tracking RMSE | `{summary['tracking_rmse_m']:.4f} m` |",
            "",
            "## Controller modes and interventions",
            "",
            "| Mode | Samples | Share |",
            "| --- | ---: | ---: |",
        ]
    )
    for mode, count in summary["mode_counts"].items():
        lines.append(f"| {_format_value(mode)} | `{count}` | `{count / summary['sample_count']:.1%}` |")
    lines.extend(
        [
            "",
            f"Intervention samples: `{summary['intervention_samples']}` "
            f"(`{summary['intervention_rate']:.1%}` of telemetry).  ",
            f"Intervention episodes: `{summary['intervention_events']}`; mode transitions: `{summary['mode_transitions']}`.  ",
            f"Controller failure samples: `{summary['controller_failure_samples']}`.",
            "",
            "## Solver timing",
            "",
            "| Metric | Value |",
            "| --- | --- |",
            f"| Mean solver time, all ticks | `{summary['solver_mean_ms']:.3f} ms` |",
            f"| P95 solver time, all ticks | `{summary['solver_p95_ms']:.3f} ms` |",
            f"| Maximum solver time, all ticks | `{summary['solver_max_ms']:.3f} ms` |",
            f"| Active filter/backup samples | `{summary['solver_active_samples']}` |",
            f"| Active-mode mean / P95 / max (ms) | `{_format_value(summary['solver_active_mean_ms'])}` / `{_format_value(summary['solver_active_p95_ms'])}` / `{_format_value(summary['solver_active_max_ms'])}` |",
            f"| Timeout limit (ms) | `{_format_value(summary['solver_timeout_limit_ms'])}` |",
            f"| Samples above timeout limit | `{_format_value(summary['solver_timeout_samples'])}` |",
            "",
            "All-tick timing includes the zero-valued LQR samples; active-mode timing uses ADAPTIVE_MPC_FILTER and BACKUP rows.",
            "",
            "## Generated artifacts",
            "",
        ]
    )
    for path in plot_paths:
        lines.append(f"- `{path.name}`")
    lines.extend(f"- Source: `{path}`" for path in run.sources)
    lines.extend(["", "## Validation notes", ""])
    if summary["reasons"]:
        lines.extend(f"- **FAIL:** {reason}" for reason in summary["reasons"])
    else:
        lines.append("- The available telemetry passed the automatic margin and integrity checks.")
    lines.extend([
        "- `MinMargin` is the nominal LQR rollout margin, including the configured safety buffer and robot radius. A negative value is a gate violation, not a measured collision.",
        "- Collision counts require explicit collision telemetry or a declared count; missing contact measurements are never reported as zero.",
        "- Rates are sample fractions; episodes are contiguous flagged samples. Solver overruns are reported independently because the controller can enter BACKUP.",
        "- This is sampled diagnostic evidence, not a continuous-time or physical safety guarantee.",
        "",
        "## Human review",
        "",
        "- [ ] Check the trajectory and controller-mode timeline against the intended scenario.",
        "- [ ] Inspect margin violations and timing outliers against the retained raw log.",
        "- [ ] Record the reviewer, decision, and any missing collision evidence before sign-off.",
    ])
    lines.append("- The strict Gate 1 command exits non-zero when this report status is `FAIL`.")
    try:
        report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    except OSError as exc:
        raise ValidationError(f"Could not write validation report {report_path}: {exc}") from exc


def generate_report(
    run: TelemetryRun,
    output_dir: str | Path,
    *,
    solver_time_limit_ms: float | None = None,
) -> dict[str, Any]:
    """Write PNG/PDF figures and the Markdown report, returning its metrics."""

    output_path = Path(output_dir)
    try:
        output_path.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        raise ValidationError(f"Could not create output directory {output_path}: {exc}") from exc
    summary = summarize_run(run, solver_time_limit_ms=solver_time_limit_ms)
    plot_paths: list[Path] = []
    for plotter in (_plot_trajectory, _plot_modes):
        plot_paths.extend(plotter(run, output_path))
    plot_paths.extend(_plot_safety(run, summary, output_path))
    plot_paths.extend(_plot_solver(run, summary, output_path))
    report_path = output_path / "gate1_validation_report.md"
    _render_report(run, summary, plot_paths, report_path)
    summary = dict(summary)
    summary["report_path"] = report_path
    summary["plot_paths"] = tuple(plot_paths)
    return summary


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate strict Gate 1 ROS 2 validation plots and Markdown report"
    )
    parser.add_argument(
        "--input",
        "-i",
        nargs="+",
        required=True,
        help="Telemetry JSON/CSV/log, harness summary.json, or one Gate 1 run directory",
    )
    parser.add_argument(
        "--metadata",
        nargs="+",
        default=(),
        help="Optional JSON object or single-row summary CSV with run metadata",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=Path("Output/Plots/Gate1"),
        help="Ignored output directory for PNG, PDF, and Markdown artifacts",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=None,
        help="Timestep used when telemetry has Tick but no time_s (default: metadata or 0.05 s)",
    )
    parser.add_argument(
        "--solver-time-limit-ms",
        type=float,
        default=None,
        help="Optional solver timeout threshold; otherwise read metadata when available",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        run = load_telemetry(args.input, metadata_inputs=args.metadata, dt=args.dt)
        summary = generate_report(
            run,
            args.output,
            solver_time_limit_ms=args.solver_time_limit_ms,
        )
    except (ValidationError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(f"Gate 1 validation: {summary['status']}")
    print(f"Report: {summary['report_path']}")
    print(f"Plots: {args.output}")
    if summary["reasons"]:
        for reason in summary["reasons"]:
            print(f"Validation failure: {reason}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
