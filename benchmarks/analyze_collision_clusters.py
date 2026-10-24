"""Cluster collision outcomes from benchmark traces without changing the runs.

The Monte Carlo runner writes decision-time traces, while the controlled
recoverability diagnostics write case tables.  This module is a deliberately
separate post-processor for those artifacts.  It keeps the raw trial counts,
uses fixed bins for the optional held-out report, and labels logged analysis as
secondary evidence.  It never treats a missing collision signal as a safe
trial. Clearance and stopping-boundary proxies are reported separately from
recorded collision or obstacle-contact outcomes.

Typical use::

    python -m benchmarks.analyze_collision_clusters \
        --input Output/Plots/Paper/ablation_results/traces \
        --output Output/Plots/Paper/ablation_results/collision_clusters

The input may be one or more JSON traces, a Monte Carlo summary containing a
``trials`` list, or a controlled case report containing ``cases``/``records``.
The resulting JSON and Markdown files are diagnostic artifacts.  A report made
from logged traces remains exploratory even when ``--split held-out`` is used;
primary controlled evidence must be explicitly labelled and supplied from a
controlled experiment.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


EVIDENCE_EXPLORATORY_LOGGED = "exploratory_logged"
EVIDENCE_PRIMARY_CONTROLLED = "primary_controlled"
EVIDENCE_ROLES = (EVIDENCE_EXPLORATORY_LOGGED, EVIDENCE_PRIMARY_CONTROLLED)

# These thresholds are intentionally declared, not fitted from the supplied
# artifacts.  The held-out report must use exactly these bins so that a split
# cannot be made to look better by changing cluster boundaries after seeing it.
FROZEN_HELD_OUT_RISK_BIN_EDGES = (0.0, 0.25, 0.50, 0.75, 1.0)
FROZEN_HELD_OUT_COMMAND_GAP_BIN_EDGES = (0.0, 0.25, 0.75, 1.25)

_RISK_ALIASES = (
    "pre_action_risk",
    "pre_risk",
    "risk",
    "risk_score",
    "cvar_risk",
)
_COMMAND_GAP_ALIASES = (
    "command_gap",
    "pre_action_command_gap",
    "latched_command_gap",
    "u0_minus_v0",
)
_COLLISION_ALIASES = (
    "collision",
    "collided",
    "is_collision",
    "collision_flag",
    "collision_event",
    "obstacle_collision",
    "obstacle_contact",
)
_COLLISION_COUNT_ALIASES = (
    "collision_count",
    "collisions",
    "collision_events",
    "obstacle_collision_count",
    "obstacle_contact_count",
)
# Neither safety flags nor geometry establish an observed obstacle collision.
# In particular, the controlled stopping sweep has no physical obstacle body.
_CLEARANCE_ALIASES = (
    "post_step_clearance",
    "minimum_clearance",
    "clearance",
    "min_clearance",
    "virtual_boundary_margin_m",
)
_TRACE_KEYS = ("trace", "telemetry", "rows", "samples", "entries")
_COLLECTION_KEYS = ("traces", "trials", "cases", "records")


@dataclass(frozen=True)
class CollisionOutcome:
    """Normalized outcome for one trace or one trial record."""

    trial_id: str
    source: str
    collision: bool | None
    collision_source: str
    pre_action_risk: float | None
    command_gap: float | None
    split: str
    boundary_proxy_violation: bool | None = None
    boundary_proxy_source: str = "unavailable"
    feature_row_index: int | None = None
    feature_step: float | None = None
    feature_selection: str = "unavailable"
    declared_splits: tuple[str, ...] = ()


def _normalise_split(split: str) -> str:
    value = str(split).strip().lower().replace("_", "-")
    aliases = {
        "exploratory": "exploratory",
        "logged": "exploratory",
        "exploratory-logged": "exploratory",
        "primary": "primary",
        "controlled": "primary",
        "primary-controlled": "primary",
        "held-out": "held-out",
        "holdout": "held-out",
    }
    if value not in aliases:
        raise ValueError("split must be exploratory, primary, or held-out")
    return aliases[value]


def _finite_float(value: Any) -> float | None:
    if isinstance(value, bool) or value is None:
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return parsed if math.isfinite(parsed) else None


def _as_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        if value in (0, 1):
            return bool(value)
        return None
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "yes", "1", "1.0"}:
            return True
        if normalized in {"false", "no", "0", "0.0"}:
            return False
    return None


def _first_value(mapping: Mapping[str, Any], aliases: Iterable[str]) -> float | None:
    for alias in aliases:
        if alias in mapping:
            value = _finite_float(mapping[alias])
            if value is not None:
                return value
    return None


def _first_bool(mapping: Mapping[str, Any], aliases: Iterable[str]) -> bool | None:
    for alias in aliases:
        if alias in mapping and mapping[alias] is not None:
            value = _as_bool(mapping[alias])
            if value is None:
                raise ValueError(f"{alias} must be a boolean or null")
            return value
    return None


def _declared_splits(value: Any) -> set[str]:
    """Collect declarations without reading arbitrary nested scenario data."""
    if isinstance(value, Mapping):
        found = set()
        if value.get("split") is not None:
            found.add(_normalise_split(value["split"]))
        for key in ("metadata", *_TRACE_KEYS, *_COLLECTION_KEYS):
            if key in value:
                found.update(_declared_splits(value[key]))
        return found
    if isinstance(value, (list, tuple)):
        return set().union(*(_declared_splits(item) for item in value))
    return set()


def _validate_split(value: Any, split: str | None = None) -> tuple[str, ...]:
    declared = _declared_splits(value)
    if len(declared) > 1:
        raise ValueError(f"input has conflicting split declarations: {sorted(declared)!r}")
    if split is not None and declared and declared != {_normalise_split(split)}:
        raise ValueError(f"input declares split {sorted(declared)!r}, expected {split!r}")
    return tuple(sorted(declared))


def _rows_and_metadata(value: Any) -> tuple[list[Mapping[str, Any]], Mapping[str, Any]]:
    """Normalize a trace-like object into rows and an optional parent record."""

    if isinstance(value, Mapping):
        containers = [key for key in _TRACE_KEYS if key in value]
        if len(containers) > 1:
            raise ValueError("input has ambiguous trace containers")
        if not containers:
            return [value], {}
        rows = value[containers[0]]
        parent = value
    elif isinstance(value, (list, tuple)):
        rows, parent = value, {}
    else:
        raise TypeError(f"trace/trial input must be an object or list, got {type(value).__name__}")
    if not isinstance(rows, (list, tuple)) or not rows:
        raise ValueError("trace must contain a non-empty list of rows")
    if not all(isinstance(row, Mapping) for row in rows):
        raise ValueError("trace contains a non-object row")
    return list(rows), parent


def _collision_count(value: Any, field: str) -> int:
    try:
        number = Decimal(str(value))
    except (InvalidOperation, ValueError) as exc:
        raise ValueError(f"{field} must be a non-negative integer") from exc
    if not number.is_finite() or number < 0 or number != number.to_integral_value():
        raise ValueError(f"{field} must be a non-negative integer")
    return int(number)


def _proxy_flag_semantics(row: Mapping[str, Any]) -> bool:
    metadata = row.get("metadata", {})
    if not isinstance(metadata, Mapping):
        raise ValueError("metadata must be an object")
    return metadata.get("collision_flag_semantics") == "boundary_proxy"


def _row_collision_signal(
    row: Mapping[str, Any], *, inherited_proxy_flag: bool = False
) -> tuple[bool | None, str]:
    # Validate every supplied count even if a flag already declares a collision.
    counts = {
        field: _collision_count(row[field], field)
        for field in _COLLISION_COUNT_ALIASES
        if field in row and row[field] is not None
    }
    proxy_flag = inherited_proxy_flag or _proxy_flag_semantics(row)
    flags = {
        field: _first_bool(row, (field,))
        for field in _COLLISION_ALIASES
        if not proxy_flag or field.startswith("obstacle_")
    }
    if any(value is True for value in flags.values()):
        return True, "explicit_collision_flag"
    for field, count in counts.items():
        if count > 0:
            return True, field
    if any(value is False for value in flags.values()):
        return False, "explicit_collision_flag"
    if counts:
        return False, ", ".join(counts)
    return None, "unavailable"


def _row_boundary_proxy(
    row: Mapping[str, Any], *, inherited_proxy_flag: bool = False
) -> tuple[bool | None, str]:
    values: dict[str, bool] = {}
    for field in _CLEARANCE_ALIASES:
        value = _finite_float(row.get(field))
        if value is not None:
            values[field] = value < 0.0
    for field in ("plant_proxy_safe", "plant_safe"):
        plant_safe = _first_bool(row, (field,))
        if plant_safe is not None:
            values[field] = not plant_safe
    if inherited_proxy_flag or _proxy_flag_semantics(row):
        for field in _COLLISION_ALIASES:
            if field.startswith("obstacle_"):
                continue
            flag = _first_bool(row, (field,))
            if flag is not None:
                values[f"{field} (declared boundary proxy)"] = flag
    if not values:
        return None, "unavailable"
    return any(values.values()), ", ".join(values)


def _summarize_signals(
    signals: Sequence[tuple[bool | None, str]],
    parent_signal: tuple[bool | None, str],
) -> tuple[bool | None, str, int | None]:
    """A positive sample proves an event; absence requires full coverage."""
    for index, (signal, source) in enumerate(signals):
        if signal is True:
            return True, source, index
    if parent_signal[0] is not None:
        return parent_signal[0], parent_signal[1], None
    if signals and all(signal is False for signal, _ in signals):
        return False, ", ".join(sorted({source for _, source in signals})), None
    source = "incomplete_telemetry" if any(signal is False for signal, _ in signals) else "unavailable"
    return None, source, None


def _collision_outcome(
    rows: Sequence[Mapping[str, Any]], parent: Mapping[str, Any]
) -> tuple[bool | None, str, int | None]:
    """Use recorded collision/obstacle contacts, never a safety or geometry proxy."""
    proxy_flag = _proxy_flag_semantics(parent)
    signals = [_row_collision_signal(row, inherited_proxy_flag=proxy_flag) for row in rows]
    parent_signal = _row_collision_signal(parent)
    if parent_signal[0] is False and any(signal is True for signal, _ in signals):
        raise ValueError("run collision declaration contradicts recorded collision samples")
    return _summarize_signals(signals, parent_signal)


def _derived_command_gap(row: Mapping[str, Any]) -> float | None:
    direct = _first_value(row, _COMMAND_GAP_ALIASES)
    if direct is not None:
        return direct

    previous = _first_value(row, ("previous_applied_v", "previous_command_v"))
    actual = _first_value(row, ("pre_v",))
    if previous is not None and actual is not None:
        return previous - actual
    initial_command = _first_value(row, ("u0",))
    initial_velocity = _first_value(row, ("v0",))
    if initial_command is not None and initial_velocity is not None:
        return initial_command - initial_velocity
    return None


def _select_feature_row(
    rows: Sequence[Mapping[str, Any]],
    collision: bool | None,
    event_index: int | None,
) -> tuple[int | None, str]:
    if event_index is not None:
        return event_index, "first_recorded_collision_tick"
    if collision is True:
        return None, "collision_tick_unavailable"
    risks = [(index, _first_value(row, _RISK_ALIASES)) for index, row in enumerate(rows)]
    finite_risks = [(index, value) for index, value in risks if value is not None]
    if finite_risks:
        return max(finite_risks, key=lambda pair: pair[1])[0], "maximum_risk_tick"
    gaps = [(index, _derived_command_gap(row)) for index, row in enumerate(rows)]
    finite_gaps = [(index, value) for index, value in gaps if value is not None]
    if finite_gaps:
        return max(finite_gaps, key=lambda pair: pair[1])[0], "maximum_command_gap_tick"
    return None, "features_unavailable"


def _outcome_from_value(value: Any, trial_id: str, source: str, split: str) -> CollisionOutcome:
    declared_splits = _validate_split(value, split)
    rows, parent = _rows_and_metadata(value)
    collision, collision_source, event_index = _collision_outcome(rows, parent)
    proxy, proxy_source, _ = _summarize_signals(
        [_row_boundary_proxy(row, inherited_proxy_flag=_proxy_flag_semantics(parent)) for row in rows],
        _row_boundary_proxy(parent),
    )
    feature_index, selection = _select_feature_row(rows, collision, event_index)
    feature_row = rows[feature_index] if feature_index is not None else {}
    return CollisionOutcome(
        trial_id=str(trial_id),
        source=str(source),
        collision=collision,
        collision_source=collision_source,
        pre_action_risk=_first_value(feature_row, _RISK_ALIASES),
        command_gap=_derived_command_gap(feature_row),
        split=split,
        boundary_proxy_violation=proxy,
        boundary_proxy_source=proxy_source,
        feature_row_index=feature_index,
        feature_step=_first_value(feature_row, ("step", "tick")),
        feature_selection=selection,
        declared_splits=declared_splits,
    )


def _coerce_edges(edges: Iterable[float]) -> tuple[float, ...]:
    values = tuple(float(value) for value in edges)
    if not values or not all(math.isfinite(value) for value in values):
        raise ValueError("cluster edges must contain at least one finite number")
    if any(right <= left for left, right in zip(values, values[1:])):
        raise ValueError("cluster edges must be strictly increasing")
    return values


def _format_number(value: float) -> str:
    return f"{value:.6g}"


def _cluster_label(value: float, edges: tuple[float, ...]) -> tuple[str, float | None, float | None]:
    if value < edges[0]:
        return f"<{_format_number(edges[0])}", None, edges[0]
    for index, lower in enumerate(edges):
        upper = edges[index + 1] if index + 1 < len(edges) else None
        if upper is None or value < upper:
            upper_label = "inf" if upper is None else _format_number(upper)
            return f"[{_format_number(lower)},{upper_label})", lower, upper
    # The loop always returns for finite values, but keeping this branch makes
    # the helper robust to future changes in the edge representation.
    return f">={_format_number(edges[-1])}", edges[-1], None


def _cluster_rows(
    outcomes: Sequence[CollisionOutcome],
    feature: str,
    edges: tuple[float, ...],
) -> list[dict[str, Any]]:
    groups: dict[str, list[CollisionOutcome]] = {}
    bounds: dict[str, tuple[float | None, float | None]] = {}
    for outcome in outcomes:
        value = getattr(outcome, feature)
        if value is None:
            label, lower, upper = "unavailable", None, None
        else:
            label, lower, upper = _cluster_label(value, edges)
        groups.setdefault(label, []).append(outcome)
        bounds[label] = (lower, upper)

    ordered_labels = list(groups)
    ordered_labels.sort(key=lambda label: (
        label == "unavailable",
        -math.inf if bounds[label][0] is None else bounds[label][0],
    ))
    rows: list[dict[str, Any]] = []
    for label in ordered_labels:
        group = groups[label]
        known = [outcome for outcome in group if outcome.collision is not None]
        collisions = sum(bool(outcome.collision) for outcome in known)
        noncollisions = len(known) - collisions
        unknown = len(group) - len(known)
        rows.append(
            {
                "feature": feature,
                "bin": label,
                "lower": bounds[label][0],
                "upper": bounds[label][1],
                "trial_count": len(group),
                "known_outcome_count": len(known),
                "collision_count": collisions,
                "noncollision_count": noncollisions,
                "unknown_outcome_count": unknown,
                "collision_rate": (
                    float(collisions / len(known)) if known else None
                ),
                "boundary_proxy_counts": _boundary_proxy_counts(group),
            }
        )
    return rows


def _raw_counts(outcomes: Sequence[CollisionOutcome]) -> dict[str, int | float | None]:
    known = [outcome for outcome in outcomes if outcome.collision is not None]
    collisions = sum(bool(outcome.collision) for outcome in known)
    noncollisions = len(known) - collisions
    return {
        "trial_count": len(outcomes),
        "known_outcome_count": len(known),
        "collision_count": collisions,
        "noncollision_count": noncollisions,
        "unknown_outcome_count": len(outcomes) - len(known),
        "collision_rate": float(collisions / len(known)) if known else None,
        "risk_available_count": sum(outcome.pre_action_risk is not None for outcome in outcomes),
        "command_gap_available_count": sum(outcome.command_gap is not None for outcome in outcomes),
    }


def _boundary_proxy_counts(outcomes: Sequence[CollisionOutcome]) -> dict[str, int]:
    return {
        "trial_count": len(outcomes),
        "boundary_proxy_violation_count": sum(row.boundary_proxy_violation is True for row in outcomes),
        "boundary_proxy_nonviolation_count": sum(row.boundary_proxy_violation is False for row in outcomes),
        "unknown_boundary_proxy_count": sum(row.boundary_proxy_violation is None for row in outcomes),
    }


def _evidence_note(evidence_role: str, split: str) -> list[str]:
    if evidence_role == EVIDENCE_PRIMARY_CONTROLLED:
        notes = [
            "The caller labelled this input as primary controlled evidence; verify the source artifact is a deterministic controlled sweep.",
        ]
    else:
        notes = [
            "Logged trace clustering is secondary exploratory analysis and is not a substitute for a controlled plant-level sweep.",
        ]
    notes.extend([
        "Missing or partially missing collision telemetry is retained as unknown; it is never counted as a non-collision unless a full-run declaration supplies the outcome.",
        "Clearance, plant_proxy_safe, and legacy plant_safe are boundary proxies, not observed physical collisions. Analytical safe flags never establish physical outcomes.",
        "Generic contact_count includes floor contacts and is never used as obstacle-contact evidence.",
        "The MuJoCo/RVO2 summary collision flag is derived from clearance; recognized summaries retain this boundary-proxy meaning. Other explicit flags retain the producer's declared semantics.",
        "Features share one pre-action tick. Outcome-dependent first-event/maximum selection is descriptive, not predictor performance or causal evidence.",
        "Counts describe trials, not independent ticks or collision-event totals. Rates exclude unknown outcomes and do not establish Gate 0 acceptance.",
    ])
    if split == "held-out":
        notes.extend(
            [
                "The held-out report uses frozen cluster edges declared in this module; no thresholds are fitted from held-out outcomes.",
                "A held-out label does not upgrade exploratory logged traces to primary controlled evidence.",
                "Split declarations are checked; an undeclared split remains a caller assertion, not proof of independent held-out selection.",
            ]
        )
    return notes


def analyze_outcomes(
    outcomes: Iterable[CollisionOutcome],
    *,
    split: str = "exploratory",
    evidence_role: str = EVIDENCE_EXPLORATORY_LOGGED,
    risk_bin_edges: Iterable[float] = FROZEN_HELD_OUT_RISK_BIN_EDGES,
    command_gap_bin_edges: Iterable[float] = FROZEN_HELD_OUT_COMMAND_GAP_BIN_EDGES,
) -> dict[str, Any]:
    """Return clustered collision counts for normalized trial outcomes.

    ``split='held-out'`` is intentionally a reporting mode.  It does not
    select, tune, or relabel trials; the caller supplies the already-frozen
    held-out artifacts.
    """

    normalized_split = _normalise_split(split)
    if evidence_role not in EVIDENCE_ROLES:
        raise ValueError(f"evidence_role must be one of {EVIDENCE_ROLES}")
    risk_edges = _coerce_edges(risk_bin_edges)
    gap_edges = _coerce_edges(command_gap_bin_edges)
    if normalized_split == "held-out":
        if risk_edges != FROZEN_HELD_OUT_RISK_BIN_EDGES:
            raise ValueError("held-out risk bins are frozen and cannot be changed")
        if gap_edges != FROZEN_HELD_OUT_COMMAND_GAP_BIN_EDGES:
            raise ValueError("held-out command-gap bins are frozen and cannot be changed")

    rows = tuple(outcomes)
    if not rows:
        raise ValueError("at least one collision outcome is required")
    for row in rows:
        if _normalise_split(row.split) != normalized_split or any(
            _normalise_split(declared) != normalized_split for declared in row.declared_splits
        ):
            raise ValueError(f"outcome {row.trial_id!r} has a conflicting split; cannot relabel as {normalized_split!r}")
    raw_counts = _raw_counts(rows)
    if normalized_split == "held-out":
        analysis_scope = "frozen_held_out"
    elif evidence_role == EVIDENCE_PRIMARY_CONTROLLED:
        analysis_scope = "primary_controlled"
    else:
        analysis_scope = "exploratory_logged"
    report = {
        "schema_version": 2,
        "analysis_scope": analysis_scope,
        "split": normalized_split,
        "frozen": normalized_split == "held-out",
        "evidence_role": evidence_role,
        "evidence_label": (
            "primary controlled evidence"
            if evidence_role == EVIDENCE_PRIMARY_CONTROLLED
            else "secondary exploratory logged analysis"
        ),
        "cluster_spec": {
            "pre_action_risk_edges": list(risk_edges),
            "command_gap_edges": list(gap_edges),
            "edge_convention": "[lower, upper), with the final bin extending to infinity",
        },
        "raw_counts": raw_counts,
        "physical_outcome_definition": "recorded collision flags/counts or obstacle-specific contacts; no geometry or safety proxies",
        "boundary_proxy_counts": _boundary_proxy_counts(rows),
        "clusters": {
            "pre_action_risk": _cluster_rows(rows, "pre_action_risk", risk_edges),
            "command_gap": _cluster_rows(rows, "command_gap", gap_edges),
        },
        "outcomes": [{**asdict(row), "declared_splits": list(row.declared_splits)} for row in rows],
        "notes": _evidence_note(evidence_role, normalized_split),
    }
    return report


def analyze_traces(
    traces: Iterable[Any] | Any,
    *,
    split: str = "exploratory",
    evidence_role: str = EVIDENCE_EXPLORATORY_LOGGED,
    sources: Sequence[str] | None = None,
    risk_bin_edges: Iterable[float] = FROZEN_HELD_OUT_RISK_BIN_EDGES,
    command_gap_bin_edges: Iterable[float] = FROZEN_HELD_OUT_COMMAND_GAP_BIN_EDGES,
) -> dict[str, Any]:
    """Analyze one or more trace/trial objects.

    Both features come from the first recorded collision tick, or from the
    maximum-risk tick when no collision is recorded (maximum-gap tick if risk
    is unavailable throughout). This outcome-dependent selection is descriptive.
    Missing features are never borrowed from another tick. Declared input splits
    must agree with the requested split, including inherited artifact metadata.
    """

    normalized_split = _normalise_split(split)
    payload = traces if isinstance(traces, Mapping) else list(traces)
    _validate_split(payload, normalized_split)
    items, generated_sources = _objects_from_payload(payload, "trace")
    if sources is None:
        sources = generated_sources
    if len(sources) != len(items):
        raise ValueError("sources must identify every trace/trial object exactly once")
    outcomes: list[CollisionOutcome] = []
    for index, item in enumerate(items):
        outcome = _outcome_from_value(item, str(index), sources[index], normalized_split)
        outcomes.append(outcome)
    return analyze_outcomes(
        outcomes,
        split=normalized_split,
        evidence_role=evidence_role,
        risk_bin_edges=risk_bin_edges,
        command_gap_bin_edges=command_gap_bin_edges,
    )


def analyze_records(
    records: Iterable[Mapping[str, Any]],
    *,
    split: str = "primary",
    evidence_role: str = EVIDENCE_PRIMARY_CONTROLLED,
    sources: Sequence[str] | None = None,
    risk_bin_edges: Iterable[float] = FROZEN_HELD_OUT_RISK_BIN_EDGES,
    command_gap_bin_edges: Iterable[float] = FROZEN_HELD_OUT_COMMAND_GAP_BIN_EDGES,
) -> dict[str, Any]:
    """Analyze trial/case records from a controlled or summary artifact."""

    rows = list(records)
    if not rows:
        raise ValueError("at least one record is required")
    if not all(isinstance(record, Mapping) for record in rows):
        raise TypeError("records must contain mapping objects")
    return analyze_traces(
        {"records": rows},
        split=split,
        evidence_role=evidence_role,
        sources=sources,
        risk_bin_edges=risk_bin_edges,
        command_gap_bin_edges=command_gap_bin_edges,
    )


def frozen_held_out_report(
    records: Iterable[Mapping[str, Any]],
    *,
    evidence_role: str = EVIDENCE_PRIMARY_CONTROLLED,
    sources: Sequence[str] | None = None,
) -> dict[str, Any]:
    """Report a caller-selected held-out artifact with immutable cluster bins."""

    return analyze_records(
        records,
        split="held-out",
        evidence_role=evidence_role,
        sources=sources,
        risk_bin_edges=FROZEN_HELD_OUT_RISK_BIN_EDGES,
        command_gap_bin_edges=FROZEN_HELD_OUT_COMMAND_GAP_BIN_EDGES,
    )


def _expand_input(path: Path) -> list[Path]:
    if not path.exists():
        raise FileNotFoundError(f"missing analysis input: {path}")
    if path.is_file():
        return [path]
    candidates = sorted(candidate for candidate in path.glob("*.json") if candidate.is_file())
    if not candidates:
        raise FileNotFoundError(f"no JSON traces found in {path}")
    return candidates


def _looks_like_trace(payload: Sequence[Any]) -> bool:
    step_keys = {"step", "tick", "time_s", "pre_v", "post_step_clearance", "requested_command_v"}
    return (
        bool(payload)
        and all(isinstance(row, Mapping) for row in payload)
        and not any(set(_TRACE_KEYS + _COLLECTION_KEYS).intersection(row) for row in payload)
        and any(step_keys.intersection(row) for row in payload)
    )


def _objects_from_payload(payload: Any, source: str) -> tuple[list[Any], list[str]]:
    _validate_split(payload)
    if isinstance(payload, (list, tuple)):
        if not payload:
            raise ValueError(f"{source} contains no trace/trial objects")
        if _looks_like_trace(payload):
            return [{"trace": list(payload)}], [source]
        objects, sources = [], []
        for index, item in enumerate(payload):
            values, paths = _objects_from_payload(item, f"{source}#{index}")
            objects.extend(values)
            sources.extend(paths)
        return objects, sources
    if not isinstance(payload, Mapping):
        raise ValueError(f"{source} must contain a JSON object or array")
    containers = [key for key in _COLLECTION_KEYS if key in payload]
    if len(containers) > 1:
        raise ValueError(f"{source} has ambiguous trial containers")
    if not containers:
        # Wrap records so a collection of trial records with step-like fields
        # cannot be reinterpreted as one trace on a later analysis call.
        value = payload if any(key in payload for key in _TRACE_KEYS) else {"trace": [payload]}
        return [value], [source]
    items = payload[containers[0]]
    if not isinstance(items, (list, tuple)) or not items:
        raise ValueError(f"{source}: {containers[0]} must contain a non-empty list")
    metadata = payload.get("metadata", {})
    if not isinstance(metadata, Mapping):
        raise ValueError(f"{source}: metadata must be an object")
    inherited = dict(metadata)
    if containers[0] == "trials" and metadata.get("rvo2_required") is True:
        # The existing runner defines TrialMetrics.collision as min(clearances)
        # < 0, not a contact measurement. Preserve that known producer contract.
        inherited["collision_flag_semantics"] = "boundary_proxy"
    if payload.get("split") is not None:
        inherited["split"] = payload["split"]
    objects, sources = [], []
    for index, item in enumerate(items):
        if isinstance(item, (list, tuple)):
            item = {"trace": list(item)}
        if not isinstance(item, Mapping):
            raise ValueError(f"{source}: trial must be an object or trace array")
        child_metadata = item.get("metadata", {})
        if not isinstance(child_metadata, Mapping):
            raise ValueError(f"{source}: metadata must be an object")
        merged_metadata = {**inherited, **child_metadata}
        if inherited.get("split") is not None:
            merged_metadata["split"] = inherited["split"]
        if inherited.get("collision_flag_semantics") == "boundary_proxy":
            merged_metadata["collision_flag_semantics"] = "boundary_proxy"
        child = {**item, "metadata": merged_metadata}
        values, paths = _objects_from_payload(child, f"{source}#{index}")
        objects.extend(values)
        sources.extend(paths)
    return objects, sources


def load_inputs(
    paths: Iterable[str | Path] | str | Path, *, split: str | None = None
) -> tuple[list[Any], list[str]]:
    """Load distinct files, retaining split metadata on each extracted trial."""

    objects: list[Any] = []
    sources: list[str] = []
    expanded: list[Path] = []
    path_values: Iterable[str | Path]
    if isinstance(paths, (str, Path)):
        path_values = (paths,)
    else:
        path_values = paths
    for raw_path in path_values:
        expanded.extend(_expand_input(Path(raw_path)))
    seen: set[Path] = set()
    for path in expanded:
        resolved = path.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise ValueError(f"could not read JSON input {path}: {exc}") from exc
        _validate_split(payload, split)
        values, value_sources = _objects_from_payload(payload, str(path))
        objects.extend(values)
        sources.extend(value_sources)
    if not objects:
        raise ValueError("no trace/trial objects were found in the supplied inputs")
    return objects, sources


def render_markdown(report: Mapping[str, Any]) -> str:
    """Render a concise human-readable report from :func:`analyze_outcomes`."""

    raw = report["raw_counts"]
    proxy = report["boundary_proxy_counts"]
    lines = [
        "# Clustered collision analysis",
        "",
        f"- **Analysis scope:** `{report['analysis_scope']}`",
        f"- **Split:** `{report['split']}`",
        f"- **Evidence role:** {report['evidence_label']}",
        f"- **Frozen bins:** `{report['frozen']}`",
        "",
        "## Raw outcome counts",
        "",
        "Recorded collision/obstacle-contact outcomes only; boundary proxies are excluded.",
        "",
        "| Trials | Known outcomes | Collisions | Non-collisions | Unknown outcomes | Collision rate |",
        "| ---: | ---: | ---: | ---: | ---: | ---: |",
        f"| {raw['trial_count']} | {raw['known_outcome_count']} | {raw['collision_count']} | {raw['noncollision_count']} | {raw['unknown_outcome_count']} | {raw['collision_rate'] if raw['collision_rate'] is not None else 'not available'} |",
        "",
        "## Boundary proxy counts (not observed collisions)",
        "",
        "| Trials | Proxy violations | Proxy nonviolations | Unknown proxy outcome |",
        "| ---: | ---: | ---: | ---: |",
        f"| {proxy['trial_count']} | {proxy['boundary_proxy_violation_count']} | {proxy['boundary_proxy_nonviolation_count']} | {proxy['unknown_boundary_proxy_count']} |",
        "",
    ]
    for title, key in (("Pre-action risk", "pre_action_risk"), ("Command gap", "command_gap")):
        lines.extend(
            [
                f"## {title} clusters",
                "",
                "| Bin | Trials | Known | Collisions | Non-collisions | Unknown | Collision rate |",
                "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
            ]
        )
        for row in report["clusters"][key]:
            rate = row["collision_rate"] if row["collision_rate"] is not None else "not available"
            lines.append(
                f"| `{row['bin']}` | {row['trial_count']} | {row['known_outcome_count']} | {row['collision_count']} | {row['noncollision_count']} | {row['unknown_outcome_count']} | {rate} |"
            )
        lines.append("")
    lines.extend(["## Interpretation notes", ""])
    lines.extend(f"- {note}" for note in report["notes"])
    lines.append("")
    return "\n".join(lines)


def write_report(report: Mapping[str, Any], output: str | Path) -> tuple[Path, Path]:
    """Write JSON and Markdown reports; return their paths."""

    output_path = Path(output)
    if output_path.suffix.lower() == ".json":
        json_path = output_path
        markdown_path = output_path.with_suffix(".md")
    else:
        output_path.mkdir(parents=True, exist_ok=True)
        json_path = output_path / "collision_cluster_report.json"
        markdown_path = output_path / "collision_cluster_report.md"
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(dict(report), indent=2, allow_nan=False), encoding="utf-8")
    markdown_path.write_text(render_markdown(report), encoding="utf-8")
    return json_path, markdown_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Cluster logged collision outcomes and emit frozen held-out diagnostics"
    )
    parser.add_argument("--input", "-i", nargs="+", required=True, help="JSON trace, summary, or case artifact")
    parser.add_argument("--output", "-o", type=Path, required=True, help="Output directory or .json report path")
    parser.add_argument(
        "--split",
        choices=("exploratory", "primary", "held-out"),
        default="exploratory",
        help="Report label; held-out uses immutable cluster bins",
    )
    parser.add_argument(
        "--evidence",
        choices=EVIDENCE_ROLES,
        default=EVIDENCE_EXPLORATORY_LOGGED,
        help="Evidence class to print in the report",
    )
    parser.add_argument(
        "--risk-bins",
        default=",".join(str(value) for value in FROZEN_HELD_OUT_RISK_BIN_EDGES),
        help="Exploratory pre-action-risk lower edges",
    )
    parser.add_argument(
        "--command-gap-bins",
        default=",".join(str(value) for value in FROZEN_HELD_OUT_COMMAND_GAP_BIN_EDGES),
        help="Exploratory command-gap lower edges",
    )
    return parser


def _parse_edges(text: str) -> tuple[float, ...]:
    values = tuple(float(part.strip()) for part in text.split(",") if part.strip())
    return _coerce_edges(values)


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        objects, sources = load_inputs(args.input, split=args.split)
        risk_edges = _parse_edges(args.risk_bins)
        gap_edges = _parse_edges(args.command_gap_bins)
        report = analyze_traces(
            objects,
            split=args.split,
            evidence_role=args.evidence,
            sources=sources,
            risk_bin_edges=risk_edges,
            command_gap_bin_edges=gap_edges,
        )
        json_path, markdown_path = write_report(report, args.output)
    except (OSError, TypeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(f"Wrote collision cluster report: {json_path}")
    print(f"Wrote collision cluster summary: {markdown_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
