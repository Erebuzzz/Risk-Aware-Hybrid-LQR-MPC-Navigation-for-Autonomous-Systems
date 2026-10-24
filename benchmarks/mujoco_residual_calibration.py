"""Deterministic residual identification for the controller-side reduced model.

This benchmark is deliberately separate from ``mujoco_calibration``.  It does
not recover MuJoCo parameters.  Instead, it keeps the controller-side
``AugmentedDifferentialDriveModel`` fixed, fits a small additive disturbance
model on primary free-space traces, and evaluates the resulting rollout on
disjoint held-out traces.

The three identified disturbances use the model's declared convention:

* ``d_v`` is a longitudinal acceleration residual in the actuator lag;
* ``d_y`` is a lateral body-velocity residual in the pose update;
* ``d_omega`` is a yaw-rate residual in the pose update.

The report is a sampled-model diagnostic.  Its finite residual envelope is the
maximum absolute endpoint residual observed on the primary traces; it is not an
intersample bound, a collision bound, or an automatic safety correction.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

from hybrid_controller.models.augmented_dynamics import (
    AugmentedDifferentialDriveModel,
    DynamicModelParams,
)


RESIDUAL_PARAMETER_NAMES = (
    "v_bias_mps2",
    "v_command_error_gain",
    "lateral_speed_gain",
    "omega_bias_rps",
    "omega_command_error_gain",
)

STATE_RESIDUAL_CHANNELS = (
    "position_x_m",
    "position_y_m",
    "yaw_rad",
    "linear_velocity_mps",
    "angular_velocity_rps",
)

# The benchmark intentionally uses the actual controller-side model defaults
# rather than fitting the actuator time constants or replacing the model with a
# simulator parameter recovery.  The identified parameters below only describe
# additive residuals passed to AugmentedDifferentialDriveModel.step().
CONTROLLER_MODEL_PARAMS = DynamicModelParams(dt=0.05, tau_v=0.10, tau_omega=0.10)


@dataclass(frozen=True)
class ResidualProbe:
    """One fixed command probe used for residual calibration."""

    name: str
    split: str
    dt: float
    friction: float
    initial_state: tuple[float, float, float, float, float]
    commands: tuple[tuple[float, float], ...]
    gravity_z: float = 0.0

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("probe name must be a nonempty string")
        if self.split not in {"primary", "held-out"}:
            raise ValueError(f"unsupported residual split: {self.split}")
        if not np.isfinite(self.dt) or self.dt <= 0.0:
            raise ValueError("probe dt must be positive and finite")
        if not np.isfinite(self.friction) or self.friction < 0.0:
            raise ValueError("probe friction must be nonnegative and finite")
        if not np.isfinite(self.gravity_z):
            raise ValueError("probe gravity_z must be finite")
        initial_state = np.asarray(self.initial_state, dtype=float)
        if initial_state.shape != (5,) or not np.all(np.isfinite(initial_state)):
            raise ValueError("initial_state must be a finite 5-vector")
        if not self.commands:
            raise ValueError("probe commands must not be empty")
        command_array = np.asarray(self.commands, dtype=float)
        if command_array.shape != (len(self.commands), 2) or not np.all(
            np.isfinite(command_array)
        ):
            raise ValueError("probe commands must be finite pairs")

    @property
    def command_array(self) -> np.ndarray:
        """Return a fresh ``(steps, 2)`` command array."""

        return np.asarray(self.commands, dtype=float).copy()


@dataclass
class ResidualTrace:
    """Fixed MuJoCo endpoint states for one residual probe."""

    probe: ResidualProbe
    states: np.ndarray
    contact_counts: np.ndarray


@dataclass(frozen=True)
class ResidualFitEstimate:
    """Residual coefficients identified from primary probes only."""

    v_bias_mps2: float
    v_command_error_gain: float
    lateral_speed_gain: float
    omega_bias_rps: float
    omega_command_error_gain: float
    cost: float
    nfev: int
    success: bool
    message: str
    fit_probe_names: tuple[str, ...]

    def as_parameter_dict(self) -> dict[str, float]:
        return {
            name: float(getattr(self, name)) for name in RESIDUAL_PARAMETER_NAMES
        }

    @property
    def coefficients(self) -> np.ndarray:
        """Return fitted coefficients in stable parameter order."""

        return np.asarray(
            [getattr(self, name) for name in RESIDUAL_PARAMETER_NAMES], dtype=float
        )


@dataclass(frozen=True)
class ResidualMetrics:
    """Sampled rollout residual metrics for one explicit split."""

    split: str
    probe_count: int
    sample_count: int
    position_x_rmse_m: float
    position_y_rmse_m: float
    position_rmse_m: float
    yaw_rmse_rad: float
    linear_velocity_rmse_mps: float
    angular_velocity_rmse_rps: float
    position_x_max_abs_m: float
    position_y_max_abs_m: float
    yaw_max_abs_rad: float
    linear_velocity_max_abs_mps: float
    angular_velocity_max_abs_rps: float
    maximum_state_abs_error: float


@dataclass(frozen=True)
class ResidualEnvelope:
    """Finite primary-sample residual envelope per state channel."""

    source_split: str
    source_probe_count: int
    source_sample_count: int
    position_x_abs_bound_m: float
    position_y_abs_bound_m: float
    yaw_abs_bound_rad: float
    linear_velocity_abs_bound_mps: float
    angular_velocity_abs_bound_rps: float
    maximum_state_abs_bound: float

    @property
    def bounds(self) -> dict[str, float]:
        return {
            "position_x_m": self.position_x_abs_bound_m,
            "position_y_m": self.position_y_abs_bound_m,
            "yaw_rad": self.yaw_abs_bound_rad,
            "linear_velocity_mps": self.linear_velocity_abs_bound_mps,
            "angular_velocity_rps": self.angular_velocity_abs_bound_rps,
        }


def _repeat_commands(
    blocks: Sequence[tuple[int, float, float]],
) -> tuple[tuple[float, float], ...]:
    commands: list[tuple[float, float]] = []
    for count, v_cmd, omega_cmd in blocks:
        if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
            raise ValueError("command block lengths must be positive integers")
        commands.extend([(float(v_cmd), float(omega_cmd))] * count)
    return tuple(commands)


def default_residual_probes() -> tuple[ResidualProbe, ...]:
    """Return fixed, independently authored primary and held-out probes."""

    return (
        ResidualProbe(
            name="residual_primary_ramp_turn",
            split="primary",
            dt=0.05,
            friction=0.65,
            initial_state=(0.0, 0.0, 0.0, 0.0, 0.0),
            commands=_repeat_commands(
                (
                    (2, 0.0, 0.0),
                    (3, 0.45, 0.0),
                    (4, 0.85, 0.35),
                    (5, 0.25, -0.55),
                    (4, -0.55, -0.25),
                    (3, 0.0, 0.0),
                    (4, 0.70, -0.40),
                )
            ),
        ),
        ResidualProbe(
            name="residual_primary_reverse_arc",
            split="primary",
            dt=0.05,
            friction=0.65,
            initial_state=(0.12, -0.08, 0.30, 0.20, -0.15),
            commands=_repeat_commands(
                (
                    (2, 0.0, 0.0),
                    (4, -0.40, 0.45),
                    (4, -0.75, 0.80),
                    (5, 0.35, -0.50),
                    (3, 1.00, 0.15),
                    (4, -0.15, -0.75),
                    (2, 0.0, 0.0),
                )
            ),
        ),
        ResidualProbe(
            name="residual_held_out_pulsed_arc",
            split="held-out",
            dt=0.05,
            friction=0.65,
            initial_state=(-0.18, 0.06, -0.45, -0.10, 0.18),
            commands=_repeat_commands(
                (
                    (1, 0.20, 0.85),
                    (2, 0.65, -0.45),
                    (3, -0.30, 0.35),
                    (2, 0.95, 0.10),
                    (3, 0.0, -0.95),
                    (2, -0.70, -0.60),
                    (2, 0.0, 0.0),
                )
            ),
        ),
        ResidualProbe(
            name="residual_held_out_asymmetric_steps",
            split="held-out",
            dt=0.05,
            friction=0.65,
            initial_state=(0.04, 0.14, 0.68, 0.08, -0.12),
            commands=_repeat_commands(
                (
                    (3, 0.30, 0.0),
                    (2, 0.80, 0.60),
                    (4, 0.05, -0.80),
                    (2, -0.60, -0.20),
                    (3, 0.50, 1.00),
                    (3, -0.20, 0.65),
                    (2, 0.0, 0.0),
                )
            ),
        ),
    )


def fixed_mujoco_config():
    """Return the fixed MuJoCo plant used only to generate comparison traces."""

    from benchmarks.mujoco_scene import MuJoCoPlantConfig

    return MuJoCoPlantConfig(
        dt=0.05,
        physics_dt=0.002,
        tau_v=0.12,
        tau_omega=0.09,
        force_gain=9.0,
        torque_gain=4.0,
        slip_gain=0.35,
        gravity_z=0.0,
    )


def probe_fingerprint(probe: ResidualProbe) -> str:
    """Hash probe content excluding split/name for leakage checks."""

    payload = {
        "dt": float(probe.dt),
        "friction": float(probe.friction),
        "initial_state": [float(value) for value in probe.initial_state],
        "commands": [[float(value) for value in command] for command in probe.commands],
        "gravity_z": float(probe.gravity_z),
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def validate_probe_partition(probes: Iterable[ResidualProbe]) -> None:
    """Reject duplicate or cross-split probes before trace generation."""

    rows = list(probes)
    if not rows:
        raise ValueError("at least one residual probe is required")
    names: set[str] = set()
    fingerprints: dict[str, str] = {}
    for probe in rows:
        if not isinstance(probe, ResidualProbe):
            raise ValueError("expected ResidualProbe values")
        probe.__post_init__()
        if probe.name in names:
            raise ValueError(f"duplicate residual probe name: {probe.name}")
        names.add(probe.name)
        fingerprint = probe_fingerprint(probe)
        previous_name = fingerprints.get(fingerprint)
        if previous_name is not None:
            raise ValueError(
                "duplicate residual probe content across the calibration partition: "
                f"{previous_name!r} and {probe.name!r}"
            )
        fingerprints[fingerprint] = probe.name


def _validated_trace(trace: ResidualTrace) -> ResidualTrace:
    """Validate and copy a complete, free-space trace before use."""

    if not isinstance(trace, ResidualTrace) or not isinstance(trace.probe, ResidualProbe):
        raise ValueError("expected a ResidualTrace with a ResidualProbe")
    trace.probe.__post_init__()
    steps = len(trace.probe.commands)
    try:
        states = np.asarray(trace.states, dtype=float)
        contact_counts = np.asarray(trace.contact_counts, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"trace {trace.probe.name!r} must contain numeric arrays") from exc
    if states.shape != (steps, 5) or not np.all(np.isfinite(states)):
        raise ValueError(
            f"trace {trace.probe.name!r} states must be finite with shape {(steps, 5)}"
        )
    if contact_counts.shape != (steps,) or not np.all(np.isfinite(contact_counts)):
        raise ValueError(
            f"trace {trace.probe.name!r} contact_counts must be finite with shape {(steps,)}"
        )
    if np.any(contact_counts < 0.0) or np.any(contact_counts != np.floor(contact_counts)):
        raise ValueError("contact_counts must contain nonnegative integer counts")
    if np.any(contact_counts > 0.0):
        raise ValueError(
            f"trace {trace.probe.name!r} contains contact samples; residual calibration "
            "requires free-space traces"
        )
    return ResidualTrace(
        probe=trace.probe,
        states=states.copy(),
        contact_counts=contact_counts.astype(int, copy=True),
    )


def _validated_trace_collection(
    traces: Iterable[ResidualTrace],
    *,
    expected_split: str | None = None,
) -> list[ResidualTrace]:
    rows = [_validated_trace(trace) for trace in traces]
    if not rows:
        raise ValueError("at least one residual trace is required")
    splits = {trace.probe.split for trace in rows}
    if len(splits) != 1:
        raise ValueError("primary and held-out residual traces cannot be mixed")
    split = next(iter(splits))
    if expected_split is not None and split != expected_split:
        raise ValueError(f"residual traces must be from the {expected_split} split only")
    names: set[str] = set()
    fingerprints: set[str] = set()
    for trace in rows:
        if trace.probe.name in names:
            raise ValueError(f"duplicate residual trace name: {trace.probe.name}")
        names.add(trace.probe.name)
        fingerprint = probe_fingerprint(trace.probe)
        if fingerprint in fingerprints:
            raise ValueError(f"duplicate residual trace content: {trace.probe.name}")
        fingerprints.add(fingerprint)
    return rows


def simulate_residual_probe(
    probe: ResidualProbe,
    config: Any | None = None,
) -> ResidualTrace:
    """Generate one deterministic MuJoCo endpoint trace for a fixed probe."""

    from benchmarks.mujoco_scene import MuJoCoPlant

    probe.__post_init__()
    plant_config = fixed_mujoco_config() if config is None else config
    if not np.isclose(plant_config.dt, probe.dt):
        raise ValueError("plant and residual probe dt must match")
    replay_config = replace(plant_config, gravity_z=probe.gravity_z)
    plant = MuJoCoPlant([], friction=probe.friction, config=replay_config)
    plant.reset(np.asarray(probe.initial_state, dtype=float))

    states: list[np.ndarray] = []
    contacts: list[int] = []
    for command in probe.command_array:
        states.append(np.asarray(plant.step(command), dtype=float).copy())
        contacts.append(plant.contact_count)
    return _validated_trace(
        ResidualTrace(
            probe=probe,
            states=np.asarray(states, dtype=float),
            contact_counts=np.asarray(contacts, dtype=int),
        )
    )


def _residual_disturbance(
    state: np.ndarray,
    control: np.ndarray,
    coefficients: Sequence[float],
) -> np.ndarray:
    """Evaluate the fixed five-parameter residual basis."""

    values = np.asarray(coefficients, dtype=float).reshape(len(RESIDUAL_PARAMETER_NAMES))
    if not np.all(np.isfinite(values)):
        raise ValueError("residual coefficients must be finite")
    state = np.asarray(state, dtype=float).reshape(5)
    control = np.asarray(control, dtype=float).reshape(2)
    if not np.all(np.isfinite(state)) or not np.all(np.isfinite(control)):
        raise ValueError("residual basis inputs must be finite")
    v_bias, v_error_gain, lateral_gain, omega_bias, omega_error_gain = values
    return np.asarray(
        [
            v_bias + v_error_gain * (control[0] - state[3]),
            lateral_gain * abs(state[3]),
            omega_bias + omega_error_gain * (control[1] - state[4]),
        ],
        dtype=float,
    )


def _rollout_predictions(
    probe: ResidualProbe,
    coefficients: Sequence[float],
) -> np.ndarray:
    model = AugmentedDifferentialDriveModel(CONTROLLER_MODEL_PARAMS)
    state = np.asarray(probe.initial_state, dtype=float).copy()
    predictions: list[np.ndarray] = []
    for command in probe.command_array:
        disturbance = _residual_disturbance(state, command, coefficients)
        state = model.step(state, command, disturbance)
        predictions.append(state.copy())
    return np.asarray(predictions, dtype=float)


def _state_residual(predicted: np.ndarray, observed: np.ndarray) -> np.ndarray:
    error = np.asarray(predicted, dtype=float) - np.asarray(observed, dtype=float)
    if error.shape != observed.shape or not np.all(np.isfinite(error)):
        raise ValueError("state residual must be finite and shape-compatible")
    error[:, 2] = np.arctan2(np.sin(error[:, 2]), np.cos(error[:, 2]))
    return error


def rollout_residuals(
    trace: ResidualTrace,
    fit: ResidualFitEstimate | Sequence[float],
) -> np.ndarray:
    """Return wrapped endpoint residuals for one trace without mutation."""

    trace = _validated_trace(trace)
    coefficients = fit.coefficients if isinstance(fit, ResidualFitEstimate) else fit
    predicted = _rollout_predictions(trace.probe, coefficients)
    return _state_residual(predicted, trace.states)


def fit_residual_model(
    primary_traces: Iterable[ResidualTrace],
    *,
    max_nfev: int = 200,
) -> ResidualFitEstimate:
    """Fit the additive residual basis on primary traces only."""

    traces = _validated_trace_collection(primary_traces, expected_split="primary")
    if isinstance(max_nfev, bool) or not isinstance(max_nfev, int) or max_nfev <= 0:
        raise ValueError("max_nfev must be a positive integer")

    from scipy.optimize import least_squares

    scale = np.asarray([1.0, 1.0, 0.5, 1.0, 1.0], dtype=float)

    def objective(coefficients: np.ndarray) -> np.ndarray:
        residual_rows: list[np.ndarray] = []
        for trace in traces:
            predicted = _rollout_predictions(trace.probe, coefficients)
            residual_rows.append(
                (_state_residual(predicted, trace.states) / scale).reshape(-1)
            )
        return np.concatenate(residual_rows)

    initial = np.zeros(len(RESIDUAL_PARAMETER_NAMES), dtype=float)
    lower = np.asarray([-5.0, -10.0, -2.0, -5.0, -10.0], dtype=float)
    upper = np.asarray([5.0, 10.0, 2.0, 5.0, 10.0], dtype=float)
    result = least_squares(
        objective,
        initial,
        bounds=(lower, upper),
        x_scale="jac",
        max_nfev=max_nfev,
        ftol=1.0e-10,
        xtol=1.0e-10,
        gtol=1.0e-10,
    )
    values = np.asarray(result.x, dtype=float)
    if not result.success or not np.all(np.isfinite(values)):
        raise RuntimeError(
            "reduced-model residual identification did not converge: "
            + str(result.message)
        )
    return ResidualFitEstimate(
        **{name: float(value) for name, value in zip(RESIDUAL_PARAMETER_NAMES, values)},
        cost=float(result.cost),
        nfev=int(result.nfev),
        success=bool(result.success),
        message=str(result.message),
        fit_probe_names=tuple(trace.probe.name for trace in traces),
    )


def evaluate_trace(trace: ResidualTrace, fit: ResidualFitEstimate) -> ResidualMetrics:
    """Evaluate one trace using a previously fitted residual model."""

    if not isinstance(fit, ResidualFitEstimate) or not fit.success:
        raise ValueError("evaluate_trace requires a converged ResidualFitEstimate")
    trace = _validated_trace(trace)
    error = rollout_residuals(trace, fit)
    return ResidualMetrics(
        split=trace.probe.split,
        probe_count=1,
        sample_count=len(error),
        position_x_rmse_m=float(np.sqrt(np.mean(error[:, 0] ** 2))),
        position_y_rmse_m=float(np.sqrt(np.mean(error[:, 1] ** 2))),
        position_rmse_m=float(np.sqrt(np.mean(error[:, :2] ** 2))),
        yaw_rmse_rad=float(np.sqrt(np.mean(error[:, 2] ** 2))),
        linear_velocity_rmse_mps=float(np.sqrt(np.mean(error[:, 3] ** 2))),
        angular_velocity_rmse_rps=float(np.sqrt(np.mean(error[:, 4] ** 2))),
        position_x_max_abs_m=float(np.max(np.abs(error[:, 0]))),
        position_y_max_abs_m=float(np.max(np.abs(error[:, 1]))),
        yaw_max_abs_rad=float(np.max(np.abs(error[:, 2]))),
        linear_velocity_max_abs_mps=float(np.max(np.abs(error[:, 3]))),
        angular_velocity_max_abs_rps=float(np.max(np.abs(error[:, 4]))),
        maximum_state_abs_error=float(np.max(np.abs(error))),
    )


def summarize_split(
    traces: Iterable[ResidualTrace],
    fit: ResidualFitEstimate,
) -> ResidualMetrics:
    """Aggregate metrics for exactly one split; mixed splits are rejected."""

    rows = _validated_trace_collection(traces)
    per_trace = [evaluate_trace(trace, fit) for trace in rows]
    sample_count = sum(row.sample_count for row in per_trace)

    def weighted_rmse(field: str) -> float:
        return float(
            np.sqrt(
                np.average(
                    [getattr(row, field) ** 2 for row in per_trace],
                    weights=[row.sample_count for row in per_trace],
                )
            )
        )

    return ResidualMetrics(
        split=per_trace[0].split,
        probe_count=len(per_trace),
        sample_count=sample_count,
        position_x_rmse_m=weighted_rmse("position_x_rmse_m"),
        position_y_rmse_m=weighted_rmse("position_y_rmse_m"),
        position_rmse_m=weighted_rmse("position_rmse_m"),
        yaw_rmse_rad=weighted_rmse("yaw_rmse_rad"),
        linear_velocity_rmse_mps=weighted_rmse("linear_velocity_rmse_mps"),
        angular_velocity_rmse_rps=weighted_rmse("angular_velocity_rmse_rps"),
        position_x_max_abs_m=float(max(row.position_x_max_abs_m for row in per_trace)),
        position_y_max_abs_m=float(max(row.position_y_max_abs_m for row in per_trace)),
        yaw_max_abs_rad=float(max(row.yaw_max_abs_rad for row in per_trace)),
        linear_velocity_max_abs_mps=float(
            max(row.linear_velocity_max_abs_mps for row in per_trace)
        ),
        angular_velocity_max_abs_rps=float(
            max(row.angular_velocity_max_abs_rps for row in per_trace)
        ),
        maximum_state_abs_error=float(max(row.maximum_state_abs_error for row in per_trace)),
    )


def build_residual_envelope(
    primary_traces: Iterable[ResidualTrace],
    fit: ResidualFitEstimate,
) -> ResidualEnvelope:
    """Build a finite, observed-sample envelope from primary residuals only."""

    traces = _validated_trace_collection(primary_traces, expected_split="primary")
    errors = [rollout_residuals(trace, fit) for trace in traces]
    stacked = np.concatenate(errors, axis=0)
    bounds = np.max(np.abs(stacked), axis=0)
    return ResidualEnvelope(
        source_split="primary",
        source_probe_count=len(traces),
        source_sample_count=len(stacked),
        position_x_abs_bound_m=float(bounds[0]),
        position_y_abs_bound_m=float(bounds[1]),
        yaw_abs_bound_rad=float(bounds[2]),
        linear_velocity_abs_bound_mps=float(bounds[3]),
        angular_velocity_abs_bound_rps=float(bounds[4]),
        maximum_state_abs_bound=float(np.max(bounds)),
    )


def envelope_evaluation(
    traces: Iterable[ResidualTrace],
    fit: ResidualFitEstimate,
    envelope: ResidualEnvelope,
) -> dict[str, Any]:
    """Count samples outside a primary-only envelope."""

    rows = _validated_trace_collection(traces)
    bounds = np.asarray(list(envelope.bounds.values()), dtype=float)
    stacked = np.concatenate([rollout_residuals(trace, fit) for trace in rows], axis=0)
    violations = np.abs(stacked) > bounds[np.newaxis, :]
    counts = {
        channel: int(np.sum(violations[:, index]))
        for index, channel in enumerate(STATE_RESIDUAL_CHANNELS)
    }
    return {
        "evaluated_split": rows[0].probe.split,
        "sample_count": int(len(stacked)),
        "violation_counts": counts,
        "violation_total": int(np.sum(violations)),
        "all_samples_within": bool(not np.any(violations)),
    }


def run_residual_calibration(*, max_nfev: int = 200) -> dict[str, Any]:
    """Fit primary residuals and evaluate disjoint held-out traces."""

    probes = default_residual_probes()
    validate_probe_partition(probes)
    plant_config = fixed_mujoco_config()
    primary_probes = [probe for probe in probes if probe.split == "primary"]
    held_out_probes = [probe for probe in probes if probe.split == "held-out"]
    if not primary_probes or not held_out_probes:
        raise ValueError("residual report requires both primary and held-out probes")
    primary_traces = [simulate_residual_probe(probe, plant_config) for probe in primary_probes]
    held_out_traces = [simulate_residual_probe(probe, plant_config) for probe in held_out_probes]

    fit = fit_residual_model(primary_traces, max_nfev=max_nfev)
    primary_metrics = summarize_split(primary_traces, fit)
    held_out_metrics = summarize_split(held_out_traces, fit)
    envelope = build_residual_envelope(primary_traces, fit)
    held_out_envelope = envelope_evaluation(held_out_traces, fit, envelope)

    return {
        "metadata": {
            "evidence_kind": "reduced_model_residual_identification",
            "controller_model": "AugmentedDifferentialDriveModel",
            "controller_state_order": ["p_x", "p_y", "yaw", "v", "omega"],
            "controller_input_order": ["v_cmd", "omega_cmd"],
            "residual_order": ["d_v", "d_y", "d_omega"],
            "primary_probe_names": [probe.name for probe in primary_probes],
            "held_out_probe_names": [probe.name for probe in held_out_probes],
            "fit_probe_names": list(fit.fit_probe_names),
            "held_out_used_for_fit": False,
            "primary_and_held_out_content_disjoint": True,
            "gravity_z_for_all_probes": 0.0,
            "friction_fixed_for_all_probes": 0.65,
            "physics_validation": "MuJoCo per-substep clock_warning_and_finite_state_checks",
            "trace_sampling": "state endpoints after each 0.05 s zero-order-held command",
            "residual_metric_definition": "predicted reduced-model state minus MuJoCo endpoint state; yaw wrapped to [-pi, pi)",
            "residual_envelope_definition": "maximum absolute sampled endpoint residual over fitted primary traces, per state channel",
            "inter_sample_limitations": [
                "The envelope is checked only at controller sample endpoints.",
                "MuJoCo physics runs at 0.002 s internally, but those inter-sample states are not part of the reported envelope.",
                "Zero-order-held commands and endpoint-only observations do not bound peak motion between samples or after the final sample.",
            ],
            "safety_correction_applied": False,
            "safety_claim_supported": False,
            "disclaimer": (
                "This is a sampled reduced-model mismatch diagnostic. The residual "
                "envelope is not automatically converted into a safety correction, "
                "constraint tightening, collision bound, or safety guarantee."
            ),
        },
        "controller_model_parameters": asdict(CONTROLLER_MODEL_PARAMS),
        "fixed_mujoco_plant_config": asdict(plant_config),
        "fit": asdict(fit),
        "estimated_residual_parameters": fit.as_parameter_dict(),
        "split_metrics": [asdict(primary_metrics), asdict(held_out_metrics)],
        "residual_envelope": asdict(envelope),
        "held_out_envelope_evaluation": held_out_envelope,
    }


def write_report(output: Path, report: dict[str, Any]) -> None:
    """Write deterministic JSON and CSV residual diagnostics."""

    output.mkdir(parents=True, exist_ok=True)
    (output / "mujoco_residual_calibration.json").write_text(
        json.dumps(report, indent=2, allow_nan=False), encoding="utf-8"
    )
    split_rows = report.get("split_metrics", [])
    if split_rows:
        with (output / "mujoco_residual_calibration_split_metrics.csv").open(
            "w", newline="", encoding="utf-8"
        ) as handle:
            writer = csv.DictWriter(handle, fieldnames=list(split_rows[0].keys()))
            writer.writeheader()
            writer.writerows(split_rows)
    envelope = report.get("residual_envelope")
    if envelope:
        with (output / "mujoco_residual_calibration_envelope.csv").open(
            "w", newline="", encoding="utf-8"
        ) as handle:
            writer = csv.DictWriter(handle, fieldnames=list(envelope.keys()))
            writer.writeheader()
            writer.writerow(envelope)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Identify controller-side reduced-model residuals against fixed MuJoCo traces"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("Output/Plots/Paper/mujoco_residual_calibration"),
    )
    parser.add_argument("--max-nfev", type=int, default=200)
    return parser


def main(argv: Sequence[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    report = run_residual_calibration(max_nfev=args.max_nfev)
    write_report(args.output, report)
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
