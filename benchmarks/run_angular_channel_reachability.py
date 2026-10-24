"""Run a deterministic diagnostic for the two-sided angular-channel trigger.

The utility evaluates the existing :func:`evaluate_angular_channel` decision
and then replays both turn directions in the headless MuJoCo plant.  It is an
experiment harness only: the diagnostic is not connected to the production
safety status, and plant outcomes are never fed back into the decision.

The primary and held-out scenario sets are generated from independent child
streams of one fixed seed.  No fit, threshold selection, or correction is
performed on either split.  Results keep the controller-trigger prediction,
the hidden actuator reachability observation, and obstacle-contact evidence in
separate fields so a trigger cannot be mistaken for collision avoidance.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from importlib.metadata import version
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PROJECT_ROOT / "src" / "hybrid_controller"
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from benchmarks.controlled_obstacles import ControlledObstacle, nonreactive_obstacle
from benchmarks.mujoco_scene import MuJoCoPlant, MuJoCoPlantConfig
from hybrid_controller.controllers.angular_channel import (
    AngularChannelDecision,
    evaluate_angular_channel,
    wrap_angle,
)
from hybrid_controller.models.footprint import (
    DEFAULT_CONTROLLER_ROBOT_RADIUS,
    MUJOCO_ROBOT_RADIUS,
)


SCHEMA_VERSION = 1
DEFAULT_SCENARIO_SEED = 20261024
PRIMARY_SCENARIO_COUNT = 24
HELD_OUT_SCENARIO_COUNT = 12
FIXED_FRICTION = 1.0
TURN_HORIZON_STEPS = 20
SIDE_TOLERANCE = 1.0e-3
REACH_TOLERANCE = 1.0e-9

# These are the existing angular-channel defaults written explicitly so that
# the report records the exact decision contract used by the experiment.
DECISION_PARAMETERS: dict[str, float] = {
    "dt": 0.05,
    "tau_omega": 0.10,
    "omega_min": -2.0,
    "omega_max": 2.0,
    "max_delta_omega": 0.15,
    "robot_radius": DEFAULT_CONTROLLER_ROBOT_RADIUS,
    "d_safe": 0.30,
    "channel_half_angle": float(np.pi / 4.0),
    "turn_gain": 2.0,
    "minimum_turn_rate": 0.35,
    "trigger_clearance": 0.0,
}

PLANT_CONFIG = MuJoCoPlantConfig(
    dt=DECISION_PARAMETERS["dt"],
    physics_dt=0.002,
    tau_v=0.10,
    tau_omega=DECISION_PARAMETERS["tau_omega"],
    force_gain=9.0,
    torque_gain=4.0,
    gravity_z=-9.81,
    slip_gain=0.0,
)

_SIDE_SIGN = {"left": 1.0, "right": -1.0}
_SPLIT_INDEX = {"primary": 0, "held-out": 1}
_DEFAULT_COUNTS = {
    "primary": PRIMARY_SCENARIO_COUNT,
    "held-out": HELD_OUT_SCENARIO_COUNT,
}


@dataclass(frozen=True)
class TurnScenario:
    """One immutable, non-reactive obstacle and initial plant state."""

    split: str
    index: int
    scenario_id: str
    initial_state: tuple[float, float, float, float, float]
    obstacle: ControlledObstacle
    previous_omega_command: float
    forward_command: float
    horizon_steps: int = TURN_HORIZON_STEPS

    def __post_init__(self) -> None:
        if self.split not in _SPLIT_INDEX:
            raise ValueError("split must be 'primary' or 'held-out'")
        if self.index < 0:
            raise ValueError("scenario index must be nonnegative")
        if not self.scenario_id:
            raise ValueError("scenario_id must not be empty")
        state = np.asarray(self.initial_state, dtype=float)
        if state.shape != (5,) or not np.all(np.isfinite(state)):
            raise ValueError("initial_state must be a finite 5-vector")
        if not np.isfinite(self.previous_omega_command):
            raise ValueError("previous_omega_command must be finite")
        if not np.isfinite(self.forward_command) or self.forward_command <= 0.0:
            raise ValueError("forward_command must be positive and finite")
        if self.horizon_steps <= 0:
            raise ValueError("horizon_steps must be positive")

    @property
    def obstacle_at_reset(self) -> dict[str, float]:
        """Return the static obstacle snapshot consumed by both paths."""

        state = self.obstacle.state_at(0.0)
        return {
            "x": float(state["x"]),
            "y": float(state["y"]),
            "radius": float(state["radius"]),
        }


def _canonical_split(split: str) -> str:
    if split == "held_out":
        split = "held-out"
    if split not in _SPLIT_INDEX:
        raise ValueError("split must be 'primary' or 'held-out'")
    return split


def _rng_for_split(split: str, seed: int) -> np.random.Generator:
    """Return an independent deterministic child stream for one split."""

    if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)):
        raise ValueError("seed must be an integer")
    if seed < 0:
        raise ValueError("seed must be nonnegative")
    children = np.random.SeedSequence(int(seed)).spawn(2)
    return np.random.default_rng(children[_SPLIT_INDEX[_canonical_split(split)]])


def generate_scenarios(
    split: str,
    *,
    seed: int = DEFAULT_SCENARIO_SEED,
    count: int | None = None,
) -> tuple[TurnScenario, ...]:
    """Generate a fixed-seed primary or held-out scenario set.

    The two streams use the same frozen generation rule but independent
    ``SeedSequence`` children.  The held-out stream is never inspected by the
    evaluator while creating a decision, fitting a parameter, or selecting a
    threshold.
    """

    canonical_split = _canonical_split(split)
    if count is None:
        count = _DEFAULT_COUNTS[canonical_split]
    if isinstance(count, bool) or not isinstance(count, (int, np.integer)) or count <= 0:
        raise ValueError("count must be a positive integer")
    count_value = int(count)

    rng = _rng_for_split(canonical_split, int(seed))
    scenarios: list[TurnScenario] = []
    for index in range(count_value):
        # Keep all obstacles in the forward angular channel while retaining
        # centerline cases for the two-sided tie-break path.
        if index % 6 == 0:
            lateral_offset = 0.0
        else:
            lateral_offset = float(rng.uniform(0.04, 0.28))
            lateral_offset *= -1.0 if index % 2 else 1.0

        if canonical_split == "primary":
            distance = float(rng.uniform(0.34, 0.62))
            radius = float(rng.uniform(0.09, 0.15))
            forward_command = float(rng.uniform(0.35, 0.75))
        else:
            # The held-out range is generated independently and is not a
            # calibration tail of the primary cases.
            distance = float(rng.uniform(0.38, 0.74))
            radius = float(rng.uniform(0.08, 0.14))
            forward_command = float(rng.uniform(0.30, 0.70))

        previous_omega = float(rng.uniform(-1.35, 1.35))
        actual_omega = float(previous_omega * rng.uniform(0.55, 0.85))
        speed = float(forward_command * rng.uniform(0.75, 1.05))
        obstacle = nonreactive_obstacle(
            distance,
            lateral_offset,
            radius,
        )
        scenarios.append(
            TurnScenario(
                split=canonical_split,
                index=index,
                scenario_id=f"{canonical_split.replace('-', '_')}_{index:03d}",
                initial_state=(0.0, 0.0, 0.0, speed, actual_omega),
                obstacle=obstacle,
                previous_omega_command=previous_omega,
                forward_command=forward_command,
            )
        )
    return tuple(scenarios)


def _decision_for_scenario(scenario: TurnScenario) -> AngularChannelDecision:
    return evaluate_angular_channel(
        np.asarray(scenario.initial_state, dtype=float),
        scenario.obstacle_at_reset,
        previous_omega_command=scenario.previous_omega_command,
        **DECISION_PARAMETERS,
    )


def _slew_limited_command(
    previous_command: float,
    target: float,
    *,
    max_delta: float,
    omega_min: float,
    omega_max: float,
) -> float:
    lower = max(omega_min, previous_command - max_delta)
    upper = min(omega_max, previous_command + max_delta)
    return float(np.clip(target, lower, upper))


def _obstacle_contact_capability(plant: Any) -> tuple[bool, str, int | None, int | None]:
    """Return raw MuJoCo obstacle-pair capability without changing telemetry."""

    try:
        robot_geom_id = int(plant.model.geom("robot_body").id)
        obstacle_geom_id = int(plant.model.geom("obstacle_0").id)
        _ = int(plant.data.ncon)
        # Accessing an empty contact array is intentionally avoided.  The
        # model/data attributes above are the capability check; contact pairs
        # are inspected only after each dynamics step.
    except (AttributeError, KeyError, TypeError, ValueError, RuntimeError):
        return False, "unavailable", None, None
    return (
        True,
        "available_via_direct_mjdata_geom_pairs",
        robot_geom_id,
        obstacle_geom_id,
    )


def _obstacle_contact_observed(
    plant: Any,
    capability: tuple[bool, str, int | None, int | None],
) -> bool | None:
    available, _, robot_geom_id, obstacle_geom_id = capability
    if not available or robot_geom_id is None or obstacle_geom_id is None:
        return None
    try:
        ncon = int(plant.data.ncon)
        for index in range(ncon):
            contact = plant.data.contact[index]
            pair = {int(contact.geom1), int(contact.geom2)}
            if pair == {int(robot_geom_id), int(obstacle_geom_id)}:
                return True
    except (AttributeError, IndexError, TypeError, ValueError, RuntimeError):
        return None
    return False


def _total_contact_count(plant: Any) -> int | None:
    """Read the existing total-contact API, which includes floor contacts."""

    try:
        return int(plant.contact_count)
    except (AttributeError, TypeError, ValueError, RuntimeError):
        return None


def _side_from_signed_value(value: float) -> str:
    if value > SIDE_TOLERANCE:
        return "left"
    if value < -SIDE_TOLERANCE:
        return "right"
    return "none"


def _side_outcome(
    scenario: TurnScenario,
    decision: AngularChannelDecision,
    side: str,
    *,
    plant_factory: Callable[..., Any] = MuJoCoPlant,
) -> dict[str, Any]:
    """Run one open-loop side in a fresh deterministic plant instance."""

    sign = _SIDE_SIGN[side]
    obstacle = scenario.obstacle_at_reset
    target_omega = sign * decision.required_turn_rate
    plant = plant_factory(
        [obstacle],
        friction=FIXED_FRICTION,
        config=PLANT_CONFIG,
    )
    plant.reset(np.asarray(scenario.initial_state, dtype=float))
    capability = _obstacle_contact_capability(plant)

    command_omega = _slew_limited_command(
        scenario.previous_omega_command,
        DECISION_PARAMETERS["omega_max"]
        if side == "left"
        else DECISION_PARAMETERS["omega_min"],
        max_delta=DECISION_PARAMETERS["max_delta_omega"],
        omega_min=DECISION_PARAMETERS["omega_min"],
        omega_max=DECISION_PARAMETERS["omega_max"],
    )
    first_command_omega = command_omega
    states = [np.asarray(scenario.initial_state, dtype=float).copy()]
    actuator_states: list[np.ndarray] = []
    obstacle_contacts: list[bool | None] = []
    total_contacts: list[int | None] = []
    for step_index in range(scenario.horizon_steps):
        if step_index > 0:
            command_omega = _slew_limited_command(
                command_omega,
                target_omega,
                max_delta=DECISION_PARAMETERS["max_delta_omega"],
                omega_min=DECISION_PARAMETERS["omega_min"],
                omega_max=DECISION_PARAMETERS["omega_max"],
            )
        state = np.asarray(
            plant.step(np.array([scenario.forward_command, command_omega], dtype=float)),
            dtype=float,
        )
        states.append(state.copy())
        try:
            actuator = np.asarray(plant.actuator_state(), dtype=float).reshape(2)
        except (AttributeError, TypeError, ValueError, RuntimeError):
            actuator = np.array([np.nan, np.nan], dtype=float)
        actuator_states.append(actuator.copy())
        obstacle_contacts.append(_obstacle_contact_observed(plant, capability))
        total_contacts.append(_total_contact_count(plant))

    state_array = np.asarray(states, dtype=float)
    actuator_array = np.asarray(actuator_states, dtype=float)
    initial = state_array[0]
    final = state_array[-1]
    yaw_delta = wrap_angle(float(final[2] - initial[2]))
    max_x = float(np.max(state_array[:, 0]))
    distances = np.hypot(state_array[:, 0] - obstacle["x"], state_array[:, 1] - obstacle["y"])
    min_clearance = float(
        np.min(distances - obstacle["radius"] - PLANT_CONFIG.robot_radius)
    )
    crossed_obstacle_plane = bool(
        max_x >= obstacle["x"] + obstacle["radius"] + PLANT_CONFIG.robot_radius
    )
    reached_yaw_side = _side_from_signed_value(yaw_delta)
    reached_position_side = _side_from_signed_value(float(final[1] - initial[1]))
    signed_first_hidden_omega = (
        float(actuator_array[0, 1]) * sign if actuator_array.size else np.nan
    )
    first_step_reached = bool(
        np.isfinite(signed_first_hidden_omega)
        and signed_first_hidden_omega + REACH_TOLERANCE >= decision.required_turn_rate
    )

    known_contacts = [value for value in obstacle_contacts if value is not None]
    if known_contacts:
        obstacle_contact: bool | None = bool(any(known_contacts))
        obstacle_contact_steps: int | None = int(sum(bool(value) for value in known_contacts))
    else:
        obstacle_contact = None
        obstacle_contact_steps = None

    if obstacle_contact is True:
        outcome_class = "obstacle_contact"
    elif obstacle_contact is False and crossed_obstacle_plane:
        outcome_class = "passed_without_obstacle_contact"
    elif obstacle_contact is False:
        outcome_class = "no_obstacle_contact_before_obstacle_plane"
    else:
        outcome_class = "obstacle_contact_telemetry_unavailable"

    known_total_contacts = [value for value in total_contacts if value is not None]
    return {
        "side": side,
        "target_turn_rate_rad_s": float(target_omega),
        "first_command_omega_rad_s": float(first_command_omega),
        "first_hidden_omega_rad_s": float(actuator_array[0, 1]),
        "first_body_omega_rad_s": float(state_array[1, 4]),
        "first_step_reached_required_turn_rate": first_step_reached,
        "final_x_m": float(final[0]),
        "final_y_m": float(final[1]),
        "final_yaw_rad": float(final[2]),
        "yaw_delta_rad": float(yaw_delta),
        "lateral_displacement_m": float(final[1] - initial[1]),
        "max_x_m": max_x,
        "minimum_geometric_clearance_m": min_clearance,
        "obstacle_plane_reached": crossed_obstacle_plane,
        "reached_yaw_side": reached_yaw_side,
        "reached_position_side": reached_position_side,
        "yaw_side_matches_command": reached_yaw_side == side,
        "position_side_matches_command": reached_position_side == side,
        "obstacle_contact": obstacle_contact,
        "obstacle_contact_steps": obstacle_contact_steps,
        "finite_no_contact_plane_crossing": (
            bool(obstacle_contact is False and crossed_obstacle_plane)
            if obstacle_contact is not None
            else None
        ),
        "total_contact_steps": (
            int(sum(value > 0 for value in known_total_contacts))
            if known_total_contacts
            else None
        ),
        "contact_telemetry": capability[1],
        "outcome_class": outcome_class,
        "steps": scenario.horizon_steps,
    }


def evaluate_scenario(
    scenario: TurnScenario,
    *,
    plant_factory: Callable[..., Any] = MuJoCoPlant,
) -> dict[str, Any]:
    """Evaluate both turn directions while keeping prediction and outcome separate."""

    decision = _decision_for_scenario(scenario)
    left = _side_outcome(scenario, decision, "left", plant_factory=plant_factory)
    right = _side_outcome(scenario, decision, "right", plant_factory=plant_factory)
    selected = left if decision.selected_side == "left" else right
    predicted_reachable = bool(decision.selected_margin >= 0.0)

    result: dict[str, Any] = {
        "split": scenario.split,
        "index": int(scenario.index),
        "scenario_id": scenario.scenario_id,
        "initial_state": [float(value) for value in scenario.initial_state],
        "previous_omega_command_rad_s": float(scenario.previous_omega_command),
        "forward_command_m_s": float(scenario.forward_command),
        "horizon_steps": int(scenario.horizon_steps),
        "obstacle": scenario.obstacle_at_reset,
        "decision": {
            **asdict(decision),
            "selected_side_reachable_prediction": predicted_reachable,
            "trigger_prediction_label": "controller_trigger_only",
        },
        "plant": {
            "left": left,
            "right": right,
            "selected_side": decision.selected_side,
            "selected_side_outcome": selected,
            "selected_side_first_step_reached_required_turn_rate": bool(
                selected["first_step_reached_required_turn_rate"]
            ),
            "actual_avoidance_evidence": selected["finite_no_contact_plane_crossing"],
            "avoidance_claim": (
                "finite_open_loop_no_contact_and_plane_crossing"
                if selected["outcome_class"] == "passed_without_obstacle_contact"
                else "not_established"
            ),
            "contact_telemetry_note": (
                "Obstacle-specific contact was inspected from direct MuJoCo "
                "geom-pair data; plant.contact_count remains total/floor-inclusive."
                if selected["obstacle_contact"] is not None
                else "Obstacle-specific contact telemetry is unavailable; no contact "
                "or avoidance claim is made."
            ),
        },
        "diagnostic_scope": {
            "production_safety_connected": False,
            "safety_claim_supported": False,
            "trigger_is_not_contact": True,
            "plant_replay_is_open_loop": True,
            "held_out_used_for_tuning": False,
        },
    }
    # Keep the most frequently inspected decision fields at the top level for
    # CSV/report consumers while retaining the complete nested decision object.
    result.update(
        {
            "decision_selected_side": decision.selected_side,
            "decision_trigger": bool(decision.trigger),
            "decision_left_margin": float(decision.left_margin),
            "decision_right_margin": float(decision.right_margin),
            "decision_selected_margin": float(decision.selected_margin),
            "decision_clearance_m": float(decision.clearance),
            "decision_required_turn_rate_rad_s": float(decision.required_turn_rate),
            "decision_selected_side_reachable_prediction": predicted_reachable,
            "plant_selected_outcome_class": selected["outcome_class"],
            "plant_selected_obstacle_contact": selected["obstacle_contact"],
            "plant_selected_first_step_reached_required_turn_rate": selected[
                "first_step_reached_required_turn_rate"
            ],
            "plant_selected_actual_avoidance_evidence": selected[
                "finite_no_contact_plane_crossing"
            ],
            "plant_selected_final_x_m": selected["final_x_m"],
            "plant_selected_final_y_m": selected["final_y_m"],
            "plant_selected_yaw_delta_rad": selected["yaw_delta_rad"],
        }
    )
    return result


def run_split(
    split: str,
    *,
    seed: int = DEFAULT_SCENARIO_SEED,
    count: int | None = None,
    plant_factory: Callable[..., Any] = MuJoCoPlant,
) -> list[dict[str, Any]]:
    """Run one frozen split; no cross-split values are used."""

    return [
        evaluate_scenario(scenario, plant_factory=plant_factory)
        for scenario in generate_scenarios(split, seed=seed, count=count)
    ]


def _contact_status(records: Sequence[Mapping[str, Any]]) -> str:
    values = [record["plant"]["selected_side_outcome"]["obstacle_contact"] for record in records]
    known = [value for value in values if value is not None]
    if not known:
        return "unavailable"
    if len(known) != len(values):
        return "mixed"
    return "available"


def summarize_split(records: Iterable[Mapping[str, Any]]) -> dict[str, Any]:
    """Summarize predictions and plant observations without calling them safety."""

    rows = list(records)
    if not rows:
        raise ValueError("at least one record is required")
    selected_outcomes = [row["plant"]["selected_side_outcome"] for row in rows]
    contact_values = [outcome["obstacle_contact"] for outcome in selected_outcomes]
    known_contact_values = [value for value in contact_values if value is not None]
    trigger_count = sum(bool(row["decision"]["trigger"]) for row in rows)
    predicted_reachable_count = sum(
        bool(row["decision"]["selected_side_reachable_prediction"]) for row in rows
    )
    first_step_reached_count = sum(
        bool(outcome["first_step_reached_required_turn_rate"])
        for outcome in selected_outcomes
    )
    passed_without_contact_count = sum(
        outcome["outcome_class"] == "passed_without_obstacle_contact"
        for outcome in selected_outcomes
    )
    return {
        "case_count": len(rows),
        "controller_trigger_prediction_count": int(trigger_count),
        "selected_side_reachable_prediction_count": int(predicted_reachable_count),
        "selected_side_first_step_reached_required_turn_rate_count": int(
            first_step_reached_count
        ),
        "selected_side_yaw_match_count": int(
            sum(outcome["yaw_side_matches_command"] for outcome in selected_outcomes)
        ),
        "selected_side_position_match_count": int(
            sum(outcome["position_side_matches_command"] for outcome in selected_outcomes)
        ),
        "obstacle_contact_telemetry_status": _contact_status(rows),
        "selected_side_obstacle_contact_observed_count": (
            int(sum(bool(value) for value in known_contact_values))
            if known_contact_values
            else None
        ),
        "selected_side_obstacle_contact_unknown_count": int(
            sum(value is None for value in contact_values)
        ),
        "selected_side_passed_without_obstacle_contact_count": (
            int(passed_without_contact_count) if known_contact_values else None
        ),
        "selected_side_actual_avoidance_evidence_count": (
            int(
                sum(
                    outcome["finite_no_contact_plane_crossing"] is True
                    for outcome in selected_outcomes
                )
            )
            if known_contact_values
            else None
        ),
        "prediction_and_plant_reach_are_separate": True,
        "contact_is_not_inferred_from_trigger": True,
        "avoidance_evidence_is_finite_open_loop_only": True,
        "held_out_used_for_tuning": False,
    }


def _source_hashes() -> dict[str, str]:
    paths = (
        Path(__file__),
        SOURCE_ROOT / "hybrid_controller" / "controllers" / "angular_channel.py",
        PROJECT_ROOT / "benchmarks" / "mujoco_scene.py",
        PROJECT_ROOT / "benchmarks" / "controlled_obstacles.py",
    )
    return {
        path.relative_to(PROJECT_ROOT).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in paths
    }


def _metadata(
    seed: int,
    primary: Sequence[Mapping[str, Any]],
    held_out: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    contact_statuses = [
        _contact_status(primary),
        _contact_status(held_out),
    ]
    contact_status = "available" if all(status == "available" for status in contact_statuses) else (
        "unavailable" if all(status == "unavailable" for status in contact_statuses) else "mixed"
    )
    return {
        "schema_version": SCHEMA_VERSION,
        "benchmark": "angular_channel_reachability",
        "evidence_kind": "diagnostic_controller_vs_mujoco_plant",
        "seed": int(seed),
        "split_counts": {
            "primary": len(primary),
            "held-out": len(held_out),
        },
        "held_out_frozen": True,
        "held_out_used_for_tuning": False,
        "production_safety_connected": False,
        "safety_claim_supported": False,
        "decision_contract": {
            "function": "hybrid_controller.controllers.angular_channel.evaluate_angular_channel",
            "parameters": dict(DECISION_PARAMETERS),
            "selected_side_reachable_rule": "selected_margin >= 0",
            "trigger_rule": "forward_channel and clearance <= trigger_clearance and selected_margin < 0",
            "robot_radius_semantics": "controller default profile",
        },
        "plant_contract": {
            "plant": "MuJoCoPlant",
            "config": asdict(PLANT_CONFIG),
            "friction": FIXED_FRICTION,
            "obstacles": "one static nonreactive controlled obstacle per case",
            "turn_replay": "both signs, command-slew limited at each control interval",
            "horizon_steps": TURN_HORIZON_STEPS,
            "plant_robot_radius_m": MUJOCO_ROBOT_RADIUS,
        },
        "contact_telemetry": {
            "status": contact_status,
            "obstacle_specific_source": (
                "direct MuJoCo data.contact geom pairs"
                if contact_status == "available"
                else "unavailable" if contact_status == "unavailable" else "mixed"
            ),
            "public_contact_count_semantics": "total mjData.ncon including floor, not obstacle-specific",
            "sample_rule": "post-control-interval contact snapshot; not a maximum over physics substeps",
        },
        "interpretation": {
            "controller_trigger": "diagnostic prediction that the selected turn is not reachable under the decision model",
            "hidden_actuator_result": "plant replay observation of first-step actuator-state turn rate",
            "avoidance": "finite open-loop outcome only; no production safety or collision guarantee",
            "unknown_contact": "null obstacle_contact means contact telemetry was unavailable, not no contact",
        },
        "provenance": {
            "python": sys.version,
            "python_executable": sys.executable,
            "platform": platform.platform(),
            "packages": {
                name: version(name) for name in ("numpy", "mujoco")
            },
            "source_sha256": _source_hashes(),
        },
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
    }


def run_experiment(
    *,
    seed: int = DEFAULT_SCENARIO_SEED,
    primary_count: int | None = None,
    held_out_count: int | None = None,
    plant_factory: Callable[..., Any] = MuJoCoPlant,
) -> dict[str, Any]:
    """Run both splits and return a JSON-ready diagnostic report."""

    if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)) or seed < 0:
        raise ValueError("seed must be a nonnegative integer")
    primary = run_split(
        "primary",
        seed=int(seed),
        count=primary_count,
        plant_factory=plant_factory,
    )
    held_out = run_split(
        "held-out",
        seed=int(seed),
        count=held_out_count,
        plant_factory=plant_factory,
    )
    metadata = _metadata(int(seed), primary, held_out)
    metadata["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
    metadata["source_sha256_after"] = _source_hashes()
    metadata["source_unchanged_during_run"] = (
        metadata["provenance"]["source_sha256"] == metadata["source_sha256_after"]
    )
    return {
        "metadata": metadata,
        "primary_metrics": summarize_split(primary),
        "held_out_metrics": summarize_split(held_out),
        "cases": primary + held_out,
    }


def _flatten_case(case: Mapping[str, Any]) -> dict[str, Any]:
    """Flatten nested decision/plant fields for a stable CSV sidecar."""

    decision = case["decision"]
    plant = case["plant"]
    row: dict[str, Any] = {
        key: value
        for key, value in case.items()
        if key not in {"decision", "plant", "diagnostic_scope", "obstacle", "initial_state"}
    }
    row.update(
        {
            "initial_x_m": case["initial_state"][0],
            "initial_y_m": case["initial_state"][1],
            "initial_yaw_rad": case["initial_state"][2],
            "initial_v_m_s": case["initial_state"][3],
            "initial_omega_rad_s": case["initial_state"][4],
            "obstacle_x_m": case["obstacle"]["x"],
            "obstacle_y_m": case["obstacle"]["y"],
            "obstacle_radius_m": case["obstacle"]["radius"],
        }
    )
    for key, value in decision.items():
        row[f"decision_{key}"] = value
    for prefix, outcome in (
        ("left", plant["left"]),
        ("right", plant["right"]),
        ("selected", plant["selected_side_outcome"]),
    ):
        for key, value in outcome.items():
            row[f"plant_{prefix}_{key}"] = value
    row["plant_selected_side"] = plant["selected_side"]
    row["plant_avoidance_claim"] = plant["avoidance_claim"]
    row["plant_contact_telemetry_note"] = plant["contact_telemetry_note"]
    return row


def write_report(report: Mapping[str, Any], output: Path) -> tuple[Path, Path]:
    """Write JSON and CSV artifacts without overwriting an earlier run."""

    output = Path(output)
    cases = report.get("cases")
    if not isinstance(cases, list) or not cases:
        raise ValueError("report cases are required")
    output.mkdir(parents=True, exist_ok=True)
    json_path = output / "angular_channel_reachability.json"
    csv_path = output / "angular_channel_reachability.csv"
    if json_path.exists() or csv_path.exists():
        raise FileExistsError("angular-channel report artifacts already exist")

    json_path.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    rows = [_flatten_case(case) for case in cases]
    fieldnames = list(rows[0].keys())
    with csv_path.open("x", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return json_path, csv_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the fixed-seed diagnostic angular-channel MuJoCo experiment"
    )
    parser.add_argument("--seed", type=int, default=DEFAULT_SCENARIO_SEED)
    parser.add_argument("--primary-count", type=int, default=None)
    parser.add_argument("--held-out-count", type=int, default=None)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("Output/Plots/Paper/angular_channel_reachability"),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    report = run_experiment(
        seed=args.seed,
        primary_count=args.primary_count,
        held_out_count=args.held_out_count,
    )
    json_path, csv_path = write_report(report, args.output)
    print(
        json.dumps(
            {
                "benchmark": report["metadata"]["benchmark"],
                "seed": report["metadata"]["seed"],
                "primary_metrics": report["primary_metrics"],
                "held_out_metrics": report["held_out_metrics"],
                "json": str(json_path),
                "csv": str(csv_path),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
