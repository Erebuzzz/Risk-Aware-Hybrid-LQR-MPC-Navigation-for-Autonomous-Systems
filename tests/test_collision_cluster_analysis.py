"""Focused tests for clustered collision and frozen held-out reporting."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from benchmarks.analyze_collision_clusters import (
    CollisionOutcome,
    EVIDENCE_EXPLORATORY_LOGGED,
    EVIDENCE_PRIMARY_CONTROLLED,
    analyze_outcomes,
    analyze_records,
    analyze_traces,
    frozen_held_out_report,
    load_inputs,
    write_report,
)


def _row(
    step: int, risk: float, gap: float, clearance: float, *, collision: bool = False
) -> dict[str, float | bool]:
    return {
        "step": float(step),
        "risk": risk,
        "pre_v": 0.5,
        "previous_applied_v": 0.5 + gap,
        "requested_command_v": 2.5,
        "post_step_clearance": clearance,
        "collision": collision,
    }


def test_logged_traces_cluster_by_decision_time_risk_and_derived_command_gap():
    traces = [
        [_row(0, 0.10, 0.10, 0.20), _row(1, 0.20, 0.15, 0.10)],
        [_row(0, 0.70, 0.20, 0.10), _row(1, 0.80, 0.30, -0.01, collision=True)],
        [_row(0, 0.50, 0.05, 0.15), _row(1, 0.60, 0.20, 0.08)],
    ]

    report = analyze_traces(traces, evidence_role=EVIDENCE_EXPLORATORY_LOGGED)

    assert report["analysis_scope"] == "exploratory_logged"
    assert report["evidence_role"] == EVIDENCE_EXPLORATORY_LOGGED
    assert report["raw_counts"] == {
        "trial_count": 3,
        "known_outcome_count": 3,
        "collision_count": 1,
        "noncollision_count": 2,
        "unknown_outcome_count": 0,
        "collision_rate": pytest.approx(1 / 3),
        "risk_available_count": 3,
        "command_gap_available_count": 3,
    }

    risk_clusters = {
        row["bin"]: row for row in report["clusters"]["pre_action_risk"]
    }
    assert risk_clusters["[0,0.25)"]["noncollision_count"] == 1
    assert risk_clusters["[0.75,1)"]["collision_count"] == 1

    gap_clusters = {row["bin"]: row for row in report["clusters"]["command_gap"]}
    assert gap_clusters["[0,0.25)"]["trial_count"] == 2
    assert gap_clusters["[0.25,0.75)"]["collision_count"] == 1


def test_missing_collision_signal_remains_unknown_and_is_not_a_safe_trial():
    report = analyze_traces([[{"step": 0, "risk": 0.9, "command_gap": 1.2}]])

    assert report["raw_counts"]["trial_count"] == 1
    assert report["raw_counts"]["collision_count"] == 0
    assert report["raw_counts"]["noncollision_count"] == 0
    assert report["raw_counts"]["unknown_outcome_count"] == 1
    risk_row = next(
        row for row in report["clusters"]["pre_action_risk"] if row["bin"] == "[0.75,1)"
    )
    assert risk_row["unknown_outcome_count"] == 1
    assert "never counted as a non-collision" in " ".join(report["notes"])


def test_nominal_safety_labels_do_not_become_physical_collision_outcomes():
    report = analyze_traces(
        [{"discrete_safe": False, "continuous_safe": False, "risk": 0.9}]
    )

    assert report["raw_counts"]["known_outcome_count"] == 0
    assert report["raw_counts"]["unknown_outcome_count"] == 1


def test_frozen_held_out_report_preserves_counts_and_evidence_role():
    records = [
        {
            "split": "held-out",
            "collision": True,
            "pre_action_risk": 0.8,
            "command_gap": 0.25,
        },
        {
            "split": "held-out",
            "collision": False,
            "pre_action_risk": 0.2,
            "command_gap": 0.75,
        },
        {
            "split": "held-out",
            "collision_count": 0,
            "pre_action_risk": 0.4,
            "command_gap": 1.25,
        },
    ]

    report = frozen_held_out_report(records)

    assert report["analysis_scope"] == "frozen_held_out"
    assert report["frozen"] is True
    assert report["evidence_role"] == EVIDENCE_PRIMARY_CONTROLLED
    assert report["raw_counts"]["collision_count"] == 1
    assert report["raw_counts"]["noncollision_count"] == 2
    assert report["cluster_spec"]["command_gap_edges"] == [0.0, 0.25, 0.75, 1.25]
    assert any("frozen cluster edges" in note for note in report["notes"])


def test_held_out_bins_cannot_be_changed_after_split_is_declared():
    outcome_report = analyze_traces(
        [{"collision": False, "pre_action_risk": 0.1, "command_gap": 0.1}],
        split="exploratory",
    )
    outcome = outcome_report["outcomes"]

    normalized = [CollisionOutcome(**outcome[0])]
    with pytest.raises(ValueError, match="held-out risk bins are frozen"):
        analyze_outcomes(normalized, split="held-out", risk_bin_edges=(0.0, 0.5))


def test_report_writer_and_loader_keep_scope_labels(tmp_path: Path):
    trace_path = tmp_path / "trace.json"
    trace_path.write_text(
        json.dumps([_row(0, 0.2, 0.1, 0.2), _row(1, 0.3, 0.2, 0.1)]),
        encoding="utf-8",
    )
    objects, sources = load_inputs([trace_path])
    report = analyze_traces(objects, sources=sources)
    json_path, markdown_path = write_report(report, tmp_path / "report")

    saved = json.loads(json_path.read_text(encoding="utf-8"))
    markdown = markdown_path.read_text(encoding="utf-8")
    assert saved["analysis_scope"] == "exploratory_logged"
    assert "secondary exploratory logged analysis" in markdown
    assert "Raw outcome counts" in markdown


def test_virtual_stopping_boundary_is_not_an_observed_collision():
    report = analyze_records([
        {
            "split": "primary", "plant_safe": False, "minimum_clearance": -0.4,
            "continuous_safe": False, "discrete_safe": False, "command_gap": 0.75,
            "plant_distance": 0.9, "obstacle_boundary_distance": 0.5,
        }
    ])
    assert report["raw_counts"]["collision_count"] == 0
    assert report["raw_counts"]["noncollision_count"] == 0
    assert report["raw_counts"]["unknown_outcome_count"] == 1
    assert report["boundary_proxy_counts"]["boundary_proxy_violation_count"] == 1
    assert report["outcomes"][0]["collision_source"] == "unavailable"


def test_controlled_sweep_v2_report_preserves_proxy_and_unknown_outcomes(tmp_path):
    payload = {
        "metadata": {"schema_version": 2, "split": "held-out"},
        "cases": [
            {
                "plant_proxy_safe": safe,
                "virtual_boundary_margin_m": margin,
                "measurement": {"status": status},
                "command_gap": gap,
                "outcome_kind": "virtual_stopping_boundary_proxy",
                "obstacle_contact_evidence": "not_measured_empty_obstacle_scene",
            }
            for safe, margin, status, gap in (
                (True, 0.2, "settled", 0.25),
                (False, -0.1, "settled", 0.75),
                (None, None, "invalid", 1.25),
            )
        ],
    }
    path = tmp_path / "controlled_sweep.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    objects, sources = load_inputs(path, split="held-out")
    report = analyze_traces(objects, sources=sources, split="held-out")

    assert report["raw_counts"]["unknown_outcome_count"] == 3
    assert report["raw_counts"]["known_outcome_count"] == 0
    assert report["raw_counts"]["collision_rate"] is None
    assert report["boundary_proxy_counts"] == {
        "trial_count": 3,
        "boundary_proxy_violation_count": 1,
        "boundary_proxy_nonviolation_count": 1,
        "unknown_boundary_proxy_count": 1,
    }
    assert all(row["declared_splits"] == ["held-out"] for row in report["outcomes"])


@pytest.mark.parametrize("field", [
    "plant_safe", "physical_safe", "safe", "state_only_safe", "continuous_safe", "discrete_safe",
])
@pytest.mark.parametrize("value", [False, True])
def test_safety_flags_alone_never_declare_physical_collision_outcomes(field, value):
    report = analyze_records([{field: value}])
    assert report["raw_counts"]["known_outcome_count"] == 0


def test_physical_flags_and_boundary_proxies_are_reported_separately():
    report = analyze_records([
        {"collision": False, "plant_safe": False, "minimum_clearance": -1.0},
        {"collision": True, "plant_safe": True, "minimum_clearance": 1.0},
    ])
    assert report["raw_counts"]["collision_count"] == 1
    assert report["raw_counts"]["noncollision_count"] == 1
    assert report["boundary_proxy_counts"]["boundary_proxy_violation_count"] == 1
    assert report["boundary_proxy_counts"]["boundary_proxy_nonviolation_count"] == 1
    assert report["outcomes"][0]["collision"] is False
    assert report["outcomes"][0]["boundary_proxy_violation"] is True
    assert report["outcomes"][1]["collision"] is True
    assert report["outcomes"][1]["boundary_proxy_violation"] is False


def test_monte_carlo_clearance_and_floor_contacts_are_not_obstacle_contacts():
    report = analyze_traces([[
        {"step": 0, "post_step_clearance": 0.1, "contact_count": 4},
        {"step": 1, "post_step_clearance": -0.1, "contact_count": 0},
    ]])
    assert report["raw_counts"]["unknown_outcome_count"] == 1
    assert report["raw_counts"]["collision_count"] == 0
    assert report["boundary_proxy_counts"]["boundary_proxy_violation_count"] == 1


@pytest.mark.parametrize("field", ["contact_count", "contact", "minimum_clearance"])
def test_zero_generic_contacts_or_positive_clearance_does_not_declare_noncollision(field):
    report = analyze_records([{field: 0}])
    assert report["raw_counts"]["unknown_outcome_count"] == 1


def test_obstacle_specific_contacts_can_supply_recorded_physical_outcomes():
    report = analyze_records([
        {"obstacle_contact_count": 3, "contact_count": 4},
        {"obstacle_contact_count": 0, "contact_count": 4},
    ])
    assert report["raw_counts"]["collision_count"] == 1
    assert report["raw_counts"]["noncollision_count"] == 1


def test_positive_obstacle_contact_is_not_hidden_by_a_false_collision_flag():
    report = analyze_records([{"collision": False, "obstacle_contact": True}])
    assert report["raw_counts"]["collision_count"] == 1


def test_known_monte_carlo_summary_collision_flag_keeps_its_geometric_semantics(tmp_path):
    # run_mujoco_monte_carlo computes this summary flag from min(clearances) < 0.
    payload = {
        "metadata": {"rvo2_required": True, "trials": 2},
        "trials": [
            {"collision": True, "minimum_clearance": -0.1},
            {"collision": False, "minimum_clearance": 0.2},
        ],
    }
    path = tmp_path / "mujoco_rvo2_ablation.json"
    path.write_text(json.dumps(payload))
    objects, sources = load_inputs(path)
    report = analyze_traces(objects, sources=sources)
    assert report["raw_counts"]["unknown_outcome_count"] == 2
    assert report["boundary_proxy_counts"]["boundary_proxy_violation_count"] == 1
    assert report["boundary_proxy_counts"]["boundary_proxy_nonviolation_count"] == 1


def test_declared_proxy_flag_does_not_hide_obstacle_specific_contact_evidence():
    report = analyze_traces({
        "metadata": {"collision_flag_semantics": "boundary_proxy"},
        "trace": [{"step": 0, "collision": False, "obstacle_contact_count": 2}],
    })
    assert report["raw_counts"]["collision_count"] == 1
    assert report["boundary_proxy_counts"]["boundary_proxy_nonviolation_count"] == 1


@pytest.mark.parametrize("count", [-1, 0.5, "-2", "1.2", float("nan"), float("inf"), True, "bad"])
@pytest.mark.parametrize("field", ["collision_count", "obstacle_contact_count"])
def test_invalid_counts_are_rejected_even_with_an_explicit_flag(field, count):
    with pytest.raises(ValueError, match="non-negative integer"):
        analyze_traces([[{"step": 0, "collision": True}, {"step": 1, "collision": False, field: count}]])


@pytest.mark.parametrize("rows", [
    [{"step": 0, "collision": False}, {"step": 1}],
    [{"step": 0}, {"step": 1, "collision_count": 0}],
    [{"step": 0, "collision": False}, {"step": 1, "collision": None}],
    [{"step": 0, "collision_count": None}, {"step": 1, "collision_count": 0}],
])
def test_partially_missing_trace_outcomes_remain_unknown(rows):
    report = analyze_traces([rows])
    assert report["raw_counts"]["known_outcome_count"] == 0
    assert report["raw_counts"]["unknown_outcome_count"] == 1


def test_observed_collision_remains_known_when_other_ticks_lack_telemetry():
    report = analyze_traces([[{"step": 0}, {"step": 1, "collision": True}, {"step": 2}]])
    assert report["raw_counts"]["collision_count"] == 1


def test_full_run_declaration_can_cover_partially_missing_sample_telemetry():
    report = analyze_traces({"collision": False, "trace": [{"step": 0}, {"step": 1, "collision": False}]})
    assert report["raw_counts"]["noncollision_count"] == 1


def test_command_gap_uses_the_pre_action_latched_command():
    report = analyze_traces([[
        {"step": 0, "risk": 0.1, "previous_applied_v": 1.0, "pre_v": 0.25,
         "requested_command_v": -2.0, "v": 4.0, "collision": True},
    ]])
    assert report["outcomes"][0]["command_gap"] == pytest.approx(0.75)


@pytest.mark.parametrize("fields", [
    {"requested_command_v": 1.2, "pre_v": 0.2},
    {"previous_applied_v": 1.2, "v": 0.2},
    {"command_v": 1.2, "velocity": 0.2},
])
def test_post_action_velocity_or_new_command_is_not_used_for_latched_gap(fields):
    report = analyze_traces([[{"step": 0, "collision": True, **fields}]])
    assert report["outcomes"][0]["command_gap"] is None


def test_initial_case_state_can_define_a_latched_gap():
    report = analyze_records([{"u0": 1.25, "v0": 0.5}])
    assert report["outcomes"][0]["command_gap"] == 0.75


def test_risk_and_gap_are_paired_at_one_max_risk_tick_for_noncollisions():
    report = analyze_traces([[
        _row(0, 0.9, 0.1, 0.1), _row(1, 0.1, 1.5, 0.2),
    ]])
    outcome = report["outcomes"][0]
    assert outcome["pre_action_risk"] == 0.9
    assert outcome["command_gap"] == pytest.approx(0.1)
    assert outcome["feature_row_index"] == 0
    assert outcome["feature_selection"] == "maximum_risk_tick"
    assert "not predictor performance" in " ".join(report["notes"])


@pytest.mark.parametrize("event_fields", [{}, {"risk": 0.4}, {"command_gap": 0.5}])
def test_missing_event_tick_features_never_come_from_other_ticks(event_fields):
    report = analyze_traces([[
        _row(0, 0.1, 0.1, 0.1),
        {"step": 1, "collision": True, **event_fields},
        _row(2, 0.9, 1.5, -0.5, collision=True),
    ]])
    outcome = report["outcomes"][0]
    assert outcome["pre_action_risk"] == event_fields.get("risk")
    assert outcome["command_gap"] == event_fields.get("command_gap")
    assert outcome["feature_row_index"] == 1


def test_parent_collision_without_an_event_tick_has_no_fabricated_features():
    report = analyze_traces({"collision": True, "trace": [{"step": 0, "risk": 0.9, "command_gap": 1.0}]})
    assert report["raw_counts"]["collision_count"] == 1
    assert report["outcomes"][0]["pre_action_risk"] is None
    assert report["outcomes"][0]["command_gap"] is None


@pytest.mark.parametrize("payload", [
    {"metadata": {"split": "primary"}, "cases": [{"collision": False}]},
    {"cases": [{"split": "primary", "collision": False}]},
    {"trace": [{"step": 0, "split": "primary", "collision": False}]},
    {"metadata": {"split": "primary"}, "trace": [{"step": 0, "collision": False}]},
])
def test_analyze_traces_rejects_primary_artifacts_as_held_out(payload):
    with pytest.raises(ValueError, match="split"):
        analyze_traces(payload, split="held-out")


def test_analyze_outcomes_does_not_relabel_a_previously_analyzed_primary_run():
    report = analyze_records([{"split": "primary", "collision": False}])
    with pytest.raises(ValueError, match="split"):
        analyze_outcomes([CollisionOutcome(**report["outcomes"][0])], split="held-out")


def test_loader_preserves_container_split_for_subsequent_analysis(tmp_path):
    path = tmp_path / "primary.json"
    path.write_text(json.dumps({"metadata": {"split": "primary"}, "trials": [{"collision": False}]}))
    objects, sources = load_inputs(path)
    with pytest.raises(ValueError, match="split"):
        analyze_traces(objects, sources=sources, split="held-out")


def test_missing_child_metadata_cannot_erase_inherited_primary_split(tmp_path):
    path = tmp_path / "primary.json"
    path.write_text(json.dumps({
        "metadata": {"split": "primary"},
        "cases": [{"metadata": {"split": None}, "collision": False}],
    }))
    objects, sources = load_inputs(path)
    with pytest.raises(ValueError, match="split"):
        analyze_traces(objects, sources=sources, split="held-out")


def test_loader_deduplicates_overlapping_directory_and_file_paths(tmp_path):
    path = tmp_path / "trace.json"
    path.write_text(json.dumps([_row(0, 0.1, 0.1, 0.1)]))
    objects, sources = load_inputs([tmp_path, path, path.resolve()])
    assert analyze_traces(objects, sources=sources)["raw_counts"]["trial_count"] == 1
    assert len(sources) == 1


def test_metadata_only_held_out_split_is_retained_in_successful_report(tmp_path):
    path = tmp_path / "heldout.json"
    path.write_text(json.dumps({"metadata": {"split": "held_out"}, "cases": [{"collision": False}]}))
    objects, sources = load_inputs(path)
    report = analyze_traces(objects, sources=sources, split="held-out")
    assert report["outcomes"][0]["declared_splits"] == ["held-out"]
    assert report["raw_counts"]["noncollision_count"] == 1


@pytest.mark.parametrize("payload", [
    {"metadata": {"split": "primary"}, "cases": [{"plant_safe": False, "minimum_clearance": -0.1}]},
    {"cases": [{"split": "primary", "plant_safe": False}]},
    {"metadata": {"split": "primary"}, "cases": [{"split": "held-out", "collision": False}]},
])
def test_cli_rejects_primary_as_held_out_without_writing_a_report(tmp_path, payload):
    path = tmp_path / "primary.json"
    output = tmp_path / "report"
    path.write_text(json.dumps(payload))
    result = subprocess.run(
        [sys.executable, "-m", "benchmarks.analyze_collision_clusters", "--input", str(path),
         "--split", "held-out", "--output", str(output)],
        cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 2
    assert "split" in result.stdout + result.stderr
    assert not output.exists()
