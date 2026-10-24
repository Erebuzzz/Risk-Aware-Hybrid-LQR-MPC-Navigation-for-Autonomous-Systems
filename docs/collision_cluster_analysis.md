# Clustered collision analysis

`benchmarks/analyze_collision_clusters.py` is a separate post-processing utility
for benchmark traces and controlled case tables. It does not rerun a controller,
change a safety threshold, or modify a controlled sweep.

## Usage

Analyze the JSON traces emitted by the MuJoCo/RVO2 runner:

```bash
python -m benchmarks.analyze_collision_clusters \
  --input Output/Plots/Paper/ablation_results/traces \
  --output Output/Plots/Paper/ablation_results/collision_clusters
```

The input may also be a Monte Carlo summary (`trials`) or a controlled report
(`cases`/`records`). Repeated paths, including directory/file overlaps and
resolved path aliases, are loaded once. The output directory receives:

- `collision_cluster_report.json` (schema version 2), with normalized outcomes,
  bin edges, raw collision/non-collision/unknown trial counts, separate boundary
  proxy counts, and per-cluster counts;
- `collision_cluster_report.md`, a concise review table.

## Recorded outcomes and boundary proxies

`raw_counts` and the cluster collision counts use **recorded collision flags or
counts**, including obstacle-specific `obstacle_contact` and
`obstacle_contact_count`. Negative, fractional, non-finite, or malformed counts
are rejected even when a collision flag is also present. A positive observation
establishes a collided trial. A non-collision requires known negative outcomes
at every supplied trace row or an explicit full-run declaration. Partially
missing telemetry without a positive observation stays `unknown_outcome_count`.
Rates divide by known trial outcomes, not all trials. A trial with several
positive samples is counted once; these are not collision-event totals.

`boundary_proxy_counts` is a separate table of violations, nonviolations, and
unknowns. It uses the sign of `post_step_clearance`, `minimum_clearance`,
`clearance`/`min_clearance`, or `virtual_boundary_margin_m`, and the inverse of
`plant_proxy_safe` (legacy `plant_safe`). These signals never
override a recorded collision/non-collision. In particular:

- The controlled stopping sweep runs an empty actual obstacle list and compares
  stopping travel with a virtual boundary. `plant_proxy_safe=False` and negative
  `virtual_boundary_margin_m` are boundary violations, **not observed collisions**.
  Invalid or unsettled measurements supply null proxy outcomes.
- The Monte Carlo trace uses conservative footprint clearance. Its generic
  `contact_count` includes floor contacts and is ignored for physical outcomes,
  even when zero. Without explicit obstacle-contact/collision telemetry, its
  physical outcome is unknown and its clearance remains a proxy.
- Legacy MuJoCo/RVO2 summaries that omit a semantics declaration compute
  `collision` from clearance. Such summaries retain that proxy meaning even when
  `metadata.rvo2_required: true`. The updated runner declares
  `metadata.collision_flag_semantics: "mujoco_obstacle_contact"` and its
  `collision` field is based on recorded static-obstacle contact samples. The
  scope is `static_mujoco_obstacle_geoms_only`; dynamic RVO2 agents are not
  MuJoCo contact geoms, so a false static-contact flag does not establish
  whole-scene collision freedom. Their negative geometric clearance remains a
  separate proxy. Other producers can declare
  `metadata.collision_flag_semantics: "boundary_proxy"` for geometric flags.
- Analytical and generic safety flags (`state_only_safe`, `continuous_safe`,
  `discrete_safe`, `safe`, `physical_safe`) never establish physical outcomes.
  Other explicit collision declarations are reported as supplied; the utility
  cannot authenticate a producer's contact sensor or independently verify them.

## Decision-time cluster features

`pre_action_risk` reads the decision-time `pre_action_risk`, `pre_risk`, `risk`,
`risk_score`, or `cvar_risk` field in that precedence order. `command_gap` uses
an explicit pre-action gap when present, otherwise
**`previous_applied_v - pre_v`** (also accepting `previous_command_v`), or the
initial-case pair `u0 - v0`. Newly requested/applied commands and post-action
`v`/`velocity` never substitute for the latched command or pre-action velocity.

Both features are taken from **one row**:

1. The first recorded collision row, if available. A feature absent there
   remains unknown, even if another earlier or later row supplies it.
2. With no recorded collision, the earliest maximum-risk row. When risk is
   absent throughout, use the earliest maximum-gap row instead. The other
   feature is still taken only from that selected row.
3. A full-run collision declaration with no identifiable event row contributes
   an outcome but no invented event-time features.

`feature_row_index`, `feature_step`, and `feature_selection` record the selection.
This **outcome-dependent descriptive selection is not predictor performance**,
a causal comparison, or independent sample-level evidence. The fixed bins do
not turn it into a prospective risk evaluation.

## Frozen held-out reporting

Use `--split held-out` only with an already-selected held-out artifact:

```bash
python -m benchmarks.analyze_collision_clusters \
  --input Output/Plots/Paper/controlled_recoverability_sweep/controlled_recoverability_sweep_held_out.json \
  --split held-out \
  --evidence primary_controlled \
  --output Output/Plots/Paper/controlled_recoverability_sweep/held_out_clusters
```

Container metadata and row-level split declarations are retained and checked
by the loader, Python analysis APIs, and CLI. A `primary` artifact cannot be
relabeled `held-out`, even when only the container metadata declares the split.
Conflicting declarations are rejected before writing reports (CLI exit code
`2`). Each normalized outcome retains `declared_splits`; no declaration means
the requested split is a caller assertion, not independently verified provenance.

The held-out mode uses the immutable edges declared in the utility
(`0, .25, .50, .75, 1.0` for risk and `0, .25, .75, 1.25` for command gap).
They are not fitted from held-out outcomes; changing them is rejected. The
report records `analysis_scope: frozen_held_out` and preserves the raw counts.
This freezes reporting bins only; it cannot prove that data were never inspected
or used for tuning. An analytical held-out grid without contact telemetry will
correctly report all physical outcomes as unknown.

Trace-derived output remains **secondary exploratory logged analysis**, even
when it is labelled held-out. `--evidence primary_controlled` is an explicit
reporting claim for deterministic controlled artifacts and should only be used
after checking the source. Controlled plant sweeps remain the primary evidence
for safety conclusions; clustered logged traces are diagnostic context. Neither
an evidence label nor an all-zero observed collision count grants Gate 0 acceptance.
