# Angular-channel reachability diagnostic

## Purpose and scope

`benchmarks/run_angular_channel_reachability.py` evaluates the existing
two-sided angular-channel decision in
`hybrid_controller.controllers.angular_channel` and replays both turn signs in
the deterministic `MuJoCoPlant`. The benchmark is a diagnostic experiment. It
does not connect the decision to production safety status, change the
controller, tune a threshold, or establish a collision/safety guarantee.

The report deliberately keeps three questions separate:

1. **Controller-trigger prediction:** did the unchanged decision return
   `trigger=True`, and what were the left, right, and selected margins?
2. **Plant reachability observation:** did the first plant actuator-lag update
   reach the selected required turn rate, and what yaw/position did each open-
   loop side produce over the finite replay horizon?
3. **Contact/avoidance evidence:** was an obstacle-specific contact observed,
   and did the finite replay cross the obstacle's forward plane without that
   contact? A trigger is not a contact sensor and is never counted as an
   avoidance outcome.

## Frozen experiment contract

The default run uses seed `20261024`, 24 primary scenarios, and 12 held-out
scenarios. Primary and held-out cases come from independent child streams of
the same `SeedSequence`; scenario generation is deterministic and no values
from held-out cases are used to fit, correct, or select anything. The utility
records `held_out_used_for_tuning: false` and
`production_safety_connected: false` in the JSON metadata.

Each scenario contains a fixed initial five-state plant state, one static
non-reactive controlled obstacle, a previous angular command, and a forward
command. Both left and right turn signs are replayed from the same reset state.
The first angular command is limited by the existing `max_delta_omega` and
subsequent commands are slew-limited toward the required turn rate. The replay
uses 20 control intervals of `0.05 s`, MuJoCo physics step `0.002 s`, fixed
friction `1.0`, zero slip, and the existing MuJoCo box footprint. These are
fixed diagnostic settings, not fitted plant parameters.

The angular decision uses its existing controller-radius default (`0.105 m`).
The MuJoCo box's circumscribed radius is recorded separately (`0.2441 m` in
the current footprint contract). This distinction is part of the diagnostic:
plant geometric clearance is not silently substituted into the decision.

## Contact telemetry semantics

The utility first checks whether the local plant exposes `model`, `data.ncon`,
and contact geom pairs. In the current MuJoCo plant, obstacle-specific contact
is available by inspecting direct `mjData.contact` pairs between `robot_body`
and `obstacle_0`. The public `MuJoCoPlant.contact_count` remains total
`mjData.ncon`, including the floor, so it is recorded only as a separate
floor-inclusive count and is not used as an obstacle-collision count.
Contact pairs are sampled after each complete `MuJoCoPlant.step()` control
interval; the utility does not claim a maximum over the internal physics
substeps. A `False` observation therefore means no obstacle pair was present
at the sampled endpoint, not that a transient intersample contact was
impossible.

If a plant fixture does not expose those raw geom pairs,
`obstacle_contact` is written as `null` and the report says
`obstacle_contact_telemetry_status: unavailable`. `null` means unknown; it does
not mean that no contact occurred. In that case the report makes no avoidance
claim.

## Reproduce

From the repository root:

```bash
.venv/bin/python -m benchmarks.run_angular_channel_reachability \
  --output Output/Plots/Paper/angular_channel_reachability_run_001
```

The command writes:

* `angular_channel_reachability.json` — metadata, per-split metrics, and every
  per-scenario decision/plant record.
* `angular_channel_reachability.csv` — a flattened sidecar for analysis.

Use `--primary-count 1 --held-out-count 1` for a deterministic smoke run. The
writer refuses to overwrite existing artifacts; use a fresh output directory
for every run.

## Reading the report

`decision_trigger` and `decision_selected_side_reachable_prediction` are
controller-model results. `plant_selected_first_step_reached_required_turn_rate`
and the left/right yaw/position fields are finite MuJoCo observations. The
`actual_avoidance_evidence` field is `true` only for the finite
no-contact/obstacle-plane-crossing outcome described below; it is not a safety
status.
`outcome_class` values are:

* `obstacle_contact` — direct obstacle/robot geom-pair contact was observed.
* `passed_without_obstacle_contact` — the finite replay crossed the declared
  obstacle plane and no obstacle contact was observed.
* `no_obstacle_contact_before_obstacle_plane` — no obstacle contact was
  observed, but the replay did not cross that plane.
* `obstacle_contact_telemetry_unavailable` — obstacle-specific contact could
  not be inspected.

`passed_without_obstacle_contact` is finite open-loop evidence for that case,
not a controller safety claim. It does not address intersample behavior,
unmodeled dynamics, hardware calibration, or operation outside this scenario
set. Held-out metrics are reported separately and are not used to alter the
decision contract.

## Recorded diagnostic result: October 24, 2026

The full run used seed `20261024`, 24 primary scenarios, and 12 held-out
scenarios under Python `3.12.15` and MuJoCo `3.17.0`:

| Metric | Primary | Held-out |
| --- | ---: | ---: |
| Cases | 24 | 12 |
| Controller trigger predictions | 11 | 3 |
| Selected-side reachable predictions | 0 | 2 |
| Selected-side first-step turn-rate reaches | 0 | 2 |
| Selected-side yaw-side matches | 17 | 12 |
| Selected-side position-side matches | 5 | 6 |
| Selected-side obstacle contacts observed | 3 | 1 |
| Finite no-contact plane crossings | 0 | 0 |

Contact telemetry was available through the named MuJoCo geom pairs. The run
therefore establishes that the static-obstacle contact path is observable, but it
does not establish avoidance: no selected-side case both crossed the declared
obstacle plane and avoided an observed contact. The result remains open-loop and
diagnostic. It does not validate the angular trigger as a safety mechanism,
exclude intersample contact, or justify connecting the trigger to production
status.
