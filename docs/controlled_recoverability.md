# Controlled MuJoCo stopping validation

## Scope and fixed contracts

`benchmarks/run_controlled_recoverability_sweep.py` measures the existing
MuJoCo plant against the unchanged state-only, continuous rate-aware, and exact
discrete stopping predictions. It applies
`[max(0, u0 - k * delta_u), 0]`, beginning at `k=0`, with
`u0 = v0 + command_gap`. There is no steering credit or reverse command.

The scene contains the robot and floor only. `obstacle_distance` is a virtual
forward travel budget after footprint allowance. A negative
`virtual_boundary_margin_m` is a budget violation, not an observed collision.
The measured `plant_proxy_safe` field must not be used as a contact sensor.

| Parameter | Primary values | Held-out values |
| --- | --- | --- |
| Initial speed `v0` (m/s) | 0.25, 0.75, 1.5, 2.5 | 0.4, 1.1, 2.0 |
| Latched command gap (m/s) | 0, 0.5, 1.0, 1.5 | 0.25, 0.75, 1.25 |
| Virtual budget (m) | 0.1, 0.5, 1.0, 1.5 | 0.25, 0.75, 1.25 |
| Linear/yaw lag (s) | 0.06, 0.10, 0.15, 0.20 | 0.08, 0.13, 0.18 |
| Command decrement (m/s per tick) | 0.05, 0.10, 0.20 | 0.075, 0.15 |
| Case count | 768 | 162 |

Both splits use a `0.05 s` control interval, `0.002 s` physics step, friction
`1.0`, zero slip, mass `2.0 kg`, gravity `-9.81 m/s^2`, force gain `9.0`, and
torque gain `4.0`. The box half-extents are `0.20 m` and `0.14 m`; the footprint
contract uses their circumscribed radius. Reset height remains `0.16 m`.
These settings are recorded in full in each report. No parameter or threshold
is fitted to either stopping grid.

## Measurement protocol v2

The protocol is frozen in `STOPPING_MEASUREMENT_PROTOCOL` before evaluation:

1. Record the maximum forward displacement at reset and each complete control
   interval. Terminal position alone can hide earlier overshoot.
2. Require zero command, both hidden actuator velocities below tolerance, and
   the norms of all body linear/angular velocity DOFs below tolerance. The
   linear tolerance is `1e-4 m/s`; the angular tolerance is `1e-4 rad/s`.
3. Require consecutive quiet endpoint samples spanning at least `0.5 s`
   (11 samples at `0.05 s`). A transient near-zero body speed does not establish
   settling while actuator lag remains active.
4. Stop with `not_settled` after 20,000 control steps if the quiet window is
   absent. A numerical warning, clock reset, or nonfinite state yields `invalid`.
   `MuJoCoPlant` checks numerical validity at every physics substep.
5. Preserve unsupported cases with reasons and partial measurements. Their
   comparison values and proxy outcomes are null, and denominators explicitly
   exclude them. They are never counted as successful stopping trials.

This is a finite, sampled observation. It supplies no bound on intersample peak
travel or motion after the settling window, and does not establish infinite-time
safety. The model and command staircase were not changed to improve agreement.

## Reproduce and retain evidence

Use the project Python 3.12 environment and a fresh output directory:

```bash
source .venv/bin/activate
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
python -m benchmarks.run_controlled_recoverability_sweep \
  --split both \
  --output Output/Plots/Paper/controlled_recoverability_run_001
```

Each split writes JSON, flattened per-case CSV, and a secondary logged-analysis
JSON sidecar. Existing artifacts are not overwritten. `--max-cases` selects a
deterministic preflight prefix and labels it `preflight`; omit it for the full
grid. The process exits nonzero when measurements are unsupported or source
hashes change during execution.

The reports include the parameter grid, measurement protocol, complete plant
configuration, generated scene hashes, Python/package versions, command line,
thread settings, Git revision, and relevant source hashes before/after the run.
When a run uses an uncommitted tree, the source hashes identify the actual code;
the Git revision alone is insufficient.

## Recorded result: October 24, 2026

The guarded run used Python **3.12.15** and MuJoCo **3.17.0**. Both reports have
`run_status: complete`, unchanged source hashes during execution, and zero
invalid or unsettled cases.

| Metric | Primary | Held-out |
| --- | ---: | ---: |
| Settled / attempted cases | 768 / 768 | 162 / 162 |
| Virtual budget respected / violated | 539 / 229 | 140 / 22 |
| Mean sampled maximum travel (m) | 0.406215 | 0.293525 |
| P95 sampled maximum travel (m) | 1.498495 | 0.860452 |
| Maximum sampled travel (m) | 2.371451 | 0.901550 |
| Discrete vs measured boundary disagreement | 264 / 768 (34.375%) | 83 / 162 (51.235%) |
| Discrete distance underpredictions | 4 | 0 |
| Largest measured minus discrete distance (m) | 0.001047 | -0.058480 |
| Discrete predicted safe but measured proxy unsafe | 0 | 0 |

Counts are deterministic grid points, not independent random trials. Changing
only the virtual budget repeats the same dynamics, so the primary underprediction
count of four must not be interpreted as four independent experiments. No
confidence interval or collision probability is inferred from these counts.
The absence of a false-safe classification at these particular budgets is not
a certificate that the discrete prediction upper-bounds plant travel.

The verified local artifacts are under:

```text
Output/Plots/Paper/controlled_recoverability_settled_v2_20261024_r2/
  controlled_recoverability_sweep_primary.json
  controlled_recoverability_sweep_primary.csv
  controlled_recoverability_sweep_primary_logged_analysis.json
  controlled_recoverability_sweep_held_out.json
  controlled_recoverability_sweep_held_out.csv
  controlled_recoverability_sweep_held_out_logged_analysis.json
```

Generated artifacts remain local and ignored by Git; reproduce them with the
command above after syncing the source. An earlier run without the `_r2` suffix
is marked `source_changed_during_run` and is not the verified result.

## Interpretation and remaining gates

The large boundary disagreement shows that the reduced actuator model and the
friction/force-driven plant are not interchangeable. The separate synthetic
identification helper checks whether known simulator parameters can be recovered
from simulator-generated probes. It does not identify the reduced model's
residual envelope against the plant and supplies no calibrated safety margin.

Gate 0 still requires plant-model mismatch calibration, a justified sampled-data
correction with explicit tail treatment, obstacle-contact evaluation, angular
avoidance validation, and a frozen evaluation of the resulting method. No
held-out result here was used to select a correction or tune a threshold.
Hardware deployment remains outside this simulation-only scope.

The collision-clustering utility can read these reports, but all physical
collision outcomes must remain unknown. Only its separate boundary-proxy table
can contain stopping-budget violations. See `collision_cluster_analysis.md` and
`mujoco_calibration.md` for the corresponding data contracts.
