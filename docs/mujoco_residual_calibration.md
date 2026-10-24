# Controller-side reduced-model residual calibration

## Scope

`benchmarks/mujoco_residual_calibration.py` is a separate diagnostic from
`benchmarks/mujoco_calibration.py`. The existing calibration helper recovers
parameters of the same MuJoCo plant that generated its synthetic traces. This
benchmark keeps the controller-side
`AugmentedDifferentialDriveModel` fixed and identifies only a small additive
residual basis from endpoint mismatches against fixed MuJoCo traces.

The reduced-model state and input contracts are:

```text
x = [p_x, p_y, yaw, v, omega]
u = [v_cmd, omega_cmd]
d = [d_v, d_y, d_omega]
```

The residuals use the model's existing convention: `d_v` is a longitudinal
acceleration residual in the actuator lag, `d_y` is a lateral body-velocity
residual in the pose update, and `d_omega` is a yaw-rate residual in the pose
update. The controller model uses `dt=0.05 s`, `tau_v=0.10 s`, and
`tau_omega=0.10 s`; those parameters are not fitted or changed.

The fixed five-parameter residual basis is:

```text
d_v     = v_bias + v_command_error_gain * (v_cmd - v)
d_y     = lateral_speed_gain * abs(v)
d_omega = omega_bias + omega_command_error_gain * (omega_cmd - omega)
```

MuJoCo traces use a fixed free-space plant configuration (`dt=0.05 s`, physics
step `0.002 s`, `tau_v=0.12 s`, `tau_omega=0.09 s`, force gain `9.0`, torque
gain `4.0`, slip gain `0.35`, gravity `0`, and friction `0.65`). Gravity is
zero so floor contact is not included in the residual fit. Any trace with a
contact sample is rejected as unsuitable residual-calibration evidence.

## Primary and held-out protocol

The benchmark has two fixed primary probes and two independently authored
held-out probes. Their initial states, command amplitudes, and command block
lengths differ. Probe content is fingerprinted before simulation; duplicate
content across the partition is rejected. The optimizer receives primary
traces only. Held-out traces are generated and evaluated only after the fit.

The implementation rejects:

* mixed primary/held-out collections;
* held-out traces passed to the fit function;
* duplicate probe content or duplicate trace content;
* nonfinite or incorrectly shaped states and contact counts;
* noninteger or negative contact counts, and any contact sample;
* nonconverged residual fits.

Yaw residuals are wrapped to `[-pi, pi)` for fitting and every reported error
metric. Metrics include separate x/y position RMSE and maximum errors,
combined position RMSE, yaw RMSE and maximum error, linear-velocity RMSE and
maximum error, and angular-velocity RMSE and maximum error.

## Residual envelope and evidence limits

The reported envelope is the maximum absolute **sampled endpoint** residual on
the fitted primary traces, separately for x position, y position, yaw, linear
velocity, and angular velocity. Held-out evaluation reports the number of
samples outside each primary-only channel bound. This is an observed-sample
envelope, not a statistical confidence interval or a generalization bound.

The report explicitly records these sample-and-hold limitations:

1. The envelope is checked at controller sample endpoints only.
2. MuJoCo integrates at `0.002 s`, but inter-sample states are not included in
   the reported envelope.
3. Zero-order-held commands and endpoint-only observations do not bound peak
   motion between samples or motion after the final sample.

Residuals are not converted into a safety correction, constraint tightening,
collision bound, or safety claim automatically. The report sets
`safety_correction_applied=false` and `safety_claim_supported=false`.

## Reproduce

From the repository root, run the existing Python 3.12 environment:

```bash
./.venv/bin/python -m pytest -q tests/test_mujoco_residual_calibration.py
./.venv/bin/python -m benchmarks.mujoco_residual_calibration \
  --output Output/Plots/Paper/mujoco_residual_calibration
```

The command writes:

```text
Output/Plots/Paper/mujoco_residual_calibration/
  mujoco_residual_calibration.json
  mujoco_residual_calibration_split_metrics.csv
  mujoco_residual_calibration_envelope.csv
```

Generated output is ignored by Git. The JSON report carries the fit probe
names, held-out probe names, controller-model parameters, fixed MuJoCo plant
configuration, residual coefficients, split metrics, primary envelope, and
held-out envelope violations.

## Recorded deterministic result (October 24, 2026)

The checked run used Python `3.12.15`, NumPy `2.5.4`, SciPy `1.18.1`, and
MuJoCo `3.17.0`. The fit converged in 11 function evaluations using the two
primary probes (49 endpoint samples). The held-out evaluation used two
disjoint probes (34 endpoint samples), with no held-out data supplied to the
fit.

| Metric | Primary | Held-out |
| --- | ---: | ---: |
| Position RMSE (m) | 0.009978 | 0.004900 |
| Yaw RMSE (rad) | 0.001145 | 0.001513 |
| Linear velocity RMSE (m/s) | 0.062486 | 0.042429 |
| Angular velocity RMSE (rad/s) | 0.017613 | 0.024981 |
| Maximum state absolute error | 0.114076 | 0.090165 |

Primary-only maximum-absolute sampled residual envelope:

| Channel | Bound |
| --- | ---: |
| x position (m) | 0.024020 |
| y position (m) | 0.008367 |
| yaw (rad) | 0.003437 |
| linear velocity (m/s) | 0.114076 |
| angular velocity (rad/s) | 0.042760 |

Three held-out angular-velocity samples exceeded the primary angular-velocity
bound. No held-out position, yaw, or linear-velocity samples exceeded their
primary bounds. These counts describe this fixed sampled benchmark only; they
are not safety or collision probabilities.
