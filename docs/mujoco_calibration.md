# MuJoCo actuator and force-transmission calibration path

## Scope

`benchmarks/mujoco_scene.py` models two distinct effects:

1. The commanded linear and angular velocities are updated by exact first-order
   actuator-lag recurrences using `tau_v` and `tau_omega`.
2. MuJoCo receives an external planar force and yaw torque. Their magnitudes are
   set by `force_gain` and `torque_gain`; floor and obstacle contact use the
   configured friction, and `slip_gain` adds the declared lateral-velocity
   diagnostic term.

The Monte Carlo runner samples friction and actuator time constants per trial,
but it does not identify force transmission or calibrate those values. Its
traces now retain the simulator configuration, hidden actuator state, and
contact telemetry so a later measured-log review can distinguish command,
actuator, and plant effects. `contact_count` is the total `mjData.ncon`,
including floor contact, from the last dynamics evaluation. It is neither an
obstacle-collision count nor a maximum over the control interval. `contact_samples`
and `contact_steps` likewise count samples with any contact, including the
floor.

## Obstacle-specific contact telemetry

`MuJoCoPlant` filters the native contact list to robot-to-named-obstacle pairs;
floor contacts are never reported as obstacle contacts. Obstacles retain their
stable `obstacle_<index>` geom names and their input-order indices. Each
`MuJoCoContactEvent` also preserves MuJoCo's exact `geom1`, `geom2`, `body1`,
and `body2` names plus the finite contact distance.

The plant exposes two explicit windows:

- `last_step_obstacle_contact_count` and
  `last_step_obstacle_contact_events` describe the final physics substep of the
  most recent control interval.
- `control_interval_obstacle_contact_count` and
  `control_interval_obstacle_contact_events` retain every obstacle contact
  point sampled across all physics substeps in that interval. Persistent
  contact points therefore appear once per substep and are not deduplicated
  into collision episodes.

The shorter `obstacle_contact_count` and `obstacle_contact_events` fields in a
Monte Carlo trace refer to the complete control interval so collision summaries
can use physical static-obstacle contact evidence. The trace also records the
explicit `last_step_obstacle_contact_*` and
`obstacle_contact_interval_*` fields. The generic `contact_count` remains raw
`mjData.ncon` and includes the floor; it must not be used as obstacle-collision
evidence. A trial's `collision` flag is true only when at least one
robot-to-static-obstacle contact sample was recorded. Dynamic RVO2 agents are
controller inputs only and are not MuJoCo contact geoms, so a false `collision`
flag does not establish whole-scene collision freedom. The separate
`geometric_collision_proxy` field preserves negative-clearance information
without relabelling it as physical contact.

To run the deterministic static-obstacle regression harness directly:

```bash
.venv/bin/python - <<'PY'
from benchmarks.mujoco_scene import run_static_obstacle_regression

result = run_static_obstacle_regression()
assert result["straight_hit"]["obstacle_contact_count"] > 0
assert result["clear_control"]["obstacle_contact_count"] == 0
print("static obstacle contact regression passed")
PY
```

The fixed scene places one `radius=0.10 m` obstacle at `(0.45, 0.0)`, uses the
existing plant at friction `0.25`, and applies the same straight command in the
two cases for twenty 50 ms control intervals. The hit case starts at the
origin; the clear control starts at `y=0.50 m`. This is a deterministic
telemetry regression, not a controller benchmark or a physical collision
guarantee.

## Numerical validity of plant evidence

`MuJoCoPlant.step()` validates the independent expected simulation clock,
`mjData.warning` counters, and finite state/acceleration/applied-force values
before and after **every physics substep**. It raises `MuJoCoSimulationError`
(a `RuntimeError`) at the first invalid substep. The exception identifies the
substep, expected/observed time where applicable, and warning name/count/lastinfo
information. The applied robot wrench is cleared in `finally`; the failed
plant can be reused only after an explicit `reset()`.

This is necessary because a finite result can be the result of an automatic
reset. MuJoCo's official 3.6.0 documentation describes automatic resets for
invalid or excessively large state/acceleration values in its
[simulation-loop section](https://mujoco.readthedocs.io/en/3.6.0/programming/simulation.html#sisimulation),
and warning counters reset with the data in its
[diagnostics section](https://mujoco.readthedocs.io/en/3.6.0/programming/simulation.html#sidiagnostics).
The independent clock check remains effective when a reset erases warning
history; the warning check catches a first-substep reset whose clock still
matches the expected `0.002 s`.

The regression test uses the real MuJoCo engine with a finite `1e12 m/s`
command. On MuJoCo **3.17.0**, it triggers `mjWARN_BADQACC` and auto-resets to
finite native state at `0.002 s`. Previously the control interval returned that
state silently; it now raises on physics substep 1. A separate injected native
reset on substep 3 verifies detection even with all warning counters cleared.
These fault-injection cases test rejection of invalid evidence, not a supported
operating range.

## Deterministic simulator-path diagnostic

Run the fixed synthetic identification and friction diagnostic with:

```bash
python -m benchmarks.mujoco_calibration \
  --output Output/Plots/Paper/mujoco_calibration
```

The helper creates JSON and CSV artifacts. It uses two **primary** probes to
fit `tau_v`, `tau_omega`, `force_gain`, `torque_gain`, and `slip_gain`, then
evaluates two different **held-out** command probes. The held-out probes are
never supplied to the optimizer and no threshold is tuned from them.
Fitting and evaluation reject incomplete, incorrectly shaped, or nonfinite
traces and invalid contact counts. `summarize_split()` rejects mixed
primary/held-out inputs. Fitting, heading RMSE, and maximum state error all use
the same wrapped yaw difference, so equivalent headings separated by `2*pi`
do not create an artificial residual.

Identification probes run with `gravity_z=0` and fixed friction `0.65`. This
keeps the body out of floor contact so force and torque transmission are not
confounded with contact friction. A separate diagnostic sweep restores the
default gravity and reports contact counts and body-speed response at friction
values `0.25`, `0.50`, `0.75`, and `1.00`. The sweep is descriptive; it does
not fit a friction coefficient or a safety margin.

The report marks itself as `synthetic_mujoco_identification`, sets
`held_out_used_for_fit=false`, and sets `safety_claim_supported=false`. The
identified parameters are recovered from traces generated by the same MuJoCo
plant, so they are a regression check for the implementation path rather than
physical calibration.
Numerical failures propagate as exceptions: the helper does not continue after
an invalid substep, replace a failed probe, or emit a recovered parameter set
from an unconverged optimizer. Probe definitions, integration settings, and
fitting bounds are fixed for these checks.

## Preflight and actual-data blocker

To check the environment and an optional measured CSV without running the
synthetic fit:

```bash
python -m benchmarks.mujoco_calibration \
  --preflight-only \
  --measured-log path/to/actuator_log.csv \
  --output Output/Plots/Paper/mujoco_calibration_preflight
```

The measured-log preflight requires aliases for command linear/angular velocity,
measured linear/angular velocity, sample time, and friction. It validates a
future input contract only; the current helper does not fit measured logs. The
current repository does not contain a measured actuator/odometry log with those
fields. In this checkout on **October 24, 2026**, the system `python` executable
reports **3.11.2**, while the project `.venv/bin/python` reports **3.12.15** and
is the interpreter used for the Python 3.12 checks below. The native RVO2
binding is also absent, so the full dynamic-obstacle
`run_mujoco_monte_carlo.py` path remains unavailable in that container. The
static-obstacle contact regression is available locally, but dynamic RVO2
agents are not MuJoCo contact geoms in the current runner. These are explicit
blockers; no physical calibration, collision guarantee, or Gate 0 acceptance is
inferred from the synthetic report.

The runtime and data checks are retained in the report under `preflight` so a
Python 3.12 environment with a measured log can be re-run without changing the
probe definitions or mixing held-out data into fitting.

## Reproduce the checks

From the project root, use the existing Python 3.12 environment:

```bash
.venv/bin/python -m pytest -q tests/test_mujoco_scene.py tests/test_mujoco_calibration.py tests/test_benchmark_trace_helpers.py
.venv/bin/python -m benchmarks.mujoco_calibration --output Output/Plots/Paper/mujoco_calibration
```

The default two primary and two held-out free-space probes and all four fixed
floor-friction cases completed with the per-substep checks enabled on Python
**3.12.15** / MuJoCo **3.17.0**. This establishes numerical completion of those
diagnostics only; physical calibration and Gate 0 acceptance remain open.
