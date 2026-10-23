# Option B Experiment Guide

This is the stronger Gazebo setup for the paper idea:

- LQR remains the nominal controller.
- The adaptive MPC safety filter intervenes only when the LQR rollout is risky.
- A command distortion node creates actuation mismatch so LMS adaptation has a real job.
- A dynamic obstacle publisher sends moving obstacle states as `[x, y, radius, vx, vy]`.

This setup supersedes the older smooth/blended Hybrid LQR-MPC idea. The old
hybrid controller is still useful as a baseline, but it should not be claimed as
the final safety method because direct switching or convex command blending does
not guarantee preservation of MPC obstacle-avoidance constraints.

## Build with Docker

From the repository root:

```bash
docker compose build jazzy
docker compose up jazzy
```

The CI-equivalent workspace validation is:

```bash
docker compose --profile ci run --rm ci
```

For a host ROS 2 installation, build both active packages from the repository
root instead of creating symlinks:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base ros2_ws/log build --symlink-install \
  --base-paths ros2_ws/src src \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install
source ros2_ws/install/setup.bash
```

## Run The Option B Scenario

```bash
ros2 launch hybrid_nav turtlebot3_adaptive_safety_filter.launch.py
```

Useful launch arguments:

```bash
ros2 launch hybrid_nav turtlebot3_adaptive_safety_filter.launch.py \
  use_actuation_mismatch:=true \
  obstacle_scenario:=crossing \
  obstacle_format:=quintuple
```

For a sinusoidal moving obstacle:

```bash
ros2 launch hybrid_nav turtlebot3_adaptive_safety_filter.launch.py \
  obstacle_scenario:=sinusoidal
```

## Gate 1 Headless Harness

The installed package provides a headless, fixed-seed runner for the active
Option B launch. Run it from the workspace root (or the Docker image's
`/ros2_ws` working directory), choosing a **new or empty** run directory:

```bash
ros2 run hybrid_nav gate1_experiment \
  --duration 30 \
  --seed 7 \
  --output-dir Output/Gate1/run_001
```

To retain artifacts on the host when the container exits, bind-mount `Output/`:

```bash
mkdir -p Output
docker compose run --rm -v "$PWD/Output:/ros2_ws/Output" jazzy \
  ros2 run hybrid_nav gate1_experiment \
  --duration 30 --seed 7 --output-dir Output/Gate1/run_001
```

The runner explicitly disables RViz and the Gazebo graphical client, isolates
DDS discovery with `ROS_LOCALHOST_ONLY=1` and `ROS_DOMAIN_ID=42`, and uses a
private Gazebo transport partition. `--seed` sets both Gazebo's seed and the
command-distortion node's seed. Numerical libraries run with one thread.
Use distinct `--ros-domain-id` values for concurrent runs.

`--duration` counts **simulation time** after all required streams have been
observed. Defaults are a 60-second wall-clock startup deadline, a 180-second
total wall-clock deadline, and a 10-second topic freshness deadline. Configure
these with `--startup-timeout-s`, `--wall-timeout-s`, and `--topic-timeout-s`.
An early launch exit, frozen simulation clock, or stale stream fails the run.
The runner stops the launch with SIGINT, waits for loggers to flush, and reports
forced shutdown as failure. It writes:

```text
Output/Gate1/run_001/gate1_ros2_launch.log
Output/Gate1/run_001/controller_raw.log
Output/Gate1/run_001/ros_topics.jsonl
Output/Gate1/run_001/telemetry.csv
Output/Gate1/run_001/summary.json
Output/Gate1/run_001/summary.csv
Output/Gate1/run_001/inputs/{params.yaml,hybrid_obstacle_world.sdf,...}
Output/Gate1/run_001/ros_logs/...
```

The launch runs in the private output directory. Its original logger output is
also retained under `run_001/Output/Logs/`; no previous run's log is reused.
The summary includes the exact launch command, seeds, input SHA-256 hashes,
topic counts, timing bounds, failure reasons, and metric definitions. ROS JSONL
events include full messages, receipt time, and observed simulation clock for
`/clock`, `/odom`, `/obstacles`, raw/distorted/relayed commands, controller
diagnostics, and distortion scales. This is a passive subscriber capture, not
a lossless rosbag. Non-finite values are represented as strings in raw JSONL
and flagged explicitly; summary JSON never writes non-standard NaN literals.

The default gate rejects missing telemetry, malformed rows, non-finite values,
missing controller ticks, controller `FAILED` messages, or any `MinMargin < 0`
row. Configure the diagnostic threshold with `--min-margin-m` and
`--max-safety-violation-rate`; `--min-rows` sets a minimum controller sample
count. Exit codes are `0` for passed checks, `3` for failed or incomplete runs,
and `2` for invalid configuration or artifact I/O failures.

**Interpretation:** `MinMargin` belongs to the **nominal LQR predicted rollout**,
including `d_safe` and robot radius. A negative value can be the expected reason
for MPC intervention; it is not proof of a physical collision or of the
filtered command violating a constraint. The default is a deliberately strict
diagnostic threshold, not a claim that Option B passes it. This harness does
not provide a safety guarantee.

The current world contains three static cylinders and walls. The moving
obstacle is published as a controller input, not spawned as a moving Gazebo
collision body. Also, `obstacle_3` has a 0.02 m radius in the world and a 0.05 m
radius in the obstacle publisher. These existing model distinctions are
preserved and must be accounted for when interpreting the diagnostics.

The scenario, random seeds, input snapshots, and simulated duration are fixed.
ROS callback scheduling, Gazebo physics, startup phase, and the existing
wall-clock MPC timeout can still vary across machines. This is repeatable
experiment configuration, not bitwise deterministic execution. Direct launch
is headless by default too; enable a GUI explicitly with
`use_rviz:=true use_gz_gui:=true` when a display is available.

## What To Record

The safety-filter node writes logs to the ignored runtime output directory:

```text
Output/Logs/adaptive_safety_filter_node_*.log
```

Each row contains:

```text
Tick,X,Y,Theta,Error,V,Omega,Mode,Risk,MinMargin,SolverMs,ThetaV,ThetaOmega
```

Use the `Mode` column for the intervention map:

- `LQR`: nominal control only
- `ADAPTIVE_MPC_FILTER`: safety filter was triggered
- `BACKUP`: MPC was infeasible or too slow, backup safety control was used

## Post-processing

Generate headless PNG/PDF diagnostics and a Markdown human-validation report
from one completed Gate 1 run:

```bash
python -m benchmarks.generate_ros_validation_report \
  --input Output/Gate1/run_001/summary.json \
  --output Output/Gate1/run_001/plots
```

The command also accepts `telemetry.csv`, JSON telemetry rows, or a controller
`.log` with the schema above. Use `--metadata summary.json` for a standalone CSV
or log, and `--dt 0.05` when the timestep is not recorded. All inputs must refer
to one run. Missing referenced artifacts, malformed rows, NaNs/infinities, and
inconsistent repeated ticks fail clearly.

The report contains run metadata, nominal-rollout margin violation counts,
controller/intervention statistics, and all-tick versus active-mode solver
timing. Actual collision counts remain **not recorded** unless supplied as
explicit collision telemetry or metadata; a negative rollout margin is not
relabelled as a collision. Human sign-off remains pending in the report.

Figures and reports are retained for failed safety checks. Exit codes are `0`
for passing available checks, `1` for failed validation, and `2` for unusable
inputs or output errors. Upstream harness failures remain failures. This strict
post-processing gate permits no negative nominal-rollout margins even if a
harness run explicitly allowed a nonzero violation rate. See
`benchmarks/README.md` for the full input and output contract. MuJoCo publication
diagnostics continue to use `benchmarks/generate_publication_plots.py` separately.

## Metrics To Report In The Paper

Report these from `summary.json`:

- `tracking_rmse_m`
- `min_safety_margin_m`
- `safety_violation_rate`
- `mpc_filter_rate`
- `backup_rate`
- `solver_mean_ms`
- `solver_p95_ms`
- `linear_jerk_rms`
- `angular_jerk_rms`
- `command_smoothness_cost`

Command jerk uses the second finite difference of logged velocity commands
over `Tick * dt`; these are requested-command metrics, not measured actuator
jerk. `command_smoothness_cost` is the mean squared change in `(V, Omega)`.
`--dt` must match both the installed controller `dt` and `1/control_rate`.
The controller's existing rounded log precision is retained, and solver
statistics include zero-time LQR rows.
