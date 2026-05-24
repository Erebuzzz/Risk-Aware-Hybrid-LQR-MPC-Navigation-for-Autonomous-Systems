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

## Build

From the repository root:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

On Windows/PowerShell with a sourced ROS2 environment, use the equivalent local setup script.

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

## What To Record

The safety-filter node writes logs to:

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

## Generate Paper Plots

After a run, replace the log path below with your newest file:

```bash
python evaluation/plot_option_b_results.py \
  --log Output/Logs/adaptive_safety_filter_node_YYYYMMDD_HHMMSS.log
```

Outputs are written to:

```text
Output/Plots/AdaptiveSafetyFilter/
```

The important plots are:

- `intervention_map.png`
- `solver_time.png`
- `risk_and_margin.png`
- `commands_and_adaptation.png`
- `summary.json`
- `summary.md`

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
