# Risk-Triggered Adaptive MPC Safety Filter with LQR Nominal Control

![Python](https://img.shields.io/badge/Python-3.10%2B-blue)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

This repository now follows a new research direction: **Risk-Triggered Adaptive MPC Safety Filtering with LQR Nominal Control** for differential-drive robot navigation.

The older smooth/blended Hybrid LQR-MPC controller is still kept in the codebase, but only as a historical baseline and ablation. It should not be presented as the final publication method, because direct LQR/MPC switching or convex command blending does not by itself guarantee obstacle-avoidance constraint preservation.

**Authors**: Kshitiz Kumar Sinha, Agolika BM

---

## Current Paper Method

The current method uses a layered control architecture:

```text
reference path
  -> LQR nominal tracking
  -> LQR rollout risk prediction
  -> adaptive MPC safety filter when risk is detected
  -> backup safety command if MPC is infeasible or too slow
```

The key idea is that MPC is no longer blended continuously with LQR. Instead, LQR gives the efficient nominal command, and adaptive MPC acts as a least-invasive safety filter only when the predicted LQR rollout becomes unsafe.

This is the method intended for new Gazebo experiments and publication work.

---

## Controller Variants

1. **LQR Controller**
   - Fast nominal trajectory tracking.
   - Good in open space.
   - Does not enforce obstacle constraints.

2. **Tube MPC / Pure MPC**
   - Solves a finite-horizon constrained optimization problem.
   - Useful as an obstacle-avoidance baseline.
   - More computationally expensive than LQR.

3. **Adaptive Nonlinear MPC**
   - Uses CasADi/IPOPT and LMS-style online parameter adaptation.
   - Estimates velocity and yaw-rate scale mismatch.
   - Useful as an adaptive MPC baseline.

4. **Legacy Blended Hybrid LQR-MPC**
   - Historical baseline only.
   - Uses smooth command blending between LQR and MPC.
   - Not the final proposed method, because blended commands may violate the safety guarantees of the MPC solution.

5. **Risk-Triggered Adaptive MPC Safety Filter**
   - Current proposed method.
   - LQR runs by default.
   - A risk predictor rolls out the LQR command sequence.
   - Adaptive MPC intervenes only when predicted clearance becomes unsafe.
   - A backup safety command is used if the MPC solver is infeasible or too slow.

---

## Option B Gazebo Scenario

Option B adds the stronger publication-oriented simulation setup:

- Actuation mismatch injection through `command_distortion_node`.
- Moving obstacle publishing through `dynamic_obstacle_publisher_node`.
- Moving obstacle messages in `[x, y, radius, vx, vy]` format.
- Metrics and plots for intervention rate, solver time, safety margin, LMS adaptation, and smoothness.

Run the current method:

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

After a run, generate paper plots:

```bash
python evaluation/plot_option_b_results.py \
  --log Output/Logs/adaptive_safety_filter_node_YYYYMMDD_HHMMSS.log
```

Plots and summaries are written to:

```text
Output/Plots/AdaptiveSafetyFilter/
```

---

## Repository Structure

```text
.
├── README.md
├── src/hybrid_controller/          # Core controller library
├── ros2_ws/src/hybrid_nav/         # ROS2/Gazebo nodes and launch files
├── evaluation/                     # Metrics, plotting, and evaluation helpers
├── docs/                           # Research proposal, plans, and guides
├── report/                         # Older blended-hybrid report history
├── Output/Logs/                    # Runtime logs from ROS2 nodes
└── requirements.txt
```

---

## Installation

Install the standalone controller package:

```bash
pip install -r requirements.txt
pip install -e src/hybrid_controller
```

Build the ROS2 workspace:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

On Windows/PowerShell with ROS2 sourced, use the equivalent local setup script.

---

## Main Files

- Current controller node: `ros2_ws/src/hybrid_nav/hybrid_nav/nodes/adaptive_safety_filter_node.py`
- Safety-filter controller: `src/hybrid_controller/hybrid_controller/controllers/adaptive_mpc_safety_filter.py`
- Risk predictor: `src/hybrid_controller/hybrid_controller/controllers/risk_predictor.py`
- Backup controller: `src/hybrid_controller/hybrid_controller/controllers/backup_safety.py`
- Option B launch file: `ros2_ws/src/hybrid_nav/launch/turtlebot3_adaptive_safety_filter.launch.py`
- Option B guide: `docs/option_b_experiment_guide.md`
- Plotting script: `evaluation/plot_option_b_results.py`

---

## Reporting Note

The older reports and blended-hybrid results are useful background, but the paper should state clearly that the old hybrid blend was not sufficient as a safety-guaranteed method. The new claim should be:

> This work proposes an LMS-adaptive predictive safety filter for LQR-based mobile robot tracking. The LQR controller provides computationally efficient nominal tracking, while the adaptive MPC safety filter minimally modifies the nominal command only when predicted future safety constraints are violated.

That framing is much safer and more publication-friendly than claiming arbitrary LQR-MPC switching or blending is guaranteed safe.
