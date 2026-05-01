# Source code location

This repository does not use a single top-level `src/` package. **Project source** lives here:

| Component | Path |
|-----------|------|
| Controller library (LQR, MPC, hybrid, trajectory, viz) | [`ros2_ws/src/hybrid_controller/hybrid_controller/`](../ros2_ws/src/hybrid_controller/hybrid_controller/) |
| ROS 2 nodes, launch files, config, worlds | [`ros2_ws/src/hybrid_nav/`](../ros2_ws/src/hybrid_nav/) |
| Standalone scripts (repo root) | [`run_simulation.py`](../run_simulation.py), [`generate_trajectory_plots.py`](../generate_trajectory_plots.py), [`generate_hybrid_scenarios.py`](../generate_hybrid_scenarios.py) |
| Evaluation scripts | [`evaluation/`](../evaluation/) |
| Unit tests | [`tests/`](../tests/) |

There are **no neural-network model weights** in this project (classical / optimization-based control only).
