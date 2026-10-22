# ROS 2 Workspace: TurtleBot3 Navigation

This workspace contains the `hybrid_nav` package for running the controller on a
TurtleBot3 Burger in Gazebo Sim. The controller library is maintained in the
repository-level `src/hybrid_controller` package and is copied into the same
colcon workspace by Docker.

## Prerequisites

- Ubuntu 24.04 or WSL2
- ROS 2 Jazzy
- Gazebo Sim (Harmonic)
- TurtleBot3 packages (`turtlebot3_gazebo`, `turtlebot3_description`)

Docker is the supported setup from the repository root:

```bash
docker compose build jazzy
docker compose up jazzy
```

The default container is headless. Add `use_gz_gui:=true use_rviz:=true` to a
direct launch command when a display-enabled host is available.

The CI-equivalent workspace check is:

```bash
docker compose --profile ci run --rm ci
```

## Host setup

```bash
# Build from the repository root after installing ROS 2 dependencies.
cd /path/to/Risk-Aware-Hybrid-LQR-MPC-Navigation-for-Autonomous-Systems
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths ros2_ws/src src
source ros2_ws/install/setup.bash

# Launch
export TURTLEBOT3_MODEL=burger
ros2 launch hybrid_nav turtlebot3_adaptive_safety_filter.launch.py use_rviz:=false
```

## Files

| File | Purpose |
|------|---------|
| `hybrid_nav/nodes/adaptive_safety_filter_node.py` | Current risk-triggered safety-filter node |
| `hybrid_nav/nodes/hybrid_controller_node.py` | Legacy blended baseline |
| `hybrid_nav/nodes/obstacle_publisher_node.py` | Publishes obstacle positions |
| `launch/turtlebot3_adaptive_safety_filter.launch.py` | Current research launch file |
| `launch/turtlebot3_hybrid.launch.py` | Baseline launch file |
| `worlds/hybrid_obstacle_world.sdf` | Gazebo world with walls and obstacles |
| `rviz/hybrid_nav.rviz` | RViz2 visualization config |
