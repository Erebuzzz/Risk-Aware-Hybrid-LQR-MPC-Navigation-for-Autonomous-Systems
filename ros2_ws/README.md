# ROS2 Workspace — TurtleBot3 Hybrid LQR-MPC Navigation

This directory contains the ROS2 package for running the hybrid LQR-MPC controller on a TurtleBot3 Burger in Gazebo.

## Prerequisites

- Ubuntu 24.04 (WSL2 supported)
- ROS2 Jazzy
- Gazebo Sim (Harmonic)
- TurtleBot3 packages (`turtlebot3_gazebo`, `turtlebot3_description`)

## Setup

```bash
# 1. Create workspace
mkdir -p ~/hybrid_ws/src
cp -r ros2_ws/hybrid_nav ~/hybrid_ws/src/

# 2. Create symlinks to the controller code
cd ~/hybrid_ws/src/hybrid_nav/hybrid_nav
ln -s /path/to/this/repo/src/hybrid_controller/hybrid_controller/controllers controllers
ln -s /path/to/this/repo/src/hybrid_controller/hybrid_controller/models models
ln -s /path/to/this/repo/src/hybrid_controller/hybrid_controller/trajectory trajectory
ln -s /path/to/this/repo/src/hybrid_controller/hybrid_controller/logging logging

# 3. Build
cd ~/hybrid_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hybrid_nav
source install/setup.bash

# 4. Launch
export TURTLEBOT3_MODEL=burger
ros2 launch hybrid_nav turtlebot3_hybrid.launch.py
```

## Files

| File | Purpose |
|------|---------|
| `hybrid_nav/nodes/hybrid_controller_node.py` | Main controller (LQR + MPC + Blending) |
| `hybrid_nav/nodes/obstacle_publisher_node.py` | Publishes obstacle positions |
| `launch/turtlebot3_hybrid.launch.py` | One-command launch file |
| `worlds/hybrid_obstacle_world.sdf` | Gazebo world with walls and obstacles |
| `rviz/hybrid_nav.rviz` | RViz2 visualization config |

See `TURTLEBOT3_IMPLEMENTATION_PLAN.md` in the repo root for full details.

## References

- **Essential citations:** [../REFERENCES.md](../REFERENCES.md) (short list).
- **Extended bibliography:** [../docs/REFERENCES.md](../docs/REFERENCES.md).
- Adaptive MPC reference implementation context: [github.com/KohlerJohannes/Adaptive](https://github.com/KohlerJohannes/Adaptive).
