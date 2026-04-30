# Risk-Aware Hybrid LQR-MPC Navigation for Autonomous Systems

This repository contains the implementation of a Risk-Aware Hybrid LQR-MPC Navigation framework for autonomous systems. The framework enables safe, efficient, and robust navigation in densely cluttered environments by dynamically blending a fast, optimal Linear Quadratic Regulator (LQR) with a predictive, constraint-aware Model Predictive Controller (MPC) based on real-time spatial risk assessment.

## Project Overview

Traditional navigation systems often face a trade-off between computational efficiency and safety in complex environments. This project introduces a hybrid controller that:
- Uses **LQR** for fast, efficient trajectory tracking in safe regions.
- Seamlessly transitions to **MPC** (or Adaptive MPC) when the system detects spatial risk, utilizing a look-ahead horizon to safely navigate around obstacles.
- Dynamically adjusts the blending weights between LQR and MPC using a continuous sigmoid risk function, preventing abrupt control discontinuities.

This architecture has been thoroughly validated using Monte Carlo statistical testing and simulated in Gazebo with a differential drive robot (TurtleBot3).

## System Architecture

The following diagram illustrates the flow of data between the Gazebo simulation environment, the ROS2 bridging layers, and the Python-based controller nodes.

```mermaid
graph TD
    %% Define Subgraphs
    subgraph Gazebo["Gazebo Simulation"]
        GZ_Sim[Gazebo Sim<br>Physics & World]
        TB3[TurtleBot3 Model]
        GZ_Sim <--> TB3
    end

    subgraph ROS_Bridge["ROS-Gazebo Bridge Layer"]
        Bridge_Odom[/odom <br> gz.msgs.Odometry/]
        Bridge_Clock[/clock <br> gz.msgs.Clock/]
        Bridge_CmdVel[/cmd_vel <br> gz.msgs.Twist/]
    end

    subgraph ROS2_Nodes["ROS2 Navigation Nodes"]
        TwistRelay(Twist Relay Node<br>geometry_msgs/TwistStamped -> Twist)
        ObsPub(Obstacle Publisher)
        
        subgraph Controllers["Controller Logic"]
            LQR_Ctrl[LQR Controller]
            MPC_Ctrl[MPC Controller]
            AMPC_Ctrl[Adaptive MPC Controller]
            Hybrid_Super[Hybrid Supervisor<br>Blends LQR + MPC]
        end
        
        CtrlNode(Controller Node)
    end

    %% Connections
    TB3 -- Odom & Clock --> Bridge_Odom
    TB3 -.-> Bridge_Clock
    
    Bridge_Odom -- "nav_msgs/Odometry" --> CtrlNode
    Bridge_Clock -- "rosgraph_msgs/Clock" --> CtrlNode
    
    ObsPub -- "Obstacle Array" --> CtrlNode
    
    CtrlNode <--> Controllers
    
    CtrlNode -- "geometry_msgs/TwistStamped<br>/cmd_vel" --> TwistRelay
    TwistRelay -- "geometry_msgs/Twist<br>/cmd_vel_bridge" --> Bridge_CmdVel
    Bridge_CmdVel -- "gz.msgs.Twist" --> TB3
```

## Setup Instructions

### Option 1: Docker (Recommended)
We provide a fully configured Docker environment that supports both ROS2 Jazzy (GZ Harmonic) and ROS2 Humble (GZ Garden).

```bash
# Launch with Jazzy (Default)
docker compose up

# Launch with Humble
docker compose up humble
```
*Note for Linux users: You may need to run `xhost +local:docker` to allow Gazebo to open a GUI window.*

### Option 2: Native ROS2 Workspace
If you have ROS2 Humble or Jazzy installed natively:

1. **Install Dependencies:**
   ```bash
   make setup
   ```
2. **Build Workspace:**
   ```bash
   make build
   ```
3. **Source Workspace:**
   ```bash
   source ros2_ws/install/setup.bash
   ```

## Run Instructions

You can run the full Gazebo simulation with RViz visualization using the provided Makefile targets:

- **Hybrid Controller** (LQR + MPC blending):
  ```bash
  make run
  ```
- **Pure MPC Controller**:
  ```bash
  make run-mpc
  ```
- **Adaptive MPC Controller**:
  ```bash
  make run-adaptive
  ```
- **Pure LQR Controller**:
  ```bash
  make run-lqr
  ```

For standalone, headless Python simulations (no ROS2 or Gazebo required):
```bash
make sim
```

## Output Structure

All generated plots, charts, and trajectory profiles are cleanly organized in the `Output/Plots/` directory.

```text
Output/
└── Plots/
    ├── AdaptiveMPC/    # Plots generated from Adaptive MPC runs
    ├── Hybrid/         # Plots generated from Hybrid LQR-MPC runs
    ├── LQR/            # Plots generated from pure LQR runs
    ├── MPC/            # Plots generated from pure MPC runs
    └── Trajectories/   # Baseline geometric trajectory profiles
```
*Note: Statistical evaluation metrics (JSON, CSV) are stored in `evaluation/results/`.*

## Modifying Parameters

Centralized configuration for the controllers and simulation parameters (e.g., lookahead horizon, safety margins, matrix weights) is located at:
`ros2_ws/src/hybrid_nav/config/params.yaml`

Modifications to this file will automatically apply on the next launch.
