# Risk-Aware Hybrid LQR–MPC Navigation for Autonomous Systems

## Work Progress Document

> Mathematical foundations and implementation details for the hybrid control architecture.
>
> **Current pivot:** the older smooth/blended Hybrid LQR-MPC architecture is retained as historical work and as a baseline only. The current publication-oriented method is Risk-Triggered Adaptive MPC Safety Filtering with LQR Nominal Control, because direct switching/blending does not by itself guarantee obstacle-avoidance safety.

---

## Table of Contents

1. [Differential Drive Robot Modeling](#1-differential-drive-robot-modeling)
2. [Linearization Around Reference Trajectory](#2-linearization-around-a-reference-trajectory)
3. [Discretization for Digital Control](#3-discretization-for-digital-control)
4. [LQR Controller Formulation](#4-lqr-controller-formulation)
5. [Reference Trajectory Generation](#5-reference-trajectory-generation)
6. [Safety-Critical Control Layer (MPC)](#6-safety-critical-control-layer-mpc)
7. [QP Formulation for MPC](#7-qp-formulation-for-mpc)
8. [MPC Node Implementation](#8-mpc-node-implementation)

---

## 1. Differential Drive Robot Modeling

### 1.1 State and Input Definitions

The robot state vector:

$$
x = \begin{bmatrix} p_x \\ p_y \\ \theta \end{bmatrix}
$$

where $p_x, p_y$ represent planar position and $\theta$ is orientation.

Control inputs:

$$
u = \begin{bmatrix} v \\ \omega \end{bmatrix}
$$

where $v$ is linear velocity and $\omega$ is angular velocity.

### 1.2 Continuous-Time Nonlinear Kinematic Model

The motion of a differential drive robot is governed by:

$$
\dot{p}_x = v\cos\theta, \qquad \dot{p}_y = v\sin\theta, \qquad \dot{\theta} = \omega
$$

These equations define how the robot's pose evolves over time based on commanded velocities.

---

## 2. Linearization Around a Reference Trajectory

### 2.1 Reference Trajectory and Error State

Let the nominal planned trajectory be:

$$
x_r(t) = \begin{bmatrix} p_{x,r}(t) \\ p_{y,r}(t) \\ \theta_r(t) \end{bmatrix}, \qquad
u_r(t) = \begin{bmatrix} v_r(t) \\ \omega_r(t) \end{bmatrix}
$$

The tracking error is defined as:

$$
\tilde{x} = x - x_r, \qquad \tilde{u} = u - u_r
$$

### 2.2 Jacobian-Based Linearization

The linearized model is obtained by computing Jacobians:

$$
A = \left.\frac{\partial f}{\partial x}\right|_{(x_r,u_r)}, \qquad
B = \left.\frac{\partial f}{\partial u}\right|_{(x_r,u_r)}
$$

Evaluating the derivatives gives:

$$
A = \begin{bmatrix}
0 & 0 & -v_r\sin\theta_r \\
0 & 0 & v_r\cos\theta_r \\
0 & 0 & 0
\end{bmatrix}, \qquad
B = \begin{bmatrix}
\cos\theta_r & 0 \\
\sin\theta_r & 0 \\
0 & 1
\end{bmatrix}
$$

The linearized error dynamics become:

$$
\dot{\tilde{x}} = A\tilde{x} + B\tilde{u}
$$

---

## 3. Discretization for Digital Control

### 3.1 Zero-Order Hold Discretization

With sampling time $T_s$, the discrete-time model is:

$$
\tilde{x}_{k+1} = A_d \tilde{x}_k + B_d \tilde{u}_k
$$

where:

$$
A_d = e^{AT_s} \approx I + A T_s, \qquad B_d \approx B T_s
$$

Explicitly:

$$
A_d = \begin{bmatrix}
1 & 0 & -v_r \sin\theta_r T_s \\
0 & 1 & v_r \cos\theta_r T_s \\
0 & 0 & 1
\end{bmatrix}, \qquad
B_d = \begin{bmatrix}
\cos\theta_r T_s & 0 \\
\sin\theta_r T_s & 0 \\
0 & T_s
\end{bmatrix}
$$

---

## 4. LQR Controller Formulation

### 4.1 Quadratic Cost Function

The LQR minimizes the infinite-horizon cost:

$$
J = \sum_{k=0}^{\infty} \left( \tilde{x}_k^{\top} Q\, \tilde{x}_k + \tilde{u}_k^{\top} R\, \tilde{u}_k \right)
$$

where:
- $Q = \text{diag}(q_x, q_y, q_\theta)$ penalizes tracking errors
- $R = \text{diag}(r_v, r_\omega)$ penalizes control effort

### 4.2 Discrete Algebraic Riccati Equation (DARE)

$$
P = A_d^{\top} P A_d - A_d^{\top} P B_d (R + B_d^{\top} P B_d)^{-1} B_d^{\top} P A_d + Q
$$

### 4.3 Optimal LQR Feedback Gain

$$
K = (R + B_d^{\top} P B_d)^{-1} B_d^{\top} P A_d
$$

### 4.4 Final Control Law

$$
\tilde{u}_k = -K\tilde{x}_k, \qquad u_k = u_{r,k} - K(x_k - x_{r,k})
$$

### 4.5 Python Implementation

```python
import numpy as np
from scipy.linalg import solve_discrete_are

def get_lqr_gain(v_r, theta_r, dt):
    # 1. Define discrete linearized matrices
    Ad = np.array([[1, 0, -v_r * np.sin(theta_r) * dt],
                   [0, 1,  v_r * np.cos(theta_r) * dt],
                   [0, 0,  1]])
    
    Bd = np.array([[np.cos(theta_r) * dt, 0],
                   [np.sin(theta_r) * dt, 0],
                   [0, dt]])

    # 2. Cost weighting matrices
    Q = np.diag([10.0, 10.0, 1.0])  # State error penalty
    R = np.diag([0.1, 0.1])         # Control effort penalty

    # 3. Solve DARE
    P = solve_discrete_are(Ad, Bd, Q, R)

    # 4. Compute optimal LQR gain
    K = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
    
    return K
```

---

## 5. Reference Trajectory Generation

### 5.1 Figure-8 (Lemniscate) Trajectory

The reference position is defined as:

$$
p_{x,r}(t) = A \sin(at), \qquad p_{y,r}(t) = A \sin(at)\cos(at)
$$

where:
- $A$ controls the spatial size of the trajectory
- $a$ controls the temporal speed of traversal

### 5.2 Heading and Velocities

**Heading angle:**
$$
\theta_r(t) = \arctan2(\dot{p}_{y,r}(t), \dot{p}_{x,r}(t))
$$

**Linear velocity:**
$$
v_r(t) = \sqrt{\dot{p}_{x,r}(t)^2 + \dot{p}_{y,r}(t)^2}
$$

**Angular velocity:**
$$
\omega_r(t_k) = \frac{\theta_r(t_{k+1}) - \theta_r(t_k)}{T_s}
$$

### 5.3 Python Implementation

```python
import numpy as np

def generate_figure_8(t_range, dt, A=2.0, a=0.5):
    t = np.arange(0, t_range, dt)
    
    # Reference position
    xr = A * np.sin(a * t)
    yr = A * np.sin(a * t) * np.cos(a * t)
    
    # Derivatives
    dxr = a * A * np.cos(a * t)
    dyr = a * A * (np.cos(a * t)**2 - np.sin(a * t)**2)
    
    # Heading
    thetar = np.arctan2(dyr, dxr)
    
    # Linear velocity
    vr = np.sqrt(dxr**2 + dyr**2)
    
    # Angular velocity (numerical derivative)
    wr = np.diff(thetar, append=thetar[-1]) / dt
    
    return np.stack([xr, yr, thetar, vr, wr], axis=1)
```

---

## 6. Safety-Critical Control Layer (MPC)

The MPC augments the nominal LQR controller with constraint-aware optimization.

### 6.1 Linear Time-Varying MPC Dynamics

At every control cycle, the dynamics are locally linearized:

$$
x_{k+1} = A_k x_k + B_k u_k
$$

### 6.2 Finite-Horizon Optimal Control Problem

$$
\min_{u_0,\dots,u_{N-1}} \left( \sum_{k=0}^{N-1} \left( \|x_k - x_{\text{ref},k}\|_Q^2 + \|u_k\|_R^2 \right) + \|x_N - x_{\text{ref},N}\|_P^2 \right)
$$

where:
- $x_k = [x_k, y_k, \theta_k]^\top$ is the predicted state
- $u_k = [v_k, \omega_k]^\top$ is the control input
- $Q \succeq 0$ penalizes state tracking error
- $R \succ 0$ penalizes control effort
- $P$ is the terminal cost ensuring stability

### 6.3 Obstacle Avoidance Constraints

For each obstacle at $(x_{\text{obs},i}, y_{\text{obs},i})$:

$$
(x_k - x_{\text{obs},i})^2 + (y_k - y_{\text{obs},i})^2 \geq d_{\text{safe}}^2
$$

### 6.4 Actuator Constraints

$$
|v_k| \leq v_{\max}, \qquad |\omega_k| \leq \omega_{\max}
$$

### 6.5 Soft-Constraint Mechanism

Using slack variables $\epsilon_k \geq 0$:

$$
(x_k - x_{\text{obs},i})^2 + (y_k - y_{\text{obs},i})^2 \geq d_{\text{safe}}^2 - \epsilon_k
$$

with added penalty:

$$
J_{\text{slack}} = \sum_{k=0}^{N-1} \rho \, \epsilon_k^2
$$

### 6.6 Warm-Start Strategy

The MPC employs warm-start by shifting the previous solution:

$$
u_0^{\text{warm}} = u_1^{\text{prev}}, \quad \dots, \quad u_{N-1}^{\text{warm}} = u_{N-2}^{\text{prev}}
$$

---

## 7. QP Formulation for MPC

### 7.1 Standard QP Structure

$$
\min_{z} \frac{1}{2} z^\top P z + q^\top z, \qquad \text{s.t. } l \leq Az \leq u
$$

### 7.2 Decision Variable Vector

$$
z = \begin{bmatrix} x_1 \\ x_2 \\ \vdots \\ x_N \\ u_0 \\ u_1 \\ \vdots \\ u_{N-1} \end{bmatrix}
$$

Dimension: $\dim(z) = 3N + 2N = 5N$

### 7.3 Constraint Structure

$$
A = \begin{bmatrix} A_{\text{dyn}} \\ A_{\text{act}} \\ A_{\text{obs}} \end{bmatrix}, \qquad
l = \begin{bmatrix} 0 \\ l_{\text{act}} \\ l_{\text{obs}} \end{bmatrix}, \qquad
u = \begin{bmatrix} 0 \\ u_{\text{act}} \\ u_{\text{obs}} \end{bmatrix}
$$

---

## 8. MPC Node Implementation

### 8.1 ROS Interfaces

**Subscribers:**
- `/odom` (nav_msgs/Odometry) - robot's estimated position and orientation
- `/mpc_obstacles` (std_msgs/Float32MultiArray) - obstacle centroids

**Publishers:**
- `/cmd_vel` (geometry_msgs/Twist) - linear and angular velocities

### 8.2 Solver Configuration

The MPC uses CVXPY/OSQP with:
- Adaptive penalty parameter updates
- Sparse matrix factorization
- Box constraints for actuator bounds
- Warm-starting enabled

### 8.3 Integration with Hybrid Architecture

The MPC output is passed to the robot's actuation system. When selected by the supervisory logic, MPC ensures controls satisfy both motion constraints and collision-avoidance requirements.

---

## 9. Hybrid Blending Supervisor

### 9.1 Blending Law

The hybrid controller uses continuous convex blending rather than hard switching:

$$
u_{blend} = w(t) \cdot u^{MPC} + (1 - w(t)) \cdot u^{LQR}
$$

where $w(t) \in [0, 1]$ is the blending weight. This preserves actuator bounds by convexity.

### 9.2 Weight Computation Pipeline

The blending weight is computed through a four-stage pipeline:

1. **Sigmoid mapping:** $w_{raw} = \sigma(k \cdot (r - r_{thresh})) = \frac{1}{1 + e^{-k(r - r_{thresh})}}$
2. **Hysteresis deadband:** Hold previous weight when $|r - r_{thresh}| < h$
3. **Rate limiting:** $|w_{k+1} - w_k| \leq \dot{w}_{max} \cdot \Delta t$ (anti-chatter)
4. **Feasibility fallback:** $w \leftarrow w \cdot \lambda^n$ after $n$ consecutive MPC failures

### 9.3 Default Parameters

| Parameter | Symbol | Default | Unit |
|-----------|--------|---------|------|
| Sigmoid steepness | $k$ | 10.0 | - |
| Risk threshold | $r_{thresh}$ | 0.3 | - |
| Max weight rate | $\dot{w}_{max}$ | 2.0 | s$^{-1}$ |
| Hysteresis band | $h$ | 0.05 | - |
| Feasibility decay | $\lambda$ | 0.8 | - |

### 9.4 Formal Guarantees

- **Lipschitz continuity:** $|w(t_1) - w(t_2)| \leq \dot{w}_{max} |t_1 - t_2|$
- **Minimum dwell time:** $\tau_{dwell} \geq 0.4$ s (20 control steps at 50 Hz)
- **Maximum switching frequency:** $f_{max} \leq 1.0$ Hz
- **No Zeno behavior:** Guaranteed by rate limiting (see Theorem 2 in `formal_proofs.md`)

---

## 10. Risk Metrics Module

### 10.1 Distance-Based Risk

$$
r_{dist}(d) = \begin{cases} 1 & d \leq d_{safe} \\ 1 - \frac{d - d_{safe}}{d_{trigger} - d_{safe}} & d_{safe} < d < d_{trigger} \\ 0 & d \geq d_{trigger} \end{cases}
$$

where $d$ is the minimum distance to obstacle edges, $d_{safe} = 0.3$ m, $d_{trigger} = 1.0$ m.

### 10.2 Predictive Risk

Evaluates predicted trajectory states for constraint violations:

$$
r_{pred} = \min\left(1, \frac{5}{N \cdot M} \sum_{k=0}^{N-1} \sum_{j=1}^{M} \left(1 - \frac{k}{2N}\right) \cdot \frac{\max(0, d_{safe} - d_{k,j})}{d_{safe}} \right)
$$

where $N$ is horizon length, $M$ is obstacle count, and earlier violations are weighted more heavily.

### 10.3 Combined Risk

$$
r_{combined} = \alpha \cdot r_{dist} + \beta \cdot r_{pred}, \quad \alpha + \beta = 1
$$

Default: $\alpha = 0.6$, $\beta = 0.4$.

### 10.4 Known Issue: Predictive Risk Circular Dependency

**Status: P0 Bug.** The predictive risk currently requires MPC predicted states, creating a circular dependency (MPC needs risk to decide activation, but risk needs MPC states). **Planned fix:** Use LQR rollout or previous-step MPC trajectory for predictive risk computation.

---

## 11. Adaptive NMPC with CasADi

### 11.1 Problem Formulation

The adaptive NMPC solves the following NLP at each control step:

$$
\min_{X, U, \Xi} \sum_{k=0}^{N-1} \left( \|x_k - x_{ref,k}\|_Q^2 + \|u_k - u_{ref,k}\|_R^2 + q_\xi \|\xi_k\|^2 \right) + \omega \sum_{k=N}^{N+M} (\cdots) + \omega \|x_{N+M} - x_{ref,N+M}\|_Q^2
$$

subject to:
- $x_{k+1} = f(x_k, u_k; \hat{\theta})$ (nonlinear dynamics with estimated parameters)
- $u_k = u_{ref,k} - K(x_k - x_{ref,k})$ for $k \in [N, N+M]$ (terminal LQR rollout)
- $\|p_k + \xi_k - p_{obs,j}\|_2 \geq r_{obs,j} + d_{safe}$ (Euclidean obstacle avoidance)
- $u_{min} \leq u_k \leq u_{max}$ (input constraints)

### 11.2 Key Design Choices

| Feature | QP-MPC (`mpc_controller.py`) | NMPC (`adaptive_mpc_controller.py`) |
|---------|------------------------------|--------------------------------------|
| Dynamics | Linearized at reference | Exact nonlinear (CasADi SX) |
| Obstacles | Linearized half-planes | Exact Euclidean norm |
| Solver | CVXPY/OSQP | CasADi + IPOPT |
| Parameters | Fixed | Online LMS adaptation |
| Terminal | Fixed weight (buggy) | LQR rollout horizon |

### 11.3 LMS Parameter Adaptation

The kinematic parameters $\theta = [v_s, \omega_s]^\top$ are estimated online:

$$
\hat{\theta}_{k+1} = \text{proj}_{[\theta_{min}, \theta_{max}]} \left[ \hat{\theta}_k + \Gamma \Phi_k^\top (x_{k+1}^{meas} - x_{k+1}^{pred}) \right]
$$

where $\Phi_k = \Delta t \begin{bmatrix} v\cos\theta & 0 \\ v\sin\theta & 0 \\ 0 & \omega \end{bmatrix}$ is the regressor matrix.

Convergence requires persistent excitation of the input signals (see `formal_proofs.md`, LMS Remark).

---

## 12. Current Architecture Status and Identified Issues

### 12.1 Critical Bug Register (Phase 0)

| ID | Module | Description | Severity |
|----|--------|-------------|----------|
| P0-A | `cvxpygen_solver.py` | CVXPYgen wrapper lacks obstacle constraints | Critical |
| P0-B | `mpc_controller.py` | Cost penalizes absolute $\|u\|$ instead of $\|u - u_{ref}\|$ | High |
| P0-C | `mpc_controller.py` | Obstacle linearization uses reference trajectory instead of warm-start | High |
| P0-D | `mpc_controller.py` | Terminal cost uses arbitrary weights instead of DARE-derived $P$ | High |
| P0-E | `lqr_controller.py` | Hardcoded fallback gain instead of DARE computation | Medium |
| P0-F | `mpc_controller.py` | Move-blocking erroneously zeroes rate penalties | Medium |

### 12.2 Planned Architecture (Post-Fix)

```
┌──────────────────────────────────────────────────┐
│           Risk-Aware Supervisory Layer            │
│   (Sigmoid + Hysteresis + Rate-Limited Blending)  │
├──────────────────┬───────────────────────────────┤
│  LQR Controller  │  Adaptive NMPC (CasADi)       │
│  DARE-derived K  │  LMS parameter estimation     │
│  (Nominal mode)  │  Euclidean obstacle avoidance  │
├──────────────────┴───────────────────────────────┤
│              System Modeling Layer                 │
│   Differential-drive kinematics (control-affine)  │
│   Online parameter adaptation: θ = [v_s, ω_s]    │
└──────────────────────────────────────────────────┘
```

### 12.3 Evaluation Status

**P5-A RESOLVED.** Stochastic noise is now enabled by default (`sigma_p = 0.05`, `sigma_theta = 0.01`). The `statistical_runner.py` includes a variance validation guard that warns if all metrics have zero standard deviation. Monte Carlo validation will produce meaningful confidence intervals.

---

## 13. Checkpoint Navigation Architecture (Phase 3)

### 13.1 CheckpointExtractor (P3-B)

Extracts sparse checkpoints from dense reference trajectories using three strategies:
- **Uniform:** Evenly spaced by index
- **Arc-length:** Evenly spaced by cumulative distance
- **Curvature-weighted:** Denser at high-curvature regions (preferred for paper)

**File:** `trajectory/checkpoint_nav.py`

### 13.2 WaypointManager (P3-A)

Manages real-time progress through checkpoints:
- Tracks arrival at each waypoint (threshold: `r_arr = 0.3 m`)
- Provides `lookahead` future waypoints to MPC
- Generates interpolated `x_refs` and `u_refs` for MPC horizon

### 13.3 CN Metrics (P3-D)

Cross-track error (XTE) computed as perpendicular distance to nearest inter-checkpoint segment. Completion rate = fraction of checkpoints reached.

### 13.4 FRP vs CN Comparison (P3-E)

Standalone runner `evaluation/frp_vs_cn_comparison.py` executes head-to-head trials on all 5 trajectory families × 30 trials × 2 modes. Outputs JSON with per-metric mean±std.

---

## 14. Multi-Trajectory Evaluation Suite (Phase 2)

### 14.1 TrajectoryFactory (P2-A)

Five trajectory families available via `TrajectoryFactory.generate()`:

| Type | Crossings | Curvature | Difficulty |
|------|-----------|-----------|------------|
| `figure8` | 1 | Balanced | Easy |
| `clover3` | 3 | High at origin | Hard |
| `rose4` | 4 | High at origin | Hard |
| `spiral` | 0 | Increasing | Medium |
| `random_wp` | Variable | Variable | Variable |

All output `[t, px, py, theta, v, omega]` arrays.

### 14.2 CLI Integration (P2-C)

`run_simulation.py --trajectory <name>` selects the reference trajectory.

---

## 15. Risk Signal Visualization (Phase 4-C)

Three new visualization methods in `Visualizer`:

1. **`plot_blend_trajectory`**: Robot path colored by blend weight (blue=LQR → purple=blend → red=MPC). Candidate paper figure.
2. **`plot_risk_heatmap`**: Arena-wide risk field with d_safe and d_trigger circles.
3. **`plot_risk_timeseries`**: Dual-axis time plot showing risk level and blend weight response.

---

## Authors

| Name | GitHub | Email |
|------|--------|-------|
| Kshitiz | [@Erebuzzz](https://github.com/Erebuzzz) | kshitiz23@iiserb.ac.in |
| Agolika | [@Agolika413](https://github.com/Agolika413) | agolika23@iiserb.ac.in |

