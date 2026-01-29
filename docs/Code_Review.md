# Code Review: Risk-Aware Hybrid LQR-MPC Navigation

## Complete Technical Documentation

> In-depth explanation of every module, class, and function in the codebase.

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Models Module](#2-models-module)
   - [differential_drive.py](#21-differential_drivepy)
   - [linearization.py](#22-linearizationpy)
3. [Controllers Module](#3-controllers-module)
   - [lqr_controller.py](#31-lqr_controllerpy)
   - [mpc_controller.py](#32-mpc_controllerpy)
4. [Trajectory Module](#4-trajectory-module)
   - [reference_generator.py](#41-reference_generatorpy)
5. [Logging Module](#5-logging-module)
   - [simulation_logger.py](#51-simulation_loggerpy)
6. [ROS2 Nodes](#6-ros2-nodes)
7. [Standalone Simulation](#7-standalone-simulation)

---

## 1. Architecture Overview

```
┌──────────────────────────────────────────────────────────────────┐
│                         Application Layer                        │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │ trajectory_node │  │    lqr_node     │  │    mpc_node     │  │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘  │
├───────────┼──────────────────────────────────────────┼───────────┤
│           │            Core Libraries                │           │
│  ┌────────▼────────────────────────────────────────▼────────┐   │
│  │                    Controllers Module                     │   │
│  │    ┌────────────────┐      ┌────────────────────┐        │   │
│  │    │ LQRController  │      │   MPCController    │        │   │
│  │    │   (DARE)       │      │    (CVXPY)         │        │   │
│  │    └───────┬────────┘      └─────────┬──────────┘        │   │
│  └────────────┼─────────────────────────┼────────────────────┘   │
│  ┌────────────▼─────────────────────────▼────────────────────┐   │
│  │                      Models Module                         │   │
│  │    ┌─────────────────────┐    ┌─────────────────────┐     │   │
│  │    │ DifferentialDrive   │    │     Linearizer      │     │   │
│  │    │   (Kinematics)      │◄───┤  (Jacobians, ZOH)   │     │   │
│  │    └─────────────────────┘    └─────────────────────┘     │   │
│  └────────────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────┐  ┌────────────────────────────┐    │
│  │ ReferenceTrajectory     │  │   SimulationLogger         │    │
│  │ Generator (Figure-8)    │  │   (CSV/JSON Export)        │    │
│  └─────────────────────────┘  └────────────────────────────┘    │
└──────────────────────────────────────────────────────────────────┘
```

---

## 2. Models Module

### 2.1 differential_drive.py

**Location:** `src/hybrid_controller/hybrid_controller/models/differential_drive.py`

This module implements the nonlinear kinematic model for a two-wheeled differential drive robot.

---

#### Data Classes

##### `RobotState`
```python
@dataclass
class RobotState:
    px: float    # x-position (meters)
    py: float    # y-position (meters)
    theta: float # orientation (radians)
```

**Purpose:** Type-safe representation of robot pose. Provides conversion utilities:
- `to_array()` → Converts to `np.ndarray([px, py, theta])`
- `from_array(arr)` → Creates `RobotState` from array

##### `ControlInput`
```python
@dataclass
class ControlInput:
    v: float      # linear velocity (m/s)
    omega: float  # angular velocity (rad/s)
```

**Purpose:** Type-safe control input representation with the same conversion methods.

---

#### Class: `DifferentialDriveRobot`

**Constants:**
| Constant | Value | Description |
|----------|-------|-------------|
| `STATE_DIM` | 3 | State dimension `[px, py, theta]` |
| `CONTROL_DIM` | 2 | Control dimension `[v, omega]` |

---

##### `__init__(self, v_max, omega_max, wheel_base)`

**Parameters:**
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `v_max` | float | 1.0 | Maximum linear velocity (m/s) |
| `omega_max` | float | 1.5 | Maximum angular velocity (rad/s) |
| `wheel_base` | float | 0.3 | Distance between wheels (m) |

**Purpose:** Initializes robot parameters for velocity limiting and wheel conversions.

---

##### `continuous_dynamics(self, state, control) → np.ndarray`

**Mathematical Foundation:**
$$
\dot{x} = f(x, u) = \begin{bmatrix} v \cos\theta \\ v \sin\theta \\ \omega \end{bmatrix}
$$

**Implementation:**
```python
dx = np.array([
    v * np.cos(theta),  # ṗ_x
    v * np.sin(theta),  # ṗ_y
    omega               # θ̇
])
```

**Purpose:** Computes instantaneous rate of change of state given current state and control input. This is the core kinematic model.

---

##### `simulate_step(self, state, control, dt, method) → np.ndarray`

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `state` | Current state `[px, py, theta]` |
| `control` | Control input `[v, omega]` |
| `dt` | Time step (seconds) |
| `method` | Integration method: `'euler'` or `'rk4'` |

**Integration Methods:**

1. **Euler (First-Order):**
$$x_{k+1} = x_k + \Delta t \cdot f(x_k, u_k)$$

2. **Runge-Kutta 4 (Fourth-Order):**
$$x_{k+1} = x_k + \frac{\Delta t}{6}(k_1 + 2k_2 + 2k_3 + k_4)$$

where:
- $k_1 = f(x_k, u_k)$
- $k_2 = f(x_k + 0.5\Delta t \cdot k_1, u_k)$
- $k_3 = f(x_k + 0.5\Delta t \cdot k_2, u_k)$
- $k_4 = f(x_k + \Delta t \cdot k_3, u_k)$

**Implementation Details:**
1. Clips control inputs to `[v_max, omega_max]` bounds
2. Applies selected integration method
3. Normalizes theta to `[-π, π]`

---

##### `simulate_trajectory(self, x0, controls, dt, method) → np.ndarray`

**Purpose:** Simulates full trajectory given initial state and sequence of controls.

**Returns:** Array of shape `(N+1, 3)` where N is number of control steps.

---

##### `clip_control(self, control) → np.ndarray`

**Purpose:** Enforces actuator limits:
$$v \in [-v_{max}, v_{max}], \quad \omega \in [-\omega_{max}, \omega_{max}]$$

---

##### `normalize_angle(angle) → float` (static)

**Algorithm:**
```python
while angle > π:   angle -= 2π
while angle < -π:  angle += 2π
```

**Purpose:** Wraps angle to `[-π, π]` range for consistent comparisons.

---

##### `compute_tracking_error(self, state, state_ref) → np.ndarray`

**Purpose:** Computes error with proper angle wrapping on the theta component.

**Returns:** `error = [px - px_ref, py - py_ref, normalize(theta - theta_ref)]`

---

##### `get_wheel_velocities(self, v, omega) → Tuple[float, float]`

**Differential Drive Kinematics:**
$$v_L = v - \frac{L}{2}\omega, \quad v_R = v + \frac{L}{2}\omega$$

where $L$ is the wheel base.

---

##### `from_wheel_velocities(self, v_left, v_right) → Tuple[float, float]`

**Inverse Kinematics:**
$$v = \frac{v_R + v_L}{2}, \quad \omega = \frac{v_R - v_L}{L}$$

---

### 2.2 linearization.py

**Location:** `src/hybrid_controller/hybrid_controller/models/linearization.py`

This module computes Jacobian-based linearization and discretization for controller design.

---

#### Class: `Linearizer`

##### `__init__(self, dt)`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `dt` | 0.02 | Sampling time $T_s$ (seconds) |

---

##### `get_jacobians(self, v_r, theta_r) → Tuple[np.ndarray, np.ndarray]`

**Mathematical Derivation:**

Given the nonlinear dynamics $\dot{x} = f(x, u)$, we compute Jacobians at the operating point $(x_r, u_r)$:

$$
A = \frac{\partial f}{\partial x}\bigg|_{(x_r, u_r)} = \begin{bmatrix}
0 & 0 & -v_r \sin\theta_r \\
0 & 0 & v_r \cos\theta_r \\
0 & 0 & 0
\end{bmatrix}
$$

$$
B = \frac{\partial f}{\partial u}\bigg|_{(x_r, u_r)} = \begin{bmatrix}
\cos\theta_r & 0 \\
\sin\theta_r & 0 \\
0 & 1
\end{bmatrix}
$$

**Physical Interpretation:**
- The A matrix captures how small changes in state affect the dynamics
- The `(0,2)` and `(1,2)` entries show coupling between orientation and position change rate
- B matrix shows how velocity and angular velocity inputs affect each state

---

##### `discretize_euler(self, A, B) → Tuple[np.ndarray, np.ndarray]`

**First-Order Approximation (Zero-Order Hold):**

$$A_d \approx I + A \cdot T_s$$
$$B_d \approx B \cdot T_s$$

**When to Use:** Valid for sufficiently small $T_s$ where higher-order terms are negligible.

---

##### `discretize_exact(self, A, B) → Tuple[np.ndarray, np.ndarray]`

**Exact Matrix Exponential Method:**

$$A_d = e^{A \cdot T_s}$$
$$B_d = \int_0^{T_s} e^{A\tau} d\tau \cdot B$$

**Implementation:** Uses augmented matrix method:
```python
augmented = [[A*dt,  B*dt],
             [0,     0   ]]
exp_aug = expm(augmented)
A_d = exp_aug[:n, :n]
B_d = exp_aug[:n, n:]
```

**When to Use:** For higher accuracy when dt is not extremely small.

---

##### `get_discrete_model_explicit(self, v_r, theta_r) → Tuple[np.ndarray, np.ndarray]`

**Direct Computation (from LaTeX document):**

$$
A_d = \begin{bmatrix}
1 & 0 & -v_r \sin\theta_r \cdot T_s \\
0 & 1 & v_r \cos\theta_r \cdot T_s \\
0 & 0 & 1
\end{bmatrix}
$$

$$
B_d = \begin{bmatrix}
\cos\theta_r \cdot T_s & 0 \\
\sin\theta_r \cdot T_s & 0 \\
0 & T_s
\end{bmatrix}
$$

**Purpose:** Most efficient method, directly computes discrete matrices without intermediate steps.

---

##### `predict_trajectory(self, x0, controls, v_refs, theta_refs) → np.ndarray`

**Linear Time-Varying (LTV) Prediction:**

For each time step, uses different linearization point:
```python
for k in range(N):
    A_d, B_d = get_discrete_model_explicit(v_refs[k], theta_refs[k])
    trajectory[k+1] = A_d @ trajectory[k] + B_d @ controls[k]
```

**Use Case:** More accurate prediction along curved trajectories where operating point varies.

---

##### `build_prediction_matrices(A_d, B_d, N) → Tuple[np.ndarray, np.ndarray]` (static)

**Purpose:** Constructs batch prediction matrices for MPC:

$$X = \Phi \cdot x_0 + \Gamma \cdot U$$

where:
- $X = [x_1, x_2, ..., x_N]^T$ (stacked states)
- $U = [u_0, u_1, ..., u_{N-1}]^T$ (stacked controls)

**Matrix Structure:**

$$\Phi = \begin{bmatrix} A_d \\ A_d^2 \\ \vdots \\ A_d^N \end{bmatrix}, \quad
\Gamma = \begin{bmatrix}
B_d & 0 & \cdots & 0 \\
A_d B_d & B_d & \cdots & 0 \\
\vdots & & \ddots & \\
A_d^{N-1} B_d & A_d^{N-2} B_d & \cdots & B_d
\end{bmatrix}$$

---

## 3. Controllers Module

### 3.1 lqr_controller.py

**Location:** `src/hybrid_controller/hybrid_controller/controllers/lqr_controller.py`

Implements discrete-time LQR for trajectory tracking using DARE (Discrete Algebraic Riccati Equation).

---

#### Class: `LQRController`

##### `__init__(self, Q_diag, R_diag, dt, v_max, omega_max)`

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `Q_diag` | [10, 10, 1] | State error weights $[q_x, q_y, q_\theta]$ |
| `R_diag` | [0.1, 0.1] | Control effort weights $[r_v, r_\omega]$ |
| `dt` | 0.02 | Sampling time |
| `v_max` | 1.0 | Linear velocity limit |
| `omega_max` | 1.5 | Angular velocity limit |

**Cost Matrices:**
$$Q = \text{diag}(q_x, q_y, q_\theta), \quad R = \text{diag}(r_v, r_\omega)$$

**Design Insight:**
- Higher Q values → more aggressive error correction
- Higher R values → smoother, slower control

---

##### `compute_gain(self, v_r, theta_r, force_recompute) → np.ndarray`

**DARE Solution:**

Solves the Discrete Algebraic Riccati Equation:
$$P = A_d^T P A_d - A_d^T P B_d (R + B_d^T P B_d)^{-1} B_d^T P A_d + Q$$

**Optimal Gain Computation:**
$$K = (R + B_d^T P B_d)^{-1} B_d^T P A_d$$

**Implementation Details:**
1. Uses operating point caching to avoid redundant computation
2. Handles $v_r = 0$ edge case (uncontrollable system)
3. Falls back to proportional gain if DARE fails

```python
# Solve DARE using scipy
P = solve_discrete_are(A_d, B_d, Q, R)

# Compute optimal gain
BtPB = B_d.T @ P @ B_d
BtPA = B_d.T @ P @ A_d
K = np.linalg.solve(R + BtPB, BtPA)
```

---

##### `compute_control(self, x, x_ref, u_ref, K) → np.ndarray`

**LQR Control Law:**

$$\tilde{x}_k = x_k - x_{r,k}$$
$$\tilde{u}_k = -K \cdot \tilde{x}_k$$
$$u_k = u_{r,k} + \tilde{u}_k$$

**Implementation Steps:**
1. Compute tracking error with angle normalization
2. Apply feedback gain: $\tilde{u} = -K \cdot \tilde{x}$
3. Add to reference control
4. Clip to actuator limits

---

##### `compute_control_at_operating_point(self, x, x_ref, u_ref) → Tuple`

**Purpose:** Convenience method that recomputes gain at current operating point and returns both control and error.

---

##### `set_weights(self, Q_diag, R_diag)`

**Purpose:** Runtime parameter modification for adaptive control. Invalidates cached gain.

---

### 3.2 mpc_controller.py

**Location:** `src/hybrid_controller/hybrid_controller/controllers/mpc_controller.py`

Implements MPC with obstacle avoidance using CVXPY for convex optimization.

---

#### Data Classes

##### `Obstacle`
```python
@dataclass
class Obstacle:
    x: float       # x-position (meters)
    y: float       # y-position (meters)
    radius: float  # obstacle radius (meters)
```

**Methods:**
- `distance_to(px, py)` → Euclidean distance from point to obstacle center
- `is_collision(px, py, d_safe)` → Checks if point is within safety distance

##### `MPCSolution`
```python
@dataclass
class MPCSolution:
    status: str                 # "optimal", "fallback", etc.
    optimal_control: np.ndarray # First control u_0
    control_sequence: np.ndarray # Full sequence (N, 2)
    predicted_states: np.ndarray # Trajectory (N+1, 3)
    cost: float                 # Optimal cost value
    solve_time_ms: float        # Solver time
    slack_used: bool            # Soft constraint activation
    iterations: int             # Solver iterations
```

---

#### Class: `MPCController`

##### `__init__(self, horizon, Q_diag, R_diag, P_diag, d_safe, slack_penalty, v_max, omega_max, dt, solver)`

**Key Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `horizon` | 10 | Prediction horizon N |
| `P_diag` | [20, 20, 2] | Terminal cost (higher for stability) |
| `d_safe` | 0.3 | Safety distance from obstacles (m) |
| `slack_penalty` | 1000 | Weight $\rho$ for soft constraints |
| `solver` | "ECOS" | CVXPY solver backend |

---

##### `solve(self, x0, x_refs, u_refs, obstacles, use_soft_constraints) → MPCSolution`

**Optimization Problem:**

$$\min_{u_0, ..., u_{N-1}} \sum_{k=0}^{N-1} \left( \|x_k - x_{ref,k}\|_Q^2 + \|u_k\|_R^2 \right) + \|x_N - x_{ref,N}\|_P^2$$

subject to:
$$x_{k+1} = A_d x_k + B_d u_k \quad \text{(dynamics)}$$
$$|v_k| \leq v_{max}, \quad |\omega_k| \leq \omega_{max} \quad \text{(actuator limits)}$$
$$\|p_k - p_{obs}\| \geq d_{safe} + r_{obs} \quad \text{(obstacle avoidance)}$$

**CVXPY Implementation:**

```python
# Decision variables
x = cp.Variable((N+1, 3))  # States
u = cp.Variable((N, 2))    # Controls
slack = cp.Variable(N * len(obstacles), nonneg=True)

# Cost function
cost = 0
for k in range(N):
    cost += cp.quad_form(x[k] - x_refs[k], Q)
    cost += cp.quad_form(u[k], R)
cost += cp.quad_form(x[N] - x_refs[N], P)
cost += slack_penalty * cp.sum_squares(slack)  # Soft constraint penalty

# Constraints
constraints = [x[0] == x0]  # Initial state
for k in range(N):
    constraints.append(x[k+1] == A_d @ x[k] + B_d @ u[k])
    constraints.append(u[k, 0] >= -v_max)
    constraints.append(u[k, 0] <= v_max)
    constraints.append(u[k, 1] >= -omega_max)
    constraints.append(u[k, 1] <= omega_max)
```

---

##### Obstacle Avoidance Constraints

**Linearization Approach:**

The nonlinear constraint $\|p - p_{obs}\| \geq d_{safe}$ is non-convex. We linearize around the reference trajectory:

$$n_x (x_k - x_{obs}) + n_y (y_k - y_{obs}) \geq d_{safe} + r_{obs} - \epsilon_k$$

where:
- $(n_x, n_y)$ = unit normal from obstacle to linearization point
- $\epsilon_k \geq 0$ is a slack variable

**Implementation:**
```python
# Direction from obstacle to linearization point
dx = px_lin - obs.x
dy = py_lin - obs.y
dist = np.sqrt(dx**2 + dy**2)
nx, ny = dx/dist, dy/dist

# Half-space constraint
constraints.append(
    nx * (x[k,0] - obs.x) + ny * (x[k,1] - obs.y)
    >= safe_dist - slack[slack_idx]
)
```

---

##### `solve_with_ltv(self, x0, x_refs, u_refs, obstacles, use_soft_constraints) → MPCSolution`

**Linear Time-Varying MPC:**

Uses different linearization points at each prediction step:
```python
for k in range(N):
    v_r = u_refs[k, 0]
    theta_r = x_refs[k, 2]
    A_d, B_d = linearizer.get_discrete_model_explicit(v_r, theta_r)
    constraints.append(x[k+1] == A_d @ x[k] + B_d @ u[k])
```

**Advantage:** More accurate for curved trajectories with varying operating points.

---

##### `get_warm_start(self) → Optional[np.ndarray]`

**Warm-Start Strategy:**

Shifts previous solution forward:
$$u_0^{warm} = u_1^{prev}, \quad u_1^{warm} = u_2^{prev}, \quad ..., \quad u_{N-1}^{warm} = u_{N-1}^{prev}$$

**Purpose:** Reduces solver iterations and improves convergence.

---

##### `_get_fallback_solution(self, x0, x_refs, u_refs, solve_time) → MPCSolution`

**Fallback Control:**

When optimization fails, uses simple proportional control:
```python
K_p = [[1.0, 0.0, 0.0],
       [0.0, 0.0, 0.5]]
u_fallback = u_refs[0] - K_p @ error
```

---

## 4. Trajectory Module

### 4.1 reference_generator.py

**Location:** `src/hybrid_controller/hybrid_controller/trajectory/reference_generator.py`

Generates Figure-8 (Lemniscate) reference trajectories for benchmarking.

---

#### Data Class: `TrajectoryPoint`

```python
@dataclass
class TrajectoryPoint:
    t: float      # Time (seconds)
    px: float     # x-position
    py: float     # y-position
    theta: float  # orientation
    v: float      # linear velocity
    omega: float  # angular velocity
```

---

#### Class: `ReferenceTrajectoryGenerator`

##### `__init__(self, A, a, dt)`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `A` | 2.0 | Spatial amplitude (meters) |
| `a` | 0.5 | Angular frequency (rad/s) |
| `dt` | 0.02 | Sampling time (seconds) |

---

##### `position(self, t) → Tuple[float, float]`

**Figure-8 Parametric Equations:**
$$p_x(t) = A \sin(at)$$
$$p_y(t) = A \sin(at) \cos(at) = \frac{A}{2} \sin(2at)$$

---

##### `velocity(self, t) → Tuple[float, float]`

**Time Derivatives:**
$$\dot{p}_x(t) = aA \cos(at)$$
$$\dot{p}_y(t) = aA(\cos^2(at) - \sin^2(at)) = aA \cos(2at)$$

---

##### `heading(self, t) → float`

$$\theta(t) = \text{atan2}(\dot{p}_y, \dot{p}_x)$$

---

##### `linear_velocity(self, t) → float`

$$v(t) = \sqrt{\dot{p}_x^2 + \dot{p}_y^2}$$

---

##### `angular_velocity(self, t) → float`

**Numerical Differentiation:**
$$\omega(t) \approx \frac{\theta(t + \Delta t) - \theta(t)}{\Delta t}$$

Includes angle wrapping to handle $\pm\pi$ discontinuities.

---

##### `generate(self, duration) → np.ndarray`

**Output Format:** Array of shape `(N, 6)`:
| Column | Content |
|--------|---------|
| 0 | time |
| 1 | px |
| 2 | py |
| 3 | theta |
| 4 | v |
| 5 | omega |

---

##### `get_trajectory_segment(self, start_idx, horizon) → Tuple`

**Purpose:** Extracts segment for MPC prediction horizon.

**Returns:** `(x_refs, u_refs)` where:
- `x_refs`: shape `(horizon, 3)`
- `u_refs`: shape `(horizon, 2)`

---

## 5. Logging Module

### 5.1 simulation_logger.py

**Location:** `src/hybrid_controller/hybrid_controller/logging/simulation_logger.py`

Comprehensive logging system with multiple output formats.

---

#### Enum: `LogEventType`

| Event | Description |
|-------|-------------|
| `STATE_UPDATE` | Robot state changes |
| `CONTROL_ACTION` | Control command issued |
| `PARAMETER_CHANGE` | Runtime parameter modification |
| `ERROR` | Error conditions |
| `CONSTRAINT_EVENT` | Constraint activations/violations |
| `SIMULATION_EVENT` | General simulation events |

---

#### Class: `SimulationLogger`

##### Key Methods

| Method | Purpose |
|--------|---------|
| `log_state()` | Records state, reference, and error |
| `log_control()` | Records control with controller type and solve time |
| `log_parameter_change()` | Tracks runtime parameter modifications |
| `log_error()` | Logs errors with process identification |
| `log_constraint_event()` | Records constraint activations |
| `log_mpc_solve()` | Logs MPC solver diagnostics |
| `log_obstacle_proximity()` | Warns about obstacle proximity |

##### Export Methods

| Method | Format | Content |
|--------|--------|---------|
| `export_to_csv()` | CSV | State history |
| `export_controls_to_csv()` | CSV | Control history |
| `export_to_json()` | JSON | All entries with metadata |

##### `get_summary() → Dict`

Returns:
```python
{
    "total_entries": int,
    "state_updates": int,
    "control_actions": int,
    "errors": int,
    "warnings": int,
    "max_error_norm": float,
    "mean_error_norm": float,
    "final_error_norm": float
}
```

---

## 6. ROS2 Nodes

### Node: `trajectory_node.py`

**Purpose:** Publishes Figure-8 reference trajectory.

**Publishers:**
| Topic | Type | Description |
|-------|------|-------------|
| `/reference_trajectory` | nav_msgs/Path | Full trajectory |
| `/current_reference` | geometry_msgs/PoseStamped | Current reference |
| `/reference_velocity` | geometry_msgs/Twist | Reference velocity |

---

### Node: `lqr_node.py`

**Purpose:** LQR trajectory tracking controller.

**Subscribers:**
| Topic | Type |
|-------|------|
| `/odom` | nav_msgs/Odometry |
| `/current_reference` | geometry_msgs/PoseStamped |

**Publishers:**
| Topic | Type |
|-------|------|
| `/cmd_vel` | geometry_msgs/Twist |

---

### Node: `mpc_node.py`

**Purpose:** MPC with obstacle avoidance.

**Additional Subscriber:**
| Topic | Type |
|-------|------|
| `/mpc_obstacles` | std_msgs/Float32MultiArray |

**Additional Publisher:**
| Topic | Type |
|-------|------|
| `/mpc_predicted_path` | nav_msgs/Path |

---

## 7. Standalone Simulation

**File:** `run_simulation.py`

**Usage:**
```bash
python run_simulation.py --mode lqr      # LQR only
python run_simulation.py --mode mpc      # MPC with obstacles
python run_simulation.py --mode compare  # Side-by-side comparison
```

**Options:**
| Flag | Description |
|------|-------------|
| `--duration` | Simulation duration (seconds) |
| `--no-plot` | Disable visualization |
| `--output-dir` | Custom output directory |

---

## Authors

| Name | GitHub | Email |
|------|--------|-------|
| Kshitiz | [@Erebuzzz](https://github.com/Erebuzzz) | kshitiz23@iiserb.ac.in |
| Agolika | [@Agolika413](https://github.com/Agolika413) | agolika23@iiserb.ac.in |
