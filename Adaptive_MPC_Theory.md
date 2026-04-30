# Theoretical and Mathematical Foundations of Adaptive Model Predictive Control (MPC)

This document provides a comprehensive theoretical and mathematical explanation of the Adaptive MPC implemented in `adaptive_mpc_controller.py`. It bridges the gap between the underlying control theory and the specific CasADi-based Python implementation.

---

## 1. Introduction to Adaptive MPC
Model Predictive Control (MPC) computes optimal control inputs by solving a constrained optimization problem over a finite future horizon at every time step. Traditional MPC assumes a perfectly known dynamic model of the system. However, real-world systems experience disturbances, wear-and-tear, or payload changes that alter their dynamics.

**Adaptive MPC** addresses this by continuously estimating the unknown system parameters online and updating the MPC's internal prediction model. This results in robust tracking and safe obstacle avoidance even with significant model parameter mismatch.

---

## 2. System Model and Parametric Uncertainty

### 2.1. Nominal Kinematic Model
The autonomous system is a differential-drive robot governed by the following continuous-time kinematics:
$$ \dot{x} = v \cos(\theta) $$
$$ \dot{y} = v \sin(\theta) $$
$$ \dot{\theta} = \omega $$

Where the state vector is $x = [x, y, \theta]^T$ and the control input vector is $u = [v, \omega]^T$ (linear and angular velocities).

### 2.2. Parametric Uncertainty Model
In this implementation, we assume that the actual velocities achieved by the robot differ from the commanded velocities due to scaling factors (e.g., due to wheel radius changes, tire deflation, or motor wear). We define a parameter vector $\hat{\theta} = [\theta_v, \theta_\omega]^T \in \mathbb{R}^2$ such that:
$$ \dot{x} = \theta_v \cdot v \cos(\theta) $$
$$ \dot{y} = \theta_v \cdot v \sin(\theta) $$
$$ \dot{\theta} = \theta_\omega \cdot \omega $$

In the code, this is implemented within the `_dynamics_ca` function:
```python
dot_x = ca.vertcat(
    theta[0] * u[0] * ca.cos(x[2]),
    theta[0] * u[0] * ca.sin(x[2]),
    theta[1] * u[1]
)
```

---

## 3. Mathematical Formulation of the MPC Problem

The optimization problem seeks to minimize a cost function subject to dynamic, physical, and safety constraints. The finite prediction horizon is $N$, and an additional terminal rollout horizon is $M$.

### 3.1. Cost Function
The objective function $J$ consists of the following components:

**1. Stage Cost (Tracking Error & Control Effort):**
Minimizes the deviation from the reference trajectory $x_{ref}$ and reference input $u_{ref}$.
$$ \sum_{k=0}^{N-1} \left( \|x_k - x_{ref,k}\|^2_Q + \|u_k - u_{ref,k}\|^2_R + q_\xi \|\xi_k\|^2 \right) $$

**2. Extended Horizon Terminal Rollout Cost:**
To guarantee stability without needing terminal constraints bounding the target perfectly, we extend the horizon with a simulated LQR feedback rollout ($M$ steps) scaled by a weight $\omega_{term}$.
$$ \omega_{term} \sum_{k=N}^{N+M-1} \left( \|x_k - x_{ref,k}\|^2_Q + \|u_k - u_{ref,k}\|^2_R + q_\xi \|\xi_k\|^2 \right) $$

**3. Terminal State Cost:**
$$ \omega_{term} \left( \|x_{N+M} - x_{ref,N+M}\|^2_Q + q_\xi \|\xi_{N+M}\|^2 \right) $$

Here, $\xi_k$ represents a **slack variable** used to soften state constraints. It ensures that the NLP solver remains feasible even if a constraint violation is mathematically unavoidable (e.g., getting pushed slightly into a boundary), penalized heavily by a large scalar weight $q_\xi$.

### 3.2. Constraints Formulation
The optimization is subjected to several rigorous constraints:

**1. Nonlinear System Dynamics:**
The next state must follow the parameter-adapted dynamics using Euler integration:
$$ x_{k+1} = x_k + dt \cdot f(x_k, u_k, \hat{\theta}) $$

**2. Terminal Rollout Control Law (for steps $k \in [N, N+M]$):**
During the extended horizon, the control inputs are bounded by a local LQR feedback controller $K$:
$$ u_k = u_{ref,k} - K(x_k - x_{ref,k}) $$
This anchors the tail of the trajectory to the reference, enhancing stability.

**3. Obstacle Avoidance (Exact Euclidean Norm):**
To avoid obstacles of radius $r_{obs}$ located at $p_{obs} = (x_{obs}, y_{obs})$ with a safety distance $d_{safe}$:
$$ \|C(x_k + \xi_k) - p_{obs}\|_2 \ge r_{obs} + d_{safe} $$
Where $C = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix}$ extracts the Cartesian coordinates. The slack variable $\xi_k$ softens this constraint to prevent solver infeasibility.

**4. Input and State Bounds:**
$$ u_{min} \le u_k \le u_{max} $$
$$ x_{min} \le x_k + \xi_k \le x_{max} $$

---

## 4. Parameter Adaptation Engine (LMS)

The `LMSAdaptation` class uses the Least Mean Squares (LMS) algorithm to estimate the scaling vector $\hat{\theta}$ online based on real-world measurements.

### 4.1. Prediction Model
Given the control applied $u_{k-1}$ and state $x_{k-1}$, the system predicts the expected next state using current estimates $\hat{\theta}$:
$$ x_{predicted} = x_{k-1} + dt \cdot f(x_{k-1}, u_{k-1}, \hat{\theta}) $$

### 4.2. Update Law (Gradient Descent)
Upon measuring the *actual* state $x_{measured}$ at step $k$, the error is:
$$ e_k = x_{measured} - x_{predicted} $$

The gradient matrix $\Phi$ containing the parameter sensitivities is computed as:
$$ \Phi = dt \cdot \begin{bmatrix} v \cos(\theta) & 0 \\ v \sin(\theta) & 0 \\ 0 & \omega \end{bmatrix} $$

The parameter update rule follows an LMS gradient step scaled by learning rate matrix $\Gamma$:
$$ \hat{\theta}_{new} = \hat{\theta}_{old} + \Gamma \Phi^T e_k $$
*(Implemented in `self.theta_hat = self.theta_hat + delta_theta` inside `update()`)*

Finally, bounds are enforced to prevent destabilizing parameter drift:
$$ \hat{\theta}_{new} = \text{clip}(\hat{\theta}_{new}, \theta_{min}, \theta_{max}) $$

---

## 5. Software Implementation Architecture

### 5.1. CasADi + IPOPT Non-Linear Programming
The optimization problem is transcribed into a massive vector of decision variables $W = [X, U, \Xi]^T$ and solved as a Nonlinear Program (NLP) using **CasADi** (for fast algorithmic differentiation) and the **IPOPT** interior-point optimizer. 

```python
nlp = {'x': vars_vec, 'f': cost, 'g': g, 'p': p}
self._solver = ca.nlpsol('adaptive_mpc', 'ipopt', nlp, opts)
```
- `'x'`: Decision variables containing all states, control inputs, and slacks across the horizon.
- `'f'`: Total cost function to minimize.
- `'g'`: Flattened array of equality dynamics constraints and inequality obstacle constraints.
- `'p'`: Parametric inputs that change every iteration but aren't optimized over (e.g., $x_0, x_{ref}, u_{ref}, p_{obs}, \hat{\theta}, K$).

### 5.2. Warm-Starting
To achieve real-time capabilities ($<20$ms solve times), the MPC solver utilizes a **warm start**. The previously computed optimal trajectory is passed via `x0=self._warm_start` as the initial guess to IPOPT, significantly reducing the required number of iterations to find the optimum in the next time step.

### 5.3. Dynamic LQR Gain Recomputation
Because the parameter estimate $\hat{\theta}$ constantly shifts the system model, the terminal stabilizing LQR feedback gain $K$ must be actively re-computed. The system re-linearizes and recalculates the steady-state discrete Algebraic Riccati Equation (DARE) continuously:
```python
P = solve_discrete_are(A_d, B_d, self.Q, self.R)
K = np.linalg.solve(self.R + B_d.T @ P @ B_d, B_d.T @ P @ A_d)
```
This guarantees the terminal penalty precisely reflects the true system dynamics. 

---

## 6. External reference (adaptive MPC)

Köhler’s companion MATLAB repository for certainty-equivalent adaptive MPC is available at [https://github.com/KohlerJohannes/Adaptive](https://github.com/KohlerJohannes/Adaptive) (MATLAB, YALMIP, MOSEK, CasADi). It accompanies *Certainty-equivalent adaptive MPC for uncertain nonlinear systems* (2026). This project’s `AdaptiveMPCController` is a separate Python/CasADi implementation for ROS 2 and TurtleBot3; cite Köhler’s repository for theoretical and methodological context, not as a line-for-line code base.
