"""
MPC Controller with CVXPY
=========================

Implements the Model Predictive Controller with obstacle avoidance using
CVXPY for easy constraint modeling and rapid prototyping.

MPC Problem Formulation:
     min  Σ_{k=0}^{N-1} (||x_k - x_ref||²_Q + ||u_k - u_ref||²_R) + ||x_N - x_ref||²_P
    s.t. x_{k+1} = A_d·x_k + B_d·u_k           (dynamics)
         |v_k| ≤ v_max, |ω_k| ≤ ω_max         (actuator limits)
         ||p_k - p_obs|| ≥ d_safe              (obstacle avoidance)

Obstacle avoidance is implemented using:
1. Linearized distance constraints
2. Soft constraints with slack variables when needed

Reference:
    Risk-Aware Hybrid LQR-MPC Navigation for Autonomous Systems
    Section: MPC Formulation with Obstacle Avoidance
    Section: QP Problem Structure
"""

import numpy as np
import cvxpy as cp
from typing import Tuple, Optional, List, Dict, Any
from dataclasses import dataclass
import time
from scipy.linalg import solve_discrete_are

from ..models.linearization import Linearizer


@dataclass
class Obstacle:
    """Circular obstacle representation."""
    x: float  # x-position (meters)
    y: float  # y-position (meters)
    radius: float  # radius (meters)
    
    def distance_to(self, px: float, py: float) -> float:
        """Compute distance from point to obstacle center."""
        return np.sqrt((px - self.x)**2 + (py - self.y)**2)
    
    def is_collision(self, px: float, py: float, d_safe: float) -> bool:
        """Check if point is within safety distance of obstacle."""
        return self.distance_to(px, py) < self.radius + d_safe


@dataclass
class MPCSolution:
    """Container for MPC solution results."""
    status: str                      # Solver status
    optimal_control: np.ndarray      # First control input u_0
    control_sequence: np.ndarray     # Full control sequence (N, 2)
    predicted_states: np.ndarray     # Predicted state trajectory (N+1, 3)
    cost: float                      # Optimal cost value
    solve_time_ms: float             # Solver time in milliseconds
    slack_used: bool                 # Whether slack variables were activated
    iterations: int                  # Solver iterations (if available)
    feasibility_margin: float = 0.0  # Max slack magnitude (0 = fully feasible)
    feasible: bool = True            # Whether solution provides usable control


class MPCController:
    """
    Model Predictive Controller with obstacle avoidance using CVXPY.
    
    Implements a finite-horizon optimal control problem with:
    - Quadratic cost function for trajectory tracking
    - Linear dynamics constraints
    - Box constraints on control inputs
    - Linearized obstacle avoidance constraints
    - Soft constraints via slack variables for feasibility
    
    Attributes:
        N: Prediction horizon
        Q, R, P: Cost weight matrices
        d_safe: Safety distance from obstacles
        v_max, omega_max: Actuator limits
    
    Example:
        mpc = MPCController(horizon=10, d_safe=0.3)
        
        obstacles = [Obstacle(x=1.0, y=0.5, radius=0.2)]
        solution = mpc.solve(x0, x_refs, u_refs, obstacles)
        
        # Apply first control
        u = solution.optimal_control
    """
    
    def __init__(self, horizon: int = 10, 
                 Q_diag: list = None, R_diag: list = None, P_diag: list = None,
                 S_diag: list = None, J_diag: list = None,
                 d_safe: float = 0.3, slack_penalty: float = 5000.0,
                 v_max: float = 1.0, omega_max: float = 1.5,
                 dt: float = 0.02, solver: str = "OSQP",
                 block_size: int = 1, w_max: float = 0.05):
        """
        Initialize MPC controller.
        
        Args:
            horizon: Prediction horizon N
            Q_diag: State tracking weight diagonal
            R_diag: Control effort weight diagonal
            P_diag: Terminal cost weight diagonal
            S_diag: Control rate-of-change weight diagonal (1st-order Δu penalty).
                    Penalizes u[k] - u[k-1] to produce smoother control.
                    Reference: Rawlings et al., Model Predictive Control, Ch. 1.3
            J_diag: Second-order jerk penalty weight diagonal (optional).
                    Penalizes u[k] - 2*u[k-1] + u[k-2] (discrete acceleration).
                    Reduces control jerk for smoother maneuvers.
                    Set to None to disable. Default: None.
            d_safe: Safety distance from obstacles (meters)
            slack_penalty: Penalty weight for constraint slack (rho)
            v_max: Maximum linear velocity (m/s)
            omega_max: Maximum angular velocity (rad/s)
            dt: Sampling time (seconds)
            solver: CVXPY solver to use (ECOS, SCS, OSQP)
            block_size: Move-blocking size (1=no blocking, 2=halve decision vars)
            w_max: Maximum disturbance bound for Tube MPC (meters)
                   Accounts for localization noise, model mismatch, actuator delays
        """
        self.N = horizon
        self.dt = dt
        self.d_safe = d_safe
        self.slack_penalty = slack_penalty
        self.v_max = v_max
        self.omega_max = omega_max
        self.solver = solver
        self.block_size = block_size
        self.w_max = w_max  # Tube MPC disturbance bound
        
        # Number of blocked control moves
        self.N_blocks = (horizon + block_size - 1) // block_size
        
        # Weight matrices
        if Q_diag is None:
            Q_diag = [10.0, 10.0, 50.0]  # Strong heading weight for stability
        if R_diag is None:
            R_diag = [0.1, 0.1]
        if S_diag is None:
            S_diag = [0.1, 0.5]  # Δu penalty: [Δv, Δω] (moderate smoothing)
        
        self.Q = np.diag(Q_diag)
        self.R = np.diag(R_diag)
        self.S = np.diag(S_diag)  # Control rate penalty (1st-order)
        
        # Second-order jerk penalty (optional)
        if J_diag is not None:
            self.J = np.diag(J_diag)
        else:
            self.J = None  # Disabled by default
        
        # Linearizer (must be created before DARE terminal cost computation)
        self.linearizer = Linearizer(dt=dt)
        
        # P0-D: Compute DARE-derived terminal cost instead of arbitrary diagonal.
        # Solves the DARE at a nominal operating point to get P_terminal that
        # satisfies the Lyapunov decrease condition for MPC stability.
        # Reference: Rawlings & Mayne (2017), Proposition 3 in formal_proofs.md
        if P_diag is not None:
            # User explicitly provided terminal weights -- respect them
            self.P = np.diag(P_diag)
        else:
            self.P = self._compute_dare_terminal_cost()
        
        # Warm-start storage
        self._prev_solution: Optional[np.ndarray] = None
        self._prev_states: Optional[np.ndarray] = None
        
        # Cold-start handling
        self._step_count = 0
        self._ramp_up_steps = 10  # Number of steps to ramp up control authority
        
        # State and control dimensions
        self.nx = 3  # [px, py, theta]
        self.nu = 2  # [v, omega]
    
    def _compute_dare_terminal_cost(self) -> np.ndarray:
        """
        Compute DARE-derived terminal cost matrix P.
        
        P0-D: Solves the Discrete Algebraic Riccati Equation at a nominal
        operating point (v=0.3, theta=0) to obtain a terminal cost matrix
        that satisfies the Lyapunov decrease condition:
        
            V_f((A-BK)x) - V_f(x) + l(x, -Kx) = 0
        
        This is the standard recipe from Rawlings & Mayne for MPC stability.
        
        Returns:
            P: Terminal cost matrix (3x3), positive definite.
        """
        # Nominal operating point for terminal cost computation
        v_nom = 0.3   # nominal speed (m/s)
        theta_nom = 0.0
        
        try:
            A_nom, B_nom = self.linearizer.get_discrete_model_explicit(v_nom, theta_nom)
            P_dare = solve_discrete_are(A_nom, B_nom, self.Q, self.R)
            
            # Verify P is positive definite (it should be by DARE theory)
            eigvals = np.linalg.eigvalsh(P_dare)
            if np.all(eigvals > 0):
                return P_dare
            else:
                print(f"Warning: DARE terminal cost has non-positive eigenvalues: {eigvals}. "
                      f"Using 10*Q fallback.")
                return 10.0 * self.Q
                
        except Exception as e:
            # Fallback: 10*Q is a conservative but stable choice
            print(f"Warning: DARE terminal cost computation failed: {e}. "
                  f"Using 10*Q fallback.")
            return 10.0 * self.Q
    
    def solve(self, x0: np.ndarray, x_refs: np.ndarray, u_refs: np.ndarray,
              obstacles: List[Obstacle] = None, 
              use_soft_constraints: bool = True) -> MPCSolution:
        """
        Solve the MPC optimization problem.
        
        Args:
            x0: Initial state [px, py, theta]
            x_refs: Reference states (N, 3) or (N+1, 3)
            u_refs: Reference controls (N, 2)
            obstacles: List of obstacles for collision avoidance
            use_soft_constraints: Enable slack variables for feasibility
            
        Returns:
            MPCSolution with optimal control and diagnostics
        """
        start_time = time.perf_counter()
        
        if obstacles is None:
            obstacles = []
        
        # Ensure reference arrays have correct shape
        if x_refs.shape[0] < self.N + 1:
            # Pad with last reference
            x_refs_padded = np.zeros((self.N + 1, 3))
            x_refs_padded[:x_refs.shape[0]] = x_refs
            x_refs_padded[x_refs.shape[0]:] = x_refs[-1]
            x_refs = x_refs_padded
        
        if u_refs.shape[0] < self.N:
            u_refs_padded = np.zeros((self.N, 2))
            u_refs_padded[:u_refs.shape[0]] = u_refs
            u_refs_padded[u_refs.shape[0]:] = u_refs[-1]
            u_refs = u_refs_padded
        
        # Get linearization point (use first reference)
        v_r = u_refs[0, 0] if abs(u_refs[0, 0]) > 0.01 else 0.1
        theta_r = x_refs[0, 2]
        
        # Get discrete-time model
        A_d, B_d = self.linearizer.get_discrete_model_explicit(v_r, theta_r)
        
        # Decision variables
        x = cp.Variable((self.N + 1, self.nx))  # States
        u = cp.Variable((self.N, self.nu))      # Controls
        
        # Slack variables for soft constraints
        if use_soft_constraints and len(obstacles) > 0:
            slack = cp.Variable(self.N * len(obstacles), nonneg=True)
        else:
            slack = None
        
        # Objective function
        cost = 0
        
        # Stage costs: Σ (||x_k - x_ref||²_Q + ||u_k - u_ref_k||²_R + ||u_k - u_{k-1}||²_S)
        # P0-B fix: Track u_ref instead of penalizing absolute control magnitude
        for k in range(self.N):
            state_error = x[k] - x_refs[k]
            cost += cp.quad_form(state_error, self.Q)
            control_error = u[k] - u_refs[k]
            cost += cp.quad_form(control_error, self.R)
            # Control rate penalty (Δu) for smoother trajectories (1st-order)
            if k > 0:
                du_rate = u[k] - u[k - 1]
                cost += cp.quad_form(du_rate, self.S)
            # Second-order jerk penalty: ||u_k - 2*u_{k-1} + u_{k-2}||^2_J
            if self.J is not None and k > 1:
                jerk = u[k] - 2 * u[k - 1] + u[k - 2]
                cost += cp.quad_form(jerk, self.J)
        
        # Terminal cost: ||x_N - x_ref_N||²_P
        terminal_error = x[self.N] - x_refs[self.N]
        cost += cp.quad_form(terminal_error, self.P)
        
        # Slack penalty
        if slack is not None:
            cost += self.slack_penalty * cp.sum_squares(slack)
        
        # Constraints
        constraints = []
        
        # Initial state constraint
        constraints.append(x[0] == x0)
        
        # Dynamics constraints: x_{k+1} = A_d·x_k + B_d·u_k
        for k in range(self.N):
            constraints.append(x[k + 1] == A_d @ x[k] + B_d @ u[k])
        
        # Actuator constraints: |v| ≤ v_max, |ω| ≤ ω_max
        # Tube MPC: minimal tightening (5%) to maintain performance while adding safety margin
        v_max_robust = self.v_max * 0.95
        omega_max_robust = self.omega_max * 0.95
        for k in range(self.N):
            constraints.append(u[k, 0] >= -v_max_robust)
            constraints.append(u[k, 0] <= v_max_robust)
            constraints.append(u[k, 1] >= -omega_max_robust)
            constraints.append(u[k, 1] <= omega_max_robust)
        
        # Obstacle avoidance constraints (linearized)
        slack_idx = 0
        for obs_idx, obs in enumerate(obstacles):
            for k in range(self.N):
                # Linearized distance constraint:
                # (p_k - p_obs)·n ≥ d_safe + r_obs
                # where n is the unit normal from obstacle to current position
                
                # P0-C fix: Use warm-start (previous predicted) trajectory
                # for linearization instead of reference trajectory.
                # This produces much better constraint normals when the
                # robot has already deviated from the reference.
                if self._prev_states is not None and k < len(self._prev_states):
                    px_lin = self._prev_states[k, 0]
                    py_lin = self._prev_states[k, 1]
                else:
                    px_lin = x_refs[k, 0]
                    py_lin = x_refs[k, 1]
                
                # Direction from obstacle to linearization point
                dx = px_lin - obs.x
                dy = py_lin - obs.y
                dist = np.sqrt(dx**2 + dy**2)
                
                if dist > 0.01:  # Avoid division by zero
                    nx = dx / dist
                    ny = dy / dist
                    
                    # Linearized constraint: n·(p - p_obs) ≥ d_safe + r_obs
                    # Tube MPC: add disturbance bound for robustness
                    safe_dist = self.d_safe + obs.radius + self.w_max
                    
                    if slack is not None:
                        constraints.append(
                            nx * (x[k, 0] - obs.x) + ny * (x[k, 1] - obs.y) 
                            >= safe_dist - slack[slack_idx]
                        )
                        slack_idx += 1
                    else:
                        constraints.append(
                            nx * (x[k, 0] - obs.x) + ny * (x[k, 1] - obs.y) 
                            >= safe_dist
                        )
        
        # Formulate and solve problem with warm-start
        problem = cp.Problem(cp.Minimize(cost), constraints)
        
        try:
            # Solve with specified solver (warm_start reduces iterations)
            problem.solve(solver=getattr(cp, self.solver), verbose=False, warm_start=True)
        except Exception as e:
            # Try fallback solver
            try:
                problem.solve(solver=cp.SCS, verbose=False)
            except:
                pass
        
        solve_time = (time.perf_counter() - start_time) * 1000  # ms
        
        # Check solution status
        if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            optimal_control = u.value[0]
            control_sequence = u.value
            predicted_states = x.value
            
            # Check if slack was used
            slack_used = False
            if slack is not None and slack.value is not None:
                slack_used = np.any(slack.value > 1e-6)
            
            # Store for warm-start
            self._prev_solution = control_sequence
            self._prev_states = predicted_states
            
            return MPCSolution(
                status="optimal",
                optimal_control=optimal_control,
                control_sequence=control_sequence,
                predicted_states=predicted_states,
                cost=problem.value,
                solve_time_ms=solve_time,
                slack_used=slack_used,
                iterations=0  # CVXPY doesn't expose iterations easily
            )
        else:
            # Return fallback (LQR-like) solution
            return self._get_fallback_solution(x0, x_refs, u_refs, solve_time)
    
    def _get_fallback_solution(self, x0: np.ndarray, x_refs: np.ndarray,
                               u_refs: np.ndarray, solve_time: float) -> MPCSolution:
        """
        Generate fallback solution when MPC fails.
        
        Uses simple proportional control as fallback.
        """
        error = x0 - x_refs[0]
        error[2] = self._normalize_angle(error[2])
        
        # Simple P control
        K_p = np.array([[1.0, 0.0, 0.0],
                        [0.0, 0.0, 0.5]])
        
        u_feedback = -K_p @ error
        u_fallback = u_refs[0] + u_feedback
        u_fallback = self._clip_control(u_fallback)
        
        return MPCSolution(
            status="fallback",
            optimal_control=u_fallback,
            control_sequence=np.tile(u_fallback, (self.N, 1)),
            predicted_states=np.tile(x0, (self.N + 1, 1)),
            cost=float('inf'),
            solve_time_ms=solve_time,
            slack_used=False,
            iterations=0
        )
    
    def solve_with_ltv(self, x0: np.ndarray, x_refs: np.ndarray, 
                       u_refs: np.ndarray, obstacles: List[Obstacle] = None,
                       use_soft_constraints: bool = True) -> MPCSolution:
        """
        Solve MPC with Linear Time-Varying (LTV) dynamics using Error Formulation.
        
        Formulation:
            min Σ ||dx_k||_Q + ||u_k||_R
            s.t. dx_{k+1} = A_d·dx_k + B_d·du_k
        
        where:
            dx_k = x_k - x_{ref,k}
            du_k = u_k - u_{ref,k}
            u_k = u_{ref,k} + du_k
        """
        start_time = time.perf_counter()
        
        if obstacles is None:
            obstacles = []
        
        # Decision variables: Deviations from reference
        dx = cp.Variable((self.N + 1, self.nx))
        
        # Move-blocking: Use fewer control decision variables
        # du_blocked has N_blocks rows, each block applies to block_size steps
        du_blocked = cp.Variable((self.N_blocks, self.nu))
        
        # Expand blocked controls to full horizon
        # du[k] = du_blocked[k // block_size]
        du_expanded = []
        for k in range(self.N):
            block_idx = k // self.block_size
            if block_idx < self.N_blocks:
                du_expanded.append(du_blocked[block_idx])
            else:
                du_expanded.append(du_blocked[-1])  # Use last block if overflow
        
        # Slack variables
        if use_soft_constraints and len(obstacles) > 0:
            slack = cp.Variable(self.N * len(obstacles), nonneg=True)
        else:
            slack = None
        
        # Objective
        cost = 0
        
        # Unwrap reference orientation to ensure continuity
        x_refs_unwrapped = x_refs.copy()
        x_refs_unwrapped[:, 2] = np.unwrap(x_refs[:, 2])
        
        # Adjust initial state theta to match the reference domain (avoid 2pi jump)
        # We want (x0_theta - ref_theta) to be in [-pi, pi]
        theta_ref_0 = x_refs_unwrapped[0, 2]
        diff = x0[2] - theta_ref_0
        diff_norm = self._normalize_angle(diff)
        x0_adjusted = x0.copy()
        x0_adjusted[2] = theta_ref_0 + diff_norm
        
        # Adaptive weight scheduling: increase heading weight during startup
        # Reference: MDPI Sensors 2024, adaptive weight MPC
        Q_active = self.Q.copy()
        if self._step_count < self._ramp_up_steps:
            ramp = (self._step_count + 1) / self._ramp_up_steps
            Q_active[2, 2] *= (2.0 - ramp)  # 2x heading weight at start, decays to 1x
        
        for k in range(self.N):
            # Penalize state error (dx) with possibly adaptive Q
            cost += cp.quad_form(dx[k], Q_active)
            
            # P0-B fix: Penalize control DEVIATION from reference, not absolute control
            cost += cp.quad_form(du_expanded[k], self.R)
            
            # P0-F fix: Apply rate penalty only at move-block boundaries.
            # Within a block, du_expanded[k] == du_expanded[k-1] by construction,
            # so the rate penalty is identically zero and wastes computation.
            if k > 0 and (k % self.block_size == 0):
                du_rate = du_expanded[k] - du_expanded[k - 1]
                cost += cp.quad_form(du_rate, self.S)
            # Second-order jerk penalty: ||du_k - 2*du_{k-1} + du_{k-2}||^2_J
            if self.J is not None and k > 1:
                jerk = du_expanded[k] - 2 * du_expanded[k - 1] + du_expanded[k - 2]
                cost += cp.quad_form(jerk, self.J)
        
        # Terminal cost
        cost += cp.quad_form(dx[self.N], self.P)
        
        if slack is not None:
            cost += self.slack_penalty * cp.sum_squares(slack)
        
        # Constraints
        constraints = []
        
        # Initial condition: dx_0 = x0_adjusted - x_refs[0]
        constraints.append(dx[0] == x0_adjusted - x_refs_unwrapped[0])
        
        # LTV dynamics: dx_{k+1} = A_d dx_k + B_d du_k
        for k in range(self.N):
            v_r = u_refs[k, 0] if abs(u_refs[k, 0]) > 0.01 else 0.1
            theta_r = x_refs_unwrapped[k, 2]
            A_d, B_d = self.linearizer.get_discrete_model_explicit(v_r, theta_r)
            constraints.append(dx[k + 1] == A_d @ dx[k] + B_d @ du_expanded[k])
        
        # Actuator constraints on TOTAL control u = u_ref + du
        # Tube MPC: minimal tightening (5%) to maintain performance while adding safety margin
        v_max_robust = self.v_max * 0.95
        omega_max_robust = self.omega_max * 0.95
        for k in range(self.N):
            u_total = u_refs[k] + du_expanded[k]
            constraints.append(u_total[0] >= -v_max_robust)
            constraints.append(u_total[0] <= v_max_robust)
            constraints.append(u_total[1] >= -omega_max_robust)
            constraints.append(u_total[1] <= omega_max_robust)
        
        # Obstacle constraints on TOTAL state x = x_ref + dx
        slack_idx = 0
        for obs in obstacles:
            for k in range(self.N):
                # P0-C fix: Use warm-start trajectory for linearization
                if self._prev_states is not None and k < len(self._prev_states):
                    px_lin = self._prev_states[k, 0]
                    py_lin = self._prev_states[k, 1]
                else:
                    px_lin = x_refs_unwrapped[k, 0]
                    py_lin = x_refs_unwrapped[k, 1]
                
                dx_obs = px_lin - obs.x
                dy_obs = py_lin - obs.y
                dist = np.sqrt(dx_obs**2 + dy_obs**2)
                
                if dist > 0.01:
                    nx, ny = dx_obs / dist, dy_obs / dist
                    # Tube MPC: add disturbance bound for robustness
                    safe_dist = self.d_safe + obs.radius + self.w_max
                    
                    # Constraint: n·(p - p_obs) ≥ safe_dist
                    # p = p_ref + dp
                    # n·(p_ref + dp - p_obs) ≥ safe_dist
                    
                    # Predicted position deviation
                    dpx = dx[k, 0]
                    dpy = dx[k, 1]
                    
                    lhs = nx * (px_lin + dpx - obs.x) + ny * (py_lin + dpy - obs.y)
                    
                    if slack is not None:
                        constraints.append(lhs >= safe_dist - slack[slack_idx])
                        slack_idx += 1
                    else:
                        constraints.append(lhs >= safe_dist)
        
        # Solve with warm-start for faster convergence
        problem = cp.Problem(cp.Minimize(cost), constraints)
        
        try:
            # OSQP with warm_start reduces iterations on consecutive solves
            problem.solve(solver=getattr(cp, self.solver), verbose=False, warm_start=True)
        except:
            try:
                problem.solve(solver=cp.SCS, verbose=False)
            except:
                pass
        
        solve_time = (time.perf_counter() - start_time) * 1000
        
        if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            slack_used = slack is not None and slack.value is not None and np.any(slack.value > 1e-6)
            slack_margin = 0.0
            if slack is not None and slack.value is not None:
                slack_margin = float(np.max(np.abs(slack.value)))
            
            # Reconstruct absolute states and controls
            dx_val = dx.value
            
            # Expand blocked controls to full horizon
            du_blocked_val = du_blocked.value
            du_val = np.zeros((self.N, self.nu))
            for k in range(self.N):
                block_idx = min(k // self.block_size, self.N_blocks - 1)
                du_val[k] = du_blocked_val[block_idx]
            
            x_pred = x_refs[:self.N+1] + dx_val
            u_pred = u_refs[:self.N] + du_val
            
            # Cold-start ramp-up: limit angular velocity during initial steps
            # This prevents aggressive heading corrections during cold-start
            if self._step_count < self._ramp_up_steps:
                ramp_factor = (self._step_count + 1) / self._ramp_up_steps
                omega_limit = self.omega_max * ramp_factor
                u_pred[0, 1] = np.clip(u_pred[0, 1], -omega_limit, omega_limit)
            
            self._step_count += 1
            self._prev_solution = u_pred
            self._prev_states = x_pred
            
            return MPCSolution(
                status="optimal",
                optimal_control=u_pred[0],
                control_sequence=u_pred,
                predicted_states=x_pred,
                cost=problem.value,
                solve_time_ms=solve_time,
                slack_used=slack_used,
                iterations=0,
                feasibility_margin=slack_margin
            )
        else:
            return self._get_fallback_solution(x0, x_refs, u_refs, solve_time)
    
    def get_warm_start(self) -> Optional[np.ndarray]:
        """
        Get warm-start control sequence from previous solution.
        
        Shifts the previous solution by one time step.
        """
        if self._prev_solution is None:
            return None
        
        # Shift solution: drop first, repeat last
        warm_start = np.zeros_like(self._prev_solution)
        warm_start[:-1] = self._prev_solution[1:]
        warm_start[-1] = self._prev_solution[-1]
        
        return warm_start
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def reset(self):
        """Reset controller state for new simulation/episode."""
        self._step_count = 0
        self._prev_solution = None
        self._prev_states = None
    
    def _clip_control(self, u: np.ndarray) -> np.ndarray:
        """Clip control to actuator limits."""
        return np.array([
            np.clip(u[0], -self.v_max, self.v_max),
            np.clip(u[1], -self.omega_max, self.omega_max)
        ])
    
    def set_obstacles(self, obstacles: List[Dict[str, float]]) -> List[Obstacle]:
        """
        Create Obstacle objects from dictionary list.
        
        Args:
            obstacles: List of dicts with 'x', 'y', 'radius' keys
            
        Returns:
            List of Obstacle objects
        """
        return [Obstacle(x=o['x'], y=o['y'], radius=o['radius']) for o in obstacles]
