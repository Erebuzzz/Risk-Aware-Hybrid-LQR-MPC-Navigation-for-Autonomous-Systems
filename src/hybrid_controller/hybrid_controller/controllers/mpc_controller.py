"""
MPC Controller with CVXPY
=========================

Implements the Model Predictive Controller with obstacle avoidance using
CVXPY for easy constraint modeling and rapid prototyping.

MPC Problem Formulation:
    min  Σ_{k=0}^{N-1} (||x_k - x_ref||²_Q + ||u_k||²_R) + ||x_N - x_ref||²_P
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
                 d_safe: float = 0.3, slack_penalty: float = 1000.0,
                 v_max: float = 1.0, omega_max: float = 1.5,
                 dt: float = 0.02, solver: str = "ECOS"):
        """
        Initialize MPC controller.
        
        Args:
            horizon: Prediction horizon N
            Q_diag: State tracking weight diagonal
            R_diag: Control effort weight diagonal
            P_diag: Terminal cost weight diagonal
            d_safe: Safety distance from obstacles (meters)
            slack_penalty: Penalty weight for constraint slack (rho)
            v_max: Maximum linear velocity (m/s)
            omega_max: Maximum angular velocity (rad/s)
            dt: Sampling time (seconds)
            solver: CVXPY solver to use (ECOS, SCS, OSQP)
        """
        self.N = horizon
        self.dt = dt
        self.d_safe = d_safe
        self.slack_penalty = slack_penalty
        self.v_max = v_max
        self.omega_max = omega_max
        self.solver = solver
        
        # Weight matrices
        if Q_diag is None:
            Q_diag = [10.0, 10.0, 1.0]
        if R_diag is None:
            R_diag = [0.1, 0.1]
        if P_diag is None:
            P_diag = [20.0, 20.0, 2.0]  # Higher terminal weight
        
        self.Q = np.diag(Q_diag)
        self.R = np.diag(R_diag)
        self.P = np.diag(P_diag)
        
        # Linearizer
        self.linearizer = Linearizer(dt=dt)
        
        # Warm-start storage
        self._prev_solution: Optional[np.ndarray] = None
        self._prev_states: Optional[np.ndarray] = None
        
        # State and control dimensions
        self.nx = 3  # [px, py, theta]
        self.nu = 2  # [v, omega]
    
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
        
        # Stage costs: Σ (||x_k - x_ref||²_Q + ||u_k||²_R)
        for k in range(self.N):
            state_error = x[k] - x_refs[k]
            cost += cp.quad_form(state_error, self.Q)
            cost += cp.quad_form(u[k], self.R)
        
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
        for k in range(self.N):
            constraints.append(u[k, 0] >= -self.v_max)
            constraints.append(u[k, 0] <= self.v_max)
            constraints.append(u[k, 1] >= -self.omega_max)
            constraints.append(u[k, 1] <= self.omega_max)
        
        # Obstacle avoidance constraints (linearized)
        slack_idx = 0
        for obs_idx, obs in enumerate(obstacles):
            for k in range(self.N):
                # Linearized distance constraint:
                # (p_k - p_obs)·n ≥ d_safe + r_obs
                # where n is the unit normal from obstacle to current position
                
                # Use reference position for linearization point
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
                    safe_dist = self.d_safe + obs.radius
                    
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
        
        # Formulate and solve problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        
        try:
            # Solve with specified solver
            problem.solve(solver=getattr(cp, self.solver), verbose=False)
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
        du = cp.Variable((self.N, self.nu))
        
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
        
        for k in range(self.N):
            # Penalize state error (dx)
            cost += cp.quad_form(dx[k], self.Q)
            
            # Penalize total control effort (u = u_ref + du)
            u_k = u_refs[k] + du[k]
            cost += cp.quad_form(u_k, self.R)
        
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
            constraints.append(dx[k + 1] == A_d @ dx[k] + B_d @ du[k])
        
        # Actuator constraints on TOTAL control u = u_ref + du
        for k in range(self.N):
            u_total = u_refs[k] + du[k]
            constraints.append(u_total[0] >= -self.v_max)
            constraints.append(u_total[0] <= self.v_max)
            constraints.append(u_total[1] >= -self.omega_max)
            constraints.append(u_total[1] <= self.omega_max)
        
        # Obstacle constraints on TOTAL state x = x_ref + dx
        slack_idx = 0
        for obs in obstacles:
            for k in range(self.N):
                # Current linearization point
                px_lin = x_refs_unwrapped[k, 0]
                py_lin = x_refs_unwrapped[k, 1]
                
                dx_obs = px_lin - obs.x
                dy_obs = py_lin - obs.y
                dist = np.sqrt(dx_obs**2 + dy_obs**2)
                
                if dist > 0.01:
                    nx, ny = dx_obs / dist, dy_obs / dist
                    safe_dist = self.d_safe + obs.radius
                    
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
        
        # Solve
        problem = cp.Problem(cp.Minimize(cost), constraints)
        
        try:
            problem.solve(solver=getattr(cp, self.solver), verbose=False)
        except:
            try:
                problem.solve(solver=cp.SCS, verbose=False)
            except:
                pass
        
        solve_time = (time.perf_counter() - start_time) * 1000
        
        if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            slack_used = slack is not None and slack.value is not None and np.any(slack.value > 1e-6)
            
            # Reconstruct absolute states and controls
            dx_val = dx.value
            du_val = du.value
            
            x_pred = x_refs[:self.N+1] + dx_val
            u_pred = u_refs[:self.N] + du_val
            
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
                iterations=0
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
