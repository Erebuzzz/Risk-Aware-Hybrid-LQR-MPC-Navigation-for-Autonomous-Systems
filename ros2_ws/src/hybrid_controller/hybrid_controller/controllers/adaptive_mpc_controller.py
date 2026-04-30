"""
Adaptive MPC Controller with CasADi
====================================

Implements a nonlinear Model Predictive Controller with:
- CasADi + IPOPT for exact nonlinear optimization
- Extended terminal horizon with LQR feedback rollout
- Time-varying reference trajectory tracking
- Per-state slack variables for constraint softening
- Exact Euclidean norm obstacle avoidance
- Online parameter adaptation via Least Mean Squares (LMS)
- Warm-start support for real-time performance

Inspired by adaptive MPC research for quadrotor control,
adapted for differential-drive robot kinematics and Figure-8 tracking.

Theory / reference implementation context: Köhler, "Certainty-equivalent adaptive MPC
for uncertain nonlinear systems" (2026), MATLAB examples at
https://github.com/KohlerJohannes/Adaptive — this Python class is not a port.

MPC Problem Formulation:
    min  Σ_{k=0}^{N-1} (||x_k - x_{ref,k}||²_Q + ||u_k - u_{ref,k}||²_R + q_ξ||ξ_k||²)
       + ω Σ_{k=N}^{N+M} (||x_k - x_{ref,k}||²_Q + ||u_k - u_{ref,k}||²_R + q_ξ||ξ_k||²)
       + ω ||x_{N+M} - x_{ref,N+M}||²_Q

    s.t. x_{k+1} = f(x_k, u_k)                       (nonlinear dynamics)
         u_k = u_{ref,k} - K·(x_k - x_{ref,k})        k ∈ [N, N+M]  (terminal LQR rollout)
         ||p_k + ξ_k - p_obs|| ≥ r_obs              (obstacle avoidance)
         u_min ≤ u_k ≤ u_max                        (input constraints)
         x_min ≤ x_k + ξ_k ≤ x_max                 (state constraints)
"""

import numpy as np
import casadi as ca
from typing import Tuple, Optional, List, Dict, Any
from scipy.linalg import solve_discrete_are
import time

from .mpc_controller import Obstacle, MPCSolution

class LMSAdaptation:
    def __init__(self, n_params: int = 2, nx: int = 3,
                 gamma: float = 0.01, theta_init: np.ndarray = None,
                 theta_bounds: np.ndarray = None):
        self.n_params = n_params
        self.nx = nx
        self.Gamma = gamma * np.eye(n_params)
        
        if theta_init is not None:
            self.theta_hat = theta_init.copy()
        else:
            self.theta_hat = np.ones(n_params)
            
        if theta_bounds is not None:
            self.theta_min = theta_bounds[:, 0]
            self.theta_max = theta_bounds[:, 1]
        else:
            self.theta_min = 0.5 * np.ones(n_params)
            self.theta_max = 2.0 * np.ones(n_params)
            
        self._last_prediction: Optional[np.ndarray] = None
        self._last_Phi: Optional[np.ndarray] = None
        self._history: List[np.ndarray] = [self.theta_hat.copy()]
    
    def predict_and_store(self, x: np.ndarray, u: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        theta = x[2]
        v, omega = u[0], u[1]
        v_s, omega_s = self.theta_hat[0], self.theta_hat[1]
        
        dot_x = np.array([
            v_s * v * np.cos(theta),
            v_s * v * np.sin(theta),
            omega_s * omega
        ])
        x_pred = x + dt * dot_x
        
        Phi = dt * np.array([
            [v * np.cos(theta), 0.0],
            [v * np.sin(theta), 0.0],
            [0.0,               omega]
        ])
        
        self._last_prediction = x_pred
        self._last_Phi = Phi
        return x_pred, Phi
    
    def update(self, x_measured: np.ndarray) -> np.ndarray:
        if self._last_prediction is None or self._last_Phi is None:
            return self.theta_hat
        
        error = x_measured - self._last_prediction
        delta_theta = self.Gamma @ self._last_Phi.T @ error
        self.theta_hat = self.theta_hat + delta_theta
        self.theta_hat = np.clip(self.theta_hat, self.theta_min, self.theta_max)
        self._history.append(self.theta_hat.copy())
        return self.theta_hat
    
    def get_history(self) -> np.ndarray:
        return np.array(self._history)
    
class AdaptiveMPCController:
    def __init__(self, prediction_horizon: int = 10, terminal_horizon: int = 5,
                 Q_diag: list = None, R_diag: list = None, T_diag: list = None,
                 omega_term: float = 10.0, q_xi: float = 1000.0,
                 d_safe: float = 0.3, v_max: float = 2.0, omega_max: float = 3.0,
                 dt: float = 0.02, enable_adaptation: bool = True,
                 adaptation_gamma: float = 0.005, theta_init: np.ndarray = None,
                 max_obstacles: int = 10):
                 
        self.N = prediction_horizon
        self.M = terminal_horizon
        self.N_ext = self.N + self.M
        self.dt = dt
        self.d_safe = d_safe
        self.v_max = v_max
        self.omega_max = omega_max
        self.max_obstacles = max_obstacles
        
        self.nx = 3
        self.nu = 2
        self.ny = 2
        self.C = np.array([[1, 0, 0], [0, 1, 0]])
        
        if Q_diag is None: Q_diag = [30.0, 30.0, 5.0]
        if R_diag is None: R_diag = [0.1, 0.1]
        
        self.Q = np.diag(Q_diag)
        self.R = np.diag(R_diag)
        self.omega_term = omega_term
        self.q_xi = q_xi
        
        self.u_min = np.array([-v_max, -omega_max])
        self.u_max = np.array([v_max, omega_max])
        self.x_max = np.array([50.0, 50.0, np.pi])
        self.x_min = -self.x_max
        
        self.enable_adaptation = enable_adaptation
        if theta_init is None: theta_init = np.array([0.85, 0.85])
        self.adaptation = LMSAdaptation(2, self.nx, adaptation_gamma, theta_init)
        
        self._solver = None
        self._lb_con = None
        self._ub_con = None
        self._warm_start = None
        
        self._build_solver()
        
        self._solve_count = 0
        self._total_solve_time = 0.0
    
    def _dynamics_ca(self, x, u, theta):
        dot_x = ca.vertcat(
            theta[0] * u[0] * ca.cos(x[2]),
            theta[0] * u[0] * ca.sin(x[2]),
            theta[1] * u[1]
        )
        return x + self.dt * dot_x
    
    def _compute_lqr_gain(self, theta: np.ndarray) -> np.ndarray:
        v_s, omega_s = theta[0], theta[1]
        v_ref, theta_ref = 0.1, 0.0
        
        A_c = np.array([[0, 0, -v_s * v_ref * np.sin(theta_ref)],
                        [0, 0,  v_s * v_ref * np.cos(theta_ref)],
                        [0, 0,  0]])
        B_c = np.array([[v_s * np.cos(theta_ref), 0],
                        [v_s * np.sin(theta_ref), 0],
                        [0,                       omega_s]])
        
        A_d = np.eye(self.nx) + self.dt * A_c
        B_d = self.dt * B_c
        
        try:
            if not hasattr(self, '_dare_cache'):
                self._dare_cache = {}
            # Quantize theta values slightly to maximize cache hits
            cache_key = (round(v_s, 3), round(omega_s, 3))
            
            if cache_key in self._dare_cache:
                P = self._dare_cache[cache_key]
            else:
                P = solve_discrete_are(A_d, B_d, self.Q, self.R)
                self._dare_cache[cache_key] = P
                
            K = np.linalg.solve(self.R + B_d.T @ P @ B_d, B_d.T @ P @ A_d)
        except Exception:
            K = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        return K
    
    def _build_solver(self):
        N, M, N_ext = self.N, self.M, self.N_ext
        nx, nu = self.nx, self.nu
        
        # Variables
        X = ca.SX.sym('X', nx, N_ext + 1)
        U = ca.SX.sym('U', nu, N_ext)
        Xi = ca.SX.sym('Xi', nx, N_ext + 1)
        
        # Parameters
        X0 = ca.SX.sym('X0', nx)
        theta = ca.SX.sym('theta', 2)
        X_ref = ca.SX.sym('X_ref', nx, N_ext + 1)
        U_ref = ca.SX.sym('U_ref', nu, N_ext)
        K_loc = ca.SX.sym('K_loc', nu, nx)
        obs_params = ca.SX.sym('obs', 3, self.max_obstacles)
        
        C = ca.DM(self.C)
        cost = 0
        con_eq = [X[:, 0] - X0]
        con_ineq = []
        
        Q = ca.DM(self.Q)
        R = ca.DM(self.R)
        
        # Free control steps
        for k in range(N):
            x_next = self._dynamics_ca(X[:, k], U[:, k], theta)
            con_eq.append(X[:, k + 1] - x_next)
            
            dx = X[:, k] - X_ref[:, k]
            du = U[:, k] - U_ref[:, k]
            cost += dx.T @ Q @ dx + du.T @ R @ du + self.q_xi * ca.sumsqr(Xi[:, k])
            
        # Terminal LQR rollout steps
        for k in range(N, N + M):
            dx = X[:, k] - X_ref[:, k]
            con_eq.append(U[:, k] - U_ref[:, k] + K_loc @ dx)
            
            x_next = self._dynamics_ca(X[:, k], U[:, k], theta)
            con_eq.append(X[:, k + 1] - x_next)
            
            du = U[:, k] - U_ref[:, k]
            cost += self.omega_term * (dx.T @ Q @ dx + du.T @ R @ du + self.q_xi * ca.sumsqr(Xi[:, k]))
            
        # Terminal cost
        dx_final = X[:, N + M] - X_ref[:, N + M]
        cost += self.omega_term * (dx_final.T @ Q @ dx_final + self.q_xi * ca.sumsqr(Xi[:, N + M]))
        
        # Obstacle avoidance (Euclidean norm)
        for j in range(self.max_obstacles):
            obs_x, obs_y, obs_r = obs_params[0, j], obs_params[1, j], obs_params[2, j]
            for k in range(N_ext + 1):
                pos_k = C @ (X[:, k] + Xi[:, k])
                dist_k = ca.norm_2(pos_k - ca.vertcat(obs_x, obs_y))
                con_ineq.append(dist_k - obs_r - self.d_safe)
                
        # Output constraints parsing
        con_input = [U[:, k] for k in range(N_ext)]
        con_state = [X[:, k] + Xi[:, k] for k in range(N_ext + 1)]
        
        g_eq = ca.vertcat(*con_eq)
        g_obs = ca.vertcat(*con_ineq) if len(con_ineq) > 0 else ca.vertcat()
        g_input = ca.vertcat(*con_input)
        g_state = ca.vertcat(*con_state)
        
        g = ca.vertcat(g_eq, g_obs, g_input, g_state)
        
        n_eq, n_obs, n_input, n_state = g_eq.shape[0], g_obs.shape[0], g_input.shape[0], g_state.shape[0]
        lb_g = np.zeros(n_eq + n_obs + n_input + n_state)
        ub_g = np.zeros(n_eq + n_obs + n_input + n_state)
        
        lb_g[:n_eq] = 0.0
        ub_g[:n_eq] = 0.0
        lb_g[n_eq:n_eq + n_obs] = 0.0
        ub_g[n_eq:n_eq + n_obs] = 1e6
        
        idx = n_eq + n_obs
        lb_g[idx:idx + n_input] = np.tile(self.u_min, N_ext)
        ub_g[idx:idx + n_input] = np.tile(self.u_max, N_ext)
        
        idx2 = idx + n_input
        lb_g[idx2:idx2 + n_state] = np.tile(self.x_min, N_ext + 1)
        ub_g[idx2:idx2 + n_state] = np.tile(self.x_max, N_ext + 1)
        
        vars_vec = ca.vertcat(
            ca.reshape(X, nx * (N_ext + 1), 1),
            ca.reshape(U, nu * N_ext, 1),
            ca.reshape(Xi, nx * (N_ext + 1), 1)
        )
        p = ca.vertcat(
            X0, theta,
            ca.reshape(X_ref, nx * (N_ext + 1), 1),
            ca.reshape(U_ref, nu * N_ext, 1),
            ca.reshape(K_loc, nu * nx, 1),
            ca.reshape(obs_params, 3 * self.max_obstacles, 1)
        )
        
        nlp = {'x': vars_vec, 'f': cost, 'g': g, 'p': p}
        opts = {
            'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0,
            'ipopt.max_iter': 100, 'ipopt.warm_start_init_point': 'yes',
            'ipopt.tol': 1e-3, 'ipopt.acceptable_tol': 5e-3, 'ipopt.acceptable_iter': 3,
            'ipopt.linear_solver': 'mumps'
        }
        self._solver = ca.nlpsol('adaptive_mpc', 'ipopt', nlp, opts)
        self._lb_con = lb_g
        self._ub_con = ub_g
        self._n_vars = vars_vec.shape[0]
        self._warm_start = np.zeros(self._n_vars)
        self._warm_start_lam_x = None
        self._warm_start_lam_g = None
        
    def solve(self, x0: np.ndarray, y_target: np.ndarray,
              obstacles: List[Obstacle] = None,
              x_refs: np.ndarray = None, u_refs: np.ndarray = None) -> MPCSolution:
        return pos_solve(self, x0, obstacles, x_refs, u_refs, y_target=y_target)
        
    def solve_tracking(self, x0: np.ndarray, x_refs: np.ndarray,
                       u_refs: np.ndarray, obstacles: List[Obstacle] = None) -> MPCSolution:
        return pos_solve(self, x0, obstacles, x_refs, u_refs)
    
    def adapt_parameters(self, x_measured: np.ndarray, x_prev: np.ndarray, u_prev: np.ndarray):
        if not self.enable_adaptation: return
        self.adaptation.predict_and_store(x_prev, u_prev, self.dt)
        self.adaptation.update(x_measured)
        self._K_lqr = self._compute_lqr_gain(self.adaptation.theta_hat)
        
    def get_stats(self):
        return {
            'solve_count': self._solve_count,
            'avg_solve_time_ms': self._total_solve_time / max(1, self._solve_count),
            'param_history_len': len(self.adaptation._history)
        }
        
    @property
    def param_estimates(self) -> np.ndarray:
        return self.adaptation.theta_hat.copy()

def pos_solve(self, x0, obstacles, x_refs, u_refs, y_target=None):
    start_time = time.perf_counter()
    if obstacles is None: obstacles = []
    
    if x_refs is None or u_refs is None:
        raise ValueError("AdaptiveMPC requires x_refs and u_refs for trajectory tracking.")
        
    N_ext, nx, nu = self.N_ext, self.nx, self.nu
    
    if len(x_refs) < N_ext + 1:
        x_refs_pad = np.zeros((N_ext + 1, nx))
        x_refs_pad[:len(x_refs)] = x_refs
        x_refs_pad[len(x_refs):] = x_refs[-1]
        x_refs = x_refs_pad
        
    if len(u_refs) < N_ext:
        u_refs_pad = np.zeros((N_ext, nu))
        u_refs_pad[:len(u_refs)] = u_refs
        u_refs_pad[len(u_refs):] = u_refs[-1]
        u_refs = u_refs_pad
        
    x_refs_unwrapped = x_refs.copy()
    x_refs_unwrapped[:, 2] = np.unwrap(x_refs[:, 2])
    
    theta_ref_0 = x_refs_unwrapped[0, 2]
    diff = x0[2] - theta_ref_0
    while diff > np.pi: diff -= 2*np.pi
    while diff < -np.pi: diff += 2*np.pi
    
    x0_adj = x0.copy()
    x0_adj[2] = theta_ref_0 + diff
    
    theta_hat = self.adaptation.theta_hat
    K_cur = self._compute_lqr_gain(theta_hat)
    
    obs_array = np.zeros((3, self.max_obstacles))
    for i, obs in enumerate(obstacles[:self.max_obstacles]):
        obs_array[0, i], obs_array[1, i], obs_array[2, i] = obs.x, obs.y, obs.radius
    for i in range(len(obstacles), self.max_obstacles):
        obs_array[0, i], obs_array[1, i], obs_array[2, i] = 1000.0, 1000.0, 0.01
        
    p_val = np.concatenate([
        x0_adj,
        theta_hat,
        x_refs_unwrapped[:N_ext+1].flatten(order='F'),
        u_refs[:N_ext].flatten(order='F'),
        K_cur.flatten(order='F'),
        obs_array.flatten(order='F')
    ])
    
    try:
        kwargs = {'x0': self._warm_start, 'p': p_val, 'lbg': self._lb_con, 'ubg': self._ub_con}
        if getattr(self, '_warm_start_lam_x', None) is not None:
            kwargs['lam_x0'] = self._warm_start_lam_x
            kwargs['lam_g0'] = self._warm_start_lam_g
        sol = self._solver(**kwargs)
        sol_x = np.array(sol['x']).flatten()
        solver_stats = self._solver.stats()
        success = solver_stats.get('success', False) or solver_stats.get('return_status', '') == 'Solve_Succeeded'
    except Exception as e:
        print(f"[AdaptiveMPC] Solver failed: {e}")
        return self._get_fallback_solution(x0, y_target if y_target is not None else x_refs[-1,:2], x_refs, u_refs, (time.perf_counter() - start_time) * 1000)
    
    solve_time = (time.perf_counter() - start_time) * 1000
    if not success:
        return self._get_fallback_solution(x0, y_target if y_target is not None else x_refs[-1,:2], x_refs, u_refs, solve_time)
        
    # Extract solution
    idx = 0
    X_sol = sol_x[idx:idx + nx * (N_ext + 1)].reshape(nx, N_ext + 1, order='F')
    idx += nx * (N_ext + 1)
    
    U_sol = sol_x[idx:idx + nu * N_ext].reshape(nu, N_ext, order='F')
    idx += nu * N_ext
    
    Xi_sol = sol_x[idx:idx + nx * (N_ext + 1)].reshape(nx, N_ext + 1, order='F')
    
    self._solve_count += 1
    self._total_solve_time += solve_time
    
    slack_used = np.any(np.abs(Xi_sol) > 1e-6)
    self._warm_start = sol_x.copy()
    if 'lam_x' in sol and 'lam_g' in sol:
        self._warm_start_lam_x = np.array(sol['lam_x']).flatten()
        self._warm_start_lam_g = np.array(sol['lam_g']).flatten()
    
    u_opt = np.clip(U_sol[:, 0], self.u_min, self.u_max)
    
    return MPCSolution(
        status="optimal",
        optimal_control=u_opt,
        control_sequence=U_sol[:, :self.N].T,
        predicted_states=X_sol[:, :self.N + 1].T,
        cost=float(sol['f']),
        solve_time_ms=solve_time,
        slack_used=slack_used,
        iterations=int(solver_stats.get('iter_count', 0))
    )

def _get_fallback_solution(self, x0, y_target, x_refs, u_refs, solve_time):
    error_xy = x0[:2] - y_target
    dist = np.linalg.norm(error_xy)
    if dist > 0.01:
        desired_theta = np.arctan2(-error_xy[1], -error_xy[0])
        theta_error = desired_theta - x0[2]
        while theta_error > np.pi: theta_error -= 2*np.pi
        while theta_error < -np.pi: theta_error += 2*np.pi
        v = min(0.5, dist) * np.cos(theta_error)
        omega = 2.0 * theta_error
    else:
        v, omega = 0.0, 0.0
    u_fallback = np.clip(np.array([v, omega]), self.u_min, self.u_max)
    return MPCSolution(
        status="fallback", optimal_control=u_fallback,
        control_sequence=np.tile(u_fallback, (self.N, 1)),
        predicted_states=np.tile(x0, (self.N + 1, 1)),
        cost=float('inf'), solve_time_ms=solve_time, slack_used=False, iterations=0
    )

AdaptiveMPCController._get_fallback_solution = _get_fallback_solution
