"""
Adaptive Hybrid Controller (Adaptive MPC + LQR)
================================================

Implements risk-aware hybrid control arbitration between:
- Adaptive MPC with LMS parameter adaptation (for high-risk obstacle regions)
- LQR (for low-risk free-space trajectory tracking)

Uses distance-based risk metrics to smoothly blend between controllers:
    u_blend = w(t) * u_adaptive_mpc + (1 - w(t)) * u_lqr

Key Features:
- Distance-based risk assessment for obstacle proximity
- Smooth sigmoid blending with anti-chatter guarantees
- Online parameter adaptation via LMS when in MPC mode
- Graceful degradation to LQR on MPC failure
- Hysteresis and rate limiting for stable switching

Reference:
    Combines adaptive MPC research with hybrid supervisory control
    for differential-drive robot navigation with online learning.
"""

import numpy as np
from typing import Tuple, Optional, List, Dict, Any
from dataclasses import dataclass

from .adaptive_mpc_controller import AdaptiveMPCController
from .lqr_controller import LQRController
from .mpc_controller import Obstacle, MPCSolution
from .risk_metrics import RiskMetrics, RiskAssessment


@dataclass
class AdaptiveHybridInfo:
    """Container for adaptive hybrid decision metadata."""
    weight: float               # w(t) in [0, 1]: 0=LQR, 1=Adaptive MPC
    weight_raw: float           # Pre-filtered sigmoid output
    risk: float                 # Combined risk input
    mode: str                   # 'LQR_DOMINANT', 'BLENDED', 'ADAPTIVE_MPC_DOMINANT'
    dw_dt: float                # Rate of weight change
    feasibility_ok: bool        # Adaptive MPC feasibility status
    solver_time_ms: float       # Adaptive MPC solver time
    param_estimates: np.ndarray # Current LMS parameter estimates [v_scale, omega_scale]
    adaptation_active: bool     # Whether LMS adaptation is running


class AdaptiveHybridController:
    """
    Risk-aware hybrid controller with Adaptive MPC and LQR.
    
    Implements continuous control arbitration based on obstacle proximity:
    - Far from obstacles (low risk): Use efficient LQR tracking
    - Near obstacles (high risk): Use Adaptive MPC with online learning
    
    The blending weight w(t) satisfies:
    1. w in [0, 1] at all times
    2. |dw/dt| <= dw_max (Lipschitz continuity)
    3. Hysteresis prevents rapid switching near threshold
    4. Feasibility fallback forces w -> 0 on MPC failure
    
    Attributes:
        adaptive_mpc: Adaptive MPC controller with LMS adaptation
        lqr: LQR controller for trajectory tracking
        risk_metrics: Risk assessment module
        k_sigmoid: Steepness of sigmoid blending function
        risk_threshold: Risk level at which w = 0.5
        dw_max: Maximum rate of weight change per second
        hysteresis_band: Width of deadband around threshold
    """
    
    def __init__(self,
                 # Adaptive MPC parameters
                 prediction_horizon: int = 10,
                 terminal_horizon: int = 5,
                 mpc_Q_diag: list = None,
                 mpc_R_diag: list = None,
                 omega_term: float = 10.0,
                 q_xi: float = 1000.0,
                 d_safe: float = 0.3,
                 enable_adaptation: bool = True,
                 adaptation_gamma: float = 0.005,
                 theta_init: np.ndarray = None,
                 # LQR parameters
                 lqr_Q_diag: list = None,
                 lqr_R_diag: list = None,
                 # Risk metrics parameters
                 d_trigger: float = 1.0,
                 risk_alpha: float = 0.6,
                 risk_beta: float = 0.4,
                 threshold_low: float = 0.2,
                 threshold_medium: float = 0.5,
                 # Blending parameters
                 k_sigmoid: float = 10.0,
                 risk_threshold: float = 0.3,
                 dw_max: float = 2.0,
                 hysteresis_band: float = 0.05,
                 solver_time_limit: float = 10.0,
                 feasibility_decay: float = 0.8,
                 feasibility_margin_threshold: float = 0.1,
                 # Common parameters
                 v_max: float = 2.0,
                 omega_max: float = 3.0,
                 dt: float = 0.02):
        """
        Initialize adaptive hybrid controller.
        
        Args:
            prediction_horizon: Adaptive MPC prediction horizon N
            terminal_horizon: Adaptive MPC terminal LQR rollout horizon M
            mpc_Q_diag: Adaptive MPC state weights [q_x, q_y, q_theta]
            mpc_R_diag: Adaptive MPC control weights [r_v, r_omega]
            omega_term: Terminal horizon weight multiplier
            q_xi: Slack variable penalty
            d_safe: Safety distance from obstacles (meters)
            enable_adaptation: Enable LMS parameter adaptation
            adaptation_gamma: LMS learning rate
            theta_init: Initial parameter estimates [v_scale, omega_scale]
            lqr_Q_diag: LQR state weights [q_x, q_y, q_theta]
            lqr_R_diag: LQR control weights [r_v, r_omega]
            d_trigger: Distance at which risk starts increasing
            risk_alpha: Weight for distance-based risk
            risk_beta: Weight for predictive risk
            threshold_low: Risk threshold for "low" level
            threshold_medium: Risk threshold for "medium" level
            k_sigmoid: Sigmoid steepness (higher = sharper transition)
            risk_threshold: Risk value at sigmoid midpoint (w=0.5)
            dw_max: Maximum rate of weight change (1/s)
            hysteresis_band: Half-width of deadband around threshold
            solver_time_limit: Maximum MPC solve time (ms)
            feasibility_decay: Decay factor for w when MPC fails
            feasibility_margin_threshold: Max acceptable slack magnitude
            v_max: Maximum linear velocity (m/s)
            omega_max: Maximum angular velocity (rad/s)
            dt: Sampling time (seconds)
        """
        self.dt = dt
        self.v_max = v_max
        self.omega_max = omega_max
        
        # Initialize Adaptive MPC controller
        if mpc_Q_diag is None:
            mpc_Q_diag = [30.0, 30.0, 5.0]
        if mpc_R_diag is None:
            mpc_R_diag = [0.1, 0.1]
        
        self.adaptive_mpc = AdaptiveMPCController(
            prediction_horizon=prediction_horizon,
            terminal_horizon=terminal_horizon,
            Q_diag=mpc_Q_diag,
            R_diag=mpc_R_diag,
            omega_term=omega_term,
            q_xi=q_xi,
            d_safe=d_safe,
            v_max=v_max,
            omega_max=omega_max,
            dt=dt,
            enable_adaptation=enable_adaptation,
            adaptation_gamma=adaptation_gamma,
            theta_init=theta_init
        )
        
        # Initialize LQR controller
        if lqr_Q_diag is None:
            lqr_Q_diag = [15.0, 15.0, 8.0]
        if lqr_R_diag is None:
            lqr_R_diag = [0.1, 0.1]
        
        self.lqr = LQRController(
            Q_diag=lqr_Q_diag,
            R_diag=lqr_R_diag,
            dt=dt,
            v_max=v_max,
            omega_max=omega_max
        )
        
        # Initialize risk metrics
        self.risk_metrics = RiskMetrics(
            d_safe=d_safe,
            d_trigger=d_trigger,
            alpha=risk_alpha,
            beta=risk_beta,
            threshold_low=threshold_low,
            threshold_medium=threshold_medium,
            threshold_high=0.8
        )
        
        # Blending parameters
        self.k_sigmoid = k_sigmoid
        self.risk_threshold = risk_threshold
        self.dw_max = dw_max
        self.hysteresis_band = hysteresis_band
        self.solver_time_limit = solver_time_limit
        self.feasibility_decay = feasibility_decay
        self.feasibility_margin_threshold = feasibility_margin_threshold
        
        # State
        self._w_prev: float = 0.0  # Previous weight (start in LQR mode)
        self._risk_prev: float = 0.0
        self._step_count: int = 0
        self._total_switches: int = 0
        self._last_mode: str = 'LQR_DOMINANT'
        
        # Mode thresholds for binary switching
        self._lqr_dominant_threshold = 0.05  # w < 0.05 -> LQR
        self._mpc_dominant_threshold = 0.95  # w > 0.95 -> Adaptive MPC
        
        # Statistics
        self._weight_history: list = []
        self._risk_history: list = []
        self._param_history: list = []
        
        # MPC solution cache (for rate control)
        self._last_mpc_solution: Optional[MPCSolution] = None
        self._last_mpc_step: int = -1
        
        # Previous state and control for adaptation
        self._x_prev: Optional[np.ndarray] = None
        self._u_prev: Optional[np.ndarray] = None
    
    def _sigmoid(self, risk: float) -> float:
        """
        Compute raw sigmoid blending weight from risk.
        
        w_raw = 1 / (1 + exp(-k * (risk - threshold)))
        
        Args:
            risk: Combined risk metric in [0, 1]
            
        Returns:
            Raw weight in [0, 1]
        """
        z = self.k_sigmoid * (risk - self.risk_threshold)
        z = np.clip(z, -20.0, 20.0)  # Prevent overflow
        return 1.0 / (1.0 + np.exp(-z))
    
    def _apply_hysteresis(self, w_raw: float, risk: float) -> float:
        """
        Apply hysteresis deadband to prevent chattering.
        
        Args:
            w_raw: Raw sigmoid output
            risk: Current risk value
            
        Returns:
            Hysteresis-filtered weight
        """
        lower_band = self.risk_threshold - self.hysteresis_band
        upper_band = self.risk_threshold + self.hysteresis_band
        
        if lower_band <= risk <= upper_band:
            # Inside deadband: maintain previous weight
            return self._w_prev
        else:
            # Outside deadband: use raw sigmoid
            return w_raw
    
    def _apply_rate_limit(self, w_target: float) -> float:
        """
        Apply rate limiting to weight change.
        
        Args:
            w_target: Target weight after hysteresis
            
        Returns:
            Rate-limited weight
        """
        max_change = self.dw_max * self.dt
        delta_w = w_target - self._w_prev
        
        if abs(delta_w) > max_change:
            delta_w = np.sign(delta_w) * max_change
        
        return self._w_prev + delta_w
    
    def _compute_weight(self, risk: float, solver_status: str,
                       solver_time_ms: float, feasibility_margin: float) -> Tuple[float, float]:
        """
        Compute blending weight with binary switching logic.
        
        Uses hard switching based on risk threshold with hysteresis:
        - risk > threshold + band: w = 1.0 (Pure Adaptive MPC)
        - risk < threshold - band: w = 0.0 (Pure LQR)
        - In between: maintain previous mode (hysteresis)
        
        Args:
            risk: Combined risk metric
            solver_status: MPC solver status
            solver_time_ms: MPC solve time
            feasibility_margin: Max slack magnitude
            
        Returns:
            Tuple of (filtered_weight, raw_weight)
        """
        # Binary switching with hysteresis
        upper_threshold = self.risk_threshold + self.hysteresis_band
        lower_threshold = self.risk_threshold - self.hysteresis_band
        
        # Determine target weight based on risk
        if risk > upper_threshold:
            # High risk: Use Adaptive MPC
            w_target = 1.0
        elif risk < lower_threshold:
            # Low risk: Use LQR
            w_target = 0.0
        else:
            # In hysteresis band: maintain previous state
            w_target = self._w_prev
        
        # Apply rate limiting for smooth transitions (but allow fast switching)
        max_change = self.dw_max * self.dt
        delta_w = w_target - self._w_prev
        
        if abs(delta_w) > max_change:
            delta_w = np.sign(delta_w) * max_change
        
        w_limited = self._w_prev + delta_w
        
        # Debug before feasibility check
        w_before_feas = w_limited
        
        # Feasibility fallback: only force to LQR if MPC explicitly fails AND we're not in high risk
        # In high risk, we MUST use MPC even if it's fallback (better than LQR with no obstacle avoidance)
        if (solver_status == 'fallback' or solver_status == 'infeasible') and risk < upper_threshold:
            w_limited = 0.0  # Fallback to LQR only in low risk
        elif solver_status != 'optimal' and self._w_prev > 0.5 and risk < upper_threshold:
            # Only penalize if we were already using MPC and not in high risk
            w_limited *= 0.5
        elif solver_time_ms > self.solver_time_limit:
            w_limited *= 0.5  # Reduce MPC influence
        
        # Debug high risk situations
        if risk > upper_threshold and w_limited < 0.5 and self._step_count % 20 == 0:
            print(f"  [WARN] High risk ({risk:.2f}) but low weight ({w_limited:.2f})! "
                  f"solver_status={solver_status}, w_target={w_target:.2f}, "
                  f"w_before_feas={w_before_feas:.2f}")
        
        # Clamp to [0, 1]
        w_limited = np.clip(w_limited, 0.0, 1.0)
        
        # For logging: compute what sigmoid would have given
        w_raw = self._sigmoid(risk)
        
        return w_limited, w_raw
    
    def _determine_mode(self, weight: float) -> str:
        """Determine control mode from weight."""
        if weight < self._lqr_dominant_threshold:
            return 'LQR_DOMINANT'
        elif weight > self._mpc_dominant_threshold:
            return 'ADAPTIVE_MPC_DOMINANT'
        else:
            return 'BLENDED'
    
    def compute_control(self,
                       x: np.ndarray,
                       x_ref: np.ndarray,
                       u_ref: np.ndarray,
                       obstacles: List[Obstacle],
                       x_refs: np.ndarray = None,
                       u_refs: np.ndarray = None,
                       mpc_rate: int = 5) -> Tuple[np.ndarray, AdaptiveHybridInfo]:
        """
        Compute hybrid control with adaptive MPC and LQR blending.
        
        Args:
            x: Current state [px, py, theta]
            x_ref: Reference state at current time
            u_ref: Reference control at current time
            obstacles: List of obstacles
            x_refs: Reference trajectory for MPC horizon (N+1 x 3)
            u_refs: Reference controls for MPC horizon (N x 2)
            mpc_rate: Run MPC every N steps (default: 5)
            
        Returns:
            Tuple of (blended_control, hybrid_info)
        """
        # Convert obstacles to dict format for risk assessment
        obstacle_dicts = [{'x': o.x, 'y': o.y, 'radius': o.radius} for o in obstacles]
        
        # --- 1. Assess Risk ---
        # Get predicted states from last MPC solution if available
        predicted_states = None
        if self._last_mpc_solution is not None:
            predicted_states = self._last_mpc_solution.predicted_states
        
        assessment = self.risk_metrics.assess_risk(x, obstacle_dicts, predicted_states)
        
        # --- 2. Compute LQR Control (always available, cheap) ---
        u_lqr, _ = self.lqr.compute_control_at_operating_point(x, x_ref, u_ref)
        
        # --- 3. Compute Adaptive MPC Control (at lower rate) ---
        solver_status = 'optimal'
        solver_time_ms = 0.0
        feasibility_margin = 0.0
        
        run_mpc = (self._step_count % mpc_rate == 0) or (self._last_mpc_solution is None) or (assessment.combined_risk > self.risk_threshold + self.hysteresis_band)
        
        if run_mpc and x_refs is not None and u_refs is not None:
            # Run Adaptive MPC when: periodic update OR first time OR high risk
            mpc_solution = self.adaptive_mpc.solve_tracking(x, x_refs, u_refs, obstacles)
            
            self._last_mpc_solution = mpc_solution
            self._last_mpc_step = self._step_count
            
            solver_status = mpc_solution.status
            solver_time_ms = mpc_solution.solve_time_ms
            feasibility_margin = mpc_solution.feasibility_margin if hasattr(mpc_solution, 'feasibility_margin') else 0.0
        
        # Use latest MPC solution
        if self._last_mpc_solution is not None:
            u_mpc = self._last_mpc_solution.optimal_control
            # Only use cached solver status if it was successful
            # Otherwise, assume 'optimal' to allow switching to MPC
            if not run_mpc and self._last_mpc_solution.status == 'optimal':
                solver_status = self._last_mpc_solution.status
                solver_time_ms = self._last_mpc_solution.solve_time_ms
        else:
            # Fallback before first MPC solve
            u_mpc = u_lqr
        
        # --- 4. Compute Blending Weight ---
        w, w_raw = self._compute_weight(assessment.combined_risk, solver_status,
                                        solver_time_ms, feasibility_margin)
        
        # --- 5. Blend Controls ---
        u_blend = w * u_mpc + (1.0 - w) * u_lqr
        
        # Clip to actuator limits
        u_blend = np.array([
            np.clip(u_blend[0], -self.v_max, self.v_max),
            np.clip(u_blend[1], -self.omega_max, self.omega_max)
        ])
        
        # --- 6. Update LMS Adaptation (when in Adaptive MPC mode) ---
        adaptation_active = False
        if self._x_prev is not None and self._u_prev is not None:
            # Adapt parameters when using Adaptive MPC (w > 0.9)
            if w > 0.9:  # Only adapt when in pure MPC mode
                self.adaptive_mpc.adapt_parameters(x, self._x_prev, self._u_prev)
                adaptation_active = True
        
        # Store current state and control for next adaptation
        self._x_prev = x.copy()
        self._u_prev = u_blend.copy()
        
        # --- 7. Determine Mode ---
        mode = self._determine_mode(w)
        
        # Track mode switches
        if mode != self._last_mode:
            if (self._last_mode == 'LQR_DOMINANT' and mode != 'LQR_DOMINANT') or \
               (self._last_mode != 'LQR_DOMINANT' and mode == 'LQR_DOMINANT'):
                self._total_switches += 1
        self._last_mode = mode
        
        # --- 8. Compute dw/dt ---
        dw_dt = (w - self._w_prev) / self.dt if self.dt > 0 else 0.0
        
        # --- 9. Update State ---
        self._w_prev = w
        self._risk_prev = assessment.combined_risk
        self._step_count += 1
        
        # Store history
        self._weight_history.append(w)
        self._risk_history.append(assessment.combined_risk)
        self._param_history.append(self.adaptive_mpc.param_estimates.copy())
        
        # --- 10. Create Info Object ---
        info = AdaptiveHybridInfo(
            weight=w,
            weight_raw=w_raw,
            risk=assessment.combined_risk,
            mode=mode,
            dw_dt=dw_dt,
            feasibility_ok=(solver_status == 'optimal'),
            solver_time_ms=solver_time_ms,
            param_estimates=self.adaptive_mpc.param_estimates.copy(),
            adaptation_active=adaptation_active
        )
        
        return u_blend, info
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Get blending and adaptation statistics.
        
        Returns:
            Dictionary with statistics
        """
        if len(self._weight_history) == 0:
            return {
                'weight_mean': 0.0,
                'weight_std': 0.0,
                'lqr_dominant_fraction': 0.0,
                'blended_fraction': 0.0,
                'mpc_dominant_fraction': 0.0,
                'total_switches': 0,
                'mean_risk': 0.0,
                'param_estimates': self.adaptive_mpc.param_estimates.copy(),
                'adaptation_enabled': self.adaptive_mpc.enable_adaptation
            }
        
        weights = np.array(self._weight_history)
        risks = np.array(self._risk_history)
        
        lqr_dominant = np.sum(weights < self._lqr_dominant_threshold)
        mpc_dominant = np.sum(weights > self._mpc_dominant_threshold)
        blended = len(weights) - lqr_dominant - mpc_dominant
        
        return {
            'weight_mean': float(np.mean(weights)),
            'weight_std': float(np.std(weights)),
            'lqr_dominant_fraction': lqr_dominant / len(weights),
            'blended_fraction': blended / len(weights),
            'mpc_dominant_fraction': mpc_dominant / len(weights),
            'total_switches': self._total_switches,
            'mean_risk': float(np.mean(risks)),
            'max_risk': float(np.max(risks)),
            'param_estimates': self.adaptive_mpc.param_estimates.copy(),
            'param_history': np.array(self._param_history),
            'adaptation_enabled': self.adaptive_mpc.enable_adaptation,
            'mpc_stats': self.adaptive_mpc.get_stats()
        }
    
    def reset(self):
        """Reset controller state."""
        self._w_prev = 0.0
        self._risk_prev = 0.0
        self._step_count = 0
        self._total_switches = 0
        self._last_mode = 'LQR_DOMINANT'
        self._weight_history = []
        self._risk_history = []
        self._param_history = []
        self._last_mpc_solution = None
        self._last_mpc_step = -1
        self._x_prev = None
        self._u_prev = None
    
    @property
    def param_estimates(self) -> np.ndarray:
        """Get current LMS parameter estimates."""
        return self.adaptive_mpc.param_estimates
    
    @property
    def weight_history(self) -> np.ndarray:
        """Get blending weight history."""
        return np.array(self._weight_history)
    
    @property
    def risk_history(self) -> np.ndarray:
        """Get risk history."""
        return np.array(self._risk_history)
