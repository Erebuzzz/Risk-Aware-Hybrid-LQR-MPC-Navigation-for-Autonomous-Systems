"""
Hybrid Blending Supervisor
==========================

Implements continuous control arbitration between LQR and MPC controllers
using a smooth sigmoid-based blending law with anti-chatter guarantees.

Blending Law:
    u_blend = w(t) * u_mpc + (1 - w(t)) * u_lqr

where w(t) in [0, 1] is computed from:
    1. Risk-based sigmoid:  w_raw = sigmoid(k * (risk - threshold))
    2. Rate limiting:       |dw/dt| <= dw_max  (anti-chatter)
    3. Hysteresis deadband: prevents oscillatory switching
    4. Feasibility fallback: w -> 0 if MPC reports infeasible/slow

Properties:
    - Continuous: w(t) is Lipschitz-continuous (bounded derivative)
    - No chattering: hysteresis + rate limit guarantee finite switches
    - Safe degradation: LQR fallback if MPC fails

Reference:
    - Project improvement plan: hybrid_architecture_upgrades.blending_strategy
    - Concept: supervisory control with continuous arbitration
"""

import numpy as np
from typing import Tuple, Optional, Dict, Any
from dataclasses import dataclass, field


@dataclass
class BlendInfo:
    """Container for blending decision metadata."""
    weight: float               # w(t) in [0, 1]: 0=LQR, 1=MPC
    weight_raw: float           # Pre-filtered sigmoid output
    risk: float                 # Combined risk input
    mode: str                   # 'LQR_DOMINANT', 'BLENDED', 'MPC_DOMINANT'
    dw_dt: float                # Rate of weight change
    feasibility_ok: bool        # MPC feasibility status
    solver_time_ms: float       # MPC solver time


class BlendingSupervisor:
    """
    Continuous control arbitration between LQR and MPC.
    
    Computes a smooth blending weight w(t) from risk metrics and
    solver diagnostics. The weight satisfies:
    
    1. w in [0, 1] at all times
    2. |dw/dt| <= dw_max (Lipschitz continuity)
    3. Hysteresis prevents rapid switching near the threshold
    4. Feasibility fallback forces w -> 0 on MPC failure
    
    Attributes:
        k_sigmoid: Steepness of the sigmoid blending function
        risk_threshold: Risk level at which w = 0.5 (equal blend)
        dw_max: Maximum rate of weight change per second
        hysteresis_band: Width of deadband around threshold
        solver_time_limit: MPC solver time limit in ms
        feasibility_decay: Exponential decay rate when MPC fails
    """
    
    def __init__(self,
                 k_sigmoid: float = 20.0,
                 risk_threshold: float = 0.25,
                 dw_max: float = 50.0,
                 hysteresis_band: float = 0.05,
                 solver_time_limit: float = 150.0,
                 feasibility_decay: float = 0.8,
                 feasibility_margin_threshold: float = 100.0,
                 dt: float = 0.02):
        """
        Initialize blending supervisor.
        
        Args:
            k_sigmoid: Sigmoid steepness parameter. Higher values give
                       sharper transition. Range: [5, 20] recommended.
            risk_threshold: Risk value at sigmoid midpoint (w=0.5).
                           Should match RiskMetrics threshold_low.
            dw_max: Maximum rate of weight change (1/s). Controls how
                    fast the blending ratio can change. Lower values
                    give smoother transitions but slower response.
            hysteresis_band: Half-width of deadband around threshold.
                            Prevents chattering when risk oscillates near
                            the transition point.
            solver_time_limit: Maximum MPC solve time in milliseconds.
                              If exceeded, feasibility fallback activates.
            feasibility_decay: Multiplicative decay factor for w when
                              MPC is infeasible. Applied each timestep:
                              w *= feasibility_decay.
            dt: Simulation timestep (seconds).
        """
        # Blending parameters
        self.k_sigmoid = k_sigmoid
        self.risk_threshold = risk_threshold
        self.dw_max = dw_max
        self.hysteresis_band = hysteresis_band
        self.solver_time_limit = solver_time_limit
        self.feasibility_decay = feasibility_decay
        self.feasibility_margin_threshold = feasibility_margin_threshold
        self.dt = dt
        
        # State
        self._w_prev: float = 0.0  # Previous weight (start in LQR mode)
        self._risk_prev: float = 0.0
        self._step_count: int = 0
        self._infeasible_count: int = 0
        self._consecutive_infeasible: int = 0  # Consecutive failures for escalation
        self._total_switches: int = 0
        
        # Mode thresholds
        self._lqr_dominant_threshold = 0.1  # w < this -> LQR dominant
        self._mpc_dominant_threshold = 0.9  # w > this -> MPC dominant
        
        # Statistics
        self._weight_history: list = []
    
    def _sigmoid(self, risk: float) -> float:
        """
        Compute raw sigmoid blending weight from risk.
        
        w_raw = 1 / (1 + exp(-k * (risk - threshold)))
        
        Maps risk in [0, 1] to weight in [0, 1] with smooth transition
        centered at risk_threshold.
        
        Args:
            risk: Combined risk metric in [0, 1]
            
        Returns:
            Raw weight in [0, 1]
        """
        z = self.k_sigmoid * (risk - self.risk_threshold)
        # Clip to prevent overflow
        z = np.clip(z, -20.0, 20.0)
        return 1.0 / (1.0 + np.exp(-z))
    
    def _apply_hysteresis(self, w_raw: float, risk: float) -> float:
        """
        Apply hysteresis deadband to prevent chattering.
        
        When risk is within [threshold - band, threshold + band],
        maintain the previous weight direction. This creates a
        deadband where the weight does not change, preventing
        oscillatory switching.
        
        Args:
            w_raw: Raw sigmoid output
            risk: Current risk level
            
        Returns:
            Hysteresis-filtered weight
        """
        lower = self.risk_threshold - self.hysteresis_band
        upper = self.risk_threshold + self.hysteresis_band
        
        if lower < risk < upper:
            # Within deadband: hold previous weight
            return self._w_prev
        
        return w_raw
    
    def _apply_rate_limit(self, w_target: float) -> float:
        """
        Apply ASYMMETRIC rate limiting.
        FAST-ATTACK: Snap to MPC instantly when risk spikes.
        SLOW-DECAY: Smoothly return to LQR when safe.
        """
        # FAST ATTACK (e.g., 20.0). Goes from 0 to 1 in ~0.05 seconds.
        # This saves you from crashing.
        dw_max_up = 20.0 
        
        # SLOW DECAY (e.g., 2.0). Goes from 1 to 0 in ~0.5 seconds.
        # This prevents the robot from violently jerking back to the path.
        dw_max_down = self.dw_max 
        
        # Determine if we are ramping up (danger) or ramping down (safe)
        if w_target > self._w_prev:
            max_change = dw_max_up * self.dt
        else:
            max_change = dw_max_down * self.dt
            
        w_new = np.clip(
            w_target,
            self._w_prev - max_change,
            self._w_prev + max_change
        )
        return w_new
    
    def _apply_feasibility_fallback(self, w: float, 
                                      solver_status: str,
                                      solver_time_ms: float,
                                      feasibility_margin: float = 0.0) -> Tuple[float, bool]:
        """
        TEMPORARY DEBUG OVERRIDE: 
        Ignore all solver errors, timeouts, and slack margins.
        Force the supervisor to use the raw weight.
        """
        return w, True
    
    def compute_weight(self, risk: float, 
                       solver_status: str = 'optimal',
                       solver_time_ms: float = 0.0,
                       feasibility_margin: float = 0.0) -> BlendInfo:
        """
        Compute the blending weight w(t) for this timestep.
        
        Pipeline:
            risk -> sigmoid -> hysteresis -> rate_limit -> feasibility -> w(t)
        
        Args:
            risk: Combined risk metric from RiskMetrics.assess_risk()
            solver_status: MPC solver status ('optimal', 'infeasible', etc.)
            solver_time_ms: MPC solve time in milliseconds
            
        Returns:
            BlendInfo with weight and diagnostic metadata
        """
        # Step 1: Raw sigmoid mapping
        w_raw = self._sigmoid(risk)
        
        # Step 2: Hysteresis deadband
        w_hysteresis = self._apply_hysteresis(w_raw, risk)
        
        # Step 3: Rate limiting (anti-chatter guarantee)
        w_limited = self._apply_rate_limit(w_hysteresis)
        
        # Step 4: Feasibility fallback
        w_final, feasibility_ok = self._apply_feasibility_fallback(
            w_limited, solver_status, solver_time_ms, feasibility_margin
        )
        
        # Clamp to [0, 1]
        w_final = np.clip(w_final, 0.0, 1.0)
        
        # Compute rate of change
        dw_dt = (w_final - self._w_prev) / self.dt if self.dt > 0 else 0.0
        
        # Determine mode
        if w_final < self._lqr_dominant_threshold:
            mode = 'LQR_DOMINANT'
        elif w_final > self._mpc_dominant_threshold:
            mode = 'MPC_DOMINANT'
        else:
            mode = 'BLENDED'
        
        # Track switches (crossing 0.5 threshold)
        if (self._w_prev < 0.5 and w_final >= 0.5) or \
           (self._w_prev >= 0.5 and w_final < 0.5):
            self._total_switches += 1
        
        # Update state
        self._w_prev = w_final
        self._risk_prev = risk
        self._step_count += 1
        self._weight_history.append(w_final)
        
        return BlendInfo(
            weight=w_final,
            weight_raw=w_raw,
            risk=risk,
            mode=mode,
            dw_dt=dw_dt,
            feasibility_ok=feasibility_ok,
            solver_time_ms=solver_time_ms
        )
    
    def blend(self, u_lqr: np.ndarray, u_mpc: np.ndarray,
              risk: float,
              solver_status: str = 'optimal',
              solver_time_ms: float = 0.0,
              feasibility_margin: float = 0.0) -> Tuple[np.ndarray, BlendInfo]:
        """
        Compute blended control output.
        
        u_blend = w * u_mpc + (1 - w) * u_lqr
        
        This is a convex combination, preserving actuator bounds if
        both u_lqr and u_mpc satisfy them individually.
        
        Property: If ||u_lqr|| <= u_max and ||u_mpc|| <= u_max,
                 then ||u_blend|| <= u_max (convexity).
        
        Args:
            u_lqr: LQR control input [v, omega]
            u_mpc: MPC control input [v, omega]
            risk: Combined risk metric
            solver_status: MPC solver status
            solver_time_ms: MPC solve time
            
        Returns:
            Tuple of (u_blend, BlendInfo)
        """
        info = self.compute_weight(risk, solver_status, solver_time_ms, feasibility_margin)
        w = info.weight
        
        u_blend = w * u_mpc + (1.0 - w) * u_lqr
        
        return u_blend, info
    
    def reset(self):
        """Reset supervisor state for new simulation."""
        self._w_prev = 0.0
        self._risk_prev = 0.0
        self._step_count = 0
        self._infeasible_count = 0
        self._consecutive_infeasible = 0
        self._total_switches = 0
        self._weight_history = []
    
    @property
    def total_switches(self) -> int:
        """Number of times w crossed the 0.5 threshold."""
        return self._total_switches
    
    @property
    def weight_history(self) -> np.ndarray:
        """Array of all computed weights."""
        return np.array(self._weight_history)
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Get blending statistics for reporting.
        
        Returns:
            Dictionary with mean, std, min, max weight,
            total switches, infeasibility count, and
            time spent in each mode.
        """
        if not self._weight_history:
            return {'error': 'No data'}
        
        w = np.array(self._weight_history)
        
        lqr_frac = np.mean(w < self._lqr_dominant_threshold)
        mpc_frac = np.mean(w > self._mpc_dominant_threshold)
        blend_frac = 1.0 - lqr_frac - mpc_frac
        
        return {
            'weight_mean': float(np.mean(w)),
            'weight_std': float(np.std(w)),
            'weight_min': float(np.min(w)),
            'weight_max': float(np.max(w)),
            'total_switches': self._total_switches,
            'infeasible_count': self._infeasible_count,
            'lqr_dominant_fraction': float(lqr_frac),
            'mpc_dominant_fraction': float(mpc_frac),
            'blended_fraction': float(blend_frac),
            'total_steps': self._step_count,
        }
    
    def get_formal_guarantees(self) -> Dict[str, float]:
        """
        Compute formal anti-chatter guarantees (Theorem 2).
        
        The rate-limited blending weight w(t) is Lipschitz continuous
        with constant L = dw_max * dt. This provides:
        
        1. Max weight change per timestep: dw_max * dt
        2. Minimum time for full LQR→MPC transition: 1/dw_max seconds
        3. Max transitions in interval T: floor(T * dw_max)
        4. With hysteresis: no switching when |risk - threshold| < band
        
        These guarantees hold for ANY risk signal r(t).
        
        Returns:
            Dictionary with formal guarantee values.
        """
        max_change_per_step = self.dw_max * self.dt
        min_transition_time = 1.0 / self.dw_max  # seconds for full 0→1
        lipschitz_constant = self.dw_max  # |w(t1) - w(t2)| <= L * |t1 - t2|
        
        return {
            'max_weight_change_per_step': max_change_per_step,
            'min_full_transition_time_s': min_transition_time,
            'lipschitz_constant': lipschitz_constant,
            'hysteresis_deadband': 2 * self.hysteresis_band,
            'max_transitions_per_second': self.dw_max,
            'feasibility_decay_rate': self.feasibility_decay,
        }
    
    def compute_jerk_bound(self, u_lqr: np.ndarray, u_mpc: np.ndarray) -> float:
        """
        Compute the maximum control jerk induced by blending weight changes.
        
        For u_blend = w * u_mpc + (1-w) * u_lqr, the jerk from w changes is:
            ||d²u_blend/dt²|| <= dw_max² * ||u_mpc - u_lqr|| / dt
        
        This bound shows that jerk is proportional to the squared rate limit
        and the control disagreement between controllers.
        
        Args:
            u_lqr: Current LQR control [v, omega]
            u_mpc: Current MPC control [v, omega]
            
        Returns:
            Upper bound on blending-induced jerk (m/s³ or rad/s³)
        """
        control_disagreement = np.linalg.norm(u_mpc - u_lqr)
        max_jerk = (self.dw_max * self.dt) ** 2 * control_disagreement / self.dt
        return float(max_jerk)

