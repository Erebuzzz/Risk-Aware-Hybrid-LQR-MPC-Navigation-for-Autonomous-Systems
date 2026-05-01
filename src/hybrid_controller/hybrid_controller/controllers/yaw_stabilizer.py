"""
Yaw Stabilizer
==============

High-rate PID yaw controller for transient damping.

This inner-loop controller runs at a higher frequency than MPC (typically 5x)
to absorb heading transients during:
- Trajectory startup (cold-start)
- Obstacle encounters
- Controller handovers (LQR ↔ MPC)

Reference:
    Risk-Aware Hybrid LQR-MPC Navigation for Autonomous Systems
    Section: Hierarchical Control Architecture
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional


@dataclass
class YawStabilizerState:
    """Container for yaw stabilizer internal state."""
    error: float = 0.0
    integral: float = 0.0
    derivative: float = 0.0
    output: float = 0.0
    saturated: bool = False


class YawStabilizer:
    """
    High-rate PID yaw controller for transient damping.
    
    Designed to run at 5x the MPC rate (e.g., 250 Hz vs 50 Hz) to provide
    fast heading correction during transient phases.
    
    The controller operates in three modes:
    1. ACTIVE: Full PID control for heading stabilization
    2. PASSTHROUGH: Passes MPC omega command directly (low error)
    3. BLENDED: Smooth transition between active and passthrough
    
    Attributes:
        kp: Proportional gain (heading error → omega)
        ki: Integral gain (accumulated error → omega)
        kd: Derivative gain (error rate → omega)
        dt: Control timestep (seconds)
        omega_max: Maximum angular velocity limit (rad/s)
        
    Example:
        stabilizer = YawStabilizer(kp=3.0, ki=0.1, kd=0.5, dt=0.004)
        
        for step in simulation:
            omega_mpc = mpc.solve(...)
            theta_ref = mpc.predicted_heading[0]
            
            omega_cmd = stabilizer.compute(
                theta_current, theta_ref, omega_mpc
            )
            robot.apply(v_mpc, omega_cmd)
    """
    
    def __init__(self, 
                 kp: float = 3.0,
                 ki: float = 0.1,
                 kd: float = 0.5,
                 dt: float = 0.004,
                 omega_max: float = 3.0,
                 error_threshold_active: float = 0.1,    # ~6°
                 error_threshold_passthrough: float = 0.05,  # ~3°
                 integral_limit: float = 1.0,
                 derivative_filter_tau: float = 0.02):
        """
        Initialize yaw stabilizer.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain  
            dt: Control timestep (seconds)
            omega_max: Angular velocity limit (rad/s)
            error_threshold_active: Error above which PID is fully active (rad)
            error_threshold_passthrough: Error below which MPC passes through (rad)
            integral_limit: Anti-windup limit for integral term
            derivative_filter_tau: Low-pass filter time constant for derivative
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.omega_max = omega_max
        
        # Mode thresholds
        self.error_threshold_active = error_threshold_active
        self.error_threshold_passthrough = error_threshold_passthrough
        
        # Anti-windup
        self.integral_limit = integral_limit
        
        # Derivative filtering
        self.derivative_filter_tau = derivative_filter_tau
        self._alpha = dt / (derivative_filter_tau + dt)
        
        # State
        self._integral = 0.0
        self._prev_error = 0.0
        self._filtered_derivative = 0.0
        self._prev_output = 0.0
        
        # Diagnostics
        self._mode = "PASSTHROUGH"
        self._blend_factor = 0.0
    
    def reset(self):
        """Reset controller state (call on mode switch or discontinuity)."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._filtered_derivative = 0.0
        self._prev_output = 0.0
        self._mode = "PASSTHROUGH"
        self._blend_factor = 0.0
    
    def compute(self, 
                theta: float, 
                theta_ref: float, 
                omega_mpc: float = 0.0) -> float:
        """
        Compute stabilized angular velocity command.
        
        Args:
            theta: Current heading angle (rad)
            theta_ref: Reference heading angle (rad)
            omega_mpc: MPC angular velocity command (rad/s)
            
        Returns:
            Stabilized angular velocity command (rad/s)
        """
        # Compute wrapped heading error
        error = self._wrap_angle(theta_ref - theta)
        error_abs = abs(error)
        
        # Determine mode and blend factor
        if error_abs >= self.error_threshold_active:
            self._mode = "ACTIVE"
            self._blend_factor = 1.0
        elif error_abs <= self.error_threshold_passthrough:
            self._mode = "PASSTHROUGH"
            self._blend_factor = 0.0
        else:
            self._mode = "BLENDED"
            # Linear interpolation between thresholds
            range_width = self.error_threshold_active - self.error_threshold_passthrough
            self._blend_factor = (error_abs - self.error_threshold_passthrough) / range_width
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self._integral += error * self.dt
        self._integral = np.clip(self._integral, -self.integral_limit, self.integral_limit)
        i_term = self.ki * self._integral
        
        # Derivative term with low-pass filter
        raw_derivative = (error - self._prev_error) / self.dt
        self._filtered_derivative = (self._alpha * raw_derivative + 
                                      (1 - self._alpha) * self._filtered_derivative)
        d_term = self.kd * self._filtered_derivative
        
        # PID output
        omega_pid = p_term + i_term + d_term
        
        # Blend PID with MPC command
        omega_blended = self._blend_factor * omega_pid + (1 - self._blend_factor) * omega_mpc
        
        # Apply rate limiting for smooth transitions
        max_rate = 10.0  # rad/s²
        omega_rate = (omega_blended - self._prev_output) / self.dt
        if abs(omega_rate) > max_rate:
            omega_blended = self._prev_output + np.sign(omega_rate) * max_rate * self.dt
        
        # Saturate to limits
        omega_output = np.clip(omega_blended, -self.omega_max, self.omega_max)
        
        # Store state for next iteration
        self._prev_error = error
        self._prev_output = omega_output
        
        return omega_output
    
    def get_state(self) -> YawStabilizerState:
        """Get current controller state for logging/diagnostics."""
        return YawStabilizerState(
            error=self._prev_error,
            integral=self._integral,
            derivative=self._filtered_derivative,
            output=self._prev_output,
            saturated=abs(self._prev_output) >= self.omega_max * 0.99
        )
    
    @property
    def mode(self) -> str:
        """Current operating mode: ACTIVE, BLENDED, or PASSTHROUGH."""
        return self._mode
    
    @property
    def blend_factor(self) -> float:
        """Current blend factor: 1.0 = full PID, 0.0 = full MPC passthrough."""
        return self._blend_factor
    
    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle


class CascadeController:
    """
    Cascade controller combining outer-loop MPC with inner-loop yaw stabilizer.
    
    Architecture:
        MPC (outer) → [v_cmd, theta_ref] → YawStabilizer (inner) → [v_cmd, omega_cmd]
    
    The outer MPC provides velocity and heading reference, while the inner
    yaw stabilizer converts heading reference to angular velocity with
    fast transient rejection.
    """
    
    def __init__(self, 
                 mpc_dt: float = 0.02,
                 inner_dt: float = 0.004,
                 stabilizer_params: dict = None):
        """
        Initialize cascade controller.
        
        Args:
            mpc_dt: MPC control period (seconds)
            inner_dt: Inner loop control period (seconds)
            stabilizer_params: Optional dict of YawStabilizer parameters
        """
        self.mpc_dt = mpc_dt
        self.inner_dt = inner_dt
        self.inner_rate_multiplier = int(mpc_dt / inner_dt)
        
        # Initialize yaw stabilizer
        params = stabilizer_params or {}
        params['dt'] = inner_dt
        self.stabilizer = YawStabilizer(**params)
        
        # Cache MPC outputs
        self._v_mpc = 0.0
        self._theta_ref = 0.0
        self._omega_mpc = 0.0
    
    def set_mpc_command(self, v: float, theta_ref: float, omega: float):
        """
        Set commands from outer MPC loop.
        
        Called once per MPC cycle to update reference for inner loop.
        """
        self._v_mpc = v
        self._theta_ref = theta_ref
        self._omega_mpc = omega
    
    def compute_inner(self, theta: float) -> Tuple[float, float]:
        """
        Compute inner-loop control.
        
        Called multiple times per MPC cycle (at inner loop rate).
        
        Args:
            theta: Current heading (rad)
            
        Returns:
            Tuple of (v_cmd, omega_cmd)
        """
        omega_cmd = self.stabilizer.compute(
            theta, self._theta_ref, self._omega_mpc
        )
        return self._v_mpc, omega_cmd
    
    def reset(self):
        """Reset controller state."""
        self.stabilizer.reset()
        self._v_mpc = 0.0
        self._theta_ref = 0.0
        self._omega_mpc = 0.0
