"""
LQR Controller
==============

Implements the discrete-time Linear Quadratic Regulator (LQR) for 
trajectory tracking.

Discrete Algebraic Riccati Equation (DARE):
    P = A_dᵀ·P·A_d - A_dᵀ·P·B_d·(R + B_dᵀ·P·B_d)⁻¹·B_dᵀ·P·A_d + Q

Optimal Gain:
    K = (R + B_dᵀ·P·B_d)⁻¹·B_dᵀ·P·A_d

Control Law:
    ũ_k = -K·x̃_k
    u_k = u_{r,k} + ũ_k

where x̃_k = x_k - x_{r,k} is the tracking error.

Reference:
    Risk-Aware Hybrid LQR-MPC Navigation for Autonomous Systems
    Section: LQR Controller Formulation
    Section: Discrete-Time LQR Formulation
"""

import numpy as np
from typing import Tuple, Optional
from scipy.linalg import solve_discrete_are

from ..models.linearization import Linearizer


class LQRController:
    """
    Discrete-time LQR controller for trajectory tracking.
    
    Computes the optimal feedback gain by solving the Discrete Algebraic
    Riccati Equation (DARE) and applies the control law for tracking
    a reference trajectory.
    
    Attributes:
        Q: State error weight matrix (3x3)
        R: Control effort weight matrix (2x2)
        K: Current LQR feedback gain matrix (2x3)
        dt: Sampling time (seconds)
    
    Example:
        lqr = LQRController(Q_diag=[10, 10, 1], R_diag=[0.1, 0.1], dt=0.02)
        
        # Compute gain at operating point
        K = lqr.compute_gain(v_r=0.5, theta_r=0.0)
        
        # Compute control input
        u = lqr.compute_control(x, x_ref, u_ref)
    """
    
    def __init__(self, Q_diag: list = None, R_diag: list = None, 
                 dt: float = 0.02, v_max: float = 1.0, omega_max: float = 1.5):
        """
        Initialize LQR controller.
        
        Args:
            Q_diag: Diagonal elements of state weight matrix [q_x, q_y, q_theta]
            R_diag: Diagonal elements of control weight matrix [r_v, r_omega]
            dt: Sampling time (seconds)
            v_max: Maximum linear velocity (m/s)
            omega_max: Maximum angular velocity (rad/s)
        """
        # Default weights from LaTeX document
        if Q_diag is None:
            Q_diag = [10.0, 10.0, 1.0]
        if R_diag is None:
            R_diag = [0.1, 0.1]
        
        self.Q = np.diag(Q_diag)
        self.R = np.diag(R_diag)
        self.dt = dt
        self.v_max = v_max
        self.omega_max = omega_max
        
        # Linearizer for computing A_d, B_d
        self.linearizer = Linearizer(dt=dt)
        
        # Current gain matrix (computed lazily)
        self.K: Optional[np.ndarray] = None
        self.P: Optional[np.ndarray] = None  # Riccati solution
        
        # Last operating point
        self._last_v_r: float = 0.0
        self._last_theta_r: float = 0.0
    
    def compute_gain(self, v_r: float, theta_r: float, 
                     force_recompute: bool = False) -> np.ndarray:
        """
        Compute the optimal LQR feedback gain at the given operating point.
        
        Solves the DARE:
            P = A_dᵀ·P·A_d - A_dᵀ·P·B_d·(R + B_dᵀ·P·B_d)⁻¹·B_dᵀ·P·A_d + Q
            
        Then computes:
            K = (R + B_dᵀ·P·B_d)⁻¹·B_dᵀ·P·A_d
        
        Args:
            v_r: Reference linear velocity (m/s)
            theta_r: Reference orientation (radians)
            force_recompute: If True, recompute even if operating point unchanged
            
        Returns:
            Optimal gain matrix K of shape (2, 3)
        """
        # Check if we need to recompute
        if not force_recompute and self.K is not None:
            if abs(v_r - self._last_v_r) < 1e-6 and abs(theta_r - self._last_theta_r) < 1e-6:
                return self.K
        
        # Get discrete-time model at operating point
        A_d, B_d = self.linearizer.get_discrete_model_explicit(v_r, theta_r)
        
        # Handle edge case where v_r = 0 (system becomes uncontrollable in x-y)
        if abs(v_r) < 1e-6:
            # Use small velocity to maintain controllability
            A_d, B_d = self.linearizer.get_discrete_model_explicit(0.01, theta_r)
        
        try:
            # Solve DARE
            self.P = solve_discrete_are(A_d, B_d, self.Q, self.R)
            
            # Compute optimal gain
            # K = (R + B_dᵀ·P·B_d)⁻¹·B_dᵀ·P·A_d
            BtPB = B_d.T @ self.P @ B_d
            BtPA = B_d.T @ self.P @ A_d
            self.K = np.linalg.solve(self.R + BtPB, BtPA)
            
        except Exception as e:
            # Fallback to simple proportional gain if DARE fails
            # This can happen at singular points
            self.K = np.array([
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0]
            ])
            print(f"Warning: DARE solver failed, using fallback gain. Error: {e}")
        
        # Store operating point
        self._last_v_r = v_r
        self._last_theta_r = theta_r
        
        return self.K
    
    def compute_control(self, x: np.ndarray, x_ref: np.ndarray,
                        u_ref: np.ndarray, K: np.ndarray = None) -> np.ndarray:
        """
        Compute the control input using LQR feedback.
        
        Control law:
            x̃_k = x_k - x_{r,k}
            ũ_k = -K·x̃_k
            u_k = u_{r,k} + ũ_k
        
        Args:
            x: Current state [px, py, theta]
            x_ref: Reference state [px_ref, py_ref, theta_ref]
            u_ref: Reference control [v_ref, omega_ref]
            K: Optional gain matrix (uses self.K if None)
            
        Returns:
            Control input u = [v, omega]
        """
        if K is None:
            if self.K is None:
                # Compute gain using reference velocity and orientation
                self.compute_gain(u_ref[0], x_ref[2])
            K = self.K
        
        # Compute tracking error
        error = x - x_ref
        
        # Normalize angle error to [-pi, pi]
        error[2] = self._normalize_angle(error[2])
        
        # Compute feedback control: ũ = -K·x̃
        u_feedback = -K @ error
        
        # Add to reference: u = u_r + ũ
        u = u_ref + u_feedback
        
        # Clip to actuator limits
        u = self._clip_control(u)
        
        return u
    
    def compute_control_at_operating_point(self, x: np.ndarray, 
                                           x_ref: np.ndarray,
                                           u_ref: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute control with gain recomputation at the current operating point.
        
        Args:
            x: Current state
            x_ref: Reference state
            u_ref: Reference control
            
        Returns:
            Tuple of (control, tracking_error)
        """
        # Recompute gain at current operating point
        K = self.compute_gain(u_ref[0], x_ref[2])
        
        # Compute tracking error
        error = x - x_ref
        error[2] = self._normalize_angle(error[2])
        
        # Compute control
        u = self.compute_control(x, x_ref, u_ref, K)
        
        return u, error
    
    def get_lqr_gain(self, v_r: float, theta_r: float, dt: float = None) -> np.ndarray:
        """
        Compute LQR gain (compatible with LaTeX Python snippet).
        
        Args:
            v_r: Reference linear velocity
            theta_r: Reference orientation
            dt: Sampling time (uses self.dt if None)
            
        Returns:
            Optimal LQR gain K
        """
        if dt is not None and abs(dt - self.dt) > 1e-9:
            # Create temporary linearizer with different dt
            temp_linearizer = Linearizer(dt=dt)
            A_d, B_d = temp_linearizer.get_discrete_model_explicit(v_r, theta_r)
        else:
            A_d, B_d = self.linearizer.get_discrete_model_explicit(v_r, theta_r)
        
        # Solve DARE
        P = solve_discrete_are(A_d, B_d, self.Q, self.R)
        
        # Compute optimal LQR gain
        K = np.linalg.inv(self.R + B_d.T @ P @ B_d) @ (B_d.T @ P @ A_d)
        
        return K
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def _clip_control(self, u: np.ndarray) -> np.ndarray:
        """Clip control inputs to actuator limits."""
        return np.array([
            np.clip(u[0], -self.v_max, self.v_max),
            np.clip(u[1], -self.omega_max, self.omega_max)
        ])
    
    def get_cost_matrices(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return the Q and R cost matrices."""
        return self.Q.copy(), self.R.copy()
    
    def set_weights(self, Q_diag: list = None, R_diag: list = None) -> None:
        """
        Update the cost weights and invalidate cached gain.
        
        Args:
            Q_diag: New state weight diagonal
            R_diag: New control weight diagonal
        """
        if Q_diag is not None:
            self.Q = np.diag(Q_diag)
        if R_diag is not None:
            self.R = np.diag(R_diag)
        
        # Invalidate cached gain
        self.K = None
        self.P = None
    
    @property
    def gain_computed(self) -> bool:
        """Check if a gain has been computed."""
        return self.K is not None
