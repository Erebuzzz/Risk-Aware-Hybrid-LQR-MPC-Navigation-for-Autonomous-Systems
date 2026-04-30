"""
Differential Drive Robot Model
==============================

Implements the nonlinear kinematic model for a differential drive robot.

State Vector:
    x = [p_x, p_y, θ]^T
    - p_x: x-position in world frame
    - p_y: y-position in world frame
    - θ: orientation (heading angle)

Control Inputs:
    u = [v, ω]^T
    - v: linear velocity
    - ω: angular velocity

Continuous-Time Dynamics:
    ṗ_x = v·cos(θ)
    ṗ_y = v·sin(θ)
    θ̇   = ω

Reference:
    Risk-Aware Hybrid LQR-MPC Navigation for Autonomous Systems
    Section: Differential Drive Robot Modeling
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class RobotState:
    """Robot state representation."""
    px: float  # x-position (meters)
    py: float  # y-position (meters)
    theta: float  # orientation (radians)
    
    def to_array(self) -> np.ndarray:
        """Convert to numpy array."""
        return np.array([self.px, self.py, self.theta])
    
    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'RobotState':
        """Create from numpy array."""
        return cls(px=arr[0], py=arr[1], theta=arr[2])
    
    def __repr__(self) -> str:
        return f"RobotState(px={self.px:.3f}, py={self.py:.3f}, theta={self.theta:.3f})"


@dataclass
class ControlInput:
    """Control input representation."""
    v: float  # linear velocity (m/s)
    omega: float  # angular velocity (rad/s)
    
    def to_array(self) -> np.ndarray:
        """Convert to numpy array."""
        return np.array([self.v, self.omega])
    
    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'ControlInput':
        """Create from numpy array."""
        return cls(v=arr[0], omega=arr[1])
    
    def __repr__(self) -> str:
        return f"ControlInput(v={self.v:.3f}, omega={self.omega:.3f})"


class DifferentialDriveRobot:
    """
    Differential drive robot kinematic model.
    
    Implements the nonlinear kinematics and provides methods for:
    - Computing state derivatives
    - Simulating forward dynamics
    - State propagation with Euler/RK4 integration
    
    Attributes:
        v_max: Maximum linear velocity (m/s)
        omega_max: Maximum angular velocity (rad/s)
        wheel_base: Distance between wheels (m)
    
    Example:
        robot = DifferentialDriveRobot(v_max=1.0, omega_max=1.5)
        state = np.array([0, 0, 0])
        control = np.array([0.5, 0.1])
        next_state = robot.simulate_step(state, control, dt=0.02)
    """
    
    # State and control dimensions
    STATE_DIM = 3  # [px, py, theta]
    CONTROL_DIM = 2  # [v, omega]
    
    def __init__(self, v_max: float = 1.0, omega_max: float = 1.5,
                 wheel_base: float = 0.3):
        """
        Initialize differential drive robot model.
        
        Args:
            v_max: Maximum linear velocity (m/s)
            omega_max: Maximum angular velocity (rad/s)
            wheel_base: Distance between wheels (m)
        """
        self.v_max = v_max
        self.omega_max = omega_max
        self.wheel_base = wheel_base
    
    def continuous_dynamics(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        """
        Compute the continuous-time state derivative.
        
        Implements:
            ṗ_x = v·cos(θ)
            ṗ_y = v·sin(θ)
            θ̇   = ω
        
        Args:
            state: Current state [px, py, theta]
            control: Control input [v, omega]
            
        Returns:
            State derivative [ṗ_x, ṗ_y, θ̇]
        """
        px, py, theta = state
        v, omega = control
        
        dx = np.array([
            v * np.cos(theta),  # ṗ_x
            v * np.sin(theta),  # ṗ_y
            omega               # θ̇
        ])
        
        return dx
    
    def simulate_step(self, state: np.ndarray, control: np.ndarray, 
                      dt: float, method: str = 'euler') -> np.ndarray:
        """
        Simulate one time step of robot motion.
        
        Args:
            state: Current state [px, py, theta]
            control: Control input [v, omega]
            dt: Time step (seconds)
            method: Integration method ('euler' or 'rk4')
            
        Returns:
            Next state after time step dt
        """
        # Clip control inputs to limits
        control = self.clip_control(control)
        
        if method == 'euler':
            # Euler integration: x_{k+1} = x_k + dt * f(x_k, u_k)
            dx = self.continuous_dynamics(state, control)
            next_state = state + dt * dx
        elif method == 'rk4':
            # Runge-Kutta 4th order
            k1 = self.continuous_dynamics(state, control)
            k2 = self.continuous_dynamics(state + 0.5 * dt * k1, control)
            k3 = self.continuous_dynamics(state + 0.5 * dt * k2, control)
            k4 = self.continuous_dynamics(state + dt * k3, control)
            next_state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        else:
            raise ValueError(f"Unknown integration method: {method}")
        
        # Normalize theta to [-pi, pi]
        next_state[2] = self.normalize_angle(next_state[2])
        
        return next_state
    
    def simulate_trajectory(self, x0: np.ndarray, controls: np.ndarray,
                            dt: float, method: str = 'euler') -> np.ndarray:
        """
        Simulate a full trajectory given initial state and control sequence.
        
        Args:
            x0: Initial state [px, py, theta]
            controls: Control sequence of shape (N, 2)
            dt: Time step (seconds)
            method: Integration method
            
        Returns:
            State trajectory of shape (N+1, 3) including initial state
        """
        N = len(controls)
        trajectory = np.zeros((N + 1, self.STATE_DIM))
        trajectory[0] = x0.copy()
        
        for k in range(N):
            trajectory[k + 1] = self.simulate_step(
                trajectory[k], controls[k], dt, method
            )
        
        return trajectory
    
    def clip_control(self, control: np.ndarray) -> np.ndarray:
        """
        Clip control inputs to actuator limits.
        
        Args:
            control: Control input [v, omega]
            
        Returns:
            Clipped control within [v_max, omega_max] bounds
        """
        clipped = np.array([
            np.clip(control[0], -self.v_max, self.v_max),
            np.clip(control[1], -self.omega_max, self.omega_max)
        ])
        return clipped
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize angle to [-pi, pi] range.
        
        Args:
            angle: Angle in radians
            
        Returns:
            Normalized angle in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def compute_tracking_error(self, state: np.ndarray, 
                               state_ref: np.ndarray) -> np.ndarray:
        """
        Compute tracking error with proper angle wrapping.
        
        Args:
            state: Current state [px, py, theta]
            state_ref: Reference state [px_ref, py_ref, theta_ref]
            
        Returns:
            Tracking error [e_x, e_y, e_theta]
        """
        error = state - state_ref
        # Normalize angle error
        error[2] = self.normalize_angle(error[2])
        return error
    
    def get_wheel_velocities(self, v: float, omega: float) -> Tuple[float, float]:
        """
        Convert (v, omega) to left and right wheel velocities.
        
        Args:
            v: Linear velocity (m/s)
            omega: Angular velocity (rad/s)
            
        Returns:
            Tuple of (left_wheel_vel, right_wheel_vel)
        """
        # v = (v_r + v_l) / 2
        # omega = (v_r - v_l) / L
        v_left = v - (self.wheel_base / 2) * omega
        v_right = v + (self.wheel_base / 2) * omega
        return v_left, v_right
    
    def from_wheel_velocities(self, v_left: float, v_right: float) -> Tuple[float, float]:
        """
        Convert wheel velocities to (v, omega).
        
        Args:
            v_left: Left wheel velocity (m/s)
            v_right: Right wheel velocity (m/s)
            
        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        v = (v_right + v_left) / 2
        omega = (v_right - v_left) / self.wheel_base
        return v, omega
