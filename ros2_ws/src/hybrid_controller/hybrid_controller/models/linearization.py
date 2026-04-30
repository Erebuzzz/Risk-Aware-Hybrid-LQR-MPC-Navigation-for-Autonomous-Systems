"""
Linearization Module
====================

Implements Jacobian-based linearization and discretization for the
differential drive robot model.

Continuous-Time Jacobians:
    A = ∂f/∂x = [[0, 0, -v_r·sin(θ_r)],
                 [0, 0,  v_r·cos(θ_r)],
                 [0, 0,  0]]

    B = ∂f/∂u = [[cos(θ_r), 0],
                 [sin(θ_r), 0],
                 [0,        1]]

Discrete-Time Approximation (Zero-Order Hold):
    A_d ≈ I + A·T_s
    B_d ≈ B·T_s

Reference:
    Risk-Aware Hybrid LQR-MPC Navigation for Autonomous Systems
    Section: Linearization Around a Reference Trajectory
    Section: Discretization for Digital Control
"""

import numpy as np
from typing import Tuple
from scipy.linalg import expm


class Linearizer:
    """
    Linearization and discretization for differential drive robot.
    
    Computes Jacobian matrices around a reference operating point and
    provides discrete-time state-space models for LQR and MPC controllers.
    
    Example:
        linearizer = Linearizer(dt=0.02)
        
        # Get discrete model at operating point
        A_d, B_d = linearizer.get_discrete_model(v_r=0.5, theta_r=0.0)
        
        # Use for state prediction
        x_next = A_d @ x + B_d @ u
    """
    
    # Dimensions
    STATE_DIM = 3   # [px, py, theta]
    CONTROL_DIM = 2  # [v, omega]
    
    def __init__(self, dt: float = 0.02):
        """
        Initialize the linearizer.
        
        Args:
            dt: Sampling time T_s (seconds)
        """
        self.dt = dt
    
    def get_jacobians(self, v_r: float, theta_r: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute continuous-time Jacobian matrices at the operating point.
        
        The Jacobians are computed as:
            A = ∂f/∂x = [[0, 0, -v_r·sin(θ_r)],
                         [0, 0,  v_r·cos(θ_r)],
                         [0, 0,  0]]

            B = ∂f/∂u = [[cos(θ_r), 0],
                         [sin(θ_r), 0],
                         [0,        1]]
        
        Args:
            v_r: Reference linear velocity (m/s)
            theta_r: Reference orientation (radians)
            
        Returns:
            Tuple of (A, B) continuous-time Jacobian matrices
        """
        # A matrix: ∂f/∂x
        A = np.array([
            [0, 0, -v_r * np.sin(theta_r)],
            [0, 0,  v_r * np.cos(theta_r)],
            [0, 0,  0]
        ])
        
        # B matrix: ∂f/∂u
        B = np.array([
            [np.cos(theta_r), 0],
            [np.sin(theta_r), 0],
            [0,               1]
        ])
        
        return A, B
    
    def discretize_euler(self, A: np.ndarray, B: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Discretize using first-order Euler approximation (Zero-Order Hold).
        
        Computes:
            A_d ≈ I + A·T_s
            B_d ≈ B·T_s
        
        This approximation is valid for sufficiently small T_s.
        
        Args:
            A: Continuous-time state matrix
            B: Continuous-time input matrix
            
        Returns:
            Tuple of (A_d, B_d) discrete-time matrices
        """
        A_d = np.eye(self.STATE_DIM) + A * self.dt
        B_d = B * self.dt
        
        return A_d, B_d
    
    def discretize_exact(self, A: np.ndarray, B: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Discretize using exact matrix exponential (more accurate).
        
        Computes:
            A_d = exp(A·T_s)
            B_d = ∫₀^{T_s} exp(A·τ) dτ · B
        
        For the differential drive case where A has specific structure,
        this can be computed in closed form or via matrix exponential.
        
        Args:
            A: Continuous-time state matrix
            B: Continuous-time input matrix
            
        Returns:
            Tuple of (A_d, B_d) discrete-time matrices
        """
        # Compute A_d = exp(A·T_s)
        A_d = expm(A * self.dt)
        
        # For B_d, use the augmented matrix method
        # [A  B]      [exp(A*dt)  ∫exp(A*τ)dτ·B]
        # [0  0]  =>  [0          I            ]
        n = self.STATE_DIM
        m = self.CONTROL_DIM
        
        # Build augmented matrix
        augmented = np.zeros((n + m, n + m))
        augmented[:n, :n] = A * self.dt
        augmented[:n, n:] = B * self.dt
        
        # Compute matrix exponential of augmented system
        exp_aug = expm(augmented)
        
        # Extract B_d from upper right block
        B_d = exp_aug[:n, n:]
        
        return A_d, B_d
    
    def get_discrete_model(self, v_r: float, theta_r: float,
                           method: str = 'euler') -> Tuple[np.ndarray, np.ndarray]:
        """
        Get the discrete-time state-space model at the given operating point.
        
        Combines Jacobian computation and discretization.
        
        Args:
            v_r: Reference linear velocity (m/s)
            theta_r: Reference orientation (radians)
            method: Discretization method ('euler' or 'exact')
            
        Returns:
            Tuple of (A_d, B_d) discrete-time state-space matrices
            
        Example:
            A_d, B_d = linearizer.get_discrete_model(v_r=0.5, theta_r=0.0)
            # State prediction: x_{k+1} = A_d @ x_k + B_d @ u_k
        """
        # Get continuous-time Jacobians
        A, B = self.get_jacobians(v_r, theta_r)
        
        # Discretize
        if method == 'euler':
            return self.discretize_euler(A, B)
        elif method == 'exact':
            return self.discretize_exact(A, B)
        else:
            raise ValueError(f"Unknown discretization method: {method}")
    
    def get_discrete_model_explicit(self, v_r: float, theta_r: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get discrete model using explicit formulas (from LaTeX document).
        
        Direct computation without intermediate Jacobian step:
            A_d = [[1, 0, -v_r·sin(θ_r)·T_s],
                   [0, 1,  v_r·cos(θ_r)·T_s],
                   [0, 0,  1]]
                   
            B_d = [[cos(θ_r)·T_s, 0],
                   [sin(θ_r)·T_s, 0],
                   [0,            T_s]]
        
        Args:
            v_r: Reference linear velocity (m/s)
            theta_r: Reference orientation (radians)
            
        Returns:
            Tuple of (A_d, B_d) discrete-time matrices
        """
        sin_theta = np.sin(theta_r)
        cos_theta = np.cos(theta_r)
        
        A_d = np.array([
            [1, 0, -v_r * sin_theta * self.dt],
            [0, 1,  v_r * cos_theta * self.dt],
            [0, 0,  1]
        ])
        
        B_d = np.array([
            [cos_theta * self.dt, 0],
            [sin_theta * self.dt, 0],
            [0,                   self.dt]
        ])
        
        return A_d, B_d
    
    def predict_trajectory(self, x0: np.ndarray, controls: np.ndarray,
                           v_refs: np.ndarray, theta_refs: np.ndarray) -> np.ndarray:
        """
        Predict state trajectory using Linear Time-Varying (LTV) model.
        
        Uses different linearization points at each time step for
        more accurate prediction along the trajectory.
        
        Args:
            x0: Initial state [px, py, theta]
            controls: Control sequence of shape (N, 2)
            v_refs: Reference velocities for linearization (N,)
            theta_refs: Reference orientations for linearization (N,)
            
        Returns:
            Predicted state trajectory of shape (N+1, 3)
        """
        N = len(controls)
        trajectory = np.zeros((N + 1, self.STATE_DIM))
        trajectory[0] = x0.copy()
        
        for k in range(N):
            # Get discrete model at this operating point
            A_d, B_d = self.get_discrete_model_explicit(v_refs[k], theta_refs[k])
            
            # Predict next state
            trajectory[k + 1] = A_d @ trajectory[k] + B_d @ controls[k]
        
        return trajectory
    
    def predict_horizon(self, x0: np.ndarray, u_seq: np.ndarray,
                        v_r: float, theta_r: float) -> np.ndarray:
        """
        Predict states over a horizon using a fixed linearization point.
        
        Args:
            x0: Initial state [px, py, theta]
            u_seq: Control sequence of shape (N, 2)
            v_r: Reference velocity for linearization
            theta_r: Reference orientation for linearization
            
        Returns:
            State trajectory of shape (N+1, 3)
        """
        A_d, B_d = self.get_discrete_model_explicit(v_r, theta_r)
        
        N = len(u_seq)
        trajectory = np.zeros((N + 1, self.STATE_DIM))
        trajectory[0] = x0.copy()
        
        for k in range(N):
            trajectory[k + 1] = A_d @ trajectory[k] + B_d @ u_seq[k]
        
        return trajectory
    
    @staticmethod
    def build_prediction_matrices(A_d: np.ndarray, B_d: np.ndarray, 
                                   N: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Build batch prediction matrices for MPC.
        
        Constructs matrices such that:
            X = Phi @ x0 + Gamma @ U
            
        where X = [x_1, x_2, ..., x_N]^T is the stacked state vector
        and U = [u_0, u_1, ..., u_{N-1}]^T is the stacked control vector.
        
        Args:
            A_d: Discrete state matrix (n x n)
            B_d: Discrete input matrix (n x m)
            N: Prediction horizon
            
        Returns:
            Tuple of (Phi, Gamma) prediction matrices
        """
        n = A_d.shape[0]  # State dimension
        m = B_d.shape[1]  # Control dimension
        
        # Phi matrix: relates initial state to future states
        # Phi = [A; A^2; A^3; ...; A^N]
        Phi = np.zeros((n * N, n))
        A_power = np.eye(n)
        for i in range(N):
            A_power = A_power @ A_d
            Phi[i*n:(i+1)*n, :] = A_power
        
        # Gamma matrix: relates control inputs to future states
        # Lower block triangular structure
        Gamma = np.zeros((n * N, m * N))
        for i in range(N):
            for j in range(i + 1):
                A_power_ij = np.linalg.matrix_power(A_d, i - j)
                Gamma[i*n:(i+1)*n, j*m:(j+1)*m] = A_power_ij @ B_d
        
        return Phi, Gamma
