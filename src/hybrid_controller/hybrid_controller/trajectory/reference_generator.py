"""
Reference Trajectory Generator
==============================

Implements the Figure-8 (Lemniscate) reference trajectory for benchmarking
trajectory tracking performance.

Trajectory Definition:
    p_{x,r}(t) = A·sin(a·t)
    p_{y,r}(t) = A·sin(a·t)·cos(a·t)
    θ_r(t) = arctan2(ṗ_{y,r}, ṗ_{x,r})
    v_r(t) = √(ṗ_{x,r}² + ṗ_{y,r}²)
    ω_r(t) = Δθ_r / T_s

where:
    A - controls the spatial size of the trajectory
    a - controls the temporal speed of traversal

Reference:
    Risk-Aware Hybrid LQR-MPC Navigation for Autonomous Systems
    Section: Reference Trajectory Generation
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class TrajectoryPoint:
    """Single point on the reference trajectory."""
    t: float        # Time (seconds)
    px: float       # x-position (meters)
    py: float       # y-position (meters)
    theta: float    # orientation (radians)
    v: float        # linear velocity (m/s)
    omega: float    # angular velocity (rad/s)
    
    def get_state(self) -> np.ndarray:
        """Get state vector [px, py, theta]."""
        return np.array([self.px, self.py, self.theta])
    
    def get_control(self) -> np.ndarray:
        """Get control vector [v, omega]."""
        return np.array([self.v, self.omega])


class ReferenceTrajectoryGenerator:
    """
    Figure-8 (Lemniscate) reference trajectory generator.
    
    Generates smooth, time-varying reference trajectories for trajectory
    tracking benchmarks. The figure-8 shape exercises both left and right
    turns and produces rich excitation for controller tuning.
    
    Attributes:
        A: Spatial amplitude (meters)
        a: Angular frequency (rad/s)
        dt: Sampling time (seconds)
    
    Example:
        generator = ReferenceTrajectoryGenerator(A=2.0, a=0.5, dt=0.02)
        trajectory = generator.generate(duration=20.0)
        
        # Get reference at specific time step
        x_ref, u_ref = generator.get_reference_at_time(t=5.0)
    """
    
    def __init__(self, A: float = 2.0, a: float = 0.5, dt: float = 0.02):
        """
        Initialize trajectory generator.
        
        Args:
            A: Spatial amplitude - controls size of trajectory (meters)
            a: Angular frequency - controls speed of traversal (rad/s)
            dt: Sampling time for discrete trajectory (seconds)
        """
        self.A = A
        self.a = a
        self.dt = dt
        
        # Cached trajectory data
        self._trajectory: Optional[np.ndarray] = None
        self._duration: float = 0.0
    
    def position(self, t: float) -> Tuple[float, float]:
        """
        Compute reference position at time t.
        
        p_{x,r}(t) = A·sin(a·t)
        p_{y,r}(t) = A·sin(a·t)·cos(a·t) = A/2·sin(2at)
        
        Args:
            t: Time in seconds
            
        Returns:
            Tuple of (px, py) position
        """
        px = self.A * np.sin(self.a * t)
        py = self.A * np.sin(self.a * t) * np.cos(self.a * t)
        return px, py
    
    def velocity(self, t: float) -> Tuple[float, float]:
        """
        Compute reference velocity (time derivatives of position) at time t.
        
        ṗ_{x,r}(t) = a·A·cos(a·t)
        ṗ_{y,r}(t) = a·A·(cos²(a·t) - sin²(a·t)) = a·A·cos(2at)
        
        Args:
            t: Time in seconds
            
        Returns:
            Tuple of (dpx, dpy) velocity components
        """
        dpx = self.a * self.A * np.cos(self.a * t)
        dpy = self.a * self.A * (np.cos(self.a * t)**2 - np.sin(self.a * t)**2)
        return dpx, dpy
    
    def heading(self, t: float) -> float:
        """
        Compute reference heading angle at time t.
        
        θ_r(t) = arctan2(ṗ_{y,r}, ṗ_{x,r})
        
        Args:
            t: Time in seconds
            
        Returns:
            Heading angle in radians
        """
        dpx, dpy = self.velocity(t)
        return np.arctan2(dpy, dpx)
    
    def linear_velocity(self, t: float) -> float:
        """
        Compute reference linear velocity magnitude at time t.
        
        v_r(t) = √(ṗ_{x,r}² + ṗ_{y,r}²)
        
        Args:
            t: Time in seconds
            
        Returns:
            Linear velocity magnitude (m/s)
        """
        dpx, dpy = self.velocity(t)
        return np.sqrt(dpx**2 + dpy**2)
    
    def angular_velocity(self, t: float) -> float:
        """
        Compute reference angular velocity at time t using numerical derivative.
        
        ω_r(t) ≈ (θ_r(t + dt) - θ_r(t)) / dt
        
        Args:
            t: Time in seconds
            
        Returns:
            Angular velocity (rad/s)
        """
        theta_now = self.heading(t)
        theta_next = self.heading(t + self.dt)
        
        # Handle angle wrapping
        dtheta = theta_next - theta_now
        while dtheta > np.pi:
            dtheta -= 2 * np.pi
        while dtheta < -np.pi:
            dtheta += 2 * np.pi
        
        return dtheta / self.dt
    
    def get_reference_at_time(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get reference state and control at a specific time.
        
        Args:
            t: Time in seconds
            
        Returns:
            Tuple of (x_ref, u_ref) where:
                x_ref = [px, py, theta]
                u_ref = [v, omega]
        """
        px, py = self.position(t)
        theta = self.heading(t)
        v = self.linear_velocity(t)
        omega = self.angular_velocity(t)
        
        x_ref = np.array([px, py, theta])
        u_ref = np.array([v, omega])
        
        return x_ref, u_ref
    
    def generate(self, duration: float) -> np.ndarray:
        """
        Generate discrete-time reference trajectory.
        
        Args:
            duration: Total trajectory duration (seconds)
            
        Returns:
            Trajectory array of shape (N, 6) where columns are:
                [t, px, py, theta, v, omega]
        """
        t = np.arange(0, duration, self.dt)
        N = len(t)
        
        trajectory = np.zeros((N, 6))
        trajectory[:, 0] = t
        
        for k in range(N):
            tk = t[k]
            px, py = self.position(tk)
            theta = self.heading(tk)
            v = self.linear_velocity(tk)
            omega = self.angular_velocity(tk)
            
            trajectory[k, 1] = px
            trajectory[k, 2] = py
            trajectory[k, 3] = theta
            trajectory[k, 4] = v
            trajectory[k, 5] = omega
        
        # Cache trajectory
        self._trajectory = trajectory
        self._duration = duration
        
        return trajectory
    
    def generate_figure_8(self, t_range: float, dt: float,
                          A: float = None, a: float = None) -> np.ndarray:
        """
        Generate Figure-8 trajectory (compatible with LaTeX Python snippet).
        
        Args:
            t_range: Duration of trajectory (seconds)
            dt: Time step (seconds)
            A: Override amplitude (uses self.A if None)
            a: Override frequency (uses self.a if None)
            
        Returns:
            Trajectory array of shape (N, 5) where columns are:
                [x, y, theta, v, omega]
        """
        if A is None:
            A = self.A
        if a is None:
            a = self.a
        
        t = np.arange(0, t_range, dt)
        
        # Reference position
        xr = A * np.sin(a * t)
        yr = A * np.sin(a * t) * np.cos(a * t)
        
        # Derivatives
        dxr = a * A * np.cos(a * t)
        dyr = a * A * (np.cos(a * t)**2 - np.sin(a * t)**2)
        
        # Heading
        thetar = np.arctan2(dyr, dxr)
        
        # Linear velocity
        vr = np.sqrt(dxr**2 + dyr**2)
        
        # Angular velocity (numerical derivative)
        wr = np.diff(thetar, append=thetar[-1]) / dt
        
        # Handle angle wrapping in angular velocity
        wr = np.where(wr > np.pi/dt, wr - 2*np.pi/dt, wr)
        wr = np.where(wr < -np.pi/dt, wr + 2*np.pi/dt, wr)
        
        return np.stack([xr, yr, thetar, vr, wr], axis=1)
    
    def get_reference_at_index(self, k: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get reference state and control at a specific index from cached trajectory.
        
        Args:
            k: Time step index
            
        Returns:
            Tuple of (x_ref, u_ref)
        """
        if self._trajectory is None:
            raise ValueError("Trajectory not generated. Call generate() first.")
        
        if k >= len(self._trajectory):
            k = len(self._trajectory) - 1  # Clamp to last point
        
        point = self._trajectory[k]
        x_ref = np.array([point[1], point[2], point[3]])  # px, py, theta
        u_ref = np.array([point[4], point[5]])  # v, omega
        
        return x_ref, u_ref
    
    def get_trajectory_segment(self, start_idx: int, 
                                horizon: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get a segment of the trajectory for MPC prediction.
        
        Args:
            start_idx: Starting index
            horizon: Number of steps to extract
            
        Returns:
            Tuple of (x_refs, u_refs) where:
                x_refs has shape (horizon, 3)
                u_refs has shape (horizon, 2)
        """
        if self._trajectory is None:
            raise ValueError("Trajectory not generated. Call generate() first.")
        
        N = len(self._trajectory)
        x_refs = np.zeros((horizon, 3))
        u_refs = np.zeros((horizon, 2))
        
        for i in range(horizon):
            idx = min(start_idx + i, N - 1)
            point = self._trajectory[idx]
            x_refs[i] = [point[1], point[2], point[3]]
            u_refs[i] = [point[4], point[5]]
        
        return x_refs, u_refs
    
    @property
    def num_points(self) -> int:
        """Number of points in the cached trajectory."""
        if self._trajectory is None:
            return 0
        return len(self._trajectory)
    
    def get_trajectory_bounds(self) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """
        Get spatial bounds of the trajectory.
        
        Returns:
            Tuple of ((x_min, x_max), (y_min, y_max))
        """
        if self._trajectory is None:
            # Compute analytically
            x_max = self.A
            y_max = self.A / 2  # Maximum of sin(2t) is 1
            return ((-x_max, x_max), (-y_max, y_max))
        
        x_min, x_max = self._trajectory[:, 1].min(), self._trajectory[:, 1].max()
        y_min, y_max = self._trajectory[:, 2].min(), self._trajectory[:, 2].max()
        return ((x_min, x_max), (y_min, y_max))
