"""
Trajectory Factory (Phase 2)
==============================

Provides a unified API for generating diverse reference trajectories 
for evaluation and benchmarking of the hybrid LQR-MPC controller.

Supported trajectory families:
    1. Figure-8 (Lemniscate) - existing baseline
    2. Three-leaf clover - high curvature at crossings
    3. Rose curve (4-petal) - repeated origin crossings
    4. Inward spiral - monotonically decreasing radius
    5. Random waypoint - non-periodic, Catmull-Rom spline

Output format: np.ndarray of shape (N, 6):
    [t, px, py, theta, v, omega]

Reference:
    Risk-Aware Hybrid LQR-MPC Navigation, Phase 2 - Multi-Trajectory Suite
"""

import numpy as np
from typing import Optional, Dict, Any
from scipy.interpolate import CubicSpline


class TrajectoryFactory:
    """
    Factory for generating diverse reference trajectories.
    
    All trajectories return arrays of shape (N, 6):
        [t, px, py, theta, v_ref, omega_ref]
    
    Usage:
        factory = TrajectoryFactory()
        traj = factory.generate('figure8', duration=20.0, dt=0.02, A=2.0)
    """
    
    SUPPORTED_TYPES = ['figure8', 'clover3', 'rose4', 'spiral', 'random_wp']
    
    def generate(self, traj_type: str, duration: float = 20.0, 
                 dt: float = 0.02, A: float = 2.0, 
                 **kwargs) -> np.ndarray:
        """
        Generate a reference trajectory.
        
        Args:
            traj_type: One of 'figure8', 'clover3', 'rose4', 'spiral', 'random_wp'
            duration: Total trajectory duration (seconds)
            dt: Sampling time (seconds)
            A: Spatial amplitude (meters)
            **kwargs: Additional trajectory-specific parameters
            
        Returns:
            Trajectory array of shape (N, 6): [t, px, py, theta, v, omega]
        """
        generators = {
            'figure8': self._generate_figure8,
            'clover3': self._generate_clover3,
            'rose4': self._generate_rose4,
            'spiral': self._generate_spiral,
            'random_wp': self._generate_random_wp,
        }
        
        if traj_type not in generators:
            raise ValueError(f"Unknown trajectory type '{traj_type}'. "
                           f"Supported: {self.SUPPORTED_TYPES}")
        
        return generators[traj_type](duration, dt, A, **kwargs)
    
    def _finalize_trajectory(self, t: np.ndarray, px: np.ndarray, 
                              py: np.ndarray, dt: float) -> np.ndarray:
        """
        Compute heading, velocity, and angular velocity from position data.
        
        Args:
            t: Time vector [N]
            px, py: Position arrays [N]
            dt: Time step
            
        Returns:
            Trajectory array [N, 6]: [t, px, py, theta, v, omega]
        """
        N = len(t)
        
        # Compute derivatives using central differences
        # Use central differences for smoother velocity estimation to reduce noise
        dpx = np.gradient(px, dt)
        dpy = np.gradient(py, dt)
        # Apply a gentle moving average filter to smooth numerical derivatives
        if len(dpx) > 4:
            dpx = np.convolve(dpx, np.ones(3)/3.0, mode='same')
            dpy = np.convolve(dpy, np.ones(3)/3.0, mode='same')
        
        # Heading angle
        theta = np.arctan2(dpy, dpx)
        
        # Unwrap heading for continuity
        theta = np.unwrap(theta)
        
        # Linear velocity
        v = np.sqrt(dpx**2 + dpy**2)
        
        # Angular velocity (numerical derivative of heading)
        omega = np.gradient(theta, dt)
        if len(omega) > 4:
            omega = np.convolve(omega, np.ones(3)/3.0, mode='same')
        
        # Clamp extreme angular velocities
        # Widened to [-5.0, 5.0] to accommodate sharp trajectories like rose4/clover3
        omega = np.clip(omega, -5.0, 5.0)
        
        # Assemble trajectory
        traj = np.stack([t, px, py, theta, v, omega], axis=1)
        
        return traj
    
    def _generate_figure8(self, duration: float, dt: float, A: float,
                           a: float = 0.5, **kwargs) -> np.ndarray:
        """
        Generate a figure-8 (lemniscate) trajectory.
        
        Equations:
            p_x(t) = A * sin(a*t)
            p_y(t) = A * sin(a*t) * cos(a*t)
            
        Properties:
            - Smooth, single crossing
            - Balanced curvature
            - Good for baseline evaluation
        """
        t = np.arange(0, duration, dt)
        px = A * np.sin(a * t)
        py = A * np.sin(a * t) * np.cos(a * t)
        
        return self._finalize_trajectory(t, px, py, dt)
    
    def _generate_clover3(self, duration: float, dt: float, A: float,
                           **kwargs) -> np.ndarray:
        """
        Generate a three-leaf clover trajectory.
        
        Equations (polar: r = A*sin(3θ), converted to Cartesian):
            p_x(t) = A * sin(3t) * cos(t)
            p_y(t) = A * sin(3t) * sin(t)
            
        Properties:
            - Three symmetric lobes
            - High curvature at origin crossings
            - Aggressive heading changes
        """
        # Scale time so one full period maps to duration
        t = np.arange(0, duration, dt)
        phase = 2.0 * np.pi * t / duration
        
        px = A * np.sin(3 * phase) * np.cos(phase)
        py = A * np.sin(3 * phase) * np.sin(phase)
        
        return self._finalize_trajectory(t, px, py, dt)
    
    def _generate_rose4(self, duration: float, dt: float, A: float,
                         **kwargs) -> np.ndarray:
        """
        Generate a four-petal rose curve trajectory.
        
        Equations (polar: r = A*cos(2θ), converted to Cartesian):
            p_x(t) = A * cos(2t) * cos(t)
            p_y(t) = A * cos(2t) * sin(t)
            
        Properties:
            - Four petals
            - Repeated origin crossings
            - Tests zero-velocity / heading singularities
        """
        t = np.arange(0, duration, dt)
        phase = 2.0 * np.pi * t / duration
        
        px = A * np.cos(2 * phase) * np.cos(phase)
        py = A * np.cos(2 * phase) * np.sin(phase)
        
        return self._finalize_trajectory(t, px, py, dt)
    
    def _generate_spiral(self, duration: float, dt: float, A: float,
                          n_turns: float = 3.0, **kwargs) -> np.ndarray:
        """
        Generate an inward spiral trajectory.
        
        Equations:
            r(t) = A * (1 - t/T)
            p_x(t) = r(t) * cos(2π * n_turns * t / T)
            p_y(t) = r(t) * sin(2π * n_turns * t / T)
            
        Properties:
            - Monotonically decreasing radius
            - No crossings
            - Continuous angular motion
            - LQR may struggle near center (low velocity)
        """
        t = np.arange(0, duration, dt)
        T = duration
        
        # Linearly decreasing radius
        r = A * (1.0 - t / T)
        # Ensure radius doesn't go negative
        r = np.maximum(r, 0.01)
        
        # Angular parameter
        angle = 2.0 * np.pi * n_turns * t / T
        
        px = r * np.cos(angle)
        py = r * np.sin(angle)
        
        return self._finalize_trajectory(t, px, py, dt)
    
    def _generate_random_wp(self, duration: float, dt: float, A: float,
                             n_waypoints: int = 12, min_spacing: float = 0.8,
                             seed: int = None, **kwargs) -> np.ndarray:
        """
        Generate a random waypoint trajectory smoothed with cubic spline.
        
        Generation:
            1. Draw n_waypoints uniformly in [-A, A]^2
            2. Enforce minimum inter-waypoint spacing
            3. Smooth with cubic spline interpolation
            
        Properties:
            - Non-periodic
            - Unpredictable geometry
            - Tests global planning assumptions
            - Reference may intersect obstacles
        """
        rng = np.random.RandomState(seed)
        
        # Generate waypoints with minimum spacing enforcement
        waypoints = [np.array([0.0, 0.0])]  # Start at origin
        
        max_attempts = 1000
        attempts = 0
        while len(waypoints) < n_waypoints and attempts < max_attempts:
            candidate = rng.uniform(-A, A, size=2)
            
            # Check minimum spacing
            too_close = False
            for wp in waypoints:
                if np.linalg.norm(candidate - wp) < min_spacing:
                    too_close = True
                    break
            
            if not too_close:
                waypoints.append(candidate)
            attempts += 1
        
        # Close the loop (return to near-origin)
        waypoints.append(np.array([0.0, 0.0]))
        waypoints = np.array(waypoints)
        
        # Parameterize by cumulative arc length
        n_wp = len(waypoints)
        arc_lengths = np.zeros(n_wp)
        for i in range(1, n_wp):
            arc_lengths[i] = arc_lengths[i-1] + np.linalg.norm(waypoints[i] - waypoints[i-1])
        
        # Normalize to [0, duration]
        arc_lengths = arc_lengths / arc_lengths[-1] * duration
        
        # Cubic spline interpolation
        cs_x = CubicSpline(arc_lengths, waypoints[:, 0], bc_type='clamped')
        cs_y = CubicSpline(arc_lengths, waypoints[:, 1], bc_type='clamped')
        
        t = np.arange(0, duration, dt)
        px = cs_x(t)
        py = cs_y(t)
        
        return self._finalize_trajectory(t, px, py, dt)
    
    @staticmethod
    def get_trajectory_info(traj_type: str) -> Dict[str, Any]:
        """Return metadata about a trajectory type."""
        info = {
            'figure8': {
                'name': 'Figure-8 (Lemniscate)',
                'crossings': 1,
                'curvature': 'balanced',
                'difficulty': 'easy',
            },
            'clover3': {
                'name': 'Three-Leaf Clover',
                'crossings': 3,
                'curvature': 'high at origin',
                'difficulty': 'hard',
            },
            'rose4': {
                'name': 'Four-Petal Rose',
                'crossings': 4,
                'curvature': 'high at origin',
                'difficulty': 'hard',
            },
            'spiral': {
                'name': 'Inward Spiral',
                'crossings': 0,
                'curvature': 'increasing',
                'difficulty': 'medium',
            },
            'random_wp': {
                'name': 'Random Waypoint',
                'crossings': 'variable',
                'curvature': 'variable',
                'difficulty': 'variable',
            },
        }
        return info.get(traj_type, {})
