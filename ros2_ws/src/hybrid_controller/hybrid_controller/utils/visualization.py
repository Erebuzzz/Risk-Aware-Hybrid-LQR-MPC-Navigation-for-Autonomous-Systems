"""
Visualization Utilities
=======================

Provides plotting and visualization functions for simulation results.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.collections import PatchCollection
from typing import List, Dict, Optional, Tuple
import os


class Visualizer:
    """
    Visualization utilities for trajectory tracking and obstacle avoidance.
    
    Provides methods for:
    - Trajectory comparison plots
    - Control effort visualization
    - Tracking error plots
    - Obstacle avoidance visualization
    - Animated simulation playback
    """
    
    def __init__(self, output_dir: Optional[str] = None):
        """
        Initialize visualizer.
        
        Args:
            output_dir: Directory to save output figures
        """
        if output_dir is None:
            output_dir = os.path.join("Output", "Plots")
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        # Plot style settings
        plt.style.use('default')
        self.colors = {
            'reference': '#2E86AB',      # Blue
            'actual': '#E94F37',         # Red
            'lqr': '#4CAF50',           # Green
            'mpc': '#FF9800',           # Orange
            'obstacle': '#9E9E9E',      # Gray
            'safe_zone': '#FFCDD2',     # Light red
        }
    
    def plot_trajectory(self, states: np.ndarray, reference: np.ndarray,
                        title: str = "Trajectory Tracking",
                        save_path: str = None) -> plt.Figure:
        """
        Plot trajectory comparison between actual and reference.
        
        Args:
            states: Actual state trajectory (N, 3) or (N, 2) for [x, y]
            reference: Reference trajectory (N, 3) or (N, 2) for [x, y]
            title: Plot title
            save_path: Optional path to save figure
            
        Returns:
            Matplotlib figure object
        """
        fig, ax = plt.subplots(1, 1, figsize=(10, 8))
        
        # Extract x, y positions
        actual_x = states[:, 0]
        actual_y = states[:, 1]
        ref_x = reference[:, 0]
        ref_y = reference[:, 1]
        
        # Plot reference trajectory
        ax.plot(ref_x, ref_y, '--', color=self.colors['reference'], 
                linewidth=2, label='Reference', alpha=0.8)
        
        # Plot actual trajectory
        ax.plot(actual_x, actual_y, '-', color=self.colors['actual'],
                linewidth=2, label='Actual')
        
        # Mark start and end points
        ax.plot(actual_x[0], actual_y[0], 'go', markersize=10, label='Start')
        ax.plot(actual_x[-1], actual_y[-1], 'rs', markersize=10, label='End')
        
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_title(title, fontsize=14)
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        plt.tight_layout()
        
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches='tight')
        
        return fig
    
    def plot_tracking_error(self, errors: np.ndarray, dt: float,
                            title: str = "Tracking Error",
                            save_path: str = None) -> plt.Figure:
        """
        Plot tracking error over time.
        
        Args:
            errors: Error trajectory (N, 3) for [e_x, e_y, e_theta]
            dt: Time step
            title: Plot title
            save_path: Optional path to save figure
            
        Returns:
            Matplotlib figure object
        """
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        
        N = len(errors)
        t = np.arange(N) * dt
        
        # Position errors
        axes[0, 0].plot(t, errors[:, 0], 'b-', linewidth=1.5, label='e_x')
        axes[0, 0].plot(t, errors[:, 1], 'r-', linewidth=1.5, label='e_y')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Position Error (m)')
        axes[0, 0].set_title('Position Tracking Error')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        # Heading error
        axes[0, 1].plot(t, np.rad2deg(errors[:, 2]), 'g-', linewidth=1.5)
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Heading Error (deg)')
        axes[0, 1].set_title('Heading Tracking Error')
        axes[0, 1].grid(True, alpha=0.3)
        
        # Error norm
        error_norm = np.linalg.norm(errors[:, :2], axis=1)
        axes[1, 0].plot(t, error_norm, 'm-', linewidth=1.5)
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Position Error Norm (m)')
        axes[1, 0].set_title('Position Error Magnitude')
        axes[1, 0].grid(True, alpha=0.3)
        
        # Combined metric
        total_error = np.sqrt(errors[:, 0]**2 + errors[:, 1]**2 + 0.1*errors[:, 2]**2)
        axes[1, 1].plot(t, total_error, 'k-', linewidth=1.5)
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Weighted Error')
        axes[1, 1].set_title('Combined Tracking Error')
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.suptitle(title, fontsize=14, y=1.02)
        plt.tight_layout()
        
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches='tight')
        
        return fig
    
    def plot_control_inputs(self, controls: np.ndarray, dt: float,
                            v_max: float = 1.0, omega_max: float = 1.5,
                            title: str = "Control Inputs",
                            save_path: str = None) -> plt.Figure:
        """
        Plot control inputs over time.
        
        Args:
            controls: Control trajectory (N, 2) for [v, omega]
            dt: Time step
            v_max: Maximum linear velocity (for reference line)
            omega_max: Maximum angular velocity (for reference line)
            title: Plot title
            save_path: Optional path to save figure
            
        Returns:
            Matplotlib figure object
        """
        fig, axes = plt.subplots(2, 1, figsize=(10, 6))
        
        N = len(controls)
        t = np.arange(N) * dt
        
        # Linear velocity
        axes[0].plot(t, controls[:, 0], 'b-', linewidth=1.5, label='v')
        axes[0].axhline(y=v_max, color='r', linestyle='--', alpha=0.5, label=f'v_max = {v_max}')
        axes[0].axhline(y=-v_max, color='r', linestyle='--', alpha=0.5)
        axes[0].set_xlabel('Time (s)')
        axes[0].set_ylabel('Linear Velocity (m/s)')
        axes[0].set_title('Linear Velocity')
        axes[0].legend(loc='best')
        axes[0].grid(True, alpha=0.3)
        
        # Angular velocity
        axes[1].plot(t, controls[:, 1], 'g-', linewidth=1.5, label='ω')
        axes[1].axhline(y=omega_max, color='r', linestyle='--', alpha=0.5, label=f'ω_max = {omega_max}')
        axes[1].axhline(y=-omega_max, color='r', linestyle='--', alpha=0.5)
        axes[1].set_xlabel('Time (s)')
        axes[1].set_ylabel('Angular Velocity (rad/s)')
        axes[1].set_title('Angular Velocity')
        axes[1].legend(loc='best')
        axes[1].grid(True, alpha=0.3)
        
        plt.suptitle(title, fontsize=14)
        plt.tight_layout()
        
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches='tight')
        
        return fig
    
    def plot_with_obstacles(self, states: np.ndarray, reference: np.ndarray,
                            obstacles: List[Dict], d_safe: float = 0.3,
                            title: str = "Trajectory with Obstacles",
                            save_path: str = None) -> plt.Figure:
        """
        Plot trajectory with obstacles and safety zones.
        
        Args:
            states: Actual state trajectory (N, 3)
            reference: Reference trajectory (N, 3)
            obstacles: List of obstacle dicts with 'x', 'y', 'radius'
            d_safe: Safety distance
            title: Plot title
            save_path: Optional path to save figure
            
        Returns:
            Matplotlib figure object
        """
        fig, ax = plt.subplots(1, 1, figsize=(10, 8))
        
        # Plot obstacles with safety zones
        for obs in obstacles:
            # Safety zone (light color)
            safe_circle = Circle((obs['x'], obs['y']), 
                                  obs['radius'] + d_safe,
                                  color=self.colors['safe_zone'], alpha=0.3)
            ax.add_patch(safe_circle)
            
            # Obstacle (solid)
            obs_circle = Circle((obs['x'], obs['y']), 
                                 obs['radius'],
                                 color=self.colors['obstacle'], alpha=0.8)
            ax.add_patch(obs_circle)
        
        # Plot trajectories
        ax.plot(reference[:, 0], reference[:, 1], '--', 
                color=self.colors['reference'], linewidth=2, 
                label='Reference', alpha=0.8)
        
        ax.plot(states[:, 0], states[:, 1], '-', 
                color=self.colors['actual'], linewidth=2, 
                label='Actual')
        
        # Mark start and end
        ax.plot(states[0, 0], states[0, 1], 'go', markersize=10, label='Start')
        ax.plot(states[-1, 0], states[-1, 1], 'rs', markersize=10, label='End')
        
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_title(title, fontsize=14)
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        plt.tight_layout()
        
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches='tight')
        
        return fig
    
    def plot_comparison(self, lqr_states: np.ndarray, mpc_states: np.ndarray,
                        reference: np.ndarray, obstacles: List[Dict] = None,
                        d_safe: float = 0.3,
                        title: str = "LQR vs MPC Comparison",
                        save_path: str = None) -> plt.Figure:
        """
        Plot comparison between LQR and MPC trajectories.
        
        Args:
            lqr_states: LQR state trajectory (N, 3)
            mpc_states: MPC state trajectory (N, 3)
            reference: Reference trajectory (N, 3)
            obstacles: Optional list of obstacles
            d_safe: Safety distance
            title: Plot title
            save_path: Optional path to save figure
            
        Returns:
            Matplotlib figure object
        """
        fig, ax = plt.subplots(1, 1, figsize=(12, 8))
        
        # Plot obstacles if provided
        if obstacles:
            for obs in obstacles:
                safe_circle = Circle((obs['x'], obs['y']), 
                                      obs['radius'] + d_safe,
                                      color=self.colors['safe_zone'], alpha=0.3)
                ax.add_patch(safe_circle)
                
                obs_circle = Circle((obs['x'], obs['y']), 
                                     obs['radius'],
                                     color=self.colors['obstacle'], alpha=0.8)
                ax.add_patch(obs_circle)
        
        # Plot reference
        ax.plot(reference[:, 0], reference[:, 1], '--', 
                color=self.colors['reference'], linewidth=2, 
                label='Reference', alpha=0.7)
        
        # Plot LQR trajectory
        ax.plot(lqr_states[:, 0], lqr_states[:, 1], '-', 
                color=self.colors['lqr'], linewidth=2, 
                label='LQR')
        
        # Plot MPC trajectory
        ax.plot(mpc_states[:, 0], mpc_states[:, 1], '-', 
                color=self.colors['mpc'], linewidth=2, 
                label='MPC')
        
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_title(title, fontsize=14)
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        plt.tight_layout()
        
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches='tight')
        
        return fig
    
    def save_all_figures(self, prefix: str = "") -> None:
        """Close and save all current figures."""
        plt.close('all')
    
    # ── P4-C: Risk Signal Visualization ─────────────────────────────
    
    def plot_blend_trajectory(self, states: np.ndarray, blend_weights: np.ndarray,
                               reference: np.ndarray, obstacles: List[Dict] = None,
                               d_safe: float = 0.35,
                               title: str = "Trajectory Colored by Blend Weight",
                               save_path: str = None) -> plt.Figure:
        """
        Plot robot trajectory colored by blend weight (P4-C).
        
        Blue = LQR dominant (w→0), Red = MPC dominant (w→1).
        Gradient shows blended regions.
        
        This can become a core paper figure showing supervisor behavior.
        
        Args:
            states: Robot states [N, 3]
            blend_weights: Blend weights [N-1] or [N], values in [0, 1]
            reference: Reference trajectory [N, 3]
            obstacles: Optional obstacle dicts
            d_safe: Safety distance for visualization
            title: Plot title
            save_path: Optional save path
        """
        fig, ax = plt.subplots(1, 1, figsize=(12, 9))
        
        # Draw obstacles
        if obstacles:
            for obs in obstacles:
                safe_circle = Circle((obs['x'], obs['y']),
                                      obs['radius'] + d_safe,
                                      color='#FFCDD2', alpha=0.2,
                                      linestyle='--', linewidth=0.5)
                ax.add_patch(safe_circle)
                obs_circle = Circle((obs['x'], obs['y']),
                                     obs['radius'],
                                     color='#616161', alpha=0.7)
                ax.add_patch(obs_circle)
        
        # Reference trajectory
        ax.plot(reference[:, 0], reference[:, 1], '--',
                color='#9E9E9E', linewidth=1.5, alpha=0.6, label='Reference')
        
        # Color-mapped trajectory segments
        N = min(len(states) - 1, len(blend_weights))
        from matplotlib.collections import LineCollection
        from matplotlib.colors import LinearSegmentedColormap
        
        # Custom colormap: blue (LQR) → purple (blend) → red (MPC)
        cmap = LinearSegmentedColormap.from_list(
            'lqr_mpc', ['#1565C0', '#7B1FA2', '#C62828'])
        
        points = states[:N+1, :2].reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        weights = blend_weights[:N]
        lc = LineCollection(segments, cmap=cmap, norm=plt.Normalize(0, 1))
        lc.set_array(weights)
        lc.set_linewidth(3)
        ax.add_collection(lc)
        
        # Colorbar
        cbar = plt.colorbar(lc, ax=ax, shrink=0.8, pad=0.02)
        cbar.set_label('Blend Weight w(t)\n(0=LQR, 1=MPC)', fontsize=11)
        
        # Start/end markers
        ax.plot(states[0, 0], states[0, 1], 'go', markersize=12, 
                label='Start', zorder=5)
        ax.plot(states[-1, 0], states[-1, 1], 'r^', markersize=12,
                label='End', zorder=5)
        
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.legend(loc='upper right', fontsize=10)
        ax.grid(True, alpha=0.2)
        ax.set_aspect('equal')
        plt.tight_layout()
        
        if save_path:
            fig.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig
    
    def plot_risk_heatmap(self, obstacles: List[Dict],
                           d_safe: float = 0.35, d_trigger: float = 3.0,
                           arena_range: Tuple = (-3.0, 3.0, -2.5, 2.5),
                           resolution: float = 0.05,
                           title: str = "Risk Heatmap",
                           save_path: str = None) -> plt.Figure:
        """
        Plot risk heatmap over the arena (P4-C).
        
        Shows the spatial risk field with obstacle influence circles.
        
        Args:
            obstacles: Obstacle dicts
            d_safe: Safety distance
            d_trigger: Risk influence distance
            arena_range: (x_min, x_max, y_min, y_max)
            resolution: Grid resolution (m)
            title: Plot title
            save_path: Optional save path
        """
        fig, ax = plt.subplots(1, 1, figsize=(12, 9))
        
        x_min, x_max, y_min, y_max = arena_range
        x_grid = np.arange(x_min, x_max, resolution)
        y_grid = np.arange(y_min, y_max, resolution)
        X, Y = np.meshgrid(x_grid, y_grid)
        
        # Compute risk at each grid point
        Z = np.zeros_like(X)
        for obs in obstacles:
            dist_to_center = np.sqrt((X - obs['x'])**2 + (Y - obs['y'])**2)
            dist_to_edge = dist_to_center - obs['radius']
            
            # Risk function: linear interpolation
            risk = np.where(dist_to_edge <= d_safe, 1.0,
                   np.where(dist_to_edge >= d_trigger, 0.0,
                   1.0 - (dist_to_edge - d_safe) / (d_trigger - d_safe)))
            Z = np.maximum(Z, risk)
        
        # Heatmap
        im = ax.pcolormesh(X, Y, Z, cmap='YlOrRd', shading='auto',
                           vmin=0, vmax=1)
        cbar = plt.colorbar(im, ax=ax, shrink=0.8)
        cbar.set_label('Risk Level', fontsize=11)
        
        # Obstacle circles and influence zones
        for obs in obstacles:
            # d_trigger circle (influence boundary)
            trigger_circle = Circle((obs['x'], obs['y']),
                                     obs['radius'] + d_trigger,
                                     fill=False, edgecolor='#FF6F00',
                                     linestyle=':', linewidth=1.5,
                                     label='d_trigger' if obs == obstacles[0] else '')
            ax.add_patch(trigger_circle)
            
            # d_safe circle
            safe_circle = Circle((obs['x'], obs['y']),
                                  obs['radius'] + d_safe,
                                  fill=False, edgecolor='#D32F2F',
                                  linestyle='--', linewidth=1.5,
                                  label='d_safe' if obs == obstacles[0] else '')
            ax.add_patch(safe_circle)
            
            # Obstacle body
            obs_circle = Circle((obs['x'], obs['y']),
                                 obs['radius'],
                                 color='#212121', alpha=0.8)
            ax.add_patch(obs_circle)
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.legend(loc='upper right', fontsize=10)
        ax.set_aspect('equal')
        plt.tight_layout()
        
        if save_path:
            fig.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig
    
    def plot_risk_timeseries(self, blend_weights: np.ndarray,
                              risk_values: np.ndarray,
                              dt: float = 0.02,
                              title: str = "Risk and Blend Weight Over Time",
                              save_path: str = None) -> plt.Figure:
        """
        Plot blend weight and risk signal over time (P4-C).
        
        Dual-axis plot showing how the supervisor responds to risk.
        
        Args:
            blend_weights: Blend weight array [N]
            risk_values: Combined risk array [N]
            dt: Time step
            title: Plot title
            save_path: Optional save path
        """
        fig, ax1 = plt.subplots(1, 1, figsize=(12, 5))
        
        N = min(len(blend_weights), len(risk_values))
        t = np.arange(N) * dt
        
        # Risk signal
        color_risk = '#E65100'
        ax1.fill_between(t, 0, risk_values[:N], alpha=0.15, color=color_risk)
        ax1.plot(t, risk_values[:N], color=color_risk, linewidth=1.5,
                 label='Combined Risk', alpha=0.8)
        ax1.set_xlabel('Time (s)', fontsize=12)
        ax1.set_ylabel('Risk Level', fontsize=12, color=color_risk)
        ax1.tick_params(axis='y', labelcolor=color_risk)
        ax1.set_ylim(-0.05, 1.05)
        
        # Blend weight on secondary axis
        ax2 = ax1.twinx()
        color_blend = '#1565C0'
        ax2.plot(t, blend_weights[:N], color=color_blend, linewidth=2,
                 label='Blend Weight w(t)')
        ax2.set_ylabel('Blend Weight w(t)', fontsize=12, color=color_blend)
        ax2.tick_params(axis='y', labelcolor=color_blend)
        ax2.set_ylim(-0.05, 1.05)
        
        # Combined legend
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2,
                   loc='upper right', fontsize=10)
        
        ax1.set_title(title, fontsize=14, fontweight='bold')
        ax1.grid(True, alpha=0.2)
        plt.tight_layout()
        
        if save_path:
            fig.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig
