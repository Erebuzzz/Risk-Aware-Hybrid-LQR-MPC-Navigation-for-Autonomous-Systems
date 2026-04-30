"""
Checkpoint Navigation Module (Phase 3)
========================================

Implements the Checkpoint Navigation (CN) paradigm as an alternative
to Full Reference Path (FRP) tracking.

Components:
    - CheckpointExtractor: Extract waypoints from dense reference trajectories
    - WaypointManager: Track progress and provide active waypoint targets
    - CNMetrics: Compute CN-specific evaluation metrics (XTE, completion rate)
    - cn_mpc_cost: Modified MPC cost for waypoint-to-waypoint navigation

CN Paradigm:
    Instead of tracking the full dense reference path, the controller
    navigates between sparse checkpoints. This gives the MPC more
    freedom to deviate from the reference for obstacle avoidance,
    reducing collision rates in dense environments.

Reference:
    Risk-Aware Hybrid LQR-MPC Navigation, Phase 3 - Dual Navigation Paradigm
"""

import numpy as np
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass, field


# ── P3-B: CheckpointExtractor ───────────────────────────────────────

class CheckpointExtractor:
    """
    Extract sparse checkpoints from a dense reference trajectory (P3-B).
    
    Strategies:
        - 'uniform': Evenly spaced by time index
        - 'curvature': Denser at high-curvature regions
        - 'arc_length': Evenly spaced by arc length
    
    Usage:
        extractor = CheckpointExtractor(n_checkpoints=12, strategy='curvature')
        waypoints = extractor.extract(trajectory)
    """
    
    def __init__(self, n_checkpoints: int = 12, strategy: str = 'curvature',
                 min_spacing: float = 0.3):
        """
        Args:
            n_checkpoints: Target number of checkpoints
            strategy: Extraction strategy ('uniform', 'curvature', 'arc_length')
            min_spacing: Minimum distance between consecutive checkpoints (m)
        """
        self.n_checkpoints = n_checkpoints
        self.strategy = strategy
        self.min_spacing = min_spacing
    
    def extract(self, trajectory: np.ndarray) -> np.ndarray:
        """
        Extract checkpoints from a dense trajectory.
        
        Args:
            trajectory: Dense trajectory [N, 6] = [t, px, py, theta, v, omega]
                        or [N, 3] = [px, py, theta]
            
        Returns:
            Checkpoints [M, 3] = [px, py, theta] where M <= n_checkpoints
        """
        # Handle both formats
        if trajectory.shape[1] >= 6:
            positions = trajectory[:, 1:4]  # [px, py, theta]
        elif trajectory.shape[1] >= 3:
            positions = trajectory[:, :3]
        else:
            raise ValueError(f"Trajectory must have >= 3 columns, got {trajectory.shape[1]}")
        
        if self.strategy == 'uniform':
            return self._extract_uniform(positions)
        elif self.strategy == 'curvature':
            return self._extract_curvature(positions)
        elif self.strategy == 'arc_length':
            return self._extract_arc_length(positions)
        else:
            raise ValueError(f"Unknown strategy: {self.strategy}")
    
    def _extract_uniform(self, positions: np.ndarray) -> np.ndarray:
        """Uniformly spaced checkpoints by index."""
        N = len(positions)
        indices = np.linspace(0, N - 1, self.n_checkpoints, dtype=int)
        return positions[indices].copy()
    
    def _extract_arc_length(self, positions: np.ndarray) -> np.ndarray:
        """Checkpoints evenly spaced by arc length."""
        N = len(positions)
        
        # Compute cumulative arc length
        diffs = np.diff(positions[:, :2], axis=0)
        seg_lengths = np.sqrt(np.sum(diffs**2, axis=1))
        cum_arc = np.concatenate([[0.0], np.cumsum(seg_lengths)])
        total_arc = cum_arc[-1]
        
        if total_arc < 1e-6:
            return self._extract_uniform(positions)
        
        # Target arc length spacing
        target_arcs = np.linspace(0, total_arc, self.n_checkpoints)
        
        # Find closest indices
        indices = []
        for s in target_arcs:
            idx = np.argmin(np.abs(cum_arc - s))
            indices.append(idx)
        
        # Remove duplicates while preserving order
        seen = set()
        unique_indices = []
        for idx in indices:
            if idx not in seen:
                seen.add(idx)
                unique_indices.append(idx)
        
        return positions[unique_indices].copy()
    
    def _extract_curvature(self, positions: np.ndarray) -> np.ndarray:
        """
        Checkpoints denser at high-curvature regions.
        
        Uses curvature-weighted arc-length parameterization to place
        more checkpoints where heading changes rapidly.
        """
        N = len(positions)
        if N < 5:
            return self._extract_uniform(positions)
        
        # Compute discrete curvature (heading rate of change)
        theta = positions[:, 2]
        dtheta = np.abs(np.diff(np.unwrap(theta)))
        
        # Compute arc length segments
        diffs = np.diff(positions[:, :2], axis=0)
        seg_lengths = np.sqrt(np.sum(diffs**2, axis=1))
        
        # Curvature-weighted cumulative parameter
        # Higher curvature = more weight = more checkpoints placed there
        curvature = np.zeros(N - 1)
        for i in range(N - 1):
            if seg_lengths[i] > 1e-6:
                curvature[i] = dtheta[i] / seg_lengths[i]
        
        # Weight: base arc length + curvature bonus
        weights = seg_lengths + 0.5 * curvature * seg_lengths
        cum_weight = np.concatenate([[0.0], np.cumsum(weights)])
        total_weight = cum_weight[-1]
        
        if total_weight < 1e-6:
            return self._extract_uniform(positions)
        
        # Target weight spacing
        target_weights = np.linspace(0, total_weight, self.n_checkpoints)
        
        indices = []
        for w in target_weights:
            idx = np.argmin(np.abs(cum_weight - w))
            if idx not in indices:
                indices.append(idx)
        
        # Ensure start and end are included
        if 0 not in indices:
            indices.insert(0, 0)
        if N - 1 not in indices:
            indices.append(N - 1)
        
        return positions[sorted(indices)].copy()


# ── P3-A: WaypointManager ──────────────────────────────────────────

@dataclass
class WaypointStatus:
    """Status of waypoint navigation progress."""
    current_index: int
    total_waypoints: int
    distance_to_current: float
    completed: bool
    fraction_complete: float
    active_waypoint: np.ndarray
    next_waypoint: Optional[np.ndarray]


class WaypointManager:
    """
    Manage progress through a sequence of checkpoints (P3-A).
    
    Tracks which checkpoint the robot is navigating toward,
    advances to the next when within the arrival radius,
    and provides the MPC with current/next waypoint targets.
    
    Attributes:
        waypoints: Array of checkpoints [M, 3] = [px, py, theta]
        arrival_radius: Distance threshold to consider a waypoint reached (m)
        lookahead: Number of future waypoints to expose to MPC
    """
    
    def __init__(self, waypoints: np.ndarray, arrival_radius: float = 0.3,
                 lookahead: int = 3):
        """
        Args:
            waypoints: Checkpoint array [M, 3]
            arrival_radius: Waypoint arrival threshold (m)
            lookahead: Number of waypoints to provide for MPC horizon
        """
        self.waypoints = waypoints.copy()
        self.arrival_radius = arrival_radius
        self.lookahead = lookahead
        self._current_idx = 0
        self._reached_flags = np.zeros(len(waypoints), dtype=bool)
    
    def update(self, state: np.ndarray) -> WaypointStatus:
        """
        Update waypoint progress based on current robot state.
        
        Advances to next waypoint if within arrival_radius of current target.
        
        Args:
            state: Current robot state [px, py, theta]
            
        Returns:
            WaypointStatus with navigation progress
        """
        if self._current_idx >= len(self.waypoints):
            # All waypoints completed
            return WaypointStatus(
                current_index=len(self.waypoints) - 1,
                total_waypoints=len(self.waypoints),
                distance_to_current=0.0,
                completed=True,
                fraction_complete=1.0,
                active_waypoint=self.waypoints[-1],
                next_waypoint=None
            )
        
        # Distance to current waypoint
        wp = self.waypoints[self._current_idx]
        dist = np.sqrt((state[0] - wp[0])**2 + (state[1] - wp[1])**2)
        
        # Check arrival
        while dist < self.arrival_radius and self._current_idx < len(self.waypoints) - 1:
            self._reached_flags[self._current_idx] = True
            self._current_idx += 1
            wp = self.waypoints[self._current_idx]
            dist = np.sqrt((state[0] - wp[0])**2 + (state[1] - wp[1])**2)
        
        # Check if final waypoint reached
        if self._current_idx == len(self.waypoints) - 1 and dist < self.arrival_radius:
            self._reached_flags[self._current_idx] = True
        
        completed = np.all(self._reached_flags)
        
        # Next waypoint
        next_wp = None
        if self._current_idx + 1 < len(self.waypoints):
            next_wp = self.waypoints[self._current_idx + 1]
        
        return WaypointStatus(
            current_index=self._current_idx,
            total_waypoints=len(self.waypoints),
            distance_to_current=dist,
            completed=completed,
            fraction_complete=float(np.sum(self._reached_flags)) / len(self.waypoints),
            active_waypoint=wp.copy(),
            next_waypoint=next_wp
        )
    
    def get_lookahead_waypoints(self) -> np.ndarray:
        """
        Get the next `lookahead` waypoints for MPC reference.
        
        Returns:
            Waypoint array [L, 3] where L = min(lookahead, remaining)
        """
        end_idx = min(self._current_idx + self.lookahead, len(self.waypoints))
        return self.waypoints[self._current_idx:end_idx].copy()
    
    def get_reference_for_mpc(self, horizon: int, dt: float,
                               current_state: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate x_refs and u_refs for MPC in CN mode.
        
        Interpolates between current position and upcoming waypoints
        to create a smooth reference for the MPC horizon.
        
        Args:
            horizon: MPC prediction horizon N
            dt: Time step
            current_state: Current robot state [px, py, theta]
            
        Returns:
            Tuple of (x_refs [N+1, 3], u_refs [N, 2])
        """
        lookahead_wps = self.get_lookahead_waypoints()
        
        if len(lookahead_wps) == 0:
            # No waypoints left; hold position
            x_refs = np.tile(current_state, (horizon + 1, 1))
            u_refs = np.zeros((horizon, 2))
            return x_refs, u_refs
        
        # Build reference by linear interpolation toward waypoints
        x_refs = np.zeros((horizon + 1, 3))
        x_refs[0] = current_state
        
        target = lookahead_wps[0]
        wp_idx = 0
        
        for k in range(1, horizon + 1):
            # Direction to current target
            dx = target[0] - x_refs[k-1, 0]
            dy = target[1] - x_refs[k-1, 1]
            dist = np.sqrt(dx**2 + dy**2)
            
            if dist < self.arrival_radius and wp_idx + 1 < len(lookahead_wps):
                wp_idx += 1
                target = lookahead_wps[wp_idx]
                dx = target[0] - x_refs[k-1, 0]
                dy = target[1] - x_refs[k-1, 1]
                dist = np.sqrt(dx**2 + dy**2)
            
            if dist > 0.01:
                # Move toward target at nominal speed
                v_nom = min(0.5, dist / (dt * (horizon - k + 1)))
                theta_target = np.arctan2(dy, dx)
                x_refs[k, 0] = x_refs[k-1, 0] + v_nom * np.cos(theta_target) * dt
                x_refs[k, 1] = x_refs[k-1, 1] + v_nom * np.sin(theta_target) * dt
                x_refs[k, 2] = theta_target
            else:
                x_refs[k] = x_refs[k-1]
        
        # Compute reference controls from reference states
        u_refs = np.zeros((horizon, 2))
        for k in range(horizon):
            dx = x_refs[k+1, 0] - x_refs[k, 0]
            dy = x_refs[k+1, 1] - x_refs[k, 1]
            u_refs[k, 0] = np.sqrt(dx**2 + dy**2) / dt  # v_ref
            dtheta = x_refs[k+1, 2] - x_refs[k, 2]
            # Normalize angle difference
            while dtheta > np.pi: dtheta -= 2 * np.pi
            while dtheta < -np.pi: dtheta += 2 * np.pi
            u_refs[k, 1] = dtheta / dt  # omega_ref
        
        return x_refs, u_refs
    
    def reset(self):
        """Reset waypoint progress."""
        self._current_idx = 0
        self._reached_flags[:] = False
    
    @property
    def current_index(self) -> int:
        return self._current_idx
    
    @property
    def n_reached(self) -> int:
        return int(np.sum(self._reached_flags))


# ── P3-D: CN-Specific Metrics ──────────────────────────────────────

class CNMetrics:
    """
    Checkpoint Navigation metrics (P3-D).
    
    Computes CN-specific evaluation metrics that differ from FRP mode:
        - Mean cross-track error (XTE) to the nearest path segment
        - Checkpoint completion rate
        - Completion time
        - Per-segment timing
    """
    
    @staticmethod
    def compute_cross_track_error(states: np.ndarray, 
                                   waypoints: np.ndarray) -> np.ndarray:
        """
        Compute cross-track error at each timestep.
        
        XTE is the perpendicular distance from the robot position
        to the nearest line segment between consecutive waypoints.
        
        Args:
            states: Robot states [N, 3]
            waypoints: Checkpoint array [M, 3]
            
        Returns:
            XTE array [N] (meters)
        """
        N = len(states)
        M = len(waypoints)
        xte = np.zeros(N)
        
        for k in range(N):
            px, py = states[k, 0], states[k, 1]
            min_dist = float('inf')
            
            for i in range(M - 1):
                # Segment from waypoint i to i+1
                ax, ay = waypoints[i, 0], waypoints[i, 1]
                bx, by = waypoints[i+1, 0], waypoints[i+1, 1]
                
                # Project point onto segment
                abx, aby = bx - ax, by - ay
                apx, apy = px - ax, py - ay
                
                ab_sq = abx**2 + aby**2
                if ab_sq < 1e-12:
                    # Degenerate segment
                    dist = np.sqrt(apx**2 + apy**2)
                else:
                    t = max(0, min(1, (apx * abx + apy * aby) / ab_sq))
                    proj_x = ax + t * abx
                    proj_y = ay + t * aby
                    dist = np.sqrt((px - proj_x)**2 + (py - proj_y)**2)
                
                min_dist = min(min_dist, dist)
            
            xte[k] = min_dist
        
        return xte
    
    @staticmethod
    def compute_completion_metrics(wp_manager: WaypointManager,
                                    total_time: float) -> Dict[str, float]:
        """
        Compute checkpoint completion metrics.
        
        Args:
            wp_manager: WaypointManager after simulation
            total_time: Total simulation time (seconds)
            
        Returns:
            Dict with completion_rate, n_reached, n_total, completion_time
        """
        n_total = len(wp_manager.waypoints)
        n_reached = wp_manager.n_reached
        
        return {
            'completion_rate': float(n_reached) / max(1, n_total),
            'n_reached': n_reached,
            'n_total': n_total,
            'completion_time': total_time,
        }
    
    @staticmethod
    def compute_summary(states: np.ndarray, waypoints: np.ndarray,
                         wp_manager: WaypointManager,
                         total_time: float) -> Dict[str, float]:
        """
        Compute full CN metric summary.
        
        Args:
            states: Robot states [N, 3]
            waypoints: Checkpoints [M, 3]
            wp_manager: WaypointManager after simulation
            total_time: Total simulation time
            
        Returns:
            Dict with all CN metrics
        """
        xte = CNMetrics.compute_cross_track_error(states, waypoints)
        completion = CNMetrics.compute_completion_metrics(wp_manager, total_time)
        
        return {
            'mean_xte': float(np.mean(xte)),
            'max_xte': float(np.max(xte)),
            'std_xte': float(np.std(xte)),
            'p95_xte': float(np.percentile(xte, 95)),
            **completion,
        }
