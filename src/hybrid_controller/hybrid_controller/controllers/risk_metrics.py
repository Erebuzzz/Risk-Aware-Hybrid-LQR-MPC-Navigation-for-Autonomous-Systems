"""
Risk Metrics Module
===================

Implements risk quantification for the hybrid LQR-MPC controller.

Risk is measured using:
1. Distance-based risk: Geometric proximity to obstacles
2. Predictive risk: Predicted constraint violations over horizon
3. Uncertainty-aware risk: (Future) Sensing/estimation uncertainty

Reference:
    Project Proposal Section 2: Risk Quantification
"""

import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass


@dataclass
class RiskAssessment:
    """Container for risk assessment results."""
    distance_risk: float       # [0, 1] normalized distance-based risk
    predictive_risk: float     # [0, 1] predicted constraint violation risk
    combined_risk: float       # [0, 1] weighted combination
    min_obstacle_distance: float  # Minimum distance to any obstacle (m)
    nearest_obstacle_id: int   # Index of nearest obstacle
    use_mpc: bool             # Recommendation: True = use MPC, False = use LQR
    risk_level: str           # "low", "medium", "high", "critical"


class RiskMetrics:
    """
    Risk quantification for hybrid controller switching.
    
    Computes risk metrics based on:
    - Geometric proximity to obstacles
    - Predicted trajectory constraint violations
    
    Attributes:
        d_safe: Safety distance threshold (meters)
        d_trigger: Distance to start considering MPC (meters)
        alpha: Weight for distance risk in combined metric
        beta: Weight for predictive risk in combined metric
        threshold_low: Risk below this is "low"
        threshold_medium: Risk below this is "medium"
        threshold_high: Risk below this is "high"
    """
    
    def __init__(self, 
                 d_safe: float = 0.3,
                 d_trigger: float = 1.0,
                 alpha: float = 0.6,
                 beta: float = 0.4,
                 threshold_low: float = 0.2,
                 threshold_medium: float = 0.5,
                 threshold_high: float = 0.8):
        """
        Initialize risk metrics calculator.
        
        Args:
            d_safe: Safety distance from obstacles (meters)
            d_trigger: Distance at which risk starts increasing (meters)
            alpha: Weight for distance-based risk [0, 1]
            beta: Weight for predictive risk [0, 1]
            threshold_low: Risk level threshold for "low"
            threshold_medium: Risk level threshold for "medium"
            threshold_high: Risk level threshold for "high"
        """
        self.d_safe = d_safe
        self.d_trigger = d_trigger
        self.alpha = alpha
        self.beta = beta
        self.threshold_low = threshold_low
        self.threshold_medium = threshold_medium
        self.threshold_high = threshold_high
        
        # Ensure weights sum to 1
        total = self.alpha + self.beta
        self.alpha /= total
        self.beta /= total
    
    def compute_distance_risk(self, 
                               state: np.ndarray, 
                               obstacles: List[Dict]) -> Tuple[float, float, int]:
        """
        Compute distance-based risk from current position to obstacles.
        
        Risk function: r(d) = max(0, 1 - (d - d_safe) / (d_trigger - d_safe))
        
        Args:
            state: Current robot state [px, py, theta]
            obstacles: List of obstacle dicts with 'x', 'y', 'radius'
            
        Returns:
            Tuple of (risk_value, min_distance, nearest_obstacle_id)
        """
        if not obstacles:
            return 0.0, float('inf'), -1
        
        px, py = state[0], state[1]
        min_distance = float('inf')
        nearest_id = -1
        max_risk = 0.0
        
        for i, obs in enumerate(obstacles):
            # Distance from robot to obstacle edge
            dist_to_center = np.sqrt((px - obs['x'])**2 + (py - obs['y'])**2)
            dist_to_edge = dist_to_center - obs['radius']
            
            if dist_to_edge < min_distance:
                min_distance = dist_to_edge
                nearest_id = i
            
            # Compute risk for this obstacle
            if dist_to_edge <= self.d_safe:
                # Inside safety zone or colliding
                risk = 1.0
            elif dist_to_edge >= self.d_trigger:
                # Far from obstacle
                risk = 0.0
            else:
                # Linear interpolation between d_safe and d_trigger
                risk = 1.0 - (dist_to_edge - self.d_safe) / (self.d_trigger - self.d_safe)
            
            max_risk = max(max_risk, risk)
        
        return max_risk, min_distance, nearest_id
    
    def compute_predictive_risk(self,
                                 predicted_states: np.ndarray,
                                 obstacles: List[Dict]) -> float:
        """
        Compute risk based on predicted trajectory constraint violations.
        
        Checks if any predicted state over the horizon violates safety constraints.
        
        Args:
            predicted_states: Predicted states (N, 3) over horizon
            obstacles: List of obstacle dicts
            
        Returns:
            Predictive risk value [0, 1]
        """
        if not obstacles or predicted_states is None or len(predicted_states) == 0:
            return 0.0
        
        N = len(predicted_states)
        violation_count = 0
        total_violation_severity = 0.0
        
        for k, state in enumerate(predicted_states):
            px, py = state[0], state[1]
            
            for obs in obstacles:
                dist_to_center = np.sqrt((px - obs['x'])**2 + (py - obs['y'])**2)
                dist_to_edge = dist_to_center - obs['radius']
                
                if dist_to_edge < self.d_safe:
                    violation_count += 1
                    # Weight earlier violations more heavily (imminent danger)
                    time_weight = 1.0 - (k / N) * 0.5
                    severity = (self.d_safe - dist_to_edge) / self.d_safe
                    total_violation_severity += time_weight * severity
        
        # Normalize by maximum possible violations
        max_violations = N * len(obstacles)
        if max_violations > 0:
            return min(1.0, total_violation_severity / max_violations * 5)  # Scale factor
        return 0.0
    
    def assess_risk(self,
                    state: np.ndarray,
                    obstacles: List[Dict],
                    predicted_states: np.ndarray = None) -> RiskAssessment:
        """
        Compute comprehensive risk assessment.
        
        Args:
            state: Current robot state [px, py, theta]
            obstacles: List of obstacle dicts
            predicted_states: Optional predicted states for predictive risk
            
        Returns:
            RiskAssessment with all risk metrics
        """
        # Distance-based risk
        dist_risk, min_dist, nearest_id = self.compute_distance_risk(state, obstacles)
        
        # Predictive risk
        if predicted_states is not None:
            pred_risk = self.compute_predictive_risk(predicted_states, obstacles)
        else:
            pred_risk = 0.0
        
        # Combined risk (weighted average)
        combined = self.alpha * dist_risk + self.beta * pred_risk
        
        # Determine risk level
        if combined < self.threshold_low:
            risk_level = "low"
        elif combined < self.threshold_medium:
            risk_level = "medium"
        elif combined < self.threshold_high:
            risk_level = "high"
        else:
            risk_level = "critical"
        
        # Switching recommendation
        # Use MPC if risk is medium or above
        use_mpc = combined >= self.threshold_low
        
        return RiskAssessment(
            distance_risk=dist_risk,
            predictive_risk=pred_risk,
            combined_risk=combined,
            min_obstacle_distance=min_dist,
            nearest_obstacle_id=nearest_id,
            use_mpc=use_mpc,
            risk_level=risk_level
        )
    
    def get_risk_summary(self, assessment: RiskAssessment) -> str:
        """Get human-readable risk summary."""
        return (f"Risk: {assessment.risk_level.upper()} "
                f"(combined={assessment.combined_risk:.2f}, "
                f"dist={assessment.distance_risk:.2f}, "
                f"pred={assessment.predictive_risk:.2f}, "
                f"min_d={assessment.min_obstacle_distance:.2f}m)")
