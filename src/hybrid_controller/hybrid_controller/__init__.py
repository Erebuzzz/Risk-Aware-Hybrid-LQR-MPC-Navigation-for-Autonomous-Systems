"""
Hybrid Controller Package
=========================

Risk-aware LQR/MPC navigation controllers for autonomous systems.

The original smooth/blended Hybrid LQR-MPC controller is retained as a legacy
baseline. The current research method is a risk-triggered adaptive MPC safety
filter around an LQR nominal controller.

This package implements:
- Phase 1: LQR-based trajectory tracking
- Phase 2: MPC-based safety-critical control with obstacle avoidance
- Current: LQR nominal control with risk-triggered adaptive MPC safety filtering

Modules:
    - models: Robot kinematics and linearization
    - controllers: LQR and MPC implementations
    - trajectory: Reference trajectory generation
    - nodes: ROS2 node implementations
    - logging: Comprehensive simulation logging
    - utils: Visualization and helper utilities
"""

__version__ = "1.0.0"
__author__ = "Developer"
