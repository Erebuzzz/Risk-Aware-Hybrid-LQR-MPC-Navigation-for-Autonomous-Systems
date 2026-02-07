"""
Hybrid Controller Package
=========================

Risk-Aware Hybrid LQR-MPC Navigation Controller for Autonomous Systems.

This package implements:
- Phase 1: LQR-based trajectory tracking
- Phase 2: MPC-based safety-critical control with obstacle avoidance

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
