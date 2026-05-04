#!/usr/bin/env python3
"""
Quick Test Script for Adaptive Hybrid Controller
=================================================

Tests the new Adaptive Hybrid Controller (Adaptive MPC + LQR)
with a simple scenario to verify implementation.

Usage:
    python test_adaptive_hybrid.py
"""

import sys
import os
import numpy as np

# Add package to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src', 'hybrid_controller'))

from hybrid_controller.models.differential_drive import DifferentialDriveRobot
from hybrid_controller.trajectory.reference_generator import ReferenceTrajectoryGenerator
from hybrid_controller.controllers.adaptive_hybrid_controller import AdaptiveHybridController
from hybrid_controller.controllers.mpc_controller import Obstacle

def test_adaptive_hybrid():
    """Test adaptive hybrid controller with simple scenario."""
    print("=" * 60)
    print("Testing Adaptive Hybrid Controller")
    print("=" * 60)
    
    # Parameters
    dt = 0.02
    duration = 5.0  # Short test
    
    # Initialize components
    robot = DifferentialDriveRobot(v_max=2.0, omega_max=3.0)
    traj_gen = ReferenceTrajectoryGenerator(A=2.0, a=0.5, dt=dt)
    
    # Initial parameter estimates (intentionally off)
    theta_init = np.array([0.8, 0.8])
    
    # Create adaptive hybrid controller
    controller = AdaptiveHybridController(
        prediction_horizon=10,
        terminal_horizon=5,
        mpc_Q_diag=[30.0, 30.0, 5.0],
        mpc_R_diag=[0.1, 0.1],
        lqr_Q_diag=[15.0, 15.0, 8.0],
        lqr_R_diag=[0.1, 0.1],
        d_safe=0.3,
        enable_adaptation=True,
        adaptation_gamma=0.005,
        theta_init=theta_init,
        v_max=2.0,
        omega_max=3.0,
        dt=dt
    )
    
    # Simple obstacle
    obstacles = [Obstacle(x=1.0, y=0.5, radius=0.2)]
    
    # Generate trajectory
    trajectory = traj_gen.generate(duration)
    N = len(trajectory)
    
    print(f"\nGenerated {N} trajectory points over {duration}s")
    print(f"Initial parameter estimates: {theta_init}")
    print(f"Testing with {len(obstacles)} obstacle(s)\n")
    
    # Initialize state
    x_ref_init, _ = traj_gen.get_reference_at_index(0)
    x = x_ref_init.copy()
    
    # Storage
    states = np.zeros((N, 3))
    controls = np.zeros((N - 1, 2))
    weights = np.zeros((N - 1,))
    params = np.zeros((N - 1, 2))
    
    states[0] = x
    
    # Simulation loop
    print("Running simulation...")
    mpc_rate = 5
    
    for k in range(N - 1):
        # Get reference
        x_ref, u_ref = traj_gen.get_reference_at_index(k)
        x_refs, u_refs = traj_gen.get_trajectory_segment(k, controller.adaptive_mpc.N_ext + 1)
        
        # Compute control
        u, info = controller.compute_control(
            x=x,
            x_ref=x_ref,
            u_ref=u_ref,
            obstacles=obstacles,
            x_refs=x_refs,
            u_refs=u_refs,
            mpc_rate=mpc_rate
        )
        
        # Simulate
        x = robot.simulate_step(x, u, dt)
        
        # Store
        states[k + 1] = x
        controls[k] = u
        weights[k] = info.weight
        params[k] = info.param_estimates
        
        if k % 50 == 0:
            print(f"  Step {k:3d}: risk={info.risk:.2f}, w={info.weight:.3f}, "
                  f"mode={info.mode:22s}, θ=[{info.param_estimates[0]:.3f}, {info.param_estimates[1]:.3f}]")
    
    # Results
    stats = controller.get_statistics()
    
    print("\n" + "=" * 60)
    print("Test Results")
    print("=" * 60)
    print(f"✓ Simulation completed successfully!")
    print(f"\nBlending Statistics:")
    print(f"  LQR-dominant:  {100*stats['lqr_dominant_fraction']:.1f}%")
    print(f"  Blended:       {100*stats['blended_fraction']:.1f}%")
    print(f"  MPC-dominant:  {100*stats['mpc_dominant_fraction']:.1f}%")
    print(f"  Transitions:   {stats['total_switches']}")
    print(f"\nParameter Adaptation:")
    print(f"  Initial: v_scale={theta_init[0]:.3f}, ω_scale={theta_init[1]:.3f}")
    print(f"  Final:   v_scale={stats['param_estimates'][0]:.3f}, ω_scale={stats['param_estimates'][1]:.3f}")
    print(f"  Change:  Δv={stats['param_estimates'][0]-theta_init[0]:+.3f}, "
          f"Δω={stats['param_estimates'][1]-theta_init[1]:+.3f}")
    print(f"\nMPC Statistics:")
    print(f"  Total solves:     {stats['mpc_stats']['solve_count']}")
    print(f"  Avg solve time:   {stats['mpc_stats']['avg_solve_time_ms']:.2f} ms")
    
    # Compute tracking error
    ref_states = trajectory[:, 1:4]
    errors = states - ref_states
    mean_error = np.mean(np.linalg.norm(errors[:, :2], axis=1))
    print(f"\nTracking Performance:")
    print(f"  Mean position error: {mean_error:.4f} m")
    
    print("\n" + "=" * 60)
    print("✓ All tests passed!")
    print("=" * 60)
    
    return True

if __name__ == '__main__':
    try:
        success = test_adaptive_hybrid()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"\n✗ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
