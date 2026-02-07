#!/usr/bin/env python3
"""
Standalone Simulation
=====================

Run the LQR and MPC simulations without ROS2 dependencies.
Useful for testing, validation, and generating plots.

Usage:
    python run_simulation.py --mode lqr
    python run_simulation.py --mode mpc
    python run_simulation.py --mode compare
"""

import sys
import os
import argparse
import numpy as np

# Add the package to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src', 'hybrid_controller'))

from hybrid_controller.models.differential_drive import DifferentialDriveRobot
from hybrid_controller.models.linearization import Linearizer
from hybrid_controller.trajectory.reference_generator import ReferenceTrajectoryGenerator
from hybrid_controller.controllers.lqr_controller import LQRController
from hybrid_controller.controllers.mpc_controller import MPCController, Obstacle
from hybrid_controller.logging.simulation_logger import SimulationLogger
from hybrid_controller.utils.visualization import Visualizer


def run_lqr_simulation(duration: float = 20.0, dt: float = 0.02,
                       visualize: bool = True) -> dict:
    """
    Run LQR trajectory tracking simulation.
    
    Args:
        duration: Simulation duration (seconds)
        dt: Time step (seconds)
        visualize: Generate plots
        
    Returns:
        Dictionary with simulation results
    """
    print("=" * 60)
    print("LQR Trajectory Tracking Simulation")
    print("=" * 60)
    
    # Initialize components
    robot = DifferentialDriveRobot(v_max=2.0, omega_max=3.0)  # Increased limits to prevent saturation
    traj_gen = ReferenceTrajectoryGenerator(A=2.0, a=0.5, dt=dt)
    lqr = LQRController(Q_diag=[15.0, 15.0, 8.0], R_diag=[0.1, 0.1], dt=dt, v_max=2.0, omega_max=3.0)
    logger = SimulationLogger(log_dir='logs', log_level='INFO', node_name='lqr_sim')
    
    # Generate reference trajectory
    trajectory = traj_gen.generate(duration)
    N = len(trajectory)
    
    print(f"Generated {N} trajectory points over {duration}s")
    
    # Initialize state at the reference trajectory's starting point
    x_ref_init, _ = traj_gen.get_reference_at_index(0)
    x = x_ref_init.copy()  # Start at reference position and heading
    
    # Storage
    states = np.zeros((N, 3))
    controls = np.zeros((N - 1, 2))
    errors = np.zeros((N - 1, 3))
    
    states[0] = x
    
    # Simulation loop
    for k in range(N - 1):
        # Get reference
        x_ref, u_ref = traj_gen.get_reference_at_index(k)
        
        # Compute LQR control
        u, error = lqr.compute_control_at_operating_point(x, x_ref, u_ref)
        
        # Log
        logger.log_state(k, x, x_ref, error)
        logger.log_control(k, u, 'LQR')
        
        # Simulate robot
        x = robot.simulate_step(x, u, dt)
        
        # Store
        states[k + 1] = x
        controls[k] = u
        errors[k] = error
        
        if k % 100 == 0:
            error_norm = np.linalg.norm(error[:2])
            print(f"  k={k:4d}: error_norm={error_norm:.4f}, u=[{u[0]:.3f}, {u[1]:.3f}]")
    
    # Final error
    final_error = np.linalg.norm(errors[-1, :2])
    mean_error = np.mean(np.linalg.norm(errors[:, :2], axis=1))
    
    print(f"\nResults:")
    print(f"  Mean tracking error: {mean_error:.4f} m")
    print(f"  Final tracking error: {final_error:.4f} m")
    
    # Export logs
    logger.finalize()
    
    # Visualization
    if visualize:
        viz = Visualizer(output_dir='outputs')
        ref_states = trajectory[:, 1:4]  # [px, py, theta]
        
        viz.plot_trajectory(states, ref_states, 
                           title="LQR Trajectory Tracking",
                           save_path="outputs/lqr_tracking.png")
        
        viz.plot_tracking_error(errors, dt,
                               title="LQR Tracking Error",
                               save_path="outputs/lqr_error.png")
        
        viz.plot_control_inputs(controls, dt,
                               title="LQR Control Inputs",
                               save_path="outputs/lqr_control.png")
        
        print("\nPlots saved to outputs/")
    
    return {
        'states': states,
        'controls': controls,
        'errors': errors,
        'reference': trajectory[:, 1:4],
        'mean_error': mean_error,
        'final_error': final_error
    }


def run_mpc_simulation(duration: float = 20.0, dt: float = 0.02,
                       with_obstacles: bool = True,
                       visualize: bool = True) -> dict:
    """
    Run MPC obstacle avoidance simulation.
    
    Args:
        duration: Simulation duration (seconds)
        dt: Time step (seconds)
        with_obstacles: Include obstacles
        visualize: Generate plots
        
    Returns:
        Dictionary with simulation results
    """
    print("=" * 60)
    print("MPC Obstacle Avoidance Simulation")
    print("=" * 60)
    
    # Initialize components
    robot = DifferentialDriveRobot(v_max=2.0, omega_max=3.0)
    traj_gen = ReferenceTrajectoryGenerator(A=2.0, a=0.5, dt=dt)
    mpc = MPCController(
        horizon=10,
        Q_diag=[15.0, 15.0, 8.0],
        R_diag=[0.1, 0.1],
        P_diag=[30.0, 30.0, 5.0],
        d_safe=0.3,
        slack_penalty=1000.0,
        dt=dt,
        v_max=2.0,
        omega_max=3.0,
        solver='ECOS'
    )
    logger = SimulationLogger(log_dir='logs', log_level='INFO', node_name='mpc_sim')
    
    # Define obstacles (positioned along the Figure-8)
    if with_obstacles:
        obstacles = [
            Obstacle(x=1.0, y=0.5, radius=0.2),
            Obstacle(x=-0.5, y=-1.0, radius=0.25),
            Obstacle(x=1.5, y=-0.3, radius=0.15),
        ]
        print(f"Added {len(obstacles)} obstacles")
    else:
        obstacles = []
    
    # Generate reference trajectory
    trajectory = traj_gen.generate(duration)
    N = len(trajectory)
    
    print(f"Generated {N} trajectory points over {duration}s")
    
    # Initialize state at the reference trajectory's starting point
    x_ref_init, _ = traj_gen.get_reference_at_index(0)
    x = x_ref_init.copy()  # Start at reference position and heading
    
    # Storage
    states = np.zeros((N, 3))
    controls = np.zeros((N - 1, 2))
    errors = np.zeros((N - 1, 3))
    solve_times = []
    
    states[0] = x
    
    # Simulation loop (MPC runs at lower rate)
    mpc_rate = 5  # Run MPC every N steps
    
    for k in range(N - 1):
        # Get reference segment
        x_refs, u_refs = traj_gen.get_trajectory_segment(k, mpc.N + 1)
        
        # Compute MPC control at lower rate
        if k % mpc_rate == 0:
            solution = mpc.solve_with_ltv(x, x_refs, u_refs, obstacles)
            solve_times.append(solution.solve_time_ms)
            
            if solution.slack_used:
                logger.log_constraint_event(k, 'slack_activated', 
                                           {'reason': 'feasibility'})
        
        u = solution.optimal_control
        error = x - x_refs[0]
        error[2] = robot.normalize_angle(error[2])
        
        # Log
        logger.log_state(k, x, x_refs[0], error)
        logger.log_control(k, u, 'MPC', solution.solve_time_ms)
        
        # Simulate robot
        x = robot.simulate_step(x, u, dt)
        
        # Store
        states[k + 1] = x
        controls[k] = u
        errors[k] = error
        
        if k % 100 == 0:
            error_norm = np.linalg.norm(error[:2])
            print(f"  k={k:4d}: error_norm={error_norm:.4f}, solve={solution.solve_time_ms:.2f}ms")
    
    # Results
    mean_error = np.mean(np.linalg.norm(errors[:, :2], axis=1))
    final_error = np.linalg.norm(errors[-1, :2])
    mean_solve_time = np.mean(solve_times)
    
    print(f"\nResults:")
    print(f"  Mean tracking error: {mean_error:.4f} m")
    print(f"  Final tracking error: {final_error:.4f} m")
    print(f"  Mean MPC solve time: {mean_solve_time:.2f} ms")
    
    # Check collisions
    collision_count = 0
    for state in states:
        for obs in obstacles:
            if obs.is_collision(state[0], state[1], mpc.d_safe):
                collision_count += 1
                break
    
    if with_obstacles:
        print(f"  Collision events: {collision_count}")
    
    logger.finalize()
    
    # Visualization
    if visualize:
        viz = Visualizer(output_dir='outputs')
        ref_states = trajectory[:, 1:4]
        
        obstacle_dicts = [{'x': o.x, 'y': o.y, 'radius': o.radius} for o in obstacles]
        
        viz.plot_with_obstacles(states, ref_states, obstacle_dicts, mpc.d_safe,
                               title="MPC Obstacle Avoidance",
                               save_path="outputs/mpc_obstacle_avoidance.png")
        
        viz.plot_tracking_error(errors, dt,
                               title="MPC Tracking Error",
                               save_path="outputs/mpc_error.png")
        
        viz.plot_control_inputs(controls, dt,
                               title="MPC Control Inputs",
                               save_path="outputs/mpc_control.png")
        
        print("\nPlots saved to outputs/")
    
    return {
        'states': states,
        'controls': controls,
        'errors': errors,
        'reference': trajectory[:, 1:4],
        'mean_error': mean_error,
        'collision_count': collision_count,
        'mean_solve_time': mean_solve_time
    }


def run_comparison(duration: float = 20.0, dt: float = 0.02) -> None:
    """
    Run comparison between LQR and MPC with obstacles.
    """
    print("=" * 60)
    print("LQR vs MPC Comparison Simulation")
    print("=" * 60)
    
    # Define obstacles
    obstacles = [
        Obstacle(x=1.0, y=0.5, radius=0.2),
        Obstacle(x=-0.5, y=-1.0, radius=0.25),
    ]
    obstacle_dicts = [{'x': o.x, 'y': o.y, 'radius': o.radius} for o in obstacles]
    
    # Run LQR (without visibility of obstacles)
    print("\n--- Running LQR (obstacle-unaware) ---")
    robot = DifferentialDriveRobot(v_max=2.0, omega_max=3.0)
    traj_gen = ReferenceTrajectoryGenerator(A=2.0, a=0.5, dt=dt)
    lqr = LQRController(Q_diag=[15.0, 15.0, 8.0], R_diag=[0.1, 0.1], dt=dt, v_max=2.0, omega_max=3.0)
    trajectory = traj_gen.generate(duration)
    N = len(trajectory)
    
    x_ref_init, _ = traj_gen.get_reference_at_index(0)
    x_lqr = x_ref_init.copy()
    lqr_states = np.zeros((N, 3))
    lqr_states[0] = x_lqr
    
    for k in range(N - 1):
        x_ref, u_ref = traj_gen.get_reference_at_index(k)
        u, _ = lqr.compute_control_at_operating_point(x_lqr, x_ref, u_ref)
        x_lqr = robot.simulate_step(x_lqr, u, dt)
        lqr_states[k + 1] = x_lqr
    
    # Check LQR collisions
    lqr_collisions = sum(1 for s in lqr_states 
                         for o in obstacles 
                         if o.distance_to(s[0], s[1]) < o.radius + 0.3)
    print(f"LQR collision events: {lqr_collisions}")
    
    # Run MPC (obstacle-aware)
    print("\n--- Running MPC (obstacle-aware) ---")
    traj_gen.generate(duration)  # Reset
    mpc = MPCController(horizon=10, Q_diag=[15.0, 15.0, 8.0], R_diag=[0.1, 0.1], 
                       P_diag=[30.0, 30.0, 5.0], d_safe=0.3, dt=dt, 
                       v_max=2.0, omega_max=3.0)
    
    x_mpc = x_ref_init.copy()  # Use same initial state as LQR
    mpc_states = np.zeros((N, 3))
    mpc_states[0] = x_mpc
    
    for k in range(N - 1):
        x_refs, u_refs = traj_gen.get_trajectory_segment(k, mpc.N + 1)
        solution = mpc.solve_with_ltv(x_mpc, x_refs, u_refs, obstacles)
        x_mpc = robot.simulate_step(x_mpc, solution.optimal_control, dt)
        mpc_states[k + 1] = x_mpc
    
    mpc_collisions = sum(1 for s in mpc_states 
                         for o in obstacles 
                         if o.distance_to(s[0], s[1]) < o.radius + 0.3)
    print(f"MPC collision events: {mpc_collisions}")
    
    # Comparison plot
    viz = Visualizer(output_dir='outputs')
    viz.plot_comparison(lqr_states, mpc_states, trajectory[:, 1:4],
                       obstacle_dicts, d_safe=0.3,
                       title="LQR vs MPC: Obstacle Avoidance Comparison",
                       save_path="outputs/comparison.png")
    
    print("\nComparison plot saved to outputs/comparison.png")


def main():
    parser = argparse.ArgumentParser(description='Run hybrid LQR-MPC simulation')
    parser.add_argument('--mode', type=str, default='lqr',
                       choices=['lqr', 'mpc', 'compare'],
                       help='Simulation mode: lqr, mpc, or compare')
    parser.add_argument('--duration', type=float, default=20.0,
                       help='Simulation duration in seconds')
    parser.add_argument('--no-plot', action='store_true',
                       help='Disable plotting')
    
    args = parser.parse_args()
    
    # Create output directories
    os.makedirs('outputs', exist_ok=True)
    os.makedirs('logs', exist_ok=True)
    
    if args.mode == 'lqr':
        run_lqr_simulation(duration=args.duration, visualize=not args.no_plot)
    elif args.mode == 'mpc':
        run_mpc_simulation(duration=args.duration, visualize=not args.no_plot)
    elif args.mode == 'compare':
        run_comparison(duration=args.duration)
    
    print("\nSimulation complete!")


if __name__ == '__main__':
    main()
