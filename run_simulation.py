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
from hybrid_controller.controllers.risk_metrics import RiskMetrics
from hybrid_controller.controllers.yaw_stabilizer import YawStabilizer
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
                               v_max=2.0, omega_max=3.0,
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
                       visualize: bool = True,
                       scenario: str = "default") -> dict:
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
    # OPTIMIZED MPC PARAMETERS for industry-standard tolerances
    # Target: heading ≤5°, latency ≤50ms, slack ≤5
    mpc = MPCController(
        horizon=6,                        # Reduced: 10→6 for faster solves
        Q_diag=[15.0, 15.0, 50.0],       # Aggressive heading: 20→50
        R_diag=[0.1, 0.1],
        P_diag=[30.0, 30.0, 40.0],       # Strong terminal heading: 15→40
        d_safe=0.3,
        slack_penalty=5000.0,             # Stricter: 1000→5000
        dt=dt,
        v_max=2.0,
        omega_max=3.0,
        solver='OSQP',                    # Faster: ECOS→OSQP
        block_size=2                      # Move-blocking: halves decision vars
    )
    
    # Yaw stabilizer for cold-start bootstrap (first N steps)
    yaw_stabilizer = YawStabilizer(
        kp=3.0, ki=0.1, kd=0.5,
        dt=dt,
        omega_max=3.0
    )
    BOOTSTRAP_STEPS = 10  # Use yaw stabilizer for first 10 steps
    
    logger = SimulationLogger(log_dir='logs', log_level='INFO', node_name='mpc_sim')
    
    # Define obstacles based on scenario
    # Multiple scenarios for comprehensive validation (real-world applicability)
    if with_obstacles:
        if scenario == "sparse":
            # Sparse: Few obstacles, far from trajectory - easy scenario
            obstacles = [
                Obstacle(x=1.5, y=0.8, radius=0.2),
            ]
        elif scenario == "dense":
            # Dense: Many obstacles, close to trajectory - challenging scenario
            obstacles = [
                Obstacle(x=1.0, y=0.5, radius=0.2),
                Obstacle(x=-0.5, y=-1.0, radius=0.25),
                Obstacle(x=1.5, y=-0.3, radius=0.15),
                Obstacle(x=-1.5, y=0.5, radius=0.2),
                Obstacle(x=0.0, y=0.8, radius=0.15),
            ]
        elif scenario == "corridor":
            # Corridor: Obstacles forming narrow passages - real-world hallway simulation
            obstacles = [
                Obstacle(x=1.0, y=0.3, radius=0.15),
                Obstacle(x=1.0, y=0.7, radius=0.15),
                Obstacle(x=-0.8, y=-0.7, radius=0.15),
                Obstacle(x=-0.3, y=-1.2, radius=0.15),
            ]
        else:  # default
            # Default: Original scenario for baseline comparison
            obstacles = [
                Obstacle(x=1.0, y=0.5, radius=0.2),
                Obstacle(x=-0.5, y=-1.0, radius=0.25),
                Obstacle(x=1.5, y=-0.3, radius=0.15),
            ]
        print(f"Scenario: {scenario} | Added {len(obstacles)} obstacles")
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
        
        # MPC control at lower rate
        if k % mpc_rate == 0:
            solution = mpc.solve_with_ltv(x, x_refs, u_refs, obstacles)
            solve_times.append(solution.solve_time_ms)
            
            if solution.slack_used:
                logger.log_constraint_event(k, 'slack_activated', 
                                           {'reason': 'feasibility'})
        
        u = solution.optimal_control
        controller_name = 'MPC'
        solve_time = solution.solve_time_ms
        
        error = x - x_refs[0]
        error[2] = robot.normalize_angle(error[2])
        
        # Log
        logger.log_state(k, x, x_refs[0], error)
        logger.log_control(k, u, controller_name, solve_time)
        
        # Simulate robot
        x = robot.simulate_step(x, u, dt)
        
        # Store
        states[k + 1] = x
        controls[k] = u
        errors[k] = error
        
        if k % 100 == 0:
            error_norm = np.linalg.norm(error[:2])
            ctrl_info = f"solve={solve_time:.2f}ms" if controller_name == 'MPC' else "bootstrap"
            print(f"  k={k:4d}: error_norm={error_norm:.4f}, {ctrl_info}")
    
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
                               v_max=2.0, omega_max=3.0,
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
    
    # Run MPC (obstacle-aware) with optimized weights
    print("\n--- Running MPC (obstacle-aware) ---")
    traj_gen.generate(duration)  # Reset
    mpc = MPCController(horizon=6, 
                       Q_diag=[15.0, 15.0, 50.0],   # Optimized heading
                       R_diag=[0.1, 0.1], 
                       P_diag=[30.0, 30.0, 40.0],   # Strong terminal heading
                       d_safe=0.3, slack_penalty=5000.0, dt=dt, 
                       v_max=2.0, omega_max=3.0, solver='OSQP')
    
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


def run_hybrid_simulation(duration: float = 20.0, dt: float = 0.02,
                          visualize: bool = True,
                          scenario: str = "default") -> dict:
    """
    Run risk-aware hybrid LQR-MPC simulation.
    
    Uses RiskMetrics to dynamically switch between LQR (low risk) and MPC (high risk).
    
    Args:
        duration: Simulation duration (seconds)
        dt: Time step (seconds)
        visualize: Generate plots
        scenario: Obstacle scenario
        
    Returns:
        Dictionary with simulation results
    """
    print("=" * 60)
    print("Hybrid LQR-MPC Risk-Aware Simulation")
    print("=" * 60)
    
    # Initialize components
    robot = DifferentialDriveRobot(v_max=2.0, omega_max=3.0)
    traj_gen = ReferenceTrajectoryGenerator(A=2.0, a=0.5, dt=dt)
    
    # LQR for low-risk scenarios
    lqr = LQRController(Q_diag=[15.0, 15.0, 8.0], R_diag=[0.1, 0.1], 
                        dt=dt, v_max=2.0, omega_max=3.0)
    
    # MPC for high-risk scenarios (optimized for industry tolerances)
    mpc = MPCController(
        horizon=6,
        Q_diag=[15.0, 15.0, 50.0],
        R_diag=[0.1, 0.1],
        P_diag=[30.0, 30.0, 40.0],
        d_safe=0.3,
        slack_penalty=5000.0,
        dt=dt,
        v_max=2.0,
        omega_max=3.0,
        solver='OSQP'
    )
    
    # Risk metrics for switching
    risk_metrics = RiskMetrics(
        d_safe=0.3,
        d_trigger=1.0,
        alpha=0.6,  # Distance risk weight
        beta=0.4,   # Predictive risk weight
        threshold_low=0.2,
        threshold_medium=0.5
    )
    
    logger = SimulationLogger(log_dir='logs', log_level='INFO', node_name='hybrid_sim')
    
    # Define obstacles based on scenario
    if scenario == "sparse":
        obstacles = [Obstacle(x=1.5, y=0.8, radius=0.2)]
    elif scenario == "dense":
        obstacles = [
            Obstacle(x=1.0, y=0.5, radius=0.2),
            Obstacle(x=-0.5, y=-1.0, radius=0.25),
            Obstacle(x=1.5, y=-0.3, radius=0.15),
            Obstacle(x=-1.5, y=0.5, radius=0.2),
            Obstacle(x=0.0, y=0.8, radius=0.15),
        ]
    elif scenario == "corridor":
        obstacles = [
            Obstacle(x=1.0, y=0.3, radius=0.15),
            Obstacle(x=1.0, y=0.7, radius=0.15),
            Obstacle(x=-0.8, y=-0.7, radius=0.15),
            Obstacle(x=-0.3, y=-1.2, radius=0.15),
        ]
    else:  # default
        obstacles = [
            Obstacle(x=1.0, y=0.5, radius=0.2),
            Obstacle(x=-0.5, y=-1.0, radius=0.25),
            Obstacle(x=1.5, y=-0.3, radius=0.15),
        ]
    
    obstacle_dicts = [{'x': o.x, 'y': o.y, 'radius': o.radius} for o in obstacles]
    print(f"Scenario: {scenario} | Added {len(obstacles)} obstacles")
    
    # Generate trajectory
    trajectory = traj_gen.generate(duration)
    N = len(trajectory)
    
    # Initialize state
    x_ref_init, _ = traj_gen.get_reference_at_index(0)
    x = x_ref_init.copy()
    
    # Storage
    states = np.zeros((N, 3))
    controls = np.zeros((N - 1, 2))
    errors = np.zeros((N - 1, 3))
    risk_history = np.zeros((N - 1,))
    controller_used = []  # 'LQR' or 'MPC' at each step
    
    states[0] = x
    
    # Switching statistics
    lqr_steps = 0
    mpc_steps = 0
    switches = 0
    prev_controller = None
    
    # Hysteresis: don't switch too frequently
    min_dwell_steps = 10  # Stay on a controller for at least 10 steps
    steps_since_switch = 0
    
    print(f"\nStarting hybrid simulation ({N} steps)...")
    
    for k in range(N - 1):
        x_ref, u_ref = traj_gen.get_reference_at_index(k)
        
        # Compute risk
        assessment = risk_metrics.assess_risk(x, obstacle_dicts)
        risk_history[k] = assessment.combined_risk
        
        # Controller selection with hysteresis
        if steps_since_switch >= min_dwell_steps:
            use_mpc = assessment.use_mpc
        else:
            # Keep previous controller during dwell time
            use_mpc = prev_controller == 'MPC' if prev_controller else assessment.use_mpc
        
        current_controller = 'MPC' if use_mpc else 'LQR'
        
        # Detect switch
        if prev_controller is not None and current_controller != prev_controller:
            switches += 1
            steps_since_switch = 0
        else:
            steps_since_switch += 1
        
        prev_controller = current_controller
        controller_used.append(current_controller)
        
        # Compute control
        if use_mpc:
            x_refs, u_refs = traj_gen.get_trajectory_segment(k, mpc.N + 1)
            solution = mpc.solve_with_ltv(x, x_refs, u_refs, obstacles)
            u = solution.optimal_control
            mpc_steps += 1
        else:
            u, _ = lqr.compute_control_at_operating_point(x, x_ref, u_ref)
            lqr_steps += 1
        
        # Log
        error = x - x_ref
        logger.log_state(k, x, x_ref, error)
        logger.log_control(k, u, current_controller)
        
        # Simulate
        x = robot.simulate_step(x, u, dt)
        
        # Store
        states[k + 1] = x
        controls[k] = u
        errors[k] = error
        
        if k % 100 == 0:
            print(f"  k={k:4d}: risk={assessment.combined_risk:.2f} [{assessment.risk_level:8s}] "
                  f"controller={current_controller}")
    
    # Statistics
    mean_error = np.mean(np.linalg.norm(errors[:, :2], axis=1))
    final_error = np.linalg.norm(errors[-1, :2])
    
    print(f"\nResults:")
    print(f"  Mean tracking error: {mean_error:.4f} m")
    print(f"  Final tracking error: {final_error:.4f} m")
    print(f"  LQR steps: {lqr_steps} ({100*lqr_steps/(N-1):.1f}%)")
    print(f"  MPC steps: {mpc_steps} ({100*mpc_steps/(N-1):.1f}%)")
    print(f"  Controller switches: {switches}")
    
    logger.finalize()
    
    # Visualization
    if visualize:
        viz = Visualizer(output_dir='outputs')
        ref_states = trajectory[:, 1:4]
        
        viz.plot_with_obstacles(states, ref_states, obstacle_dicts, mpc.d_safe,
                               title="Hybrid LQR-MPC Trajectory",
                               save_path="outputs/hybrid_trajectory.png")
        
        viz.plot_tracking_error(errors, dt,
                               title="Hybrid Tracking Error",
                               save_path="outputs/hybrid_error.png")
        
        viz.plot_control_inputs(controls, dt,
                               v_max=2.0, omega_max=3.0,
                               title="Hybrid Control Inputs",
                               save_path="outputs/hybrid_control.png")
        
        # Plot risk history
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 4))
        t = np.arange(N - 1) * dt
        ax.plot(t, risk_history, 'b-', linewidth=1.5, label='Combined Risk')
        ax.axhline(risk_metrics.threshold_low, color='g', linestyle='--', 
                  alpha=0.5, label=f'Low threshold ({risk_metrics.threshold_low})')
        ax.axhline(risk_metrics.threshold_medium, color='orange', linestyle='--',
                  alpha=0.5, label=f'Medium threshold ({risk_metrics.threshold_medium})')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Risk Level')
        ax.set_title('Risk History and Controller Switching')
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.savefig('outputs/hybrid_risk.png', dpi=150, bbox_inches='tight')
        plt.close(fig)
        
        print("\nPlots saved to outputs/")
    
    return {
        'states': states,
        'controls': controls,
        'errors': errors,
        'risk_history': risk_history,
        'controller_used': controller_used,
        'lqr_steps': lqr_steps,
        'mpc_steps': mpc_steps,
        'switches': switches,
        'mean_error': mean_error
    }


def main():
    parser = argparse.ArgumentParser(description='Run hybrid LQR-MPC simulation')
    parser.add_argument('--mode', type=str, default='lqr',
                       choices=['lqr', 'mpc', 'compare', 'hybrid'],
                       help='Simulation mode: lqr, mpc, compare, or hybrid')
    parser.add_argument('--duration', type=float, default=20.0,
                       help='Simulation duration in seconds')
    parser.add_argument('--scenario', type=str, default='default',
                       choices=['default', 'sparse', 'dense', 'corridor'],
                       help='Obstacle scenario: default, sparse, dense, or corridor')
    parser.add_argument('--no-plot', action='store_true',
                       help='Disable plotting')
    
    args = parser.parse_args()
    
    # Create output directories
    os.makedirs('outputs', exist_ok=True)
    os.makedirs('logs', exist_ok=True)
    
    if args.mode == 'lqr':
        run_lqr_simulation(duration=args.duration, visualize=not args.no_plot)
    elif args.mode == 'mpc':
        run_mpc_simulation(duration=args.duration, visualize=not args.no_plot, 
                          scenario=args.scenario)
    elif args.mode == 'compare':
        run_comparison(duration=args.duration)
    elif args.mode == 'hybrid':
        run_hybrid_simulation(duration=args.duration, visualize=not args.no_plot,
                             scenario=args.scenario)
    
    print("\nSimulation complete!")


if __name__ == '__main__':
    main()
