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
from hybrid_controller.controllers.hybrid_blender import BlendingSupervisor
from hybrid_controller.controllers.adaptive_hybrid_controller import AdaptiveHybridController
from hybrid_controller.controllers.yaw_stabilizer import YawStabilizer
from hybrid_controller.models.actuator_dynamics import ActuatorDynamics, ActuatorParams
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
    traj_gen = ReferenceTrajectoryGenerator(A=2.0, a=0.5, dt=dt, T_blend=0.5)
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
        block_size=2,                     # Move-blocking: halves decision vars
        w_max=0.05                        # Tube MPC: disturbance bound (5cm)
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
                          scenario: str = "default",
                          actuator_params: ActuatorParams = None) -> dict:
    """
    Run risk-aware hybrid LQR-MPC simulation with continuous blending.
    
    Uses BlendingSupervisor for smooth control arbitration:
        u = w(t) * u_mpc + (1 - w(t)) * u_lqr
    
    where w(t) is computed from risk metrics with anti-chatter guarantees.
    
    Args:
        duration: Simulation duration (seconds)
        dt: Time step (seconds)
        visualize: Generate plots
        visualize: Generate plots
        scenario: Obstacle scenario
        actuator_params: Optional parameters for hardware realism (lag, delay, noise)
        
    Returns:
        Dictionary with simulation results including blend weights and jerk metrics
    """
    print("=" * 60)
    print("Hybrid LQR-MPC Smooth Blending Simulation")
    print("=" * 60)
    
    # Initialize components
    robot = DifferentialDriveRobot(v_max=2.0, omega_max=3.0)
    traj_gen = ReferenceTrajectoryGenerator(A=2.0, a=0.5, dt=dt, T_blend=0.5)
    
    # LQR for low-risk regions (efficient, stable)
    lqr = LQRController(Q_diag=[15.0, 15.0, 8.0], R_diag=[0.1, 0.1], 
                        dt=dt, v_max=2.0, omega_max=3.0)
    
    # MPC for high-risk regions (obstacle-aware, optimal)
    mpc = MPCController(
        horizon=5,
        Q_diag=[80.0, 80.0, 120.0],
        R_diag=[0.1, 0.1],
        P_diag=[20.0, 20.0, 40.0],
        S_diag=[0.1, 0.5],
        J_diag=[0.05, 0.3],     # Second-order jerk penalty (Phase 5A)
        d_safe=0.3,
        slack_penalty=5000.0,
        dt=dt,
        v_max=2.0,
        omega_max=3.0,
        solver='OSQP',
        block_size=2,
        w_max=0.05
    )
    
    # Risk metrics
    risk_metrics = RiskMetrics(
        d_safe=0.3,
        d_trigger=1.0,
        alpha=0.6,
        beta=0.4,
        threshold_low=0.2,
        threshold_medium=0.5
    )
    
    # Blending supervisor (core novelty)
    blender = BlendingSupervisor(
        k_sigmoid=10.0,
        risk_threshold=0.3,
        dw_max=2.0,
        hysteresis_band=0.05,
        solver_time_limit=5.0,
        feasibility_decay=0.8,
        dt=dt
    )
    
    logger = SimulationLogger(log_dir='logs', log_level='INFO', node_name='hybrid_sim')
    
    # Initialize Actuator Dynamics if params are provided
    actuator = None
    if actuator_params:
        actuator = ActuatorDynamics(actuator_params, dt)
    
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
    blend_weights = np.zeros((N - 1,))
    solve_times = []
    
    states[0] = x
    
    # MPC rate control
    mpc_rate = 5
    mpc_solution = None
    
    print(f"\nStarting smooth blending simulation ({N} steps)...")
    
    for k in range(N - 1):
        # 1. Measurement (add sensor noise if actuator params provided)
        x_measured = x.copy()
        if actuator_params:
            if actuator_params.noise_v_std > 0: # Reuse noise param for state noise proxy
                 # Ideally we'd have separate sensor noise params, but for now reuse
                 x_measured[0] += np.random.normal(0, actuator_params.noise_v_std * dt)
                 x_measured[1] += np.random.normal(0, actuator_params.noise_v_std * dt)
                 x_measured[2] += np.random.normal(0, actuator_params.noise_omega_std * dt)
        
        x_ref, u_ref = traj_gen.get_reference_at_index(k)
        
        # --- Compute risk ---
        assessment = risk_metrics.assess_risk(x_measured, obstacle_dicts)
        risk_history[k] = assessment.combined_risk
        
        # --- Compute LQR control (always available, cheap) ---
        u_lqr, _ = lqr.compute_control_at_operating_point(x_measured, x_ref, u_ref)
        
        # --- Compute MPC control (at lower rate) ---
        solver_status = 'optimal'
        solver_time_ms = 0.0
        
        if k % mpc_rate == 0:
            x_refs, u_refs = traj_gen.get_trajectory_segment(k, mpc.N + 1)
            mpc_solution = mpc.solve_with_ltv(x_measured, x_refs, u_refs, obstacles)
            solver_status = mpc_solution.status
            solver_time_ms = mpc_solution.solve_time_ms
            solve_times.append(solver_time_ms)
        
        # Use latest MPC solution (or LQR if MPC hasn't run yet)
        if mpc_solution is not None:
            u_mpc = mpc_solution.optimal_control
        else:
            u_mpc = u_lqr  # Fallback before first MPC solve
        
        # --- Blend controls ---
        feas_margin = mpc_solution.feasibility_margin if mpc_solution is not None else 0.0
        u, blend_info = blender.blend(
            u_lqr=u_lqr,
            u_mpc=u_mpc,
            risk=assessment.combined_risk,
            solver_status=solver_status,
            solver_time_ms=solver_time_ms,
            feasibility_margin=feas_margin
        )
        
        blend_weights[k] = blend_info.weight
        
        # --- Compute jerk (from previous controls) ---
        linear_jerk = 0.0
        angular_jerk = 0.0
        if k >= 2:
            dv_curr = (controls[k-1, 0] - controls[k-2, 0]) / dt
            dv_prev = (controls[k-2, 0] - controls[max(0, k-3), 0]) / dt if k >= 3 else 0.0
            linear_jerk = (dv_curr - dv_prev) / dt
            
            domega_curr = (controls[k-1, 1] - controls[k-2, 1]) / dt
            domega_prev = (controls[k-2, 1] - controls[max(0, k-3), 1]) / dt if k >= 3 else 0.0
            angular_jerk = (domega_curr - domega_prev) / dt
        
        # --- Log ---
        error = x - x_ref
        error[2] = robot.normalize_angle(error[2])
        
        logger.log_state(k, x, x_ref, error)
        logger.log_control(k, u, f"HYBRID(w={blend_info.weight:.2f})", solver_time_ms)
        logger.log_hybrid_step(k, blend_info.weight, assessment.combined_risk,
                               blend_info.mode, linear_jerk, angular_jerk)
        
        # --- Simulate ---
        u_applied = u.copy()
        if actuator:
            u_applied[0], u_applied[1] = actuator.update(u[0], u[1])
            
        x = robot.simulate_step(x, u_applied, dt)
        
        # --- Store ---
        states[k + 1] = x
        controls[k] = u
        errors[k] = error
        
        if k % 100 == 0:
            error_norm = np.linalg.norm(error[:2])
            print(f"  k={k:4d}: risk={assessment.combined_risk:.2f} "
                  f"w={blend_info.weight:.3f} [{blend_info.mode:14s}] "
                  f"err={error_norm:.4f}")
    
    # --- Results ---
    mean_error = np.mean(np.linalg.norm(errors[:, :2], axis=1))
    final_error = np.linalg.norm(errors[-1, :2])
    mean_solve_time = np.mean(solve_times) if solve_times else 0.0
    
    # Jerk metrics
    jerk_metrics = SimulationLogger.compute_jerk_metrics(controls, dt)
    
    # Blending statistics
    blend_stats = blender.get_statistics()
    
    print(f"\nResults:")
    print(f"  Mean tracking error: {mean_error:.4f} m")
    print(f"  Final tracking error: {final_error:.4f} m")
    print(f"  Mean MPC solve time: {mean_solve_time:.2f} ms")
    print(f"\nBlending Statistics:")
    print(f"  Weight mean: {blend_stats['weight_mean']:.3f}")
    print(f"  LQR-dominant: {100*blend_stats['lqr_dominant_fraction']:.1f}%")
    print(f"  Blended: {100*blend_stats['blended_fraction']:.1f}%")
    print(f"  MPC-dominant: {100*blend_stats['mpc_dominant_fraction']:.1f}%")
    print(f"  Weight transitions (w crosses 0.5): {blend_stats['total_switches']}")
    print(f"\nJerk Metrics:")
    print(f"  Linear jerk  - peak: {jerk_metrics['linear_jerk_peak']:.2f}, "
          f"RMS: {jerk_metrics['linear_jerk_rms']:.2f}, "
          f"p95: {jerk_metrics['linear_jerk_p95']:.2f}")
    print(f"  Angular jerk - peak: {jerk_metrics['angular_jerk_peak']:.2f}, "
          f"RMS: {jerk_metrics['angular_jerk_rms']:.2f}, "
          f"p95: {jerk_metrics['angular_jerk_p95']:.2f}")
    
    logger.finalize()
    
    # --- Visualization ---
    if visualize:
        viz = Visualizer(output_dir='outputs')
        ref_states = trajectory[:, 1:4]
        
        viz.plot_with_obstacles(states, ref_states, obstacle_dicts, mpc.d_safe,
                               title="Hybrid Smooth-Blend Trajectory",
                               save_path="outputs/hybrid_trajectory.png")
        
        viz.plot_tracking_error(errors, dt,
                               title="Hybrid Tracking Error",
                               save_path="outputs/hybrid_error.png")
        
        viz.plot_control_inputs(controls, dt,
                               v_max=2.0, omega_max=3.0,
                               title="Hybrid Control Inputs",
                               save_path="outputs/hybrid_control.png")
        
        # Plot blending weight and risk
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        t = np.arange(N - 1) * dt
        
        # Panel 1: Risk and blend weight
        ax1 = axes[0]
        ax1.plot(t, risk_history, 'b-', linewidth=1.0, alpha=0.7, label='Risk')
        ax1.plot(t, blend_weights, 'r-', linewidth=2.0, label='Blend Weight w(t)')
        ax1.axhline(blender.risk_threshold, color='g', linestyle='--', 
                    alpha=0.4, label=f'Threshold ({blender.risk_threshold})')
        ax1.fill_between(t, 0, blend_weights, alpha=0.15, color='red')
        ax1.set_ylabel('Value')
        ax1.set_title('Continuous Blending: Risk-Based Control Arbitration')
        ax1.legend(loc='upper right')
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(-0.05, 1.05)
        
        # Panel 2: Control inputs with blend regions
        ax2 = axes[1]
        ax2.plot(t, controls[:, 0], 'b-', linewidth=1.0, label='v (m/s)')
        ax2.plot(t, controls[:, 1], 'r-', linewidth=1.0, label='omega (rad/s)')
        ax2.set_ylabel('Control')
        ax2.set_title('Blended Control Inputs')
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.3)
        
        # Panel 3: Jerk
        ax3 = axes[2]
        if len(controls) >= 3:
            dv = np.diff(controls[:, 0]) / dt
            domega = np.diff(controls[:, 1]) / dt
            lj = np.diff(dv) / dt
            aj = np.diff(domega) / dt
            t_jerk = np.arange(len(lj)) * dt
            ax3.plot(t_jerk, lj, 'b-', linewidth=0.8, alpha=0.7, label='Linear jerk')
            ax3.plot(t_jerk, aj, 'r-', linewidth=0.8, alpha=0.7, label='Angular jerk')
        ax3.set_ylabel('Jerk')
        ax3.set_xlabel('Time (s)')
        ax3.set_title('Control Jerk (d^2u/dt^2)')
        ax3.legend(loc='upper right')
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        fig.savefig('outputs/hybrid_blending.png', dpi=150, bbox_inches='tight')
        plt.close(fig)
        
        print("\nPlots saved to outputs/")
    
    return {
        'states': states,
        'controls': controls,
        'errors': errors,
        'risk_history': risk_history,
        'blend_weights': blend_weights,
        'blend_stats': blend_stats,
        'jerk_metrics': jerk_metrics,
        'mean_error': mean_error,
        'final_error': final_error,
        'mean_solve_time': mean_solve_time,
    }


def run_adaptive_hybrid_simulation(duration: float = 20.0, dt: float = 0.02,
                                   visualize: bool = True,
                                   scenario: str = "default",
                                   actuator_params: ActuatorParams = None) -> dict:
    """
    Run adaptive hybrid simulation with Adaptive MPC (LMS) + LQR.
    
    Uses distance-based risk metrics to switch between:
    - Adaptive MPC with online parameter learning (near obstacles)
    - LQR for efficient tracking (far from obstacles)
    
    Key Features:
    - LMS adaptation of velocity/angular velocity scaling factors
    - Smooth sigmoid blending with anti-chatter
    - Distance-based risk assessment
    - Online learning when in high-risk regions
    
    Args:
        duration: Simulation duration (seconds)
        dt: Time step (seconds)
        visualize: Generate plots
        scenario: Obstacle scenario
        actuator_params: Optional actuator dynamics parameters
        
    Returns:
        Dictionary with simulation results including parameter adaptation history
    """
    print("=" * 60)
    print("Adaptive Hybrid (Adaptive MPC + LQR) Simulation")
    print("=" * 60)
    
    # Initialize components
    robot = DifferentialDriveRobot(v_max=2.0, omega_max=3.0)
    traj_gen = ReferenceTrajectoryGenerator(A=2.0, a=0.5, dt=dt, T_blend=0.5)
    
    # Initialize Adaptive Hybrid Controller
    # Initial parameter estimates slightly off to demonstrate adaptation
    theta_init = np.array([0.85, 0.85])  # True values are [1.0, 1.0]
    
    adaptive_hybrid = AdaptiveHybridController(
        # Adaptive MPC parameters
        prediction_horizon=10,
        terminal_horizon=5,
        mpc_Q_diag=[30.0, 30.0, 5.0],
        mpc_R_diag=[0.1, 0.1],
        omega_term=10.0,
        q_xi=1000.0,
        d_safe=0.3,
        enable_adaptation=True,
        adaptation_gamma=0.005,  # Same as standalone adaptive MPC
        theta_init=theta_init,
        # LQR parameters
        lqr_Q_diag=[15.0, 15.0, 8.0],
        lqr_R_diag=[0.1, 0.1],
        # Risk metrics - clear switching based on distance only
        d_trigger=0.8,  # Start risk at 0.8m from obstacle
        risk_alpha=1.0,  # Only use distance risk
        risk_beta=0.0,   # Ignore predictive risk
        threshold_low=0.2,
        threshold_medium=0.5,
        # Blending parameters - binary switching
        k_sigmoid=20.0,
        risk_threshold=0.35,  # Switch at risk=0.35: <0.3=LQR, >0.4=MPC
        dw_max=50.0,  # Instant switching
        hysteresis_band=0.05,
        solver_time_limit=50.0,  # Generous timeout
        feasibility_decay=0.8,
        # Common
        v_max=2.0,
        omega_max=3.0,
        dt=dt
    )
    
    logger = SimulationLogger(log_dir='logs', log_level='INFO', node_name='adaptive_hybrid_sim')
    
    # Initialize actuator dynamics if provided
    actuator = None
    if actuator_params:
        actuator = ActuatorDynamics(actuator_params, dt)
    
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
    print(f"Initial parameter estimates: v_scale={theta_init[0]:.3f}, omega_scale={theta_init[1]:.3f}")
    
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
    blend_weights = np.zeros((N - 1,))
    param_history = np.zeros((N - 1, 2))
    adaptation_active = np.zeros((N - 1,), dtype=bool)
    
    states[0] = x
    
    # MPC rate control
    mpc_rate = 5
    
    print(f"\nStarting adaptive hybrid simulation ({N} steps)...")
    print("Legend: w=blend_weight, r=risk, theta=[v_scale, omega_scale], adapt=adaptation_active")
    
    for k in range(N - 1):
        # Get reference
        x_ref, u_ref = traj_gen.get_reference_at_index(k)
        x_refs, u_refs = traj_gen.get_trajectory_segment(k, adaptive_hybrid.adaptive_mpc.N_ext + 1)
        
        # Compute hybrid control
        u, info = adaptive_hybrid.compute_control(
            x=x,
            x_ref=x_ref,
            u_ref=u_ref,
            obstacles=obstacles,
            x_refs=x_refs,
            u_refs=u_refs,
            mpc_rate=mpc_rate
        )
        
        # Store info
        risk_history[k] = info.risk
        blend_weights[k] = info.weight
        param_history[k] = info.param_estimates
        adaptation_active[k] = info.adaptation_active
        
        # Compute error
        error = x - x_ref
        error[2] = robot.normalize_angle(error[2])
        
        # Log
        logger.log_state(k, x, x_ref, error)
        logger.log_control(k, u, f"ADAPTIVE_HYBRID(w={info.weight:.2f})", info.solver_time_ms)
        logger.log_hybrid_step(k, info.weight, info.risk, info.mode, 0.0, 0.0)
        
        # Apply actuator dynamics if enabled
        u_applied = u.copy()
        if actuator:
            u_applied[0], u_applied[1] = actuator.update(u[0], u[1])
        
        # Simulate robot
        x = robot.simulate_step(x, u_applied, dt)
        
        # Store
        states[k + 1] = x
        controls[k] = u
        errors[k] = error
        
        if k % 100 == 0:
            error_norm = np.linalg.norm(error[:2])
            adapt_str = "Y" if info.adaptation_active else "N"
            print(f"  k={k:4d}: r={info.risk:.2f} w={info.weight:.3f} [{info.mode:22s}] "
                  f"params=[{info.param_estimates[0]:.3f},{info.param_estimates[1]:.3f}] "
                  f"adapt={adapt_str} err={error_norm:.4f}")
    
    # Results
    mean_error = np.mean(np.linalg.norm(errors[:, :2], axis=1))
    final_error = np.linalg.norm(errors[-1, :2])
    
    # Get statistics
    stats = adaptive_hybrid.get_statistics()
    
    print(f"\nResults:")
    print(f"  Mean tracking error: {mean_error:.4f} m")
    print(f"  Final tracking error: {final_error:.4f} m")
    print(f"\nAdaptive Hybrid Statistics:")
    print(f"  Weight mean: {stats['weight_mean']:.3f}")
    print(f"  LQR-dominant: {100*stats['lqr_dominant_fraction']:.1f}%")
    print(f"  Blended: {100*stats['blended_fraction']:.1f}%")
    print(f"  Adaptive MPC-dominant: {100*stats['mpc_dominant_fraction']:.1f}%")
    print(f"  Weight transitions: {stats['total_switches']}")
    print(f"  Mean risk: {stats['mean_risk']:.3f}")
    print(f"  Max risk: {stats['max_risk']:.3f}")
    print(f"\nParameter Adaptation:")
    print(f"  Initial estimates: v_scale={theta_init[0]:.3f}, omega_scale={theta_init[1]:.3f}")
    print(f"  Final estimates: v_scale={stats['param_estimates'][0]:.3f}, omega_scale={stats['param_estimates'][1]:.3f}")
    print(f"  Adaptation steps: {np.sum(adaptation_active)}/{len(adaptation_active)}")
    print(f"\nMPC Statistics:")
    print(f"  Total solves: {stats['mpc_stats']['solve_count']}")
    print(f"  Avg solve time: {stats['mpc_stats']['avg_solve_time_ms']:.2f} ms")
    
    # Check collisions
    collision_count = 0
    for state in states:
        for obs in obstacles:
            if obs.is_collision(state[0], state[1], 0.3):
                collision_count += 1
                break
    print(f"  Collision events: {collision_count}")
    
    logger.finalize()
    
    # Visualization
    if visualize:
        viz = Visualizer(output_dir='outputs')
        ref_states = trajectory[:, 1:4]
        
        viz.plot_with_obstacles(states, ref_states, obstacle_dicts, 0.3,
                               title="Adaptive Hybrid (Adaptive MPC + LQR) Trajectory",
                               save_path="outputs/adaptive_hybrid_trajectory.png")
        
        viz.plot_tracking_error(errors, dt,
                               title="Adaptive Hybrid Tracking Error",
                               save_path="outputs/adaptive_hybrid_error.png")
        
        viz.plot_control_inputs(controls, dt,
                               v_max=2.0, omega_max=3.0,
                               title="Adaptive Hybrid Control Inputs",
                               save_path="outputs/adaptive_hybrid_control.png")
        
        # Plot adaptive hybrid specific metrics
        import matplotlib.pyplot as plt
        
        # Figure 1: Risk, blend weight, and adaptation
        fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
        t = np.arange(N - 1) * dt
        
        # Panel 1: Risk and blend weight
        ax1 = axes[0]
        ax1.plot(t, risk_history, 'b-', linewidth=1.0, alpha=0.7, label='Risk')
        ax1.plot(t, blend_weights, 'r-', linewidth=2.0, label='Blend Weight w(t)')
        ax1.axhline(0.3, color='g', linestyle='--', alpha=0.4, label='Risk Threshold')
        ax1.fill_between(t, 0, blend_weights, alpha=0.15, color='red')
        ax1.set_ylabel('Value')
        ax1.set_title('Adaptive Hybrid: Risk-Based Blending with LMS Adaptation')
        ax1.legend(loc='upper right')
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(-0.05, 1.05)
        
        # Panel 2: Parameter adaptation
        ax2 = axes[1]
        ax2.plot(t, param_history[:, 0], 'b-', linewidth=1.5, label='v_scale (linear velocity)')
        ax2.plot(t, param_history[:, 1], 'r-', linewidth=1.5, label='omega_scale (angular velocity)')
        ax2.axhline(1.0, color='k', linestyle='--', alpha=0.3, label='True value')
        ax2.axhline(theta_init[0], color='b', linestyle=':', alpha=0.3)
        ax2.axhline(theta_init[1], color='r', linestyle=':', alpha=0.3)
        ax2.set_ylabel('Parameter Value')
        ax2.set_title('LMS Parameter Adaptation (Online Learning)')
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.3)
        
        # Panel 3: Adaptation activity
        ax3 = axes[2]
        adapt_signal = adaptation_active.astype(float)
        ax3.fill_between(t, 0, adapt_signal, alpha=0.3, color='green', label='Adaptation Active')
        ax3.plot(t, adapt_signal, 'g-', linewidth=0.5, alpha=0.5)
        ax3.set_ylabel('Active')
        ax3.set_title('Adaptation Activity (Learning when w > 0.5)')
        ax3.set_ylim(-0.1, 1.1)
        ax3.legend(loc='upper right')
        ax3.grid(True, alpha=0.3)
        
        # Panel 4: Control inputs
        ax4 = axes[3]
        ax4.plot(t, controls[:, 0], 'b-', linewidth=1.0, label='v (m/s)')
        ax4.plot(t, controls[:, 1], 'r-', linewidth=1.0, label='omega (rad/s)')
        ax4.set_ylabel('Control')
        ax4.set_xlabel('Time (s)')
        ax4.set_title('Blended Control Inputs')
        ax4.legend(loc='upper right')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        fig.savefig('outputs/adaptive_hybrid_blending.png', dpi=150, bbox_inches='tight')
        plt.close(fig)
        
        # Figure 2: Parameter convergence detail
        fig2, ax = plt.subplots(1, 1, figsize=(10, 6))
        ax.plot(t, param_history[:, 0], 'b-', linewidth=2.0, label='v_scale', alpha=0.8)
        ax.plot(t, param_history[:, 1], 'r-', linewidth=2.0, label='omega_scale', alpha=0.8)
        ax.axhline(1.0, color='k', linestyle='--', linewidth=1.5, alpha=0.5, label='True value (1.0)')
        ax.fill_between(t, param_history[:, 0], 1.0, alpha=0.1, color='blue')
        ax.fill_between(t, param_history[:, 1], 1.0, alpha=0.1, color='red')
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Parameter Estimate', fontsize=12)
        ax.set_title('LMS Parameter Convergence (Adaptive MPC)', fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=11)
        ax.grid(True, alpha=0.3)
        fig2.tight_layout()
        fig2.savefig('outputs/adaptive_mpc_params.png', dpi=150, bbox_inches='tight')
        plt.close(fig2)
        
        print("\nPlots saved to outputs/")
    
    return {
        'states': states,
        'controls': controls,
        'errors': errors,
        'risk_history': risk_history,
        'blend_weights': blend_weights,
        'param_history': param_history,
        'adaptation_active': adaptation_active,
        'stats': stats,
        'mean_error': mean_error,
        'final_error': final_error,
        'collision_count': collision_count,
    }


def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Run hybrid LQR-MPC simulation')
    parser.add_argument('--mode', type=str, 
                        choices=['lqr', 'mpc', 'compare', 'hybrid', 'adaptive_hybrid'], 
                        default='lqr', help='Simulation mode')
    parser.add_argument('--duration', type=float, default=20.0, help='Duration in seconds')
    parser.add_argument('--dt', type=float, default=0.02, help='Time step')
    parser.add_argument('--scenario', type=str, default='default', help='Obstacle scenario')
    parser.add_argument('--no-plot', action='store_true', help='Disable plotting')
    parser.add_argument('--realistic', action='store_true', help='Enable actuator dynamics (lag, delay, noise)')
    
    args = parser.parse_args()
    
    # Create output directories
    os.makedirs('outputs', exist_ok=True)
    os.makedirs('logs', exist_ok=True)
    
    # Configure realism
    act_params = None
    if args.realistic:
        act_params = ActuatorParams(
            tau_v=0.1, tau_omega=0.1,
            delay_steps=1,
            noise_v_std=0.02, noise_omega_std=0.05
        )
    
    if args.mode == 'lqr':
        run_lqr_simulation(duration=args.duration, dt=args.dt, visualize=not args.no_plot)
    elif args.mode == 'mpc':
        run_mpc_simulation(duration=args.duration, dt=args.dt, 
                           visualize=not args.no_plot, scenario=args.scenario)
    elif args.mode == 'compare':
        run_comparison(duration=args.duration, dt=args.dt)
    elif args.mode == 'hybrid':
        run_hybrid_simulation(duration=args.duration, dt=args.dt, 
                              visualize=not args.no_plot, scenario=args.scenario,
                              actuator_params=act_params)
    elif args.mode == 'adaptive_hybrid':
        run_adaptive_hybrid_simulation(duration=args.duration, dt=args.dt,
                                       visualize=not args.no_plot, scenario=args.scenario,
                                       actuator_params=act_params)
    
    print("\nSimulation complete!")


if __name__ == '__main__':
    main()
