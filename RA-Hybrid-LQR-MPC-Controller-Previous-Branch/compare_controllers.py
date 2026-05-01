#!/usr/bin/env python3
"""
Controller Comparison Script
=============================

Compares all available controllers on the same scenario:
- LQR (no obstacle avoidance)
- MPC (obstacle avoidance)
- Hybrid (MPC + LQR blending)
- Adaptive Hybrid (Adaptive MPC + LQR with LMS)

Generates side-by-side comparison plots.

Usage:
    python compare_controllers.py
    python compare_controllers.py --scenario dense
    python compare_controllers.py --duration 15
"""

import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt

# Add package to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src', 'hybrid_controller'))

from hybrid_controller.models.differential_drive import DifferentialDriveRobot
from hybrid_controller.trajectory.reference_generator import ReferenceTrajectoryGenerator
from hybrid_controller.controllers.lqr_controller import LQRController
from hybrid_controller.controllers.mpc_controller import MPCController, Obstacle
from hybrid_controller.controllers.hybrid_blender import BlendingSupervisor
from hybrid_controller.controllers.adaptive_hybrid_controller import AdaptiveHybridController
from hybrid_controller.controllers.risk_metrics import RiskMetrics


def run_controller(controller_name, duration, dt, obstacles, scenario_name):
    """Run a single controller and return results."""
    print(f"\n{'='*60}")
    print(f"Running {controller_name}")
    print(f"{'='*60}")
    
    robot = DifferentialDriveRobot(v_max=2.0, omega_max=3.0)
    traj_gen = ReferenceTrajectoryGenerator(A=2.0, a=0.5, dt=dt)
    trajectory = traj_gen.generate(duration)
    N = len(trajectory)
    
    x_ref_init, _ = traj_gen.get_reference_at_index(0)
    x = x_ref_init.copy()
    
    states = np.zeros((N, 3))
    controls = np.zeros((N - 1, 2))
    solve_times = []
    extra_data = {}
    
    states[0] = x
    
    # Initialize controller
    if controller_name == "LQR":
        controller = LQRController(Q_diag=[15.0, 15.0, 8.0], R_diag=[0.1, 0.1], 
                                   dt=dt, v_max=2.0, omega_max=3.0)
        
        for k in range(N - 1):
            x_ref, u_ref = traj_gen.get_reference_at_index(k)
            u, _ = controller.compute_control_at_operating_point(x, x_ref, u_ref)
            x = robot.simulate_step(x, u, dt)
            states[k + 1] = x
            controls[k] = u
            
    elif controller_name == "MPC":
        controller = MPCController(horizon=6, Q_diag=[15.0, 15.0, 50.0],
                                   R_diag=[0.1, 0.1], P_diag=[30.0, 30.0, 40.0],
                                   d_safe=0.3, slack_penalty=5000.0, dt=dt,
                                   v_max=2.0, omega_max=3.0, solver='OSQP',
                                   block_size=2, w_max=0.05)
        
        mpc_rate = 5
        solution = None
        
        for k in range(N - 1):
            x_refs, u_refs = traj_gen.get_trajectory_segment(k, controller.N + 1)
            
            if k % mpc_rate == 0:
                solution = controller.solve_with_ltv(x, x_refs, u_refs, obstacles)
                solve_times.append(solution.solve_time_ms)
            
            u = solution.optimal_control if solution else u_refs[0]
            x = robot.simulate_step(x, u, dt)
            states[k + 1] = x
            controls[k] = u
            
    elif controller_name == "Hybrid":
        lqr = LQRController(Q_diag=[15.0, 15.0, 8.0], R_diag=[0.1, 0.1],
                           dt=dt, v_max=2.0, omega_max=3.0)
        mpc = MPCController(horizon=5, Q_diag=[80.0, 80.0, 120.0],
                           R_diag=[0.1, 0.1], P_diag=[20.0, 20.0, 40.0],
                           d_safe=0.3, slack_penalty=5000.0, dt=dt,
                           v_max=2.0, omega_max=3.0, solver='OSQP')
        risk_metrics = RiskMetrics(d_safe=0.3, d_trigger=1.0)
        blender = BlendingSupervisor(k_sigmoid=10.0, risk_threshold=0.3,
                                     dw_max=2.0, dt=dt)
        
        obstacle_dicts = [{'x': o.x, 'y': o.y, 'radius': o.radius} for o in obstacles]
        mpc_rate = 5
        mpc_solution = None
        weights = np.zeros((N - 1,))
        
        for k in range(N - 1):
            x_ref, u_ref = traj_gen.get_reference_at_index(k)
            x_refs, u_refs = traj_gen.get_trajectory_segment(k, mpc.N + 1)
            
            assessment = risk_metrics.assess_risk(x, obstacle_dicts)
            u_lqr, _ = lqr.compute_control_at_operating_point(x, x_ref, u_ref)
            
            if k % mpc_rate == 0:
                mpc_solution = mpc.solve_with_ltv(x, x_refs, u_refs, obstacles)
                solve_times.append(mpc_solution.solve_time_ms)
            
            u_mpc = mpc_solution.optimal_control if mpc_solution else u_lqr
            u, blend_info = blender.blend(u_lqr, u_mpc, assessment.combined_risk,
                                         mpc_solution.status if mpc_solution else 'optimal',
                                         mpc_solution.solve_time_ms if mpc_solution else 0.0,
                                         0.0)
            
            weights[k] = blend_info.weight
            x = robot.simulate_step(x, u, dt)
            states[k + 1] = x
            controls[k] = u
        
        extra_data['weights'] = weights
        
    elif controller_name == "Adaptive Hybrid":
        theta_init = np.array([0.85, 0.85])
        controller = AdaptiveHybridController(
            prediction_horizon=10, terminal_horizon=5,
            mpc_Q_diag=[30.0, 30.0, 5.0], mpc_R_diag=[0.1, 0.1],
            lqr_Q_diag=[15.0, 15.0, 8.0], lqr_R_diag=[0.1, 0.1],
            d_safe=0.3, enable_adaptation=True, adaptation_gamma=0.005,
            theta_init=theta_init, v_max=2.0, omega_max=3.0, dt=dt
        )
        
        mpc_rate = 5
        weights = np.zeros((N - 1,))
        params = np.zeros((N - 1, 2))
        
        for k in range(N - 1):
            x_ref, u_ref = traj_gen.get_reference_at_index(k)
            x_refs, u_refs = traj_gen.get_trajectory_segment(k, controller.adaptive_mpc.N_ext + 1)
            
            u, info = controller.compute_control(x, x_ref, u_ref, obstacles,
                                                x_refs, u_refs, mpc_rate)
            
            weights[k] = info.weight
            params[k] = info.param_estimates
            if info.solver_time_ms > 0:
                solve_times.append(info.solver_time_ms)
            
            x = robot.simulate_step(x, u, dt)
            states[k + 1] = x
            controls[k] = u
        
        extra_data['weights'] = weights
        extra_data['params'] = params
    
    # Compute metrics
    ref_states = trajectory[:, 1:4]
    errors = states - ref_states
    mean_error = np.mean(np.linalg.norm(errors[:, :2], axis=1))
    max_error = np.max(np.linalg.norm(errors[:, :2], axis=1))
    
    # Check collisions
    collision_count = 0
    for state in states:
        for obs in obstacles:
            if obs.is_collision(state[0], state[1], 0.3):
                collision_count += 1
                break
    
    mean_solve_time = np.mean(solve_times) if solve_times else 0.0
    
    print(f"Mean error: {mean_error:.4f} m")
    print(f"Max error: {max_error:.4f} m")
    print(f"Collisions: {collision_count}")
    print(f"Mean solve time: {mean_solve_time:.2f} ms")
    
    return {
        'states': states,
        'controls': controls,
        'errors': errors,
        'mean_error': mean_error,
        'max_error': max_error,
        'collision_count': collision_count,
        'mean_solve_time': mean_solve_time,
        'extra_data': extra_data
    }


def plot_comparison(results, obstacles, trajectory, scenario_name):
    """Create comparison plots."""
    print("\nGenerating comparison plots...")
    
    fig = plt.figure(figsize=(16, 12))
    
    # Plot 1: Trajectories
    ax1 = plt.subplot(2, 3, 1)
    ref_states = trajectory[:, 1:4]
    ax1.plot(ref_states[:, 0], ref_states[:, 1], 'k--', linewidth=2, label='Reference', alpha=0.7)
    
    colors = {'LQR': 'blue', 'MPC': 'red', 'Hybrid': 'green', 'Adaptive Hybrid': 'purple'}
    for name, result in results.items():
        ax1.plot(result['states'][:, 0], result['states'][:, 1], 
                color=colors[name], linewidth=1.5, label=name, alpha=0.8)
    
    # Plot obstacles
    for obs in obstacles:
        circle = plt.Circle((obs.x, obs.y), obs.radius, color='gray', alpha=0.3)
        ax1.add_patch(circle)
        circle_safe = plt.Circle((obs.x, obs.y), obs.radius + 0.3, 
                                color='red', fill=False, linestyle='--', alpha=0.3)
        ax1.add_patch(circle_safe)
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title(f'Trajectory Comparison - {scenario_name}')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Plot 2: Tracking Error
    ax2 = plt.subplot(2, 3, 2)
    dt = 0.02
    for name, result in results.items():
        t = np.arange(len(result['errors'])) * dt
        error_norm = np.linalg.norm(result['errors'][:, :2], axis=1)
        ax2.plot(t, error_norm, color=colors[name], linewidth=1.5, label=name, alpha=0.8)
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position Error (m)')
    ax2.set_title('Tracking Error Comparison')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Performance Metrics
    ax3 = plt.subplot(2, 3, 3)
    names = list(results.keys())
    mean_errors = [results[n]['mean_error'] for n in names]
    collisions = [results[n]['collision_count'] for n in names]
    
    x_pos = np.arange(len(names))
    width = 0.35
    
    ax3_twin = ax3.twinx()
    bars1 = ax3.bar(x_pos - width/2, mean_errors, width, label='Mean Error (m)', 
                    color=[colors[n] for n in names], alpha=0.7)
    bars2 = ax3_twin.bar(x_pos + width/2, collisions, width, label='Collisions', 
                         color='red', alpha=0.5)
    
    ax3.set_xlabel('Controller')
    ax3.set_ylabel('Mean Error (m)', color='blue')
    ax3_twin.set_ylabel('Collision Count', color='red')
    ax3.set_title('Performance Metrics')
    ax3.set_xticks(x_pos)
    ax3.set_xticklabels(names, rotation=15, ha='right')
    ax3.tick_params(axis='y', labelcolor='blue')
    ax3_twin.tick_params(axis='y', labelcolor='red')
    ax3.grid(True, alpha=0.3, axis='y')
    
    # Plot 4: Control Inputs (Linear Velocity)
    ax4 = plt.subplot(2, 3, 4)
    for name, result in results.items():
        t = np.arange(len(result['controls'])) * dt
        ax4.plot(t, result['controls'][:, 0], color=colors[name], 
                linewidth=1.0, label=name, alpha=0.7)
    
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Linear Velocity (m/s)')
    ax4.set_title('Linear Velocity Comparison')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # Plot 5: Blending Weights (for hybrid controllers)
    ax5 = plt.subplot(2, 3, 5)
    for name, result in results.items():
        if 'weights' in result['extra_data']:
            t = np.arange(len(result['extra_data']['weights'])) * dt
            ax5.plot(t, result['extra_data']['weights'], color=colors[name],
                    linewidth=2.0, label=name, alpha=0.8)
    
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Blending Weight w(t)')
    ax5.set_title('Blending Weight (0=LQR, 1=MPC)')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    ax5.set_ylim(-0.05, 1.05)
    
    # Plot 6: Parameter Adaptation (for adaptive hybrid)
    ax6 = plt.subplot(2, 3, 6)
    if 'Adaptive Hybrid' in results and 'params' in results['Adaptive Hybrid']['extra_data']:
        params = results['Adaptive Hybrid']['extra_data']['params']
        t = np.arange(len(params)) * dt
        ax6.plot(t, params[:, 0], 'b-', linewidth=2.0, label='v_scale', alpha=0.8)
        ax6.plot(t, params[:, 1], 'r-', linewidth=2.0, label='ω_scale', alpha=0.8)
        ax6.axhline(1.0, color='k', linestyle='--', alpha=0.3, label='True value')
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Parameter Estimate')
        ax6.set_title('LMS Parameter Adaptation (Adaptive Hybrid)')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
    else:
        ax6.text(0.5, 0.5, 'No adaptation data', ha='center', va='center',
                transform=ax6.transAxes, fontsize=12)
        ax6.set_title('Parameter Adaptation')
    
    plt.tight_layout()
    
    # Save
    os.makedirs('outputs', exist_ok=True)
    filename = f'outputs/controller_comparison_{scenario_name}.png'
    fig.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"Saved comparison plot: {filename}")
    
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description='Compare all controllers')
    parser.add_argument('--duration', type=float, default=10.0, help='Simulation duration')
    parser.add_argument('--scenario', type=str, default='default', 
                       choices=['default', 'sparse', 'dense', 'corridor'],
                       help='Obstacle scenario')
    args = parser.parse_args()
    
    # Define obstacles
    if args.scenario == "sparse":
        obstacles = [Obstacle(x=1.5, y=0.8, radius=0.2)]
    elif args.scenario == "dense":
        obstacles = [
            Obstacle(x=1.0, y=0.5, radius=0.2),
            Obstacle(x=-0.5, y=-1.0, radius=0.25),
            Obstacle(x=1.5, y=-0.3, radius=0.15),
            Obstacle(x=-1.5, y=0.5, radius=0.2),
            Obstacle(x=0.0, y=0.8, radius=0.15),
        ]
    elif args.scenario == "corridor":
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
    
    print("=" * 60)
    print("Controller Comparison")
    print("=" * 60)
    print(f"Scenario: {args.scenario}")
    print(f"Duration: {args.duration}s")
    print(f"Obstacles: {len(obstacles)}")
    
    # Generate reference trajectory
    dt = 0.02
    traj_gen = ReferenceTrajectoryGenerator(A=2.0, a=0.5, dt=dt)
    trajectory = traj_gen.generate(args.duration)
    
    # Run all controllers
    controllers = ["LQR", "MPC", "Hybrid", "Adaptive Hybrid"]
    results = {}
    
    for controller_name in controllers:
        results[controller_name] = run_controller(
            controller_name, args.duration, dt, obstacles, args.scenario
        )
    
    # Generate comparison plots
    plot_comparison(results, obstacles, trajectory, args.scenario)
    
    # Print summary
    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)
    print(f"{'Controller':<20} {'Mean Error (m)':<15} {'Collisions':<12} {'Solve Time (ms)'}")
    print("-" * 60)
    for name, result in results.items():
        print(f"{name:<20} {result['mean_error']:<15.4f} {result['collision_count']:<12} "
              f"{result['mean_solve_time']:.2f}")
    
    print("\n✓ Comparison complete!")


if __name__ == '__main__':
    main()
