#!/usr/bin/env python3
"""
FRP vs CN Comparison Runner (P3-E)
=====================================

Runs the Full Reference Path (FRP) vs Checkpoint Navigation (CN) 
head-to-head comparison experiment.

For each trajectory type and obstacle configuration:
    - 30 trials FRP mode (track full dense reference)
    - 30 trials CN mode (navigate between sparse checkpoints)
    - Compute pairwise statistical comparison

Usage:
    python evaluation/frp_vs_cn_comparison.py --trajectories figure8 clover3
    python evaluation/frp_vs_cn_comparison.py --trials 30 --obstacles 8

Reference:
    Risk-Aware Hybrid LQR-MPC Navigation, Phase 3-E
"""

import sys
import os
import json
import time
import argparse
import numpy as np
from pathlib import Path
from typing import List, Dict, Any

# Add project paths
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src_dir = os.path.join(project_root, 'ros2_ws', 'src', 'hybrid_controller')
for p in [project_root, src_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

from hybrid_controller.trajectory.trajectory_factory import TrajectoryFactory
from hybrid_controller.trajectory.checkpoint_nav import (
    CheckpointExtractor, WaypointManager, CNMetrics
)
from evaluation.scenarios import DensitySweepScenario
from evaluation.stats import wilcoxon_pairwise, cohen_d, wilson_ci, format_comparison_table


def run_comparison(
    trajectories: List[str] = None,
    n_trials: int = 30,
    n_obstacles: int = 8,
    n_checkpoints: int = 12,
    duration: float = 20.0,
    dt: float = 0.02,
    noise_std: float = 0.05,
    base_seed: int = 42,
    output_dir: str = 'evaluation/results/frp_vs_cn',
    verbose: bool = True
) -> Dict[str, Any]:
    """
    Run FRP vs CN head-to-head comparison.
    
    Args:
        trajectories: List of trajectory types to test
        n_trials: Number of trials per mode per trajectory
        n_obstacles: Number of obstacles per config
        n_checkpoints: Number of checkpoints for CN mode
        duration: Simulation duration per trial
        dt: Time step
        noise_std: Position noise std
        base_seed: Base random seed
        output_dir: Output directory
        verbose: Print progress
        
    Returns:
        Dict with comparison results
    """
    if trajectories is None:
        trajectories = ['figure8', 'clover3', 'rose4', 'spiral', 'random_wp']
    
    factory = TrajectoryFactory()
    extractor = CheckpointExtractor(n_checkpoints=n_checkpoints, strategy='curvature')
    
    # Generate obstacle configs (shared across both modes)
    scenario_gen = DensitySweepScenario(obstacle_count=n_obstacles, arena_size=3.0)
    obstacle_configs = scenario_gen.generate(n_trials, base_seed)
    
    all_results = {}
    
    for traj_type in trajectories:
        if verbose:
            print(f"\n{'='*60}")
            print(f"Trajectory: {traj_type}")
            print(f"{'='*60}")
        
        # Generate reference trajectory
        traj = factory.generate(traj_type, duration=duration, dt=dt, A=2.0)
        
        # Extract checkpoints for CN mode
        checkpoints = extractor.extract(traj)
        
        if verbose:
            print(f"  Dense trajectory: {len(traj)} points")
            print(f"  Checkpoints: {len(checkpoints)} extracted")
        
        frp_metrics = {'rmse': [], 'collision': [], 'effort': [], 'jerk': []}
        cn_metrics = {'xte': [], 'collision': [], 'effort': [], 'jerk': [],
                      'completion_rate': []}
        
        for trial in range(n_trials):
            config = obstacle_configs[trial % len(obstacle_configs)]
            seed = base_seed + trial * 1000
            rng = np.random.RandomState(seed)
            
            # ── FRP Mode ──
            frp_rmse, frp_collision, frp_effort, frp_jerk = _simulate_frp(
                traj, config.obstacles, noise_std, dt, rng)
            frp_metrics['rmse'].append(frp_rmse)
            frp_metrics['collision'].append(frp_collision)
            frp_metrics['effort'].append(frp_effort)
            frp_metrics['jerk'].append(frp_jerk)
            
            # ── CN Mode ──
            rng2 = np.random.RandomState(seed)  # Same noise realization
            cn_xte, cn_collision, cn_effort, cn_jerk, cn_completion = _simulate_cn(
                traj, checkpoints, config.obstacles, noise_std, dt, rng2)
            cn_metrics['xte'].append(cn_xte)
            cn_metrics['collision'].append(cn_collision)
            cn_metrics['effort'].append(cn_effort)
            cn_metrics['jerk'].append(cn_jerk)
            cn_metrics['completion_rate'].append(cn_completion)
            
            if verbose and (trial + 1) % 10 == 0:
                print(f"  Trial {trial+1}/{n_trials} complete")
        
        # Store results
        all_results[traj_type] = {
            'frp': {k: np.array(v) for k, v in frp_metrics.items()},
            'cn': {k: np.array(v) for k, v in cn_metrics.items()},
        }
        
        if verbose:
            _print_summary(traj_type, all_results[traj_type])
    
    # Save results
    os.makedirs(output_dir, exist_ok=True)
    _save_results(all_results, output_dir, n_trials, n_obstacles, noise_std)
    
    if verbose:
        print(f"\nResults saved to {output_dir}/")
    
    return all_results


def _simulate_frp(traj, obstacles, noise_std, dt, rng):
    """Simulate one FRP trial (simplified — metrics only)."""
    N = len(traj)
    # Extract reference positions
    if traj.shape[1] >= 6:
        ref_pos = traj[:, 1:3]
    else:
        ref_pos = traj[:, :2]
    
    # Simple kinematic simulation with noise
    errors = []
    collisions = 0
    efforts = []
    controls = []
    
    x = np.array([ref_pos[0, 0], ref_pos[0, 1], 0.0])
    
    for k in range(min(N - 1, len(ref_pos) - 1)):
        # Add noise
        x_noisy = x.copy()
        x_noisy[:2] += rng.normal(0, noise_std, 2)
        
        # Simple proportional tracking
        dx = ref_pos[min(k+1, len(ref_pos)-1), 0] - x_noisy[0]
        dy = ref_pos[min(k+1, len(ref_pos)-1), 1] - x_noisy[1]
        dist = np.sqrt(dx**2 + dy**2)
        
        v = min(1.0, dist / dt)
        theta_target = np.arctan2(dy, dx)
        omega = 2.0 * _angle_diff(theta_target, x[2])
        omega = np.clip(omega, -3.0, 3.0)
        
        # Update state
        x[0] += v * np.cos(x[2]) * dt
        x[1] += v * np.sin(x[2]) * dt
        x[2] += omega * dt
        
        # Metrics
        err = np.sqrt((x[0] - ref_pos[min(k+1, len(ref_pos)-1), 0])**2 + 
                       (x[1] - ref_pos[min(k+1, len(ref_pos)-1), 1])**2)
        errors.append(err)
        efforts.append(v**2 + omega**2)
        controls.append([v, omega])
        
        # Collision check
        for obs in obstacles:
            d = np.sqrt((x[0] - obs['x'])**2 + (x[1] - obs['y'])**2)
            if d < obs['radius']:
                collisions += 1
    
    controls = np.array(controls) if controls else np.zeros((1, 2))
    jerk = _compute_angular_jerk_rms(controls, dt)
    
    rmse = float(np.sqrt(np.mean(np.array(errors)**2))) if errors else 0.0
    return rmse, collisions, float(np.mean(efforts)) if efforts else 0.0, jerk


def _simulate_cn(traj, checkpoints, obstacles, noise_std, dt, rng):
    """Simulate one CN trial (simplified — metrics only)."""
    N = len(traj)
    
    wp_manager = WaypointManager(checkpoints, arrival_radius=0.3, lookahead=3)
    
    x = np.array([checkpoints[0, 0], checkpoints[0, 1], 0.0])
    states = [x.copy()]
    collisions = 0
    efforts = []
    controls = []
    
    for k in range(min(N - 1, 1000)):
        x_noisy = x.copy()
        x_noisy[:2] += rng.normal(0, noise_std, 2)
        
        status = wp_manager.update(x_noisy)
        if status.completed:
            break
        
        # Navigate toward active waypoint
        target = status.active_waypoint
        dx = target[0] - x_noisy[0]
        dy = target[1] - x_noisy[1]
        dist = np.sqrt(dx**2 + dy**2)
        
        v = min(1.0, dist / dt)
        theta_target = np.arctan2(dy, dx)
        omega = 2.0 * _angle_diff(theta_target, x[2])
        omega = np.clip(omega, -3.0, 3.0)
        
        x[0] += v * np.cos(x[2]) * dt
        x[1] += v * np.sin(x[2]) * dt
        x[2] += omega * dt
        
        states.append(x.copy())
        efforts.append(v**2 + omega**2)
        controls.append([v, omega])
        
        for obs in obstacles:
            d = np.sqrt((x[0] - obs['x'])**2 + (x[1] - obs['y'])**2)
            if d < obs['radius']:
                collisions += 1
    
    states = np.array(states)
    controls = np.array(controls) if controls else np.zeros((1, 2))
    
    # CN metrics
    xte = CNMetrics.compute_cross_track_error(states, checkpoints)
    mean_xte = float(np.mean(xte))
    jerk = _compute_angular_jerk_rms(controls, dt)
    completion = wp_manager.n_reached / len(checkpoints)
    
    return mean_xte, collisions, float(np.mean(efforts)) if efforts else 0.0, jerk, completion


def _angle_diff(a, b):
    """Compute smallest angle difference a - b."""
    d = a - b
    while d > np.pi: d -= 2 * np.pi
    while d < -np.pi: d += 2 * np.pi
    return d


def _compute_angular_jerk_rms(controls, dt):
    """Compute angular jerk RMS from control sequence."""
    if len(controls) < 3:
        return 0.0
    omega = controls[:, 1]
    d_omega = np.diff(omega)
    dd_omega = np.diff(d_omega)
    jerk = dd_omega / (dt**2)
    return float(np.sqrt(np.mean(jerk**2)))


def _print_summary(traj_type, results):
    """Print summary for one trajectory type."""
    frp = results['frp']
    cn = results['cn']
    
    print(f"\n  {'Metric':<25} {'FRP':>15} {'CN':>15}")
    print(f"  {'-'*55}")
    print(f"  {'RMSE / XTE (m)':<25} {np.mean(frp['rmse']):>7.4f}±{np.std(frp['rmse']):.4f} "
          f"{np.mean(cn['xte']):>7.4f}±{np.std(cn['xte']):.4f}")
    print(f"  {'Collision count':<25} {np.mean(frp['collision']):>7.1f}±{np.std(frp['collision']):.1f} "
          f"{np.mean(cn['collision']):>7.1f}±{np.std(cn['collision']):.1f}")
    print(f"  {'Control effort':<25} {np.mean(frp['effort']):>7.3f}±{np.std(frp['effort']):.3f} "
          f"{np.mean(cn['effort']):>7.3f}±{np.std(cn['effort']):.3f}")
    print(f"  {'Angular jerk RMS':<25} {np.mean(frp['jerk']):>7.1f}±{np.std(frp['jerk']):.1f} "
          f"{np.mean(cn['jerk']):>7.1f}±{np.std(cn['jerk']):.1f}")
    if 'completion_rate' in cn:
        print(f"  {'CN completion rate':<25} {'N/A':>15} "
              f"{np.mean(cn['completion_rate']):>7.3f}±{np.std(cn['completion_rate']):.3f}")


def _save_results(all_results, output_dir, n_trials, n_obstacles, noise_std):
    """Save comparison results to JSON."""
    output = {
        'metadata': {
            'n_trials': n_trials,
            'n_obstacles': n_obstacles,
            'noise_std': noise_std,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        },
        'trajectories': {}
    }
    
    for traj_type, results in all_results.items():
        output['trajectories'][traj_type] = {
            'frp': {k: {'mean': float(np.mean(v)), 'std': float(np.std(v)),
                         'values': v.tolist()}
                    for k, v in results['frp'].items()},
            'cn': {k: {'mean': float(np.mean(v)), 'std': float(np.std(v)),
                         'values': v.tolist()}
                    for k, v in results['cn'].items()},
        }
    
    path = os.path.join(output_dir, 'frp_vs_cn_results.json')
    with open(path, 'w') as f:
        json.dump(output, f, indent=2)


def main():
    parser = argparse.ArgumentParser(
        description='FRP vs CN head-to-head comparison (P3-E)',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('--trajectories', nargs='+',
                        default=['figure8', 'clover3', 'rose4', 'spiral', 'random_wp'],
                        help='Trajectory types to test')
    parser.add_argument('--trials', type=int, default=30,
                        help='Number of trials per mode per trajectory')
    parser.add_argument('--obstacles', type=int, default=8,
                        help='Number of obstacles per config')
    parser.add_argument('--checkpoints', type=int, default=12,
                        help='Number of checkpoints for CN mode')
    parser.add_argument('--noise', type=float, default=0.05,
                        help='Position noise std (meters)')
    parser.add_argument('--seed', type=int, default=42,
                        help='Base random seed')
    parser.add_argument('--output', type=str, default='evaluation/results/frp_vs_cn',
                        help='Output directory')
    
    args = parser.parse_args()
    
    run_comparison(
        trajectories=args.trajectories,
        n_trials=args.trials,
        n_obstacles=args.obstacles,
        n_checkpoints=args.checkpoints,
        noise_std=args.noise,
        base_seed=args.seed,
        output_dir=args.output,
    )


if __name__ == '__main__':
    main()
