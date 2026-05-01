"""
Statistical Validation Framework
=================================

Monte Carlo runner for comparing controller modes under randomized
obstacle configurations, sensor noise, and control latency.

Usage:
    python evaluation/statistical_runner.py --configs 50 --modes hybrid lqr mpc
    python evaluation/statistical_runner.py --configs 100 --noise 0.01 --delay 20

Output:
    - Console table: mean ± std for each metric × controller
    - CSV: evaluation/results/statistical_results.csv
    - JSON: evaluation/results/statistical_results.json
"""

import sys
import os
import json
import time
import argparse
import numpy as np
from pathlib import Path
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Any, Optional, Tuple

# Add project root and source directories to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src_dir = os.path.join(project_root, 'src', 'hybrid_controller')
for p in [project_root, src_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

from hybrid_controller.models.differential_drive import DifferentialDriveRobot
from hybrid_controller.controllers.lqr_controller import LQRController
from hybrid_controller.models.actuator_dynamics import ActuatorDynamics, ActuatorParams
from hybrid_controller.controllers.mpc_controller import Obstacle
from hybrid_controller.controllers.cvxpygen_solver import CVXPYgenWrapper
from hybrid_controller.models.linearization import Linearizer
from hybrid_controller.controllers.hybrid_blender import BlendingSupervisor
from hybrid_controller.controllers.risk_metrics import RiskMetrics
from hybrid_controller.trajectory.reference_generator import ReferenceTrajectoryGenerator
from hybrid_controller.logging.simulation_logger import SimulationLogger
from evaluation.scenarios import ObstacleConfig, get_generator, ScenarioGenerator


# -- Data classes ----------------------------------------------------

@dataclass 
class NoiseConfig:
    """Sensor noise, actuator lag, and control latency parameters."""
    position_noise_std: float = 0.0    # Sensor noise (meters)
    heading_noise_std: float = 0.0     # Sensor noise (radians)
    control_delay_steps: int = 0       # Pipeline delay
    tau_v: float = 0.0                 # Actuator lag (time constant)
    tau_omega: float = 0.0             # Actuator lag
    actuator_noise_std: float = 0.0    # Execution noise (v/omega)
    
    @property
    def delay_ms(self) -> float:
        return self.control_delay_steps * 20.0  # assumes dt=0.02
    
    def to_actuator_params(self) -> ActuatorParams:
        return ActuatorParams(
            tau_v=self.tau_v, 
            tau_omega=self.tau_omega,
            noise_v_std=self.actuator_noise_std,
            noise_omega_std=self.actuator_noise_std,
            delay_steps=self.control_delay_steps
        )


@dataclass
class RunMetrics:
    """Collected metrics from a single simulation run."""
    mode: str
    config_id: int
    seed: int
    mean_tracking_error: float
    max_tracking_error: float  
    final_tracking_error: float
    mean_solve_time_ms: float
    max_solve_time_ms: float
    linear_jerk_rms: float
    angular_jerk_rms: float
    linear_jerk_peak: float
    angular_jerk_peak: float
    infeasible_count: int
    collision_count: int
    completion_fraction: float  # How much of trajectory was completed
    total_control_effort: float
    blend_weight_mean: float = 0.0
    blend_weight_std: float = 0.0
    smooth_transitions: int = 0
    wall_time_s: float = 0.0
    

@dataclass
class AggregatedResults:
    """Statistics across multiple runs for one controller mode."""
    mode: str
    n_runs: int
    metrics: Dict[str, Dict[str, float]]  # metric_name -> {mean, std, min, max, p5, p95}





# -- Single simulation runner ----------------------------------------

def run_single_config(mode: str, 
                       obstacle_config: ObstacleConfig,
                       noise_config: NoiseConfig,
                       duration: float = 20.0,
                       dt: float = 0.02,
                       config_id: int = 0) -> RunMetrics:
    """
    Run a single simulation with given controller mode and obstacle config.
    
    Modes:
        - 'lqr':       Pure LQR (ignores obstacles)
        - 'mpc':       Pure MPC
        - 'hard_switch': Hard LQR/MPC switching (threshold-based)
        - 'hybrid':    Smooth blending (BlendingSupervisor)
        
    Args:
        mode: Controller mode string
        obstacle_config: Randomized obstacle layout
        noise_config: Noise and latency parameters
        duration: Simulation duration (seconds)
        dt: Time step (seconds)
        config_id: Config index for identification
        
    Returns:
        RunMetrics with all collected performance data
    """
    start_wall = time.perf_counter()
    
    # -- Initialize components --
    robot = DifferentialDriveRobot(v_max=2.0, omega_max=3.0)
    traj_gen = ReferenceTrajectoryGenerator(A=2.0, a=0.5, dt=dt, T_blend=0.5)
    
    lqr = LQRController(Q_diag=[15.0, 15.0, 8.0], R_diag=[0.1, 0.1],
                         dt=dt, v_max=2.0, omega_max=3.0)
    
    mpc = CVXPYgenWrapper(
        horizon=5, Q_diag=[80.0, 80.0, 120.0], R_diag=[0.1, 0.1],
        P_diag=[20.0, 20.0, 40.0], S_diag=[0.1, 0.5], J_diag=[0.05, 0.3],
        v_max=2.0, omega_max=3.0, solver_name='OSQP'
    )
    linearizer = Linearizer(dt=dt)
    
    risk_metrics = RiskMetrics(
        d_safe=0.3, d_trigger=1.0, alpha=0.6, beta=0.4,
        threshold_low=0.2, threshold_medium=0.5
    )
    
    blender = BlendingSupervisor(
        k_sigmoid=10.0, risk_threshold=0.3, dw_max=2.0,
        hysteresis_band=0.05, solver_time_limit=5.0,
        feasibility_decay=0.8, dt=dt
    )
    
    obstacles = obstacle_config.to_obstacle_list()
    obstacle_dicts = obstacle_config.obstacles
    
    # -- Generate trajectory --
    trajectory = traj_gen.generate(duration)
    N = len(trajectory)
    
    x_ref_init, _ = traj_gen.get_reference_at_index(0)
    x = np.array([x_ref_init[0], x_ref_init[1], x_ref_init[2]])
    
    # Storage
    states = np.zeros((N, 3))
    controls = np.zeros((N - 1, 2))
    errors = np.zeros(N)
    solve_times = []
    infeasible_count = 0
    collision_count = 0
    blend_weights = np.zeros(N - 1) if mode == 'hybrid' else None
    
    # Control delay buffer managed by ActuatorDynamics now
    actuator = ActuatorDynamics(noise_config.to_actuator_params(), dt)
    
    # MPC rate control
    mpc_rate = 5  # Run MPC every 5 steps
    mpc_solution = None
    solver_status = 'optimal'
    solver_time_ms = 0.0
    
    # -- Simulation loop --
    for k in range(N):
        # Apply noise to state measurement
        x_measured = x.copy()
        if noise_config.position_noise_std > 0:
            x_measured[0] += np.random.normal(0, noise_config.position_noise_std)
            x_measured[1] += np.random.normal(0, noise_config.position_noise_std)
        if noise_config.heading_noise_std > 0:
            x_measured[2] += np.random.normal(0, noise_config.heading_noise_std)
        
        states[k] = x
        
        # Reference
        x_ref, x_ref_dot = traj_gen.get_reference_at_index(k)
        error = robot.compute_tracking_error(x_measured, x_ref)
        errors[k] = np.linalg.norm(error[:2])
        
        if k >= N - 1:
            break
        
        # -- Compute controls based on mode --
        if mode == 'lqr':
            u = lqr.compute_control(x_measured, x_ref, x_ref_dot)
        
        elif mode == 'mpc':
            lookahead = min(mpc.N + 1, N - k)
            x_refs = np.array([traj_gen.get_reference_at_index(k + j)[0] 
                              for j in range(lookahead)])
            u_refs = np.array([traj_gen.get_reference_at_index(min(k + j, N - 2))[1][:2] 
                              for j in range(lookahead - 1)])
            
            # Pad if needed
            while len(x_refs) < mpc.N + 1:
                x_refs = np.vstack([x_refs, x_refs[-1:]])
            while len(u_refs) < mpc.N:
                u_refs = np.vstack([u_refs, u_refs[-1:]])
            
            # Use linearizer to get A_d, B_d
            v_ref = u_refs[0, 0] if abs(u_refs[0, 0]) > 0.01 else 0.1
            A_d, B_d = linearizer.get_discrete_model_explicit(v_ref, traj_gen.get_reference_at_index(k)[0][2])
            
            sol = mpc.solve_fast(x_measured, x_refs, A_d, B_d)
            u = sol.optimal_control
            solve_times.append(sol.solve_time_ms)
            if sol.status not in ('optimal', 'optimal_inaccurate'):
                infeasible_count += 1
        
        elif mode == 'hard_switch':
            # Risk-based hard switching (baseline comparison)
            assessment = risk_metrics.assess_risk(x_measured, obstacle_dicts)
            
            if assessment.combined_risk > 0.3:
                # MPC mode
                lookahead = min(mpc.N + 1, N - k)
                x_refs = np.array([traj_gen.get_reference_at_index(k + j)[0] 
                                  for j in range(lookahead)])
                u_refs = np.array([traj_gen.get_reference_at_index(min(k + j, N - 2))[1][:2] 
                                  for j in range(lookahead - 1)])
                while len(x_refs) < mpc.N + 1:
                    x_refs = np.vstack([x_refs, x_refs[-1:]])
                while len(u_refs) < mpc.N:
                    u_refs = np.vstack([u_refs, u_refs[-1:]])
                
                v_ref = u_refs[0, 0] if abs(u_refs[0, 0]) > 0.01 else 0.1
                A_d, B_d = linearizer.get_discrete_model_explicit(v_ref, traj_gen.get_reference_at_index(k)[0][2])
                
                sol = mpc.solve_fast(x_measured, x_refs, A_d, B_d)
                u = sol.optimal_control
                solve_times.append(sol.solve_time_ms)
                if sol.status not in ('optimal', 'optimal_inaccurate'):
                    infeasible_count += 1
            else:
                u = lqr.compute_control(x_measured, x_ref, x_ref_dot)
        
        elif mode == 'hybrid':
            # Smooth blending (our contribution)
            u_lqr = lqr.compute_control(x_measured, x_ref, x_ref_dot)
            assessment = risk_metrics.assess_risk(x_measured, obstacle_dicts)
            
            # MPC at reduced rate
            if k % mpc_rate == 0:
                lookahead = min(mpc.N + 1, N - k)
                x_refs = np.array([traj_gen.get_reference_at_index(k + j)[0] 
                                  for j in range(lookahead)])
                u_refs = np.array([traj_gen.get_reference_at_index(min(k + j, N - 2))[1][:2] 
                                  for j in range(lookahead - 1)])
                while len(x_refs) < mpc.N + 1:
                    x_refs = np.vstack([x_refs, x_refs[-1:]])
                while len(u_refs) < mpc.N:
                    u_refs = np.vstack([u_refs, u_refs[-1:]])
                
                v_ref = u_refs[0, 0] if abs(u_refs[0, 0]) > 0.01 else 0.1
                A_d, B_d = linearizer.get_discrete_model_explicit(v_ref, traj_gen.get_reference_at_index(k)[0][2])
                
                mpc_solution = mpc.solve_fast(x_measured, x_refs, A_d, B_d)
                solver_status = mpc_solution.status
                solver_time_ms = mpc_solution.solve_time_ms
                solve_times.append(solver_time_ms)
                
                if solver_status not in ('optimal', 'optimal_inaccurate'):
                    infeasible_count += 1
            
            u_mpc = mpc_solution.optimal_control if mpc_solution is not None else u_lqr
            feas_margin = mpc_solution.feasibility_margin if mpc_solution is not None else 0.0
            
            u, blend_info = blender.blend(
                u_lqr=u_lqr, u_mpc=u_mpc,
                risk=assessment.combined_risk,
                solver_status=solver_status,
                solver_time_ms=solver_time_ms,
                feasibility_margin=feas_margin
            )
            blend_weights[k] = blend_info.weight
        
        else:
            raise ValueError(f"Unknown mode: {mode}")
        
        # -- Apply Actuator Dynamics (delay + lag) --
        # actuator.update() handles delay buffer and first-order lag
        v_applied, omega_applied = actuator.update(u[0], u[1])
        u_applied = np.array([v_applied, omega_applied])
        
        controls[k] = u  # Log COMMANDED control
        
        # -- Simulate step --
        x = robot.simulate_step(x, u_applied, dt.real if hasattr(dt, 'real') else dt)
        
        # -- Check collisions --
        for obs in obstacles:
            dist = np.sqrt((x[0] - obs.x)**2 + (x[1] - obs.y)**2)
            if dist < obs.radius:
                collision_count += 1
    
    # -- Compute metrics --
    wall_time = time.perf_counter() - start_wall
    
    # Jerk metrics
    jerk_metrics = SimulationLogger.compute_jerk_metrics(controls, dt)
    
    # Control effort
    total_effort = float(np.sum(np.abs(controls)))
    
    # Blend stats
    bw_mean, bw_std, n_transitions = 0.0, 0.0, 0
    if mode == 'hybrid':
        stats = blender.get_statistics()
        bw_mean = stats.get('weight_mean', 0.0)
        bw_std = stats.get('weight_std', 0.0)
        n_transitions = stats.get('total_switches', 0)
    
    return RunMetrics(
        mode=mode,
        config_id=config_id,
        seed=obstacle_config.seed,
        mean_tracking_error=float(np.mean(errors)),
        max_tracking_error=float(np.max(errors)),
        final_tracking_error=float(errors[-1]),
        mean_solve_time_ms=float(np.mean(solve_times)) if solve_times else 0.0,
        max_solve_time_ms=float(np.max(solve_times)) if solve_times else 0.0,
        linear_jerk_rms=jerk_metrics.get('linear_jerk_rms', 0.0),
        angular_jerk_rms=jerk_metrics.get('angular_jerk_rms', 0.0),
        linear_jerk_peak=jerk_metrics.get('linear_jerk_peak', 0.0),
        angular_jerk_peak=jerk_metrics.get('angular_jerk_peak', 0.0),
        infeasible_count=infeasible_count,
        collision_count=collision_count,
        completion_fraction=1.0,  # All configs run full duration
        total_control_effort=total_effort,
        blend_weight_mean=bw_mean,
        blend_weight_std=bw_std,
        smooth_transitions=n_transitions,
        wall_time_s=wall_time
    )


# -- Aggregation -----------------------------------------------------

def aggregate_results(runs: List[RunMetrics], mode: str) -> AggregatedResults:
    """
    Compute mean ± std and percentiles for all metrics across runs.
    
    Args:
        runs: List of RunMetrics for a single mode
        mode: Controller mode name
        
    Returns:
        AggregatedResults with per-metric statistics
    """
    metric_names = [
        'mean_tracking_error', 'max_tracking_error', 'final_tracking_error',
        'mean_solve_time_ms', 'max_solve_time_ms',
        'linear_jerk_rms', 'angular_jerk_rms', 
        'linear_jerk_peak', 'angular_jerk_peak',
        'infeasible_count', 'collision_count',
        'total_control_effort', 'wall_time_s'
    ]
    
    if mode == 'hybrid':
        metric_names += ['blend_weight_mean', 'blend_weight_std', 'smooth_transitions']
    
    metrics = {}
    for name in metric_names:
        values = np.array([getattr(r, name) for r in runs])
        metrics[name] = {
            'mean': float(np.mean(values)),
            'std': float(np.std(values)),
            'min': float(np.min(values)),
            'max': float(np.max(values)),
            'p5': float(np.percentile(values, 5)),
            'p95': float(np.percentile(values, 95)),
        }
    
    return AggregatedResults(mode=mode, n_runs=len(runs), metrics=metrics)


# -- Table formatting ------------------------------------------------

def format_comparison_table(results: Dict[str, AggregatedResults]) -> str:
    """
    Format a comparison table of all controller modes.
    
    Returns:
        Formatted table string for console output.
    """
    # Key metrics for comparison
    display_metrics = [
        ('mean_tracking_error', 'Mean Track Err (m)', '.4f'),
        ('final_tracking_error', 'Final Track Err (m)', '.4f'),
        ('linear_jerk_rms', 'Lin Jerk RMS', '.1f'),
        ('angular_jerk_rms', 'Ang Jerk RMS', '.1f'),
        ('mean_solve_time_ms', 'Solve Time (ms)', '.2f'),
        ('infeasible_count', 'Infeasible', '.1f'),
        ('collision_count', 'Collisions', '.1f'),
        ('total_control_effort', 'Ctrl Effort', '.1f'),
    ]
    
    modes = list(results.keys())
    
    # Header
    header = f"{'Metric':<24}"
    for mode in modes:
        header += f" | {mode:^22}"
    
    separator = "-" * len(header)
    
    lines = [
        "",
        "=" * len(header),
        "Statistical Comparison Results",
        "=" * len(header),
        f"Runs per mode: {results[modes[0]].n_runs}",
        "",
        header,
        separator,
    ]
    
    for metric_key, metric_name, fmt in display_metrics:
        row = f"{metric_name:<24}"
        for mode in modes:
            if metric_key in results[mode].metrics:
                m = results[mode].metrics[metric_key]
                row += f" | {m['mean']:{fmt}} ± {m['std']:{fmt}}"
                row += " " * max(0, 22 - len(f"{m['mean']:{fmt}} ± {m['std']:{fmt}}"))
            else:
                row += f" | {'N/A':^22}"
        lines.append(row)
    
    lines.append(separator)
    lines.append("")
    
    return "\n".join(lines)


# -- Main runner -----------------------------------------------------

def run_statistical_validation(
    n_configs: int = 50,
    modes: List[str] = None,
    duration: float = 20.0,
    dt: float = 0.02,
    noise_std: float = 0.0,
    heading_noise_std: float = 0.0,
    delay_steps: int = 0,
    tau: float = 0.0,
    actuator_noise: float = 0.0,
    scenario_type: str = 'random',
    base_seed: int = 42,
    verbose: bool = True,
    output_dir: str = 'evaluation/results'
) -> Dict[str, AggregatedResults]:
    """
    Run full Monte Carlo validation across all controller modes.
    
    Args:
        n_configs: Number of randomized obstacle configs
        modes: List of controller modes to compare
        duration: Simulation duration per run
        dt: Time step
        noise_std: Position noise standard deviation (meters)
        heading_noise_std: Heading noise std (radians)
        delay_steps: Control pipeline delay (timesteps)
        base_seed: Base random seed
        scenario_type: Type of obstacle scenario (random, corridor, bugtrap, dense)
        verbose: Print progress
        output_dir: Directory for output files
        
    Returns:
        Dictionary mapping mode -> AggregatedResults
    """
    if modes is None:
        modes = ['lqr', 'mpc', 'hard_switch', 'hybrid']
    
    noise_config = NoiseConfig(
        position_noise_std=noise_std,
        heading_noise_std=heading_noise_std,
        control_delay_steps=delay_steps,
        tau_v=tau, tau_omega=tau,
        actuator_noise_std=actuator_noise
    )
    
    # Generate obstacle configs (shared across all modes)
    if verbose:
        print(f"Generating {n_configs} obstacle configurations (type={scenario_type}, seed={base_seed})...")
    
    generator = get_generator(scenario_type)
    # Instantiate the appropriate generator based on name
    if scenario_type == 'corridor':
        from evaluation.scenarios import CorridorScenario
        generator = CorridorScenario()
    elif scenario_type == 'bugtrap':
        from evaluation.scenarios import BugTrapScenario
        generator = BugTrapScenario()
    elif scenario_type == 'dense':
        from evaluation.scenarios import DenseClutterScenario
        generator = DenseClutterScenario()
    else:
        from evaluation.scenarios import RandomScenario
        generator = RandomScenario()
        
    configs = generator.generate(n_configs, base_seed)
    
    # Run all modes
    all_results: Dict[str, List[RunMetrics]] = {mode: [] for mode in modes}
    total_runs = n_configs * len(modes)
    completed = 0
    
    for mode in modes:
        if verbose:
            print(f"\n{'-'*50}")
            print(f"Running mode: {mode.upper()} ({n_configs} configs)")
            print(f"{'-'*50}")
        
        for i, config in enumerate(configs):
            try:
                metrics = run_single_config(
                    mode=mode,
                    obstacle_config=config,
                    noise_config=noise_config,
                    duration=duration,
                    dt=dt,
                    config_id=i
                )
                all_results[mode].append(metrics)
            except Exception as e:
                if verbose:
                    print(f"  Config {i} FAILED: {e}")
                # Create a failure entry
                all_results[mode].append(RunMetrics(
                    mode=mode, config_id=i, seed=config.seed,
                    mean_tracking_error=float('inf'),
                    max_tracking_error=float('inf'),
                    final_tracking_error=float('inf'),
                    mean_solve_time_ms=0.0, max_solve_time_ms=0.0,
                    linear_jerk_rms=0.0, angular_jerk_rms=0.0,
                    linear_jerk_peak=0.0, angular_jerk_peak=0.0,
                    infeasible_count=999, collision_count=999,
                    completion_fraction=0.0, total_control_effort=0.0
                ))
            
            completed += 1
            if verbose and (i + 1) % max(1, n_configs // 10) == 0:
                pct = 100 * completed / total_runs
                print(f"  Progress: {i+1}/{n_configs} ({pct:.0f}% overall)")
    
    # Aggregate results
    aggregated = {}
    for mode in modes:
        # Filter out failed runs
        valid_runs = [r for r in all_results[mode] 
                      if r.mean_tracking_error < float('inf')]
        if valid_runs:
            aggregated[mode] = aggregate_results(valid_runs, mode)
        else:
            print(f"WARNING: No valid runs for mode '{mode}'")
    
    # Print table
    if verbose:
        print(format_comparison_table(aggregated))
    
    # Save results
    os.makedirs(output_dir, exist_ok=True)
    
    # Save JSON
    json_data = {
        'metadata': {
            'n_configs': n_configs,
            'duration': duration,
            'dt': dt,
            'noise_config': asdict(noise_config),
            'base_seed': base_seed,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        },
        'results': {}
    }
    for mode, agg in aggregated.items():
        json_data['results'][mode] = {
            'n_runs': agg.n_runs,
            'metrics': agg.metrics
        }
    
    json_path = os.path.join(output_dir, 'statistical_results.json')
    with open(json_path, 'w') as f:
        json.dump(json_data, f, indent=2)
    if verbose:
        print(f"Results saved to {json_path}")
    
    # Save CSV
    csv_path = os.path.join(output_dir, 'statistical_results.csv')
    with open(csv_path, 'w') as f:
        # Header: mode, n_runs, then for each metric: mean, std
        metric_keys = list(next(iter(aggregated.values())).metrics.keys())
        header_parts = ['mode', 'n_runs']
        for mk in metric_keys:
            header_parts.extend([f'{mk}_mean', f'{mk}_std'])
        f.write(','.join(header_parts) + '\n')
        
        for mode, agg in aggregated.items():
            row_parts = [mode, str(agg.n_runs)]
            for mk in metric_keys:
                if mk in agg.metrics:
                    row_parts.extend([
                        f"{agg.metrics[mk]['mean']:.6f}",
                        f"{agg.metrics[mk]['std']:.6f}"
                    ])
                else:
                    row_parts.extend(['0', '0'])
            f.write(','.join(row_parts) + '\n')
    if verbose:
        print(f"CSV saved to {csv_path}")
    
    # Save per-run CSV for detailed analysis
    perrun_path = os.path.join(output_dir, 'per_run_results.csv')
    with open(perrun_path, 'w') as f:
        fields = [
            'mode', 'config_id', 'seed',
            'mean_tracking_error', 'max_tracking_error', 'final_tracking_error',
            'mean_solve_time_ms', 'max_solve_time_ms',
            'linear_jerk_rms', 'angular_jerk_rms',
            'linear_jerk_peak', 'angular_jerk_peak',
            'infeasible_count', 'collision_count',
            'total_control_effort', 'blend_weight_mean', 'smooth_transitions',
            'wall_time_s'
        ]
        f.write(','.join(fields) + '\n')
        
        for mode in modes:
            for r in all_results[mode]:
                if r.mean_tracking_error < float('inf'):
                    values = [str(getattr(r, field)) for field in fields]
                    f.write(','.join(values) + '\n')
    if verbose:
        print(f"Per-run data saved to {perrun_path}")
    
    return aggregated


# -- CLI -------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Monte Carlo statistical validation of controller modes',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('--configs', type=int, default=50,
                        help='Number of randomized obstacle configs')
    parser.add_argument('--modes', nargs='+', 
                        default=['lqr', 'mpc', 'hard_switch', 'hybrid'],
                        choices=['lqr', 'mpc', 'hard_switch', 'hybrid'],
                        help='Controller modes to compare')
    parser.add_argument('--duration', type=float, default=20.0,
                        help='Simulation duration (seconds)')
    parser.add_argument('--noise', type=float, default=0.0,
                        help='Position noise std (meters)')
    parser.add_argument('--heading-noise', type=float, default=0.0,
                        help='Heading noise std (radians)')
    parser.add_argument('--delay', type=int, default=0,
                        help='Control pipeline delay (timesteps)')
    parser.add_argument('--tau', type=float, default=0.0,
                        help='Actuator time constant (seconds)')
    parser.add_argument('--actuator-noise', type=float, default=0.0,
                        help='Actuator execution noise std')
    parser.add_argument('--scenario', type=str, default='random',
                        choices=['random', 'corridor', 'bugtrap', 'dense'],
                        help='Scenario type')
    parser.add_argument('--seed', type=int, default=42,
                        help='Base random seed')
    parser.add_argument('--output', type=str, default='evaluation/results',
                        help='Output directory')
    parser.add_argument('--quiet', action='store_true',
                        help='Suppress progress output')
    
    args = parser.parse_args()
    
    run_statistical_validation(
        n_configs=args.configs,
        modes=args.modes,
        duration=args.duration,
        noise_std=args.noise,
        heading_noise_std=args.heading_noise,
        delay_steps=args.delay,
        tau=args.tau,
        actuator_noise=args.actuator_noise,
        scenario_type=args.scenario,
        base_seed=args.seed,
        verbose=not args.quiet,
        output_dir=args.output
    )


if __name__ == '__main__':
    main()
