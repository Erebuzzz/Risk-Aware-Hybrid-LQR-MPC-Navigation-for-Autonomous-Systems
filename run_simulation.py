#!/usr/bin/env python3
"""
Standalone Simulation
=====================

Run the LQR, MPC, and Hybrid simulations without ROS2 dependencies.
Useful for testing, validation, and generating plots.

Usage:
    python run_simulation.py --mode lqr --trajectory figure8
    python run_simulation.py --mode mpc --trajectory clover3 --scenario dense
    python run_simulation.py --mode adaptive
    python run_simulation.py --mode hybrid --trajectory clover3
    python run_simulation.py --mode compare --trajectory rose4
"""

import argparse
import os
import sys

import numpy as np

# Add the package to path (works without pip install -e)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "ros2_ws", "src", "hybrid_controller"))

from hybrid_controller.controllers.adaptive_mpc_controller import AdaptiveMPCController
from hybrid_controller.controllers.hybrid_blender import BlendingSupervisor
from hybrid_controller.controllers.lqr_controller import LQRController
from hybrid_controller.controllers.mpc_controller import MPCController, Obstacle
from hybrid_controller.controllers.risk_metrics import RiskMetrics
from hybrid_controller.controllers.yaw_stabilizer import YawStabilizer
from hybrid_controller.logging.simulation_logger import SimulationLogger
from hybrid_controller.models.actuator_dynamics import ActuatorDynamics, ActuatorParams
from hybrid_controller.models.differential_drive import DifferentialDriveRobot
from hybrid_controller.trajectory.trajectory_factory import TrajectoryFactory
from hybrid_controller.utils.visualization import Visualizer

PLOTS_ROOT = os.path.join("results")


def _plots_sub(*parts: str) -> str:
    return os.path.join(PLOTS_ROOT, *parts)


def _generate_factory_trajectory(
    trajectory_type: str, duration: float, dt: float
) -> tuple:
    """
    Discrete reference from TrajectoryFactory with index-based accessors.

    Returns:
        trajectory: [t, px, py, theta, v, omega]
        N: sample count
        get_reference_at_index: (k) -> (x_ref [3], u_ref [2])
        get_trajectory_segment: (start_k, length) -> (x_refs, u_refs) padded
    """
    factory = TrajectoryFactory()
    trajectory = factory.generate(
        trajectory_type,
        duration=duration,
        dt=dt,
        A=2.0,
        seed=42 if trajectory_type == "random_wp" else None,
    )
    ref_states = trajectory[:, 1:4]
    ref_controls = trajectory[:, 4:6]
    n_samples = len(trajectory)

    def get_reference_at_index(index: int):
        idx = min(max(index, 0), n_samples - 1)
        return ref_states[idx].copy(), ref_controls[min(idx, n_samples - 2)].copy()

    def get_trajectory_segment(start_index: int, length: int):
        end_index = min(start_index + length, n_samples)
        x_refs = ref_states[start_index:end_index].copy()
        u_refs = ref_controls[
            start_index : min(start_index + length - 1, n_samples - 1)
        ].copy()
        while len(x_refs) < length:
            x_refs = np.vstack([x_refs, x_refs[-1:]])
        while len(u_refs) < length - 1:
            u_refs = np.vstack([u_refs, u_refs[-1:]])
        return x_refs, u_refs

    return trajectory, n_samples, get_reference_at_index, get_trajectory_segment


def run_lqr_simulation(
    duration: float = 20.0,
    dt: float = 0.02,
    visualize: bool = True,
    trajectory_type: str = "figure8",
) -> dict:
    """
    Run LQR trajectory tracking simulation.

    Args:
        duration: Simulation duration (seconds)
        dt: Time step (seconds)
        visualize: Generate plots
        trajectory_type: Geometry label (matches ``--trajectory``); plots go under
            ``results/LQR/<trajectory_type>/``.

    Returns:
        Dictionary with simulation results
    """
    print("=" * 60)
    print("LQR Trajectory Tracking Simulation")
    print("=" * 60)

    # Initialize components
    robot = DifferentialDriveRobot(
        v_max=2.0, omega_max=3.0
    )  # Increased limits to prevent saturation
    lqr = LQRController(
        Q_diag=[15.0, 15.0, 8.0], R_diag=[0.1, 0.1], dt=dt, v_max=2.0, omega_max=3.0
    )
    logger = SimulationLogger(log_dir="logs", log_level="INFO", node_name="lqr_sim")

    trajectory, n_samples, get_reference_at_index, _ = _generate_factory_trajectory(
        trajectory_type, duration, dt
    )

    print(
        f"Trajectory: {trajectory_type} | "
        f"Generated {n_samples} trajectory points over {duration}s"
    )

    # Initialize state at the reference trajectory's starting point
    x_ref_init, _ = get_reference_at_index(0)
    x = x_ref_init.copy()  # Start at reference position and heading

    # Storage
    states = np.zeros((n_samples, 3))
    controls = np.zeros((n_samples - 1, 2))
    errors = np.zeros((n_samples - 1, 3))

    states[0] = x

    # Simulation loop
    for k in range(n_samples - 1):
        # Get reference
        x_ref, u_ref = get_reference_at_index(k)

        # Compute LQR control
        u, error = lqr.compute_control_at_operating_point(x, x_ref, u_ref)

        # Log
        logger.log_state(k, x, x_ref, error)
        logger.log_control(k, u, "LQR")

        # Simulate robot
        x = robot.simulate_step(x, u, dt)

        # Store
        states[k + 1] = x
        controls[k] = u
        errors[k] = error

        if k % 100 == 0:
            error_norm = np.linalg.norm(error[:2])
            print(
                f"  k={k:4d}: error_norm={error_norm:.4f}, u=[{u[0]:.3f}, {u[1]:.3f}]"
            )

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
        lqr_dir = _plots_sub("LQR", trajectory_type)
        os.makedirs(lqr_dir, exist_ok=True)
        viz = Visualizer(output_dir=lqr_dir)
        ref_states = trajectory[:, 1:4]  # [px, py, theta]

        viz.plot_trajectory(
            states,
            ref_states,
            title="LQR Trajectory Tracking",
            save_path=os.path.join(lqr_dir, "lqr_tracking.png"),
        )

        viz.plot_tracking_error(
            errors,
            dt,
            title="LQR Tracking Error",
            save_path=os.path.join(lqr_dir, "lqr_error.png"),
        )

        viz.plot_control_inputs(
            controls,
            dt,
            v_max=2.0,
            omega_max=3.0,
            title="LQR Control Inputs",
            save_path=os.path.join(lqr_dir, "lqr_control.png"),
        )

        print(f"\nPlots saved to {lqr_dir}")

    return {
        "states": states,
        "controls": controls,
        "errors": errors,
        "reference": trajectory[:, 1:4],
        "mean_error": mean_error,
        "final_error": final_error,
    }


def run_mpc_simulation(
    duration: float = 20.0,
    dt: float = 0.02,
    with_obstacles: bool = True,
    visualize: bool = True,
    scenario: str = "default",
    trajectory_type: str = "figure8",
) -> dict:
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
    # OPTIMIZED MPC PARAMETERS for industry-standard tolerances
    # Target: heading ≤5°, latency ≤50ms, slack ≤5
    mpc = MPCController(
        horizon=6,  # Reduced: 10→6 for faster solves
        Q_diag=[15.0, 15.0, 50.0],  # Aggressive heading: 20→50
        R_diag=[0.1, 0.1],
        P_diag=[30.0, 30.0, 40.0],  # Strong terminal heading: 15→40
        d_safe=0.3,
        slack_penalty=5000.0,  # Stricter: 1000→5000
        dt=dt,
        v_max=2.0,
        omega_max=3.0,
        solver="OSQP",  # Faster: ECOS→OSQP
        block_size=2,  # Move-blocking: halves decision vars
        w_max=0.05,  # Tube MPC: disturbance bound (5cm)
    )

    # Yaw stabilizer for cold-start bootstrap (first N steps)
    yaw_stabilizer = YawStabilizer(kp=3.0, ki=0.1, kd=0.5, dt=dt, omega_max=3.0)
    BOOTSTRAP_STEPS = 10  # Use yaw stabilizer for first 10 steps

    logger = SimulationLogger(log_dir="logs", log_level="INFO", node_name="mpc_sim")

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
        print(
            f"Scenario: {scenario} | Trajectory: {trajectory_type} | "
            f"Added {len(obstacles)} obstacles"
        )
    else:
        obstacles = []

    trajectory, N, get_reference_at_index, get_trajectory_segment = (
        _generate_factory_trajectory(trajectory_type, duration, dt)
    )

    print(f"Generated {N} trajectory points over {duration}s")

    # Initialize state at the reference trajectory's starting point
    x_ref_init, _ = get_reference_at_index(0)
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
        x_refs, u_refs = get_trajectory_segment(k, mpc.N + 1)
        # We only run the MPC optimization every `mpc_rate` steps to simulate
        # a real-time system where the solver runs slower than the control loop.
        # However, for this simple simulation, we run it every step but use
        # warm starting.
        solution = mpc.solve(
            x0=x,
            x_refs=x_refs,
            u_refs=u_refs,
            obstacles=obstacles,
            use_soft_constraints=True
        )

        solve_times.append(solution.solve_time_ms)

        if solution.slack_used:
            logger.log_constraint_event(
                k, "slack_activated", {"reason": "feasibility"}
            )

        u = solution.optimal_control
        controller_name = "MPC"
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
            ctrl_info = (
                f"solve={solve_time:.2f}ms" if controller_name == "MPC" else "bootstrap"
            )
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
    plot_scenario = scenario if with_obstacles else "no_obstacles"
    if visualize:
        mpc_dir = _plots_sub("MPC", trajectory_type, plot_scenario)
        os.makedirs(mpc_dir, exist_ok=True)
        viz = Visualizer(output_dir=mpc_dir)
        ref_states = trajectory[:, 1:4]

        obstacle_dicts = [{"x": o.x, "y": o.y, "radius": o.radius} for o in obstacles]

        viz.plot_with_obstacles(
            states,
            ref_states,
            obstacle_dicts,
            mpc.d_safe,
            title="MPC Obstacle Avoidance",
            save_path=os.path.join(mpc_dir, "mpc_obstacle_avoidance.png"),
        )

        viz.plot_tracking_error(
            errors,
            dt,
            title="MPC Tracking Error",
            save_path=os.path.join(mpc_dir, "mpc_error.png"),
        )

        viz.plot_control_inputs(
            controls,
            dt,
            v_max=2.0,
            omega_max=3.0,
            title="MPC Control Inputs",
            save_path=os.path.join(mpc_dir, "mpc_control.png"),
        )

        print(f"\nPlots saved to {mpc_dir}")

    return {
        "states": states,
        "controls": controls,
        "errors": errors,
        "reference": trajectory[:, 1:4],
        "mean_error": mean_error,
        "collision_count": collision_count,
        "mean_solve_time": mean_solve_time,
    }


def run_adaptive_simulation(
    duration: float = 20.0,
    dt: float = 0.02,
    visualize: bool = True,
    scenario: str = "default",
    trajectory_type: str = "figure8",
) -> dict:
    """
    Run Adaptive NMPC (CasADi) trajectory tracking with obstacle avoidance.

    Uses the same reference generator and scenario layout as ``run_mpc_simulation``.
    """
    print("=" * 60)
    print("Adaptive MPC Obstacle Avoidance Simulation")
    print("=" * 60)

    robot = DifferentialDriveRobot(v_max=2.0, omega_max=3.0)
    ampc = AdaptiveMPCController(
        prediction_horizon=5,
        terminal_horizon=3,
        Q_diag=[30.0, 30.0, 5.0],
        R_diag=[0.1, 0.1],
        d_safe=0.3,
        v_max=2.0,
        omega_max=3.0,
        dt=dt,
        enable_adaptation=True,
        adaptation_gamma=0.005,
    )

    yaw_stabilizer = YawStabilizer(kp=3.0, ki=0.1, kd=0.5, dt=dt, omega_max=3.0)
    BOOTSTRAP_STEPS = 10

    logger = SimulationLogger(
        log_dir="logs", log_level="INFO", node_name="adaptive_mpc_sim"
    )

    if scenario == "sparse":
        obstacles = [
            Obstacle(x=1.5, y=0.8, radius=0.2),
        ]
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
    else:
        obstacles = [
            Obstacle(x=1.0, y=0.5, radius=0.2),
            Obstacle(x=-0.5, y=-1.0, radius=0.25),
            Obstacle(x=1.5, y=-0.3, radius=0.15),
        ]
    print(
        f"Scenario: {scenario} | Trajectory: {trajectory_type} | "
        f"Added {len(obstacles)} obstacles"
    )

    trajectory, N, get_reference_at_index, _ = _generate_factory_trajectory(
        trajectory_type, duration, dt
    )

    print(f"Generated {N} trajectory points over {duration}s")

    x_ref_init, _ = get_reference_at_index(0)
    x = x_ref_init.copy()

    states = np.zeros((N, 3))
    controls = np.zeros((N - 1, 2))
    errors = np.zeros((N - 1, 3))
    solve_times = []

    states[0] = x

    mpc_rate = 5
    solution = None

    for k in range(N - 1):
        x_prev = x.copy()
        x_ref_k, u_ref_k = get_reference_at_index(k)

        if k < BOOTSTRAP_STEPS:
            omega_cmd = yaw_stabilizer.compute(x[2], x_ref_k[2], float(u_ref_k[1]))
            u = np.array([float(u_ref_k[0]), omega_cmd], dtype=float)
            solve_time = 0.0
        else:
            if solution is None or k % mpc_rate == 0:
                lookahead = min(ampc.N_ext + 1, N - k)
                x_refs = np.array(
                    [get_reference_at_index(k + j)[0] for j in range(lookahead)]
                )
                u_refs = np.array(
                    [
                        get_reference_at_index(min(k + j, N - 2))[1][:2]
                        for j in range(lookahead - 1)
                    ]
                )
                solution = ampc.solve_tracking(x, x_refs, u_refs, obstacles)
                solve_times.append(solution.solve_time_ms)

            u = solution.optimal_control
            solve_time = solution.solve_time_ms

        error = x - x_ref_k
        error[2] = robot.normalize_angle(error[2])

        logger.log_state(k, x, x_ref_k, error)
        logger.log_control(k, u, "AdaptiveMPC", solve_time)

        x = robot.simulate_step(x, u, dt)

        if k > 0 and k >= BOOTSTRAP_STEPS:
            ampc.adapt_parameters(x, x_prev, u)

        states[k + 1] = x
        controls[k] = u
        errors[k] = error

        if k % 100 == 0:
            error_norm = np.linalg.norm(error[:2])
            print(f"  k={k:4d}: error_norm={error_norm:.4f}, solve={solve_time:.2f}ms")

    mean_error = np.mean(np.linalg.norm(errors[:, :2], axis=1))
    final_error = np.linalg.norm(errors[-1, :2])
    mean_solve_time = np.mean(solve_times) if solve_times else 0.0

    print(f"\nResults:")
    print(f"  Mean tracking error: {mean_error:.4f} m")
    print(f"  Final tracking error: {final_error:.4f} m")
    print(f"  Mean Adaptive MPC solve time: {mean_solve_time:.2f} ms")

    collision_count = 0
    for state in states:
        for obs in obstacles:
            if obs.is_collision(state[0], state[1], ampc.d_safe):
                collision_count += 1
                break

    print(f"  Collision events: {collision_count}")

    logger.finalize()

    if visualize:
        ampc_dir = _plots_sub("AdaptiveMPC", trajectory_type, scenario)
        os.makedirs(ampc_dir, exist_ok=True)
        viz = Visualizer(output_dir=ampc_dir)
        ref_states = trajectory[:, 1:4]
        obstacle_dicts = [{"x": o.x, "y": o.y, "radius": o.radius} for o in obstacles]

        viz.plot_with_obstacles(
            states,
            ref_states,
            obstacle_dicts,
            ampc.d_safe,
            title="Adaptive MPC Obstacle Avoidance",
            save_path=os.path.join(ampc_dir, "adaptive_mpc_obstacle_avoidance.png"),
        )

        viz.plot_tracking_error(
            errors,
            dt,
            title="Adaptive MPC Tracking Error",
            save_path=os.path.join(ampc_dir, "adaptive_mpc_error.png"),
        )

        viz.plot_control_inputs(
            controls,
            dt,
            v_max=2.0,
            omega_max=3.0,
            title="Adaptive MPC Control Inputs",
            save_path=os.path.join(ampc_dir, "adaptive_mpc_control.png"),
        )

        print(f"\nPlots saved to {ampc_dir}")

    return {
        "states": states,
        "controls": controls,
        "errors": errors,
        "reference": trajectory[:, 1:4],
        "mean_error": mean_error,
        "collision_count": collision_count,
        "mean_solve_time": mean_solve_time,
    }


def run_comparison(
    duration: float = 20.0, dt: float = 0.02, trajectory_type: str = "figure8"
) -> None:
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
    obstacle_dicts = [{"x": o.x, "y": o.y, "radius": o.radius} for o in obstacles]

    # Run LQR (without visibility of obstacles)
    print("\n--- Running LQR (obstacle-unaware) ---")
    robot = DifferentialDriveRobot(v_max=2.0, omega_max=3.0)
    lqr = LQRController(
        Q_diag=[15.0, 15.0, 8.0], R_diag=[0.1, 0.1], dt=dt, v_max=2.0, omega_max=3.0
    )
    trajectory, N, get_reference_at_index, get_trajectory_segment = (
        _generate_factory_trajectory(trajectory_type, duration, dt)
    )

    x_ref_init, _ = get_reference_at_index(0)
    x_lqr = x_ref_init.copy()
    lqr_states = np.zeros((N, 3))
    lqr_states[0] = x_lqr

    for k in range(N - 1):
        x_ref, u_ref = get_reference_at_index(k)
        u, _ = lqr.compute_control_at_operating_point(x_lqr, x_ref, u_ref)
        x_lqr = robot.simulate_step(x_lqr, u, dt)
        lqr_states[k + 1] = x_lqr

    # Check LQR collisions
    lqr_collisions = sum(
        1
        for s in lqr_states
        for o in obstacles
        if o.distance_to(s[0], s[1]) < o.radius + 0.3
    )
    print(f"LQR collision events: {lqr_collisions}")

    # Run MPC (obstacle-aware) with optimized weights
    print("\n--- Running MPC (obstacle-aware) ---")
    mpc = MPCController(
        horizon=6,
        Q_diag=[15.0, 15.0, 50.0],  # Optimized heading
        R_diag=[0.1, 0.1],
        P_diag=[30.0, 30.0, 40.0],  # Strong terminal heading
        d_safe=0.3,
        slack_penalty=5000.0,
        dt=dt,
        v_max=2.0,
        omega_max=3.0,
        solver="OSQP",
    )

    x_mpc = x_ref_init.copy()  # Use same initial state as LQR
    mpc_states = np.zeros((N, 3))
    mpc_states[0] = x_mpc

    for k in range(N - 1):
        x_refs, u_refs = get_trajectory_segment(k, mpc.N + 1)
        solution = mpc.solve_with_ltv(x_mpc, x_refs, u_refs, obstacles)
        x_mpc = robot.simulate_step(x_mpc, solution.optimal_control, dt)
        mpc_states[k + 1] = x_mpc

    mpc_collisions = sum(
        1
        for s in mpc_states
        for o in obstacles
        if o.distance_to(s[0], s[1]) < o.radius + 0.3
    )
    print(f"MPC collision events: {mpc_collisions}")

    # Comparison plot
    compare_dir = _plots_sub("Compare", trajectory_type)
    os.makedirs(compare_dir, exist_ok=True)
    comparison_path = os.path.join(compare_dir, "comparison.png")
    viz = Visualizer(output_dir=compare_dir)
    viz.plot_comparison(
        lqr_states,
        mpc_states,
        trajectory[:, 1:4],
        obstacle_dicts,
        d_safe=0.3,
        title="LQR vs MPC: Obstacle Avoidance Comparison",
        save_path=comparison_path,
    )

    print(f"\nComparison plot saved to {comparison_path}")


def run_hybrid_simulation(
    duration: float = 20.0,
    dt: float = 0.02,
    visualize: bool = True,
    scenario: str = "default",
    trajectory_type: str = "figure8",
    output_dir: str = None,
    actuator_params: ActuatorParams = None,
) -> dict:
    """
    Run risk-aware hybrid LQR-MPC simulation with continuous blending.

    Uses BlendingSupervisor for smooth control arbitration:
        u = w(t) * u_mpc + (1 - w(t)) * u_lqr

    where w(t) is computed from risk metrics with anti-chatter guarantees.

    Args:
        duration: Simulation duration (seconds)
        dt: Time step (seconds)
        visualize: Generate plots
        scenario: Obstacle scenario
        trajectory_type: Trajectory family to generate
        output_dir: Override plot directory (default: results/Hybrid/<trajectory>/<scenario>/)
        actuator_params: Optional parameters for hardware realism (lag, delay, noise)

    Returns:
        Dictionary with simulation results including blend weights and jerk metrics
    """
    print("=" * 60)
    print("Hybrid LQR-MPC Smooth Blending Simulation")
    print("=" * 60)

    # Initialize components
    robot = DifferentialDriveRobot(v_max=2.0, omega_max=3.0)
    traj_factory = TrajectoryFactory()

    # LQR for low-risk regions (efficient, stable)
    lqr = LQRController(
        Q_diag=[15.0, 15.0, 8.0], R_diag=[0.1, 0.1], dt=dt, v_max=2.0, omega_max=3.0
    )

    # MPC for high-risk regions (obstacle-aware, optimal)
    mpc = MPCController(
        horizon=5,
        Q_diag=[80.0, 80.0, 120.0],
        R_diag=[0.1, 0.1],
        P_diag=[20.0, 20.0, 40.0],
        S_diag=[0.1, 0.5],
        J_diag=[0.05, 0.3],  # Second-order jerk penalty (Phase 5A)
        d_safe=0.3,
        slack_penalty=5000.0,
        dt=dt,
        v_max=2.0,
        omega_max=3.0,
        solver="OSQP",
        block_size=2,
        w_max=0.05,
    )

    # Risk metrics
    risk_metrics = RiskMetrics(
        d_safe=0.35,
        d_trigger=3.0,
        alpha=0.6,
        beta=0.4,
        threshold_low=0.2,
        threshold_medium=0.5,
    )

    # Blending supervisor (core novelty)
    blender = BlendingSupervisor(
        k_sigmoid=8.0,
        risk_threshold=0.25,
        dw_max=2.0,
        hysteresis_band=0.08,
        solver_time_limit=50.0,
        feasibility_decay=0.8,
        dt=dt,
    )
    # Diagnostic dominance bands: w < 0.25 is mostly LQR, w > 0.75 is mostly MPC.
    blender._lqr_dominant_threshold = 0.25
    blender._mpc_dominant_threshold = 0.75

    logger = SimulationLogger(log_dir="logs", log_level="INFO", node_name="hybrid_sim")

    # Initialize Actuator Dynamics if params are provided
    actuator = None
    if actuator_params:
        actuator = ActuatorDynamics(actuator_params, dt)

    # Define obstacles based on scenario
    moving_obstacle_specs = []
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
    elif scenario == "moving":
        obstacles = [
            Obstacle(x=1.0, y=0.5, radius=0.2),
            Obstacle(x=-0.5, y=-1.0, radius=0.25),
            Obstacle(x=1.5, y=-0.3, radius=0.15),
        ]
        moving_obstacle_specs = [
            (obstacles[0], 1.0, 0.5, 0.35, 0.00, 0.12),
            (obstacles[1], -0.5, -1.0, 0.00, 0.30, 0.10),
            (obstacles[2], 1.5, -0.3, 0.25, 0.20, 0.08),
        ]
    else:  # default
        obstacles = [
            Obstacle(x=1.0, y=0.5, radius=0.2),
            Obstacle(x=-0.5, y=-1.0, radius=0.25),
            Obstacle(x=1.5, y=-0.3, radius=0.15),
        ]

    obstacle_dicts = [{"x": o.x, "y": o.y, "radius": o.radius} for o in obstacles]
    print(
        f"Scenario: {scenario} | Trajectory: {trajectory_type} | Added {len(obstacles)} obstacles"
    )

    # Generate trajectory. This is the critical path fix for generate_hybrid_scenarios.py:
    # the CLI --trajectory value now actually changes the hybrid reference path.
    trajectory = traj_factory.generate(
        trajectory_type,
        duration=duration,
        dt=dt,
        A=2.0,
        seed=42 if trajectory_type == "random_wp" else None,
    )
    ref_states = trajectory[:, 1:4]
    ref_controls = trajectory[:, 4:6]
    N = len(trajectory)

    def get_reference_at_index(index: int):
        idx = min(max(index, 0), N - 1)
        return ref_states[idx].copy(), ref_controls[min(idx, N - 2)].copy()

    def get_trajectory_segment(start_index: int, length: int):
        end_index = min(start_index + length, N)
        x_refs = ref_states[start_index:end_index].copy()
        u_refs = ref_controls[start_index : min(start_index + length - 1, N - 1)].copy()

        while len(x_refs) < length:
            x_refs = np.vstack([x_refs, x_refs[-1:]])
        while len(u_refs) < length - 1:
            u_refs = np.vstack([u_refs, u_refs[-1:]])

        return x_refs, u_refs

    # Initialize state
    x_ref_init, _ = get_reference_at_index(0)
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
            if (
                actuator_params.noise_v_std > 0
            ):  # Reuse noise param for state noise proxy
                # Ideally we'd have separate sensor noise params, but for now reuse
                x_measured[0] += np.random.normal(0, actuator_params.noise_v_std * dt)
                x_measured[1] += np.random.normal(0, actuator_params.noise_v_std * dt)
                x_measured[2] += np.random.normal(
                    0, actuator_params.noise_omega_std * dt
                )

        if moving_obstacle_specs:
            time_s = k * dt
            for obs, x0_obs, y0_obs, amp_x, amp_y, freq in moving_obstacle_specs:
                phase = 2.0 * np.pi * freq * time_s
                obs.x = x0_obs + amp_x * np.sin(phase)
                obs.y = y0_obs + amp_y * np.cos(phase)
            obstacle_dicts = [
                {"x": o.x, "y": o.y, "radius": o.radius} for o in obstacles
            ]

        x_ref, u_ref = get_reference_at_index(k)

        # --- Compute risk (with lightweight predictive component) ---
        lookahead = min(20, N - k)
        if lookahead > 1:
            _, u_refs_pred = get_trajectory_segment(k, lookahead)
            predicted_states = risk_metrics.fast_predict_rollout(
                x_measured, u_refs_pred, dt
            )
            assessment = risk_metrics.assess_risk(
                x_measured, obstacle_dicts, predicted_states
            )
        else:
            assessment = risk_metrics.assess_risk(x_measured, obstacle_dicts)
        risk_history[k] = assessment.combined_risk

        # --- Compute LQR control (always available, cheap) ---
        u_lqr, _ = lqr.compute_control_at_operating_point(x_measured, x_ref, u_ref)

        # --- Compute MPC control (at lower rate) ---
        solver_status = "optimal"
        solver_time_ms = 0.0

        if k % mpc_rate == 0:
            x_refs, u_refs = get_trajectory_segment(k, mpc.N + 1)
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
        feas_margin = (
            mpc_solution.feasibility_margin if mpc_solution is not None else 0.0
        )
        u, blend_info = blender.blend(
            u_lqr=u_lqr,
            u_mpc=u_mpc,
            risk=assessment.combined_risk,
            solver_status=solver_status,
            solver_time_ms=solver_time_ms,
            feasibility_margin=feas_margin,
        )

        blend_weights[k] = blend_info.weight

        # --- Compute jerk (from previous controls) ---
        linear_jerk = 0.0
        angular_jerk = 0.0
        if k >= 2:
            dv_curr = (controls[k - 1, 0] - controls[k - 2, 0]) / dt
            dv_prev = (
                (controls[k - 2, 0] - controls[max(0, k - 3), 0]) / dt
                if k >= 3
                else 0.0
            )
            linear_jerk = (dv_curr - dv_prev) / dt

            domega_curr = (controls[k - 1, 1] - controls[k - 2, 1]) / dt
            domega_prev = (
                (controls[k - 2, 1] - controls[max(0, k - 3), 1]) / dt
                if k >= 3
                else 0.0
            )
            angular_jerk = (domega_curr - domega_prev) / dt

        # --- Log ---
        error = x - x_ref
        error[2] = robot.normalize_angle(error[2])

        logger.log_state(k, x, x_ref, error)
        logger.log_control(k, u, f"HYBRID(w={blend_info.weight:.2f})", solver_time_ms)
        logger.log_hybrid_step(
            k,
            blend_info.weight,
            assessment.combined_risk,
            blend_info.mode,
            linear_jerk,
            angular_jerk,
        )

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
            print(
                f"  k={k:4d}: risk={assessment.combined_risk:.2f} "
                f"w={blend_info.weight:.3f} [{blend_info.mode:14s}] "
                f"err={error_norm:.4f}"
            )

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
    print(f"  LQR-dominant: {100 * blend_stats['lqr_dominant_fraction']:.1f}%")
    print(f"  Blended: {100 * blend_stats['blended_fraction']:.1f}%")
    print(f"  MPC-dominant: {100 * blend_stats['mpc_dominant_fraction']:.1f}%")
    print(f"  Weight transitions (w crosses 0.5): {blend_stats['total_switches']}")
    print(f"\nJerk Metrics:")
    print(
        f"  Linear jerk  - peak: {jerk_metrics['linear_jerk_peak']:.2f}, "
        f"RMS: {jerk_metrics['linear_jerk_rms']:.2f}, "
        f"p95: {jerk_metrics['linear_jerk_p95']:.2f}"
    )
    print(
        f"  Angular jerk - peak: {jerk_metrics['angular_jerk_peak']:.2f}, "
        f"RMS: {jerk_metrics['angular_jerk_rms']:.2f}, "
        f"p95: {jerk_metrics['angular_jerk_p95']:.2f}"
    )

    logger.finalize()

    # --- Visualization ---
    if visualize:
        run_output_dir = output_dir or _plots_sub(
            "Hybrid", trajectory_type, scenario
        )
        os.makedirs(run_output_dir, exist_ok=True)
        viz = Visualizer(output_dir=run_output_dir)

        trajectory_plot_path = os.path.join(
            run_output_dir, f"hybrid_{trajectory_type}_{scenario}_trajectory.png"
        )
        trajectory_fig = viz.plot_with_obstacles(
            states,
            ref_states,
            obstacle_dicts,
            mpc.d_safe,
            title=f"Hybrid Smooth-Blend Trajectory | {trajectory_type} | {scenario}",
            save_path=trajectory_plot_path,
        )

        viz.plot_tracking_error(
            errors,
            dt,
            title=f"Hybrid Tracking Error | {trajectory_type} | {scenario}",
            save_path=os.path.join(
                run_output_dir, f"hybrid_{trajectory_type}_{scenario}_error.png"
            ),
        )

        viz.plot_control_inputs(
            controls,
            dt,
            v_max=2.0,
            omega_max=3.0,
            title=f"Hybrid Control Inputs | {trajectory_type} | {scenario}",
            save_path=os.path.join(
                run_output_dir, f"hybrid_{trajectory_type}_{scenario}_control.png"
            ),
        )

        # Plot blending weight and risk
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        t = np.arange(N - 1) * dt

        # Panel 1: Risk and blend weight
        ax1 = axes[0]
        ax1.plot(t, risk_history, "b-", linewidth=1.0, alpha=0.7, label="Risk")
        ax1.plot(t, blend_weights, "r-", linewidth=2.0, label="Blend Weight w(t)")
        ax1.axhline(
            blender.risk_threshold,
            color="g",
            linestyle="--",
            alpha=0.4,
            label=f"Risk threshold ({blender.risk_threshold})",
        )
        ax1.axhline(
            blender._lqr_dominant_threshold,
            color="tab:blue",
            linestyle=":",
            alpha=0.5,
            label=f"LQR-dominant w<{blender._lqr_dominant_threshold}",
        )
        ax1.axhline(
            blender._mpc_dominant_threshold,
            color="tab:red",
            linestyle=":",
            alpha=0.5,
            label=f"MPC-dominant w>{blender._mpc_dominant_threshold}",
        )
        ax1.fill_between(t, 0, blend_weights, alpha=0.15, color="red")
        ax1.set_ylabel("Value")
        ax1.set_title("Continuous Blending: Risk-Based Control Arbitration")
        ax1.legend(loc="upper right")
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(-0.05, 1.05)

        # Panel 2: Control inputs with blend regions
        ax2 = axes[1]
        ax2.plot(t, controls[:, 0], "b-", linewidth=1.0, label="v (m/s)")
        ax2.plot(t, controls[:, 1], "r-", linewidth=1.0, label="omega (rad/s)")
        ax2.set_ylabel("Control")
        ax2.set_title("Blended Control Inputs")
        ax2.legend(loc="upper right")
        ax2.grid(True, alpha=0.3)

        # Panel 3: Jerk
        ax3 = axes[2]
        if len(controls) >= 3:
            dv = np.diff(controls[:, 0]) / dt
            domega = np.diff(controls[:, 1]) / dt
            lj = np.diff(dv) / dt
            aj = np.diff(domega) / dt
            t_jerk = np.arange(len(lj)) * dt
            ax3.plot(t_jerk, lj, "b-", linewidth=0.8, alpha=0.7, label="Linear jerk")
            ax3.plot(t_jerk, aj, "r-", linewidth=0.8, alpha=0.7, label="Angular jerk")
        ax3.set_ylabel("Jerk")
        ax3.set_xlabel("Time (s)")
        ax3.set_title("Control Jerk (d^2u/dt^2)")
        ax3.legend(loc="upper right")
        ax3.grid(True, alpha=0.3)

        plt.tight_layout()
        fig.savefig(
            os.path.join(
                run_output_dir, f"hybrid_{trajectory_type}_{scenario}_blending.png"
            ),
            dpi=150,
            bbox_inches="tight",
        )
        plt.close(fig)

        print(f"\nPlots saved to {run_output_dir}")

    return {
        "states": states,
        "controls": controls,
        "errors": errors,
        "risk_history": risk_history,
        "blend_weights": blend_weights,
        "blend_stats": blend_stats,
        "jerk_metrics": jerk_metrics,
        "mean_error": mean_error,
        "final_error": final_error,
        "mean_solve_time": mean_solve_time,
    }


import warnings

warnings.filterwarnings("ignore", category=UserWarning, module="cvxpy")


def main():
    # Parse arguments
    parser = argparse.ArgumentParser(
        description="Run hybrid LQR-MPC simulation",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["lqr", "mpc", "adaptive", "compare", "hybrid"],
        default="lqr",
        help="Simulation mode",
    )
    parser.add_argument(
        "--duration", type=float, default=20.0, help="Duration in seconds"
    )
    parser.add_argument("--dt", type=float, default=0.02, help="Time step")
    parser.add_argument(
        "--scenario", type=str, default="default", help="Obstacle scenario"
    )
    parser.add_argument(
        "--trajectory",
        type=str,
        default="figure8",
        choices=["figure8", "clover3", "rose4", "spiral", "random_wp"],
        help="P2-C: Trajectory family (default: figure8)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Override plot directory (default: results/Hybrid/<trajectory>/<scenario>)",
    )
    parser.add_argument("--no-plot", action="store_true", help="Disable plotting")
    parser.add_argument(
        "--realistic",
        action="store_true",
        help="Enable actuator dynamics (lag, delay, noise)",
    )

    args = parser.parse_args()

    # Create output directories
    os.makedirs(PLOTS_ROOT, exist_ok=True)
    os.makedirs("logs", exist_ok=True)

    # Configure realism
    act_params = None
    if args.realistic:
        act_params = ActuatorParams(
            tau_v=0.1,
            tau_omega=0.1,
            delay_steps=1,
            noise_v_std=0.02,
            noise_omega_std=0.05,
        )

    if args.mode == "lqr":
        run_lqr_simulation(
            duration=args.duration,
            dt=args.dt,
            visualize=not args.no_plot,
            trajectory_type=args.trajectory,
        )
    elif args.mode == "mpc":
        run_mpc_simulation(
            duration=args.duration,
            dt=args.dt,
            visualize=not args.no_plot,
            scenario=args.scenario,
            trajectory_type=args.trajectory,
        )
    elif args.mode == "adaptive":
        run_adaptive_simulation(
            duration=args.duration,
            dt=args.dt,
            visualize=not args.no_plot,
            scenario=args.scenario,
            trajectory_type=args.trajectory,
        )
    elif args.mode == "compare":
        run_comparison(
            duration=args.duration, dt=args.dt, trajectory_type=args.trajectory
        )
    elif args.mode == "hybrid":
        run_hybrid_simulation(
            duration=args.duration,
            dt=args.dt,
            visualize=not args.no_plot,
            scenario=args.scenario,
            trajectory_type=args.trajectory,
            output_dir=args.output_dir,
            actuator_params=act_params,
        )

    print("\nSimulation complete!")


if __name__ == "__main__":
    main()
