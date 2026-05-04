#!/usr/bin/env python3
"""
Asynchronous Adaptive MPC Controller Node for TurtleBot3
=========================================================

Architecture (3-thread decoupled design):
──────────────────────────────────────────
  Thread 1 — rclpy executor (main)
      • /odom subscriber   → writes to `_state_lock`-protected state
      • /obstacles subscriber → writes to `_obs_lock`-protected list
      • 20 Hz timer         → reads latest [v, ω] under `_cmd_lock`,
                              publishes TwistStamped to /cmd_vel
                              (Zero-Order Hold — never blocks)

  Thread 2 — MPC solver (daemon)
      • Continuous loop: snapshot state → solve → store [v, ω]
      • Runs as fast as IPOPT allows (~6-8 Hz with warm-start)
      • Never blocks the publisher timer

Why this works:
  The 20 Hz timer always has a command ready (even if it's the previous
  one). The MPC thread updates the command whenever it finishes a solve.
  The robot sees smooth, jitter-free control at exactly 20 Hz.

Reference:
    "Decoupled perception-planning-control" pattern, common in
    autonomous driving stacks (Autoware, Apollo).
"""

import threading
import time
import logging
import os
from datetime import datetime
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, String, Float32MultiArray

from hybrid_controller.controllers.adaptive_mpc_controller import AdaptiveMPCController
from hybrid_nav.trajectory_index import nearest_idx_closed


# ── Helpers ─────────────────────────────────────────────────────────

def euler_from_quaternion(q):
    """Extract yaw from quaternion (ZYX convention)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    """Wrap angle to [-π, π]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


class DummyObstacle:
    """Lightweight obstacle container matching the Obstacle interface."""
    __slots__ = ('x', 'y', 'radius')
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius


# ── Main Node ───────────────────────────────────────────────────────

class AsyncMPCControllerNode(Node):
    """
    Threaded Adaptive MPC node with decoupled state / solve / publish.

    Shared variables & their locks:
        _state_lock  → protects (x, y, theta, odom_ok)
        _obs_lock    → protects obstacles list
        _cmd_lock    → protects (cmd_v, cmd_omega, mpc_mode, mpc_predicted)
    """

    def __init__(self):
        super().__init__('async_mpc_controller_node')

        # ── Declare Parameters ──────────────────────────────────────
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('trajectory_amplitude', 0.5)
        self.declare_parameter('trajectory_frequency', 0.15)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('omega_max', 2.84)
        self.declare_parameter('horizon', 20)
        self.declare_parameter('d_safe', 0.50)
        self.declare_parameter('q_x', 20.0)
        self.declare_parameter('q_y', 20.0)
        self.declare_parameter('q_theta', 5.0)
        self.declare_parameter('r_v', 0.1)
        self.declare_parameter('r_omega', 0.1)

        # ── Read Parameters ─────────────────────────────────────────
        rate = self.get_parameter('control_rate').value
        self.A = self.get_parameter('trajectory_amplitude').value
        self.freq = self.get_parameter('trajectory_frequency').value
        self.dt = self.get_parameter('dt').value
        self.v_max = self.get_parameter('v_max').value
        self.omega_max = self.get_parameter('omega_max').value
        self.horizon = self.get_parameter('horizon').value
        self.d_safe = self.get_parameter('d_safe').value

        Q_diag = [
            self.get_parameter('q_x').value,
            self.get_parameter('q_y').value,
            self.get_parameter('q_theta').value,
        ]
        R_diag = [
            self.get_parameter('r_v').value,
            self.get_parameter('r_omega').value,
        ]

        # ── MPC Controller (used ONLY in solver thread) ─────────────
        self.mpc = AdaptiveMPCController(
            prediction_horizon=self.horizon,
            dt=self.dt,
            d_safe=self.d_safe,
            v_max=self.v_max,
            omega_max=self.omega_max,
            Q_diag=Q_diag,
            R_diag=R_diag,
            enable_adaptation=True,
        )
        self.get_logger().info('✅ Adaptive MPC solver built (CasADi + IPOPT).')

        # ── Generate Reference Trajectory ───────────────────────────
        self._generate_trajectory()

        # ══════════════════════════════════════════════════════════════
        #  THREAD-SAFE SHARED STATE
        # ══════════════════════════════════════════════════════════════

        # --- State (written by odom callback, read by solver thread) -
        self._state_lock = threading.Lock()
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._odom_ok = False

        # --- Obstacles (written by obs callback, read by solver) -----
        self._obs_lock = threading.Lock()
        self._obstacles: List[DummyObstacle] = []

        # --- Command (written by solver thread, read by 20 Hz timer) -
        self._cmd_lock = threading.Lock()
        self._cmd_v = 0.0
        self._cmd_omega = 0.0
        self._mpc_mode = 'WAITING_FOR_ODOM'
        self._mpc_solve_time_ms = 0.0
        self._mpc_predicted_states = None  # for RViz visualization

        # --- Solver-internal (only touched by solver thread) ---------
        self._solver_current_idx = 0
        self._solver_x_prev = None
        self._solver_u_prev = None

        # --- Diagnostics (atomic-ish writes, read by timer) ----------
        self._tick_count = 0
        self._solve_count = 0

        # ── Shutdown flag ────────────────────────────────────────────
        self._shutdown_event = threading.Event()

        # ══════════════════════════════════════════════════════════════
        #  ROS INTERFACE
        # ══════════════════════════════════════════════════════════════

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_static = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Use separate callback groups so subscriptions can run in
        # parallel with the timer on a MultiThreadedExecutor.
        sub_group = MutuallyExclusiveCallbackGroup()
        timer_group = MutuallyExclusiveCallbackGroup()

        # Subscribers
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, qos,
            callback_group=sub_group,
        )
        self.create_subscription(
            Float32MultiArray, '/obstacles', self._obs_cb, qos,
            callback_group=sub_group,
        )

        # Publishers
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', qos)
        self.err_pub = self.create_publisher(Float32, '/mpc/tracking_error', qos)
        self.mode_pub = self.create_publisher(String, '/mpc/mode', qos)
        self.ref_pub = self.create_publisher(Path, '/hybrid/reference_path', qos_static)
        self.pred_pub = self.create_publisher(Path, '/hybrid/predicted_path', 1)
        self.trail_pub = self.create_publisher(Path, '/hybrid/robot_trail', qos)

        self.robot_trail = Path()
        self.robot_trail.header.frame_id = 'odom'

        # File logger
        self._run_file_logger, log_file = self._setup_run_file_logger(
            'async_mpc_node'
        )
        self._run_file_logger.info(
            'Tick,X,Y,Theta,Error,V,Omega,SolveTimeMs,Params,Mode'
        )

        # ── 20 Hz Publisher Timer (Zero-Order Hold) ──────────────────
        self._pub_timer = self.create_timer(
            1.0 / rate, self._publish_cmd_cb,
            callback_group=timer_group,
        )

        # Publish reference path once
        self._ref_published = False
        self._pub_ref_path_once()

        # ── Start MPC Solver Thread ──────────────────────────────────
        self._solver_thread = threading.Thread(
            target=self._solver_loop,
            name='mpc_solver',
            daemon=True,
        )
        self._solver_thread.start()

        self.get_logger().info(
            f'🤖 Async MPC Node Ready  |  '
            f'pub @ {rate:.0f} Hz  |  solver → background thread  |  '
            f'log → {log_file}'
        )

    # ══════════════════════════════════════════════════════════════════
    #  TRAJECTORY GENERATION
    # ══════════════════════════════════════════════════════════════════

    def _generate_trajectory(self):
        """Generate one full figure-8 loop as the reference path."""
        w = self.freq
        T = 2 * np.pi / w
        t = np.arange(0, T, self.dt)

        self.ref_x = self.A * np.sin(w * t)
        self.ref_y = (self.A / 2.0) * np.sin(2.0 * w * t)

        dx_dt = self.A * w * np.cos(w * t)
        dy_dt = self.A * w * np.cos(2.0 * w * t)

        self.ref_theta = np.arctan2(dy_dt, dx_dt)
        self.ref_v = np.clip(np.sqrt(dx_dt**2 + dy_dt**2), 0.0, self.v_max)

        self.ref_omega = np.zeros_like(t)
        for i in range(1, len(t)):
            dtheta = normalize_angle(self.ref_theta[i] - self.ref_theta[i - 1])
            self.ref_omega[i] = dtheta / self.dt
        self.ref_omega[0] = self.ref_omega[1]

        self.N_traj = len(t)
        self.get_logger().info(
            f'📐 Trajectory: {self.N_traj} pts, '
            f'period={T:.1f}s, A={self.A}m'
        )

    # ══════════════════════════════════════════════════════════════════
    #  THREAD 1a — ODOM CALLBACK (runs on executor)
    # ══════════════════════════════════════════════════════════════════

    def _odom_cb(self, msg: Odometry):
        """Update thread-safe robot state from odometry."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = euler_from_quaternion(msg.pose.pose.orientation)

        with self._state_lock:
            self._x = x
            self._y = y
            self._theta = theta
            first_odom = not self._odom_ok
            self._odom_ok = True

        if first_odom:
            self.get_logger().info(f'📡 Odom OK: ({x:.3f}, {y:.3f})')

        # Robot trail for RViz (append is fast, no lock needed for
        # publisher since only this callback appends)
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.robot_trail.poses.append(ps)
        self.robot_trail.header.stamp = self.get_clock().now().to_msg()
        self.trail_pub.publish(self.robot_trail)

    # ══════════════════════════════════════════════════════════════════
    #  THREAD 1b — OBSTACLE CALLBACK (runs on executor)
    # ══════════════════════════════════════════════════════════════════

    def _obs_cb(self, msg: Float32MultiArray):
        """Update thread-safe obstacle list from /obstacles topic."""
        obs_list = []
        for i in range(0, len(msg.data), 3):
            if i + 2 < len(msg.data):
                obs_list.append(DummyObstacle(
                    x=float(msg.data[i]),
                    y=float(msg.data[i + 1]),
                    radius=float(msg.data[i + 2]),
                ))
        with self._obs_lock:
            self._obstacles = obs_list

    # ══════════════════════════════════════════════════════════════════
    #  THREAD 2 — MPC SOLVER (background daemon)
    # ══════════════════════════════════════════════════════════════════

    def _solver_loop(self):
        """
        Continuous MPC solve loop.

        Runs as fast as IPOPT allows. Each iteration:
          1. Snapshot state & obstacles under their locks
          2. Compute reference window
          3. Solve MPC (expensive — 50-200 ms)
          4. Store [v, ω] result under _cmd_lock
        """
        self.get_logger().info('🧮 Solver thread started.')

        # Wait for first odom before solving
        while not self._shutdown_event.is_set():
            with self._state_lock:
                ready = self._odom_ok
            if ready:
                break
            time.sleep(0.05)

        self.get_logger().info('🧮 Solver thread: odom received, entering loop.')

        while not self._shutdown_event.is_set():
            t_start = time.perf_counter()

            # ── 1. Snapshot state ────────────────────────────────────
            with self._state_lock:
                x_now = self._x
                y_now = self._y
                theta_now = self._theta

            x_current = np.array([x_now, y_now, theta_now])

            # ── 2. Snapshot obstacles ────────────────────────────────
            with self._obs_lock:
                obstacles = list(self._obstacles)  # shallow copy

            # ── 3. Online parameter adaptation ───────────────────────
            if (self._solver_x_prev is not None
                    and self._solver_u_prev is not None):
                self.mpc.adapt_parameters(
                    x_current, self._solver_x_prev, self._solver_u_prev
                )

            # ── 4. Find nearest index on closed trajectory ───────────
            self._solver_current_idx, dist_err = nearest_idx_closed(
                self.ref_x, self.ref_y,
                self._solver_current_idx,
                x_now, y_now, self.N_traj,
            )

            # ── 5. Build reference window ────────────────────────────
            x_refs = np.zeros((self.horizon + 1, 3))
            u_refs = np.zeros((self.horizon, 2))
            start_idx = (self._solver_current_idx + 2) % self.N_traj

            for k in range(self.horizon + 1):
                idx = (start_idx + k) % self.N_traj
                x_refs[k] = [
                    self.ref_x[idx],
                    self.ref_y[idx],
                    self.ref_theta[idx],
                ]
                if k < self.horizon:
                    u_refs[k] = [self.ref_v[idx], self.ref_omega[idx]]

            # ── 6. Solve MPC ─────────────────────────────────────────
            v, omega = 0.0, 0.0
            mode = 'AMPC_FAILED'
            predicted_states = None

            try:
                solution = self.mpc.solve_tracking(
                    x0=x_current,
                    x_refs=x_refs,
                    u_refs=u_refs,
                    obstacles=obstacles,
                )

                if solution.feasible:
                    v = float(solution.optimal_control[0])
                    omega = float(solution.optimal_control[1])
                    mode = f'AMPC_OK ({solution.solve_time_ms:.0f}ms)'
                    predicted_states = solution.predicted_states
                else:
                    mode = 'AMPC_INFEASIBLE'
                    # Decay previous command as fallback
                    if self._solver_u_prev is not None:
                        v = 0.0
                        omega = float(np.clip(
                            self._solver_u_prev[1] * 1.5 ,
                            -self.omega_max, self.omega_max,
                        ))
                        if abs(omega)<0.1:
                            omega = 0.5
            except Exception as e:
                self.get_logger().error(
                    f'Solver exception: {e}', throttle_duration_sec=2.0
                )

            # ════════════════════════════════════════════════════════════
            # 🚨 NEW BULLETPROOF FIX: EMERGENCY REVERSE OVERRIDE 🚨
            # If the robot physically touches the obstacle, the MPC fails.
            # Spinning in place grinds the wheels. We MUST reverse!
            for obs in obstacles:
                dist_to_obs = np.hypot(x_current[0] - obs.x, x_current[1] - obs.y)
                if dist_to_obs < (obs.radius + 0.35 ): # If within 18cm of obstacle center
                    v = -0.50    # Force reverse!
                    omega = 2.0 # Force spin to point away from the obstacle
                    mode = 'EMERGENCY_REVERSE'
                    break
            # ════════════════════════════════════════════════════════════

            # Safety clipping
            v = float(np.clip(v, -self.v_max, self.v_max))
            omega = float(np.clip(omega, -self.omega_max, self.omega_max))
            # Safety clipping
            v = float(np.clip(v, -self.v_max, self.v_max))
            omega = float(np.clip(omega, -self.omega_max, self.omega_max))

            solve_time_ms = (time.perf_counter() - t_start) * 1000.0

            # ── 7. Store result (thread-safe) ────────────────────────
            with self._cmd_lock:
                self._cmd_v = v
                self._cmd_omega = omega
                self._mpc_mode = mode
                self._mpc_solve_time_ms = solve_time_ms
                self._mpc_predicted_states = predicted_states

            # Update solver-internal state (only this thread touches)
            self._solver_x_prev = x_current
            self._solver_u_prev = np.array([v, omega])
            self._solve_count += 1

            # Periodic log from solver thread
            if self._solve_count % 20 == 0:
                params = self.mpc.param_estimates
                self.get_logger().info(
                    f'[Solver #{self._solve_count}] '
                    f'idx={self._solver_current_idx}/{self.N_traj} '
                    f'err={dist_err:.3f}m  '
                    f'v={v:.3f} ω={omega:+.3f}  '
                    f'solve={solve_time_ms:.0f}ms  '
                    f'θ̂=[{params[0]:.2f},{params[1]:.2f}]'
                )

    # ══════════════════════════════════════════════════════════════════
    #  THREAD 1c — 20 Hz PUBLISHER TIMER (Zero-Order Hold)
    # ══════════════════════════════════════════════════════════════════

    def _publish_cmd_cb(self):
        """
        Strictly 20 Hz timer callback.

        Grabs the latest [v, ω] from the solver thread and publishes
        to /cmd_vel. This NEVER blocks — if the solver hasn't finished
        a new solve yet, we republish the previous command (ZOH).
        """
        # Snapshot command under lock (fast — no blocking)
        with self._cmd_lock:
            v = self._cmd_v
            omega = self._cmd_omega
            mode = self._mpc_mode
            solve_ms = self._mpc_solve_time_ms
            pred_states = self._mpc_predicted_states

        # Snapshot state for diagnostics
        with self._state_lock:
            x = self._x
            y = self._y
            theta = self._theta
            odom_ok = self._odom_ok

        if not odom_ok:
            return

        # Publish TwistStamped
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = v
        cmd.twist.angular.z = omega
        self.cmd_pub.publish(cmd)

        # Publish diagnostics
        # Tracking error (quick computation, no lock needed)
        ref_idx = self._solver_current_idx  # atomic read
        dist_err = float(np.hypot(
            self.ref_x[ref_idx] - x,
            self.ref_y[ref_idx] - y,
        ))

        e_msg = Float32()
        e_msg.data = dist_err
        self.err_pub.publish(e_msg)

        m_msg = String()
        m_msg.data = mode
        self.mode_pub.publish(m_msg)

        # Publish predicted path for RViz
        if pred_states is not None:
            self._pub_pred_path(pred_states)

        # File logging
        params = self.mpc.param_estimates
        self._run_file_logger.info(
            f'{self._tick_count},{x:.3f},{y:.3f},{theta:.3f},'
            f'{dist_err:.3f},{v:.3f},{omega:.3f},{solve_ms:.1f},'
            f'"{params[0]:.2f}_{params[1]:.2f}",{mode}'
        )

        # Console logging (every 1 second at 20 Hz)
        if self._tick_count % 20 == 0:
            self.get_logger().info(
                f'[PUB {self._tick_count}] '
                f'pos=({x:+.2f},{y:+.2f}) '
                f'err={dist_err:.3f}m | '
                f'v={v:.3f} ω={omega:+.3f} | '
                f'solve={solve_ms:.0f}ms | {mode}'
            )

        self._tick_count += 1

    # ══════════════════════════════════════════════════════════════════
    #  RViz HELPERS
    # ══════════════════════════════════════════════════════════════════

    def _pub_ref_path_once(self):
        """Publish the full reference path once (latched/transient-local)."""
        if self._ref_published:
            return
        self._ref_published = True
        p = Path()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'odom'
        for i in range(0, self.N_traj, 5):
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.pose.position.x = float(self.ref_x[i])
            ps.pose.position.y = float(self.ref_y[i])
            ps.pose.orientation.z = float(np.sin(self.ref_theta[i] / 2))
            ps.pose.orientation.w = float(np.cos(self.ref_theta[i] / 2))
            p.poses.append(ps)
        self.ref_pub.publish(p)

    def _pub_pred_path(self, states):
        """Publish MPC predicted trajectory for RViz."""
        p = Path()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'odom'
        for i in range(states.shape[0]):
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.pose.position.x = float(states[i, 0])
            ps.pose.position.y = float(states[i, 1])
            p.poses.append(ps)
        self.pred_pub.publish(p)

    # ══════════════════════════════════════════════════════════════════
    #  FILE LOGGING
    # ══════════════════════════════════════════════════════════════════

    def _setup_run_file_logger(self, basename: str):
        log_dir = os.path.join(os.getcwd(), 'Output', 'Logs')
        os.makedirs(log_dir, exist_ok=True)
        path = os.path.join(
            log_dir,
            f'{basename}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log',
        )
        logger = logging.getLogger(f'hybrid_nav.{basename}_{id(self)}')
        logger.setLevel(logging.DEBUG)
        logger.handlers.clear()
        fh = logging.FileHandler(path, encoding='utf-8')
        fh.setFormatter(
            logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        )
        logger.addHandler(fh)
        logger.propagate = False
        return logger, path

    def _close_run_file_logger(self):
        log = getattr(self, '_run_file_logger', None)
        if not log:
            return
        for h in list(log.handlers):
            log.removeHandler(h)
            h.flush()
            h.close()
        self._run_file_logger = None

    # ══════════════════════════════════════════════════════════════════
    #  SHUTDOWN
    # ══════════════════════════════════════════════════════════════════

    def shutdown(self):
        """Cleanly stop solver thread and send zero command."""
        self._shutdown_event.set()

        # Send stop command
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self.cmd_pub.publish(cmd)

        # Wait for solver thread
        if self._solver_thread.is_alive():
            self._solver_thread.join(timeout=2.0)

        self._close_run_file_logger()
        self.get_logger().info('🛑 Async MPC Node shut down cleanly.')


# ══════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = AsyncMPCControllerNode()

    # Use MultiThreadedExecutor so odom callbacks and the 20 Hz timer
    # can run concurrently (each in its own callback group).
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('⌨️  KeyboardInterrupt received.')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
