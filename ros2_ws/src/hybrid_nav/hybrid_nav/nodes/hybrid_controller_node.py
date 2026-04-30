#!/usr/bin/env python3
"""
Hybrid Controller Node for TurtleBot3 — v5 (Pure Pursuit fix)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, String, Float32MultiArray
from rclpy.qos import DurabilityPolicy

import logging
import os
from datetime import datetime

from hybrid_nav.trajectory_index import nearest_idx_open, lookahead_idx_open


def euler_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class HybridControllerNode(Node):
    def __init__(self):
        super().__init__('hybrid_controller')

        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('trajectory_amplitude', 0.5)
        self.declare_parameter('trajectory_frequency', 0.2)
        self.declare_parameter('trajectory_duration', 120.0)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('mpc_horizon', 10)
        self.declare_parameter('d_safe', 0.35)
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('omega_max', 2.84)
        self.declare_parameter('use_hybrid', True)
        self.declare_parameter('lookahead_dist', 0.40)   # FIXED: was 0.20
        self.declare_parameter('cruise_speed', 0.10)
        self.declare_parameter('waypoint_reach_dist', 0.08)  # NEW

        rate       = self.get_parameter('control_rate').value
        self.A     = self.get_parameter('trajectory_amplitude').value
        self.freq  = self.get_parameter('trajectory_frequency').value
        dur        = self.get_parameter('trajectory_duration').value
        self.dt    = self.get_parameter('dt').value
        self.v_max = self.get_parameter('v_max').value
        self.omega_max = self.get_parameter('omega_max').value
        self.use_hybrid    = self.get_parameter('use_hybrid').value
        self.lookahead     = self.get_parameter('lookahead_dist').value
        self.cruise_speed  = self.get_parameter('cruise_speed').value
        self.wp_reach_dist = self.get_parameter('waypoint_reach_dist').value

        # ── Import advanced controllers ──────────────────────────
        self.controllers_ok = False
        try:
            from hybrid_controller.controllers.mpc_controller import MPCController, Obstacle
            from hybrid_controller.controllers.hybrid_blender import BlendingSupervisor
            from hybrid_controller.controllers.risk_metrics import RiskMetrics
            self.mpc = MPCController(
                horizon=self.get_parameter('mpc_horizon').value,
                d_safe=self.get_parameter('d_safe').value,
                v_max=self.v_max, omega_max=self.omega_max, dt=self.dt,
            )
            self.blender   = BlendingSupervisor(dt=self.dt)
            self.risk_calc = RiskMetrics(d_trigger=0.6, d_safe=self.get_parameter('d_safe').value)
            self.Obstacle  = Obstacle
            self.controllers_ok = True
            self.get_logger().info('✅ MPC available')
        except Exception as e:
            self.get_logger().warn(f'⚠️  MPC not available: {e}')

        self._generate_waypoints(dur)

        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self.odom_ok   = False
        self.obstacles = []
        self.wp_idx    = 0   # ONLY moves forward now
        self.tick_count = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)
        self.create_subscription(Float32MultiArray, '/obstacles', self._obs_cb, qos)

        self.cmd_pub  = self.create_publisher(TwistStamped, '/cmd_vel', qos)
        self.err_pub  = self.create_publisher(Float32, '/hybrid/tracking_error', qos)
        self.wgt_pub  = self.create_publisher(Float32, '/hybrid/blend_weight', qos)
        self.mode_pub = self.create_publisher(String, '/hybrid/controller_mode', qos)
        
        qos_static = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.ref_pub  = self.create_publisher(Path, '/hybrid/reference_path', qos_static)
        self.trail_pub = self.create_publisher(Path, '/hybrid/robot_trail', qos)
        
        self.robot_trail = Path()
        self.robot_trail.header.frame_id = 'odom'

        self._run_file_logger, log_file = self._setup_run_file_logger('hybrid_node')
        self._run_file_logger.info('Tick,X,Y,Theta,Error,V,Omega,Mode,BlendWeight')

        self.create_timer(1.0 / rate, self._control_loop)
        self._ref_published = False
        self._pub_ref_path_once()

        self.get_logger().info(f'🤖 Hybrid Node Ready. Logging to {log_file}')

    # ── Waypoint generation (unchanged) ───────────────────────
    def _generate_waypoints(self, duration):
        t   = np.arange(0, duration, self.dt)
        w   = self.freq
        self.dense_x     = self.A * np.sin(w * t)
        self.dense_y     = (self.A / 2.0) * np.sin(2.0 * w * t)
        dx_dt            = self.A * w * np.cos(w * t)
        dy_dt            = self.A * w * np.cos(2.0 * w * t)
        self.dense_theta = np.arctan2(dy_dt, dx_dt)
        self.dense_v     = np.clip(np.sqrt(dx_dt**2 + dy_dt**2), 0.0, self.v_max)
        self.dense_N     = len(t)

        wp_x, wp_y, wp_theta = [self.dense_x[0]], [self.dense_y[0]], [self.dense_theta[0]]
        for i in range(1, len(t)):
            d = np.hypot(self.dense_x[i] - wp_x[-1], self.dense_y[i] - wp_y[-1])
            if d >= 0.05:
                wp_x.append(self.dense_x[i])
                wp_y.append(self.dense_y[i])
                wp_theta.append(self.dense_theta[i])

        self.wp_x = np.array(wp_x)
        self.wp_y = np.array(wp_y)
        self.wp_theta = np.array(wp_theta)
        self.n_wp = len(self.wp_x)
        self.get_logger().info(f'📐 {self.n_wp} waypoints generated')

    # ── Callbacks ─────────────────────────────────────────────
    def _odom_cb(self, msg):
        self.x     = msg.pose.pose.position.x
        self.y     = msg.pose.pose.position.y
        self.theta = euler_from_quaternion(msg.pose.pose.orientation)
        if not self.odom_ok:
            self.odom_ok = True
            self.get_logger().info(f'📡 Odom OK: ({self.x:.3f},{self.y:.3f})')
            
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.robot_trail.poses.append(ps)
        self.robot_trail.header.stamp = self.get_clock().now().to_msg()
        self.trail_pub.publish(self.robot_trail)

    def _obs_cb(self, msg):
        self.obstacles = []
        for i in range(0, len(msg.data) - 2, 3):
            self.obstacles.append({
                'x': float(msg.data[i]),
                'y': float(msg.data[i+1]),
                'radius': float(msg.data[i+2]),
            })

    # ── Main Control Loop ──────────────────────────────────────
    def _control_loop(self):
        if not self.odom_ok:
            return

        self.wp_idx, _dist_wp = nearest_idx_open(
            self.wp_x, self.wp_y, self.wp_idx, self.x, self.y, self.n_wp
        )
        target_idx = lookahead_idx_open(
            self.wp_x, self.wp_y, self.wp_idx, self.lookahead, self.n_wp
        )

        tx = self.wp_x[target_idx]
        ty = self.wp_y[target_idx]

        dx = tx - self.x
        dy = ty - self.y
        dist_to_target  = np.hypot(dx, dy)
        target_heading  = np.arctan2(dy, dx)
        heading_err     = normalize_angle(target_heading - self.theta)

        alignment = max(0.0, np.cos(heading_err))
        v = self.cruise_speed * alignment

        k_omega = 2.5
        omega = float(np.clip(k_omega * heading_err, -self.omega_max, self.omega_max))
        v     = float(np.clip(v, -self.v_max, self.v_max))

        if self.wp_idx >= self.n_wp - 1:
            dist_final = float(np.hypot(self.x - self.wp_x[-1], self.y - self.wp_y[-1]))
            if dist_final <= self.wp_reach_dist:
                v = 0.0
                omega = float(np.clip(
                    k_omega * normalize_angle(self.wp_theta[-1] - self.theta),
                    -self.omega_max, self.omega_max))
            else:
                v = float(np.clip(v * min(1.0, dist_final / 0.25), -self.v_max, self.v_max))

        mode    = 'TRACK'
        v_base  = v
        omega_base = omega
        w_blend = 0.0

        # ── MPC obstacle overlay (unchanged logic) ─────────────
        if self.controllers_ok and self.use_hybrid and self.obstacles:
            try:
                x0 = np.array([self.x, self.y, self.theta])
                dist_risk, _, _ = self.risk_calc.compute_distance_risk(x0, self.obstacles)
                blend_info = self.blender.compute_weight(dist_risk)
                w_blend = blend_info.weight

                if w_blend > 0.05:
                    mode = f'AVOID(w={w_blend:.2f})'
                    horizon = self.mpc.N
                    x_refs  = np.zeros((horizon + 1, 3))
                    u_refs  = np.zeros((horizon, 2))
                    for k in range(horizon + 1):
                        idx = min(self.wp_idx + k, self.n_wp - 1)
                        x_refs[k] = [self.wp_x[idx], self.wp_y[idx], self.wp_theta[idx]]
                    for k in range(horizon):
                        u_refs[k] = [self.cruise_speed, 0.0]
                    mpc_obs = [self.Obstacle(x=o['x'], y=o['y'], radius=o['radius'])
                               for o in self.obstacles]
                    sol   = self.mpc.solve(x0, x_refs, u_refs, mpc_obs)
                    u_mpc = sol.optimal_control
                    v     = float(np.clip(w_blend*u_mpc[0] + (1-w_blend)*v_base,
                                         -self.v_max, self.v_max))
                    omega = float(np.clip(w_blend*u_mpc[1] + (1-w_blend)*omega_base,
                                         -self.omega_max, self.omega_max))
            except Exception:
                pass

        self._send(v, omega)
        self._pub_diag(dist_to_target, w_blend, mode)

        self._run_file_logger.info(
            f"{self.tick_count},{self.x:.3f},{self.y:.3f},{self.theta:.3f},"
            f"{dist_to_target:.3f},{v:.3f},{omega:.3f},{mode},{w_blend:.3f}"
        )

        if self.tick_count % 20 == 0:
            self.get_logger().info(
                f'[wp={self.wp_idx}/{self.n_wp}] '
                f'pos=({self.x:+.2f},{self.y:+.2f}) θ={self.theta:+.2f} | '
                f'tgt=({tx:+.2f},{ty:+.2f}) Δθ={heading_err:+.2f} | '
                f'v={v:.3f} ω={omega:+.3f} | {mode}'
            )
        self.tick_count += 1

    # ── Helpers (unchanged) ────────────────────────────────────
    def _send(self, v, omega):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x  = float(v)
        cmd.twist.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    def _pub_diag(self, err, w, mode):
        e = Float32(); e.data = float(err); self.err_pub.publish(e)
        wm = Float32(); wm.data = float(w); self.wgt_pub.publish(wm)
        m = String(); m.data = mode; self.mode_pub.publish(m)

    def _pub_ref_path_once(self):
        if self._ref_published:
            return
        self._ref_published = True
        p = Path()
        p.header.stamp    = self.get_clock().now().to_msg()
        p.header.frame_id = 'odom'
        for i in range(0, self.dense_N, 5):
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.pose.position.x = float(self.dense_x[i])
            ps.pose.position.y = float(self.dense_y[i])
            ps.pose.orientation.z = float(np.sin(self.dense_theta[i] / 2))
            ps.pose.orientation.w = float(np.cos(self.dense_theta[i] / 2))
            p.poses.append(ps)
        self.ref_pub.publish(p)
        self.get_logger().info(f'📍 Ref path: {len(p.poses)} poses')

    def _setup_run_file_logger(self, basename: str):
        log_dir = os.path.join(os.getcwd(), 'Output', 'Logs')
        os.makedirs(log_dir, exist_ok=True)
        path = os.path.join(
            log_dir, f'{basename}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
        logger = logging.getLogger(f'hybrid_nav.{basename}_{id(self)}')
        logger.setLevel(logging.DEBUG)
        logger.handlers.clear()
        fh = logging.FileHandler(path, encoding='utf-8')
        fh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
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


def main(args=None):
    rclpy.init(args=args)
    node = HybridControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._send(0.0, 0.0)
    finally:
        node._close_run_file_logger()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
