#!/usr/bin/env python3
"""
Pure MPC Controller Node for TurtleBot3
=======================================

This node strictly runs the pure MPC algorithm for trajectory tracking
and obstacle avoidance. It does NOT include any hybrid logic.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, String, Float32MultiArray

# Import the existing MPC controller logic
from hybrid_controller.controllers.mpc_controller import MPCController


def euler_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    """Wrap to [-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi

class DummyObstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

class MPCControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller_node')

        # ── Parameters ─────────────────────────────────────────
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('trajectory_amplitude', 0.5)
        self.declare_parameter('trajectory_frequency', 0.15)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('omega_max', 2.84)
        
        # MPC tuning weights
        self.declare_parameter('horizon', 10)
        self.declare_parameter('d_safe', 0.25)
        self.declare_parameter('q_x', 20.0)
        self.declare_parameter('q_y', 20.0)
        self.declare_parameter('q_theta', 5.0)
        self.declare_parameter('r_v', 1.0)
        self.declare_parameter('r_omega', 0.5)

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
            self.get_parameter('q_theta').value
        ]
        R_diag = [
            self.get_parameter('r_v').value,
            self.get_parameter('r_omega').value
        ]

        # ── Initialize pure MPC Controller ─────────────────────
        self.mpc = MPCController(
            horizon=self.horizon,
            Q_diag=Q_diag, 
            R_diag=R_diag,
            P_diag=[q*2.0 for q in Q_diag],
            d_safe=self.d_safe,
            slack_penalty=5000.0,
            dt=self.dt, 
            v_max=self.v_max, 
            omega_max=self.omega_max,
            solver="OSQP"
        )
        self.get_logger().info(f'✅ Pure MPC Controller Initialized. N={self.horizon}, Q={Q_diag}')

        # ── Trajectory: generate dense path ────────────────────
        self._generate_trajectory()

        # ── State ──────────────────────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_ok = False
        self.tick_count = 0
        self.current_idx = 0
        self.obstacles = []
        self.last_u = np.array([0.0, 0.0])

        # ── ROS ────────────────────────────────────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)
        self.create_subscription(Float32MultiArray, '/obstacles', self._obs_cb, qos)

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', qos)
        self.err_pub = self.create_publisher(Float32, '/mpc/tracking_error', qos)
        self.mode_pub = self.create_publisher(String, '/mpc/mode', qos)
        self.ref_pub = self.create_publisher(Path, '/hybrid/reference_path', 1)
        self.pred_pub = self.create_publisher(Path, '/hybrid/predicted_path', 1)

        self.create_timer(1.0 / rate, self._control_loop)
        self.create_timer(3.0, self._pub_ref_path_once)
        self._ref_published = False

        self.get_logger().info('🤖 MPC Node Ready to track trajectory')

    # ── Generate Trajectory ────────────────────────────────────
    def _generate_trajectory(self):
        w = self.freq
        
        # Ensure duration is exactly ONE figure-8 period (T = 2pi/w)
        T = 2 * np.pi / w
        t = np.arange(0, T, self.dt)

        # Figure-8 Kinematics
        self.ref_x = self.A * np.sin(w * t)
        self.ref_y = (self.A / 2.0) * np.sin(2.0 * w * t)
        
        dx_dt = self.A * w * np.cos(w * t)
        dy_dt = self.A * w * np.cos(2.0 * w * t)
        
        self.ref_theta = np.arctan2(dy_dt, dx_dt)
        self.ref_v = np.clip(np.sqrt(dx_dt**2 + dy_dt**2), 0.0, self.v_max)
        
        # Calculate angular velocity (omega = d(theta)/dt)
        self.ref_omega = np.zeros_like(t)
        for i in range(1, len(t)):
            dtheta = normalize_angle(self.ref_theta[i] - self.ref_theta[i-1])
            self.ref_omega[i] = dtheta / self.dt
        self.ref_omega[0] = self.ref_omega[1]
        
        self.N = len(t)
        self.get_logger().info(f'📐 Generated EXACTLY 1 loop: {self.N} points.')

    # ── Callbacks ──────────────────────────────────────────────
    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = euler_from_quaternion(msg.pose.pose.orientation)
        if not self.odom_ok:
            self.odom_ok = True
            self.get_logger().info(f'📡 Odom OK: ({self.x:.3f}, {self.y:.3f})')

    def _obs_cb(self, msg):
        obs_list = []
        for i in range(0, len(msg.data), 3):
            if i + 2 < len(msg.data):
                obs_list.append(DummyObstacle(
                    x=float(msg.data[i]),
                    y=float(msg.data[i+1]),
                    radius=float(msg.data[i+2])
                ))
        self.obstacles = obs_list

    # ── Main Control Loop ──────────────────────────────────────
    def _control_loop(self):
        if not self.odom_ok:
            return

        # 1. Find the target reference point based on nearest distance.
        # Use a wraparound moving window to prevent jumping at trajectory crossings
        window = 40
        dists = []
        indices = []
        for i in range(-window, window + 1):
            idx = (self.current_idx + i) % self.N
            d = np.sqrt((self.ref_x[idx] - self.x)**2 + (self.ref_y[idx] - self.y)**2)
            dists.append(d)
            indices.append(idx)
            
        local_min_idx = np.argmin(dists)
        self.current_idx = indices[local_min_idx]
        dist_err = dists[local_min_idx]

        # 2. Extract reference window for MPC
        x_refs = np.zeros((self.horizon + 1, 3))
        u_refs = np.zeros((self.horizon, 2))
        
        # Small lookahead offset to help with tight curves
        start_idx = (self.current_idx + 2) % self.N
        
        for k in range(self.horizon + 1):
            idx = (start_idx + k) % self.N
            x_refs[k] = [self.ref_x[idx], self.ref_y[idx], self.ref_theta[idx]]
            if k < self.horizon:
                u_refs[k] = [self.ref_v[idx], self.ref_omega[idx]]
        
        x_current = np.array([self.x, self.y, self.theta])

        # 3. Call the Pure MPC Algorithm
        v, omega = 0.0, 0.0
        mode = "MPC_FAILED"
        try:
            solution = self.mpc.solve(
                x0=x_current, 
                x_refs=x_refs, 
                u_refs=u_refs, 
                obstacles=self.obstacles,
                use_soft_constraints=True
            )
            
            if solution.feasible:
                v = float(solution.optimal_control[0])
                omega = float(solution.optimal_control[1])
                self.last_u = np.array([v, omega])
                mode = f"MPC_OK (Slack={solution.feasibility_margin:.2f})"
                
                # Publish predicted trajectory for RViz
                if solution.predicted_states is not None:
                    self._pub_pred_path(solution.predicted_states)
            else:
                self.get_logger().warn(f"MPC Infeasible!")
                mode = "MPC_INFEASIBLE"
                # Keep last valid control, but brake safely
                v = float(np.clip(self.last_u[0] * 0.8, 0.0, self.v_max))
                omega = float(np.clip(self.last_u[1] * 0.8, -self.omega_max, self.omega_max))
                self.last_u = np.array([v, omega])
                
        except Exception as e:
            self.get_logger().error(f"MPC computation failed: {e}")

        # Safety clipping
        v = float(np.clip(v, 0.0, self.v_max))
        omega = float(np.clip(omega, -self.omega_max, self.omega_max))

        # Publish command
        self._send(v, omega)

        # Publish diagnostics
        e = Float32(); e.data = float(dist_err); self.err_pub.publish(e)
        m = String(); m.data = mode; self.mode_pub.publish(m)

        # Log
        if self.tick_count % 20 == 0:
            self.get_logger().info(
                f'[MPC {self.current_idx}/{self.N}] '
                f'pos=({self.x:+.2f},{self.y:+.2f}) '
                f'err={dist_err:.3f}m | '
                f'v={v:.3f} ω={omega:+.3f} | {mode}'
            )

        self.tick_count += 1

    def _send(self, v, omega):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    def _pub_ref_path_once(self):
        if self._ref_published:
            return
        self._ref_published = True
        p = Path()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'odom'
        for i in range(0, self.N, 5):
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.pose.position.x = float(self.ref_x[i])
            ps.pose.position.y = float(self.ref_y[i])
            ps.pose.orientation.z = float(np.sin(self.ref_theta[i] / 2))
            ps.pose.orientation.w = float(np.cos(self.ref_theta[i] / 2))
            p.poses.append(ps)
        self.ref_pub.publish(p)

    def _pub_pred_path(self, states):
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


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._send(0.0, 0.0)
        node.get_logger().info('MPC Node Stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
