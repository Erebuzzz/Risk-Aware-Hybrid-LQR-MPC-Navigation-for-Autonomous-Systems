#!/usr/bin/env python3
"""
Pure LQR Controller Node for TurtleBot3
=======================================

This node strictly runs the pure LQR algorithm to track a reference trajectory.
It does NOT include MPC or any hybrid logic.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, String

# Import the existing LQR controller logic
from hybrid_controller.controllers.lqr_controller import LQRController


def euler_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    """Wrap to [-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


class LQRControllerNode(Node):
    def __init__(self):
        super().__init__('lqr_controller_node')

        # ── Parameters ─────────────────────────────────────────
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('trajectory_amplitude', 0.5)
        self.declare_parameter('trajectory_frequency', 0.2)
        self.declare_parameter('trajectory_duration', 120.0)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('omega_max', 2.84)
        
        # LQR tuning weights
        self.declare_parameter('q_x', 5.0)
        self.declare_parameter('q_y', 5.0)
        self.declare_parameter('q_theta', 2.0)
        self.declare_parameter('r_v', 1.0)
        self.declare_parameter('r_omega', 0.5)

        rate = self.get_parameter('control_rate').value
        self.A = self.get_parameter('trajectory_amplitude').value
        self.freq = self.get_parameter('trajectory_frequency').value
        dur = self.get_parameter('trajectory_duration').value
        self.dt = self.get_parameter('dt').value
        self.v_max = self.get_parameter('v_max').value
        self.omega_max = self.get_parameter('omega_max').value

        Q_diag = [
            self.get_parameter('q_x').value,
            self.get_parameter('q_y').value,
            self.get_parameter('q_theta').value
        ]
        R_diag = [
            self.get_parameter('r_v').value,
            self.get_parameter('r_omega').value
        ]

        # ── Initialize pure LQR Controller ─────────────────────
        self.lqr = LQRController(
            Q_diag=Q_diag, 
            R_diag=R_diag, 
            dt=self.dt, 
            v_max=self.v_max, 
            omega_max=self.omega_max
        )
        self.get_logger().info(f'✅ Pure LQR Controller Initialized. Q={Q_diag}, R={R_diag}')

        # ── Trajectory: generate dense path ────────────────────
        self._generate_trajectory(dur)

        # ── State ──────────────────────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_ok = False
        self.tick_count = 0
        self.current_idx = 0

        # ── ROS ────────────────────────────────────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', qos)
        self.err_pub = self.create_publisher(Float32, '/lqr/tracking_error', qos)
        self.mode_pub = self.create_publisher(String, '/lqr/mode', qos)
        
        # Publish path on the topic RViz expects
        self.ref_pub = self.create_publisher(Path, '/hybrid/reference_path', 1)

        self.create_timer(1.0 / rate, self._control_loop)
        self.create_timer(3.0, self._pub_ref_path_once)
        self._ref_published = False

        self.get_logger().info('🤖 LQR Node Ready to track trajectory')

    # ── Generate Trajectory ────────────────────────────────────
    def _generate_trajectory(self, duration):
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

    # ── Main Control Loop ──────────────────────────────────────
    def _control_loop(self):
        if not self.odom_ok:
            return

        # 1. Find the target reference point based on nearest distance.
        # Use a wraparound moving window to prevent jumping at trajectory crossings
        # and to seamlessly loop back to the start.
        window = 40  # Search 40 points behind and ahead (~2 seconds)
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

        # Use a point further ahead on the trajectory to anticipate sharp curves
        # This keeps the LQR heading error small, preventing linearization breakdown
        target_idx = (self.current_idx + 15) % self.N

        # 2. Extract reference state and control
        x_ref = np.array([
            self.ref_x[target_idx],
            self.ref_y[target_idx],
            self.ref_theta[target_idx]
        ])
        
        u_ref = np.array([
            self.ref_v[target_idx],
            self.ref_omega[target_idx]
        ])
        
        x_current = np.array([self.x, self.y, self.theta])

        # 3. Call the Pure LQR Algorithm
        try:
            u_opt = self.lqr.compute_control(x_current, x_ref, u_ref)
            v = float(u_opt[0])
            omega = float(u_opt[1])
        except Exception as e:
            self.get_logger().error(f"LQR computation failed: {e}")
            v, omega = 0.0, 0.0

        # Safety clipping - strictly prevent backward driving!
        # When LQR linearization breaks down on sharp corners, it commands v < 0.
        # We must force v >= 0.0 so the robot always drives forward and corrects its heading.
        v = float(np.clip(v, 0.0, self.v_max))
        omega = float(np.clip(omega, -self.omega_max, self.omega_max))

        # Publish command
        self._send(v, omega)

        # Publish diagnostics
        e = Float32(); e.data = float(dist_err); self.err_pub.publish(e)
        m = String(); m.data = 'LQR_TRACKING'; self.mode_pub.publish(m)

        # Log
        if self.tick_count % 20 == 0:
            self.get_logger().info(
                f'[LQR {self.current_idx}/{self.N}] '
                f'pos=({self.x:+.2f},{self.y:+.2f}) '
                f'err={dist_err:.3f}m | '
                f'v={v:.3f} ω={omega:+.3f}'
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


def main(args=None):
    rclpy.init(args=args)
    node = LQRControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._send(0.0, 0.0)
        node.get_logger().info('LQR Node Stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
