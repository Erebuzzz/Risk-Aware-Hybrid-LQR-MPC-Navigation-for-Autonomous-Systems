#!/usr/bin/env python3
"""
Pure Adaptive MPC Controller Node for TurtleBot3
================================================

This node strictly runs the Adaptive MPC algorithm for trajectory tracking
and obstacle avoidance. It does NOT include any hybrid logic.
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

from hybrid_controller.controllers.adaptive_mpc_controller import AdaptiveMPCController
from hybrid_nav.trajectory_index import nearest_idx_closed


def euler_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

class DummyObstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

class AdaptiveMPCControllerNode(Node):
    def __init__(self):
        super().__init__('adaptive_mpc_controller_node')

        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('trajectory_amplitude', 0.5)
        self.declare_parameter('trajectory_frequency', 0.15)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('omega_max', 2.84)
        
        self.declare_parameter('horizon', 10)
        self.declare_parameter('d_safe', 0.35)
        self.declare_parameter('q_x', 30.0)
        self.declare_parameter('q_y', 30.0)
        self.declare_parameter('q_theta', 5.0)
        self.declare_parameter('r_v', 0.1)
        self.declare_parameter('r_omega', 0.1)

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

        self.mpc = AdaptiveMPCController(
            prediction_horizon=self.horizon,
            dt=self.dt,
            d_safe=self.d_safe,
            v_max=self.v_max,
            omega_max=self.omega_max,
            Q_diag=Q_diag,
            R_diag=R_diag,
            enable_adaptation=True
        )
        self.get_logger().info('✅ Pure Adaptive MPC Controller Initialized.')

        self._generate_trajectory()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_ok = False
        self.tick_count = 0
        self.current_idx = 0
        self.obstacles = []
        
        self.x_prev = None
        self.u_prev = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)
        self.create_subscription(Float32MultiArray, '/obstacles', self._obs_cb, qos)

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', qos)
        self.err_pub = self.create_publisher(Float32, '/mpc/tracking_error', qos)
        self.mode_pub = self.create_publisher(String, '/mpc/mode', qos)
        
        qos_static = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.ref_pub = self.create_publisher(Path, '/hybrid/reference_path', qos_static)
        self.pred_pub = self.create_publisher(Path, '/hybrid/predicted_path', 1)
        self.trail_pub = self.create_publisher(Path, '/hybrid/robot_trail', qos)
        
        self.robot_trail = Path()
        self.robot_trail.header.frame_id = 'odom'

        self._run_file_logger, log_file = self._setup_run_file_logger('adaptive_mpc_node')
        self._run_file_logger.info('Tick,X,Y,Theta,Error,V,Omega,Params,Mode')

        self.create_timer(1.0 / rate, self._control_loop)
        self._ref_published = False
        self._pub_ref_path_once()

        self.get_logger().info(f'🤖 Adaptive MPC Node Ready. Logging to {log_file}')

    def _generate_trajectory(self):
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
            dtheta = normalize_angle(self.ref_theta[i] - self.ref_theta[i-1])
            self.ref_omega[i] = dtheta / self.dt
        self.ref_omega[0] = self.ref_omega[1]
        
        self.N = len(t)
        self.get_logger().info(f'📐 Generated EXACTLY 1 loop: {self.N} points.')

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = euler_from_quaternion(msg.pose.pose.orientation)
        if not self.odom_ok:
            self.odom_ok = True
            self.get_logger().info(f'📡 Odom OK: ({self.x:.3f}, {self.y:.3f})')
            
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.robot_trail.poses.append(ps)
        self.robot_trail.header.stamp = self.get_clock().now().to_msg()
        self.trail_pub.publish(self.robot_trail)

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

    def _control_loop(self):
        if not self.odom_ok:
            return

        x_current = np.array([self.x, self.y, self.theta])

        # Adapt parameters using previous state and control
        if self.x_prev is not None and self.u_prev is not None:
            self.mpc.adapt_parameters(x_current, self.x_prev, self.u_prev)

        self.current_idx, dist_err = nearest_idx_closed(
            self.ref_x, self.ref_y, self.current_idx, self.x, self.y, self.N
        )

        x_refs = np.zeros((self.horizon + 1, 3))
        u_refs = np.zeros((self.horizon, 2))
        
        start_idx = (self.current_idx + 2) % self.N
        for k in range(self.horizon + 1):
            idx = (start_idx + k) % self.N
            x_refs[k] = [self.ref_x[idx], self.ref_y[idx], self.ref_theta[idx]]
            if k < self.horizon:
                u_refs[k] = [self.ref_v[idx], self.ref_omega[idx]]
        
        v, omega = 0.0, 0.0
        mode = "AMPC_FAILED"
        try:
            solution = self.mpc.solve_tracking(
                x0=x_current, 
                x_refs=x_refs, 
                u_refs=u_refs, 
                obstacles=self.obstacles
            )
            
            if solution.feasible:
                v = float(solution.optimal_control[0])
                omega = float(solution.optimal_control[1])
                mode = f"AMPC_OK (Slack={solution.feasibility_margin:.2f})"
                
                if solution.predicted_states is not None:
                    self._pub_pred_path(solution.predicted_states)
            else:
                self.get_logger().warn("Adaptive MPC Infeasible!")
                mode = "AMPC_INFEASIBLE"
                if self.u_prev is not None:
                    v = float(np.clip(self.u_prev[0] * 0.8, -self.v_max, self.v_max))
                    omega = float(np.clip(self.u_prev[1] * 0.8, -self.omega_max, self.omega_max))
                
        except Exception as e:
            self.get_logger().error(f"Adaptive MPC computation failed: {e}")

        v = float(np.clip(v, -self.v_max, self.v_max))
        omega = float(np.clip(omega, -self.omega_max, self.omega_max))

        self.x_prev = x_current
        self.u_prev = np.array([v, omega])

        self._send(v, omega)

        e = Float32(); e.data = float(dist_err); self.err_pub.publish(e)
        m = String(); m.data = mode; self.mode_pub.publish(m)
        
        params = self.mpc.param_estimates if hasattr(self.mpc, 'param_estimates') else [0.0, 0.0]
        self._run_file_logger.info(
            f"{self.tick_count},{self.x:.3f},{self.y:.3f},{self.theta:.3f},"
            f"{dist_err:.3f},{v:.3f},{omega:.3f},"
            f'"{params[0]:.2f}_{params[1]:.2f}",{mode}'
        )

        if self.tick_count % 20 == 0:
            params = self.mpc.param_estimates
            self.get_logger().info(
                f'[AMPC {self.current_idx}/{self.N}] '
                f'pos=({self.x:+.2f},{self.y:+.2f}) '
                f'err={dist_err:.3f}m | '
                f'v={v:.3f} ω={omega:+.3f} | Params=[{params[0]:.2f}, {params[1]:.2f}] | {mode}'
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
    node = AdaptiveMPCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._send(0.0, 0.0)
        node.get_logger().info('Adaptive MPC Node Stopped.')
    finally:
        node._close_run_file_logger()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
