#!/usr/bin/env python3
"""
Risk-triggered adaptive MPC safety-filter node.

This node implements the new publication-oriented architecture:

    LQR nominal tracking
        -> rollout risk prediction
        -> adaptive MPC safety filter when unsafe
        -> backup stop/turn controller when MPC is unavailable

The existing blended hybrid node is intentionally left untouched so it can be
used as an experiment baseline.
"""

import logging
import os
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Float32MultiArray, String

from hybrid_controller.controllers.adaptive_mpc_safety_filter import (
    AdaptiveMPCSafetyFilter,
)
from hybrid_nav.trajectory_index import nearest_idx_closed


def euler_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class DummyObstacle:
    __slots__ = ("x", "y", "radius")

    def __init__(self, x, y, radius):
        self.x = float(x)
        self.y = float(y)
        self.radius = float(radius)


class AdaptiveSafetyFilterNode(Node):
    def __init__(self):
        super().__init__("adaptive_safety_filter_node")

        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("trajectory_amplitude", 0.5)
        self.declare_parameter("trajectory_frequency", 0.15)
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("v_max", 0.22)
        self.declare_parameter("omega_max", 2.84)
        self.declare_parameter("horizon", 10)
        self.declare_parameter("terminal_horizon", 5)
        self.declare_parameter("d_safe", 0.30)
        self.declare_parameter("q_x", 30.0)
        self.declare_parameter("q_y", 30.0)
        self.declare_parameter("q_theta", 5.0)
        self.declare_parameter("r_v", 0.1)
        self.declare_parameter("r_omega", 0.1)
        self.declare_parameter("adaptation_gamma", 0.005)
        self.declare_parameter("mpc_time_limit_ms", 250.0)

        rate = float(self.get_parameter("control_rate").value)
        self.A = float(self.get_parameter("trajectory_amplitude").value)
        self.freq = float(self.get_parameter("trajectory_frequency").value)
        self.dt = float(self.get_parameter("dt").value)
        self.v_max = float(self.get_parameter("v_max").value)
        self.omega_max = float(self.get_parameter("omega_max").value)
        self.horizon = int(self.get_parameter("horizon").value)
        terminal_horizon = int(self.get_parameter("terminal_horizon").value)
        self.d_safe = float(self.get_parameter("d_safe").value)

        q_diag = [
            float(self.get_parameter("q_x").value),
            float(self.get_parameter("q_y").value),
            float(self.get_parameter("q_theta").value),
        ]
        r_diag = [
            float(self.get_parameter("r_v").value),
            float(self.get_parameter("r_omega").value),
        ]

        self.controller = AdaptiveMPCSafetyFilter(
            prediction_horizon=self.horizon,
            terminal_horizon=terminal_horizon,
            Q_diag=q_diag,
            R_diag=r_diag,
            d_safe=self.d_safe,
            dt=self.dt,
            v_max=self.v_max,
            omega_max=self.omega_max,
            adaptation_gamma=float(self.get_parameter("adaptation_gamma").value),
            mpc_time_limit_ms=float(self.get_parameter("mpc_time_limit_ms").value),
        )

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
        qos_static = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(Odometry, "/odom", self._odom_cb, qos)
        self.create_subscription(Float32MultiArray, "/obstacles", self._obs_cb, qos)

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", qos)
        self.err_pub = self.create_publisher(
            Float32, "/adaptive_safety_filter/tracking_error", qos
        )
        self.mode_pub = self.create_publisher(
            String, "/adaptive_safety_filter/mode", qos
        )
        self.risk_pub = self.create_publisher(
            Float32, "/adaptive_safety_filter/risk", qos
        )
        self.clearance_pub = self.create_publisher(
            Float32, "/adaptive_safety_filter/min_margin", qos
        )
        self.ref_pub = self.create_publisher(Path, "/hybrid/reference_path", qos_static)
        self.pred_pub = self.create_publisher(Path, "/hybrid/predicted_path", 1)
        self.trail_pub = self.create_publisher(Path, "/hybrid/robot_trail", qos)

        self.robot_trail = Path()
        self.robot_trail.header.frame_id = "odom"

        self._run_file_logger, log_file = self._setup_run_file_logger(
            "adaptive_safety_filter_node"
        )
        self._run_file_logger.info(
            "Tick,X,Y,Theta,Error,V,Omega,Mode,Risk,MinMargin,"
            "SolverMs,ThetaV,ThetaOmega"
        )

        self.create_timer(1.0 / rate, self._control_loop)
        self._ref_published = False
        self._pub_ref_path_once()

        self.get_logger().info(
            "Adaptive safety-filter node ready. "
            f"N={self.horizon}, M={terminal_horizon}, log={log_file}"
        )

    def _generate_trajectory(self):
        w = self.freq
        period = 2.0 * np.pi / w
        t = np.arange(0.0, period, self.dt)

        self.ref_x = self.A * np.sin(w * t)
        self.ref_y = (self.A / 2.0) * np.sin(2.0 * w * t)

        dx_dt = self.A * w * np.cos(w * t)
        dy_dt = self.A * w * np.cos(2.0 * w * t)

        self.ref_theta = np.arctan2(dy_dt, dx_dt)
        self.ref_v = np.clip(np.sqrt(dx_dt**2 + dy_dt**2), 0.0, self.v_max)
        self.ref_omega = np.zeros_like(t)
        for i in range(1, len(t)):
            dtheta = normalize_angle(self.ref_theta[i] - self.ref_theta[i - 1])
            self.ref_omega[i] = np.clip(dtheta / self.dt, -self.omega_max, self.omega_max)
        if len(self.ref_omega) > 1:
            self.ref_omega[0] = self.ref_omega[1]

        self.N_traj = len(t)
        self.get_logger().info(
            f"Generated one figure-8 loop: {self.N_traj} points."
        )

    def _odom_cb(self, msg):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.theta = float(euler_from_quaternion(msg.pose.pose.orientation))
        if not self.odom_ok:
            self.odom_ok = True
            self.get_logger().info(f"Odom OK: ({self.x:.3f}, {self.y:.3f})")

        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.robot_trail.poses.append(ps)
        self.robot_trail.header.stamp = self.get_clock().now().to_msg()
        self.trail_pub.publish(self.robot_trail)

    def _obs_cb(self, msg):
        obstacles = []
        for i in range(0, len(msg.data), 3):
            if i + 2 < len(msg.data):
                obstacles.append(
                    DummyObstacle(msg.data[i], msg.data[i + 1], msg.data[i + 2])
                )
        self.obstacles = obstacles

    def _control_loop(self):
        if not self.odom_ok:
            return

        x_current = np.array([self.x, self.y, self.theta], dtype=float)

        if self.x_prev is not None and self.u_prev is not None:
            self.controller.adapt_parameters(x_current, self.x_prev, self.u_prev)

        self.current_idx, dist_err = nearest_idx_closed(
            self.ref_x,
            self.ref_y,
            self.current_idx,
            self.x,
            self.y,
            self.N_traj,
        )

        x_refs, u_refs = self._reference_window()

        try:
            result = self.controller.compute_control(
                x0=x_current,
                x_refs=x_refs,
                u_refs=u_refs,
                obstacles=self.obstacles,
            )
            v = float(result.optimal_control[0])
            omega = float(result.optimal_control[1])
        except Exception as exc:
            self.get_logger().error(f"Safety-filter computation failed: {exc}")
            result = None
            v, omega = 0.0, 0.0

        v = float(np.clip(v, -self.v_max, self.v_max))
        omega = float(np.clip(omega, -self.omega_max, self.omega_max))

        self.x_prev = x_current
        self.u_prev = np.array([v, omega], dtype=float)

        self._send(v, omega)

        if result is not None:
            self._publish_diagnostics(dist_err, result)
            if result.predicted_states is not None:
                self._pub_pred_path(result.predicted_states)

            theta_hat = result.theta_hat
            self._run_file_logger.info(
                f"{self.tick_count},{self.x:.3f},{self.y:.3f},{self.theta:.3f},"
                f"{dist_err:.3f},{v:.3f},{omega:.3f},{result.mode},"
                f"{result.risk.risk_score:.3f},{result.risk.min_margin:.3f},"
                f"{result.solver_time_ms:.1f},{theta_hat[0]:.3f},{theta_hat[1]:.3f}"
            )

            if self.tick_count % 20 == 0:
                self.get_logger().info(
                    f"[ASF {self.current_idx}/{self.N_traj}] "
                    f"err={dist_err:.3f}m risk={result.risk.risk_score:.2f} "
                    f"margin={result.risk.min_margin:.2f} "
                    f"v={v:.3f} omega={omega:+.3f} mode={result.mode}"
                )
        else:
            self._publish_failure_diagnostics(dist_err)

        self.tick_count += 1

    def _reference_window(self):
        state_len = self.controller.required_state_horizon
        control_len = self.controller.required_control_horizon
        x_refs = np.zeros((state_len, 3), dtype=float)
        u_refs = np.zeros((control_len, 2), dtype=float)

        start_idx = (self.current_idx + 2) % self.N_traj
        for k in range(state_len):
            idx = (start_idx + k) % self.N_traj
            x_refs[k] = [self.ref_x[idx], self.ref_y[idx], self.ref_theta[idx]]
            if k < control_len:
                u_refs[k] = [self.ref_v[idx], self.ref_omega[idx]]

        return x_refs, u_refs

    def _send(self, v, omega):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    def _publish_diagnostics(self, dist_err, result):
        e = Float32()
        e.data = float(dist_err)
        self.err_pub.publish(e)

        mode = String()
        mode.data = result.mode
        self.mode_pub.publish(mode)

        risk = Float32()
        risk.data = float(result.risk.risk_score)
        self.risk_pub.publish(risk)

        margin = Float32()
        margin.data = float(result.risk.min_margin)
        self.clearance_pub.publish(margin)

    def _publish_failure_diagnostics(self, dist_err):
        e = Float32()
        e.data = float(dist_err)
        self.err_pub.publish(e)
        mode = String()
        mode.data = "FAILED"
        self.mode_pub.publish(mode)

    def _pub_ref_path_once(self):
        if self._ref_published:
            return
        self._ref_published = True
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "odom"
        for i in range(0, self.N_traj, 5):
            ps = PoseStamped()
            ps.header.frame_id = "odom"
            ps.pose.position.x = float(self.ref_x[i])
            ps.pose.position.y = float(self.ref_y[i])
            ps.pose.orientation.z = float(np.sin(self.ref_theta[i] / 2.0))
            ps.pose.orientation.w = float(np.cos(self.ref_theta[i] / 2.0))
            path.poses.append(ps)
        self.ref_pub.publish(path)

    def _pub_pred_path(self, states):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "odom"
        for state in states:
            ps = PoseStamped()
            ps.header.frame_id = "odom"
            ps.pose.position.x = float(state[0])
            ps.pose.position.y = float(state[1])
            path.poses.append(ps)
        self.pred_pub.publish(path)

    def _setup_run_file_logger(self, basename):
        log_dir = os.path.join(os.getcwd(), "Output", "Logs")
        os.makedirs(log_dir, exist_ok=True)
        path = os.path.join(
            log_dir, f"{basename}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        )
        logger = logging.getLogger(f"hybrid_nav.{basename}_{id(self)}")
        logger.setLevel(logging.DEBUG)
        logger.handlers.clear()
        fh = logging.FileHandler(path, encoding="utf-8")
        fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        logger.addHandler(fh)
        logger.propagate = False
        return logger, path

    def _close_run_file_logger(self):
        log = getattr(self, "_run_file_logger", None)
        if not log:
            return
        for handler in list(log.handlers):
            log.removeHandler(handler)
            handler.flush()
            handler.close()
        self._run_file_logger = None


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveSafetyFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._send(0.0, 0.0)
        node.get_logger().info("Adaptive safety-filter node stopped.")
    finally:
        node._close_run_file_logger()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
