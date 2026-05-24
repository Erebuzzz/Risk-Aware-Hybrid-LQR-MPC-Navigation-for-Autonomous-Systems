#!/usr/bin/env python3
"""
Command distortion node for actuation-mismatch experiments.

The adaptive controller learns scale parameters only if Gazebo actually applies
different velocities from the commands it sends. This node sits between the
controller and the Gazebo relay:

    /cmd_vel_raw  ->  command_distortion_node  ->  /cmd_vel

It applies configurable linear and angular velocity scaling, optional step or
sinusoidal drift, and small command noise.
"""

import math
import random

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32


class CommandDistortionNode(Node):
    def __init__(self):
        super().__init__("command_distortion_node")

        self.declare_parameter("enabled", True)
        self.declare_parameter("input_topic", "/cmd_vel_raw")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("profile", "step")
        self.declare_parameter("lambda_v", 0.85)
        self.declare_parameter("lambda_omega", 1.15)
        self.declare_parameter("lambda_v_after", 0.70)
        self.declare_parameter("lambda_omega_after", 1.30)
        self.declare_parameter("step_time", 25.0)
        self.declare_parameter("sin_amplitude_v", 0.10)
        self.declare_parameter("sin_amplitude_omega", 0.10)
        self.declare_parameter("sin_frequency", 0.03)
        self.declare_parameter("noise_std_v", 0.005)
        self.declare_parameter("noise_std_omega", 0.02)
        self.declare_parameter("min_scale", 0.40)
        self.declare_parameter("max_scale", 1.60)
        self.declare_parameter("v_max", 0.22)
        self.declare_parameter("omega_max", 2.84)
        self.declare_parameter("seed", 7)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.profile = str(self.get_parameter("profile").value).lower()

        seed = int(self.get_parameter("seed").value)
        self.rng = random.Random(None if seed < 0 else seed)
        self.start_time = self._now_sec()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(TwistStamped, self.output_topic, qos)
        self.lambda_v_pub = self.create_publisher(
            Float32, "/command_distortion/lambda_v", qos
        )
        self.lambda_omega_pub = self.create_publisher(
            Float32, "/command_distortion/lambda_omega", qos
        )
        self.create_subscription(TwistStamped, self.input_topic, self._cmd_cb, qos)

        self.get_logger().info(
            "Command distortion ready: "
            f"{self.input_topic} -> {self.output_topic}, "
            f"enabled={self.enabled}, profile={self.profile}"
        )

    def _cmd_cb(self, msg):
        lambda_v, lambda_omega = self._current_scales()
        out = TwistStamped()
        out.header = msg.header
        out.twist = msg.twist

        if self.enabled:
            noise_v = self.rng.gauss(0.0, float(self.get_parameter("noise_std_v").value))
            noise_omega = self.rng.gauss(
                0.0, float(self.get_parameter("noise_std_omega").value)
            )
            v = lambda_v * msg.twist.linear.x + noise_v
            omega = lambda_omega * msg.twist.angular.z + noise_omega
            out.twist.linear.x = self._clip(v, -self._v_max, self._v_max)
            out.twist.angular.z = self._clip(
                omega, -self._omega_max, self._omega_max
            )

        self.pub.publish(out)
        self._publish_scales(lambda_v, lambda_omega)

    def _current_scales(self):
        base_v = float(self.get_parameter("lambda_v").value)
        base_omega = float(self.get_parameter("lambda_omega").value)
        elapsed = max(0.0, self._now_sec() - self.start_time)

        if self.profile == "step" and elapsed >= float(
            self.get_parameter("step_time").value
        ):
            lambda_v = float(self.get_parameter("lambda_v_after").value)
            lambda_omega = float(self.get_parameter("lambda_omega_after").value)
        elif self.profile == "sinusoidal":
            phase = 2.0 * math.pi * float(self.get_parameter("sin_frequency").value)
            lambda_v = base_v + float(
                self.get_parameter("sin_amplitude_v").value
            ) * math.sin(phase * elapsed)
            lambda_omega = base_omega + float(
                self.get_parameter("sin_amplitude_omega").value
            ) * math.sin(phase * elapsed + math.pi / 2.0)
        else:
            lambda_v = base_v
            lambda_omega = base_omega

        return self._bounded_scale(lambda_v), self._bounded_scale(lambda_omega)

    def _publish_scales(self, lambda_v, lambda_omega):
        msg_v = Float32()
        msg_v.data = float(lambda_v)
        self.lambda_v_pub.publish(msg_v)

        msg_omega = Float32()
        msg_omega.data = float(lambda_omega)
        self.lambda_omega_pub.publish(msg_omega)

    def _bounded_scale(self, value):
        return self._clip(
            float(value),
            float(self.get_parameter("min_scale").value),
            float(self.get_parameter("max_scale").value),
        )

    @property
    def _v_max(self):
        return float(self.get_parameter("v_max").value)

    @property
    def _omega_max(self):
        return float(self.get_parameter("omega_max").value)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _clip(value, lower, upper):
        return max(lower, min(upper, float(value)))


def main(args=None):
    rclpy.init(args=args)
    node = CommandDistortionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
