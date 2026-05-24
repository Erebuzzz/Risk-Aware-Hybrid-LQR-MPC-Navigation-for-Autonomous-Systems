#!/usr/bin/env python3
"""
Dynamic obstacle publisher for Option B experiments.

Publishes /obstacles as either:
    triplet:   [x, y, r, ...]
    quintuple: [x, y, r, vx, vy, ...]

The adaptive safety-filter node consumes both formats. The quintuple format is
used for moving-obstacle risk prediction.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray


class DynamicObstaclePublisherNode(Node):
    def __init__(self):
        super().__init__("dynamic_obstacle_publisher_node")

        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("message_format", "quintuple")
        self.declare_parameter("scenario", "crossing")
        self.declare_parameter("include_static_obstacles", True)
        self.declare_parameter("dynamic_radius", 0.07)
        self.declare_parameter("crossing_x_start", -0.65)
        self.declare_parameter("crossing_y_start", 0.02)
        self.declare_parameter("crossing_x_end", 0.65)
        self.declare_parameter("crossing_y_end", 0.02)
        self.declare_parameter("crossing_speed", 0.08)
        self.declare_parameter("start_delay", 4.0)
        self.declare_parameter("sin_center_x", 0.15)
        self.declare_parameter("sin_center_y", 0.0)
        self.declare_parameter("sin_amplitude", 0.35)
        self.declare_parameter("sin_frequency", 0.04)

        self.start_time = self._now_sec()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Float32MultiArray, "/obstacles", qos)

        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / max(rate, 1e-6), self.publish_obstacles)

        self.static_obstacles = [
            (0.30, 0.35, 0.05),
            (-0.35, -0.35, 0.05),
            (0.55, 0.00, 0.05),
        ]
        self.get_logger().info(
            "Dynamic obstacle publisher ready: "
            f"scenario={self.get_parameter('scenario').value}, "
            f"format={self.get_parameter('message_format').value}"
        )

    def publish_obstacles(self):
        msg = Float32MultiArray()
        data = []

        if bool(self.get_parameter("include_static_obstacles").value):
            for x, y, radius in self.static_obstacles:
                self._append_obstacle(data, x, y, radius, 0.0, 0.0)

        x, y, radius, vx, vy = self._dynamic_obstacle()
        self._append_obstacle(data, x, y, radius, vx, vy)

        msg.data = [float(v) for v in data]
        self.pub.publish(msg)

    def _dynamic_obstacle(self):
        scenario = str(self.get_parameter("scenario").value).lower()
        elapsed = max(0.0, self._now_sec() - self.start_time)
        elapsed = max(0.0, elapsed - float(self.get_parameter("start_delay").value))
        radius = float(self.get_parameter("dynamic_radius").value)

        if scenario == "sinusoidal":
            cx = float(self.get_parameter("sin_center_x").value)
            cy = float(self.get_parameter("sin_center_y").value)
            amplitude = float(self.get_parameter("sin_amplitude").value)
            freq = float(self.get_parameter("sin_frequency").value)
            phase = 2.0 * math.pi * freq * elapsed
            y = cy + amplitude * math.sin(phase)
            vy = amplitude * 2.0 * math.pi * freq * math.cos(phase)
            return cx, y, radius, 0.0, vy

        return self._crossing_obstacle(elapsed, radius)

    def _crossing_obstacle(self, elapsed, radius):
        x0 = float(self.get_parameter("crossing_x_start").value)
        y0 = float(self.get_parameter("crossing_y_start").value)
        x1 = float(self.get_parameter("crossing_x_end").value)
        y1 = float(self.get_parameter("crossing_y_end").value)
        speed = max(float(self.get_parameter("crossing_speed").value), 1e-6)

        dx = x1 - x0
        dy = y1 - y0
        distance = math.hypot(dx, dy)
        if distance < 1e-9:
            return x0, y0, radius, 0.0, 0.0

        segment_time = distance / speed
        cycle_time = 2.0 * segment_time
        phase = elapsed % cycle_time
        forward = phase <= segment_time
        s = phase / segment_time if forward else (cycle_time - phase) / segment_time
        direction = 1.0 if forward else -1.0

        x = x0 + s * dx
        y = y0 + s * dy
        vx = direction * speed * dx / distance
        vy = direction * speed * dy / distance
        return x, y, radius, vx, vy

    def _append_obstacle(self, data, x, y, radius, vx, vy):
        fmt = str(self.get_parameter("message_format").value).lower()
        if fmt == "triplet":
            data.extend([x, y, radius])
        else:
            data.extend([x, y, radius, vx, vy])

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstaclePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
