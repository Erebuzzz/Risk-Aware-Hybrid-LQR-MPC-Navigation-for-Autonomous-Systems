#!/usr/bin/env python3
"""
Twist Relay Node
================

Subscribes to TwistStamped on /cmd_vel and republishes as plain Twist
on /cmd_vel_bridge for ros_gz_bridge compatibility.

TurtleBot3's GZ Sim diff-drive plugin expects geometry_msgs/Twist via
the ros_gz_bridge, but our controller nodes publish TwistStamped for
proper timestamping and frame_id metadata.

This lightweight relay bridges the gap.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped


class TwistRelayNode(Node):
    def __init__(self):
        super().__init__('twist_relay')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscribe to TwistStamped from controller nodes
        self.create_subscription(
            TwistStamped, '/cmd_vel', self._relay_cb, qos
        )

        # Publish plain Twist for the ros_gz_bridge
        self.bridge_pub = self.create_publisher(Twist, '/cmd_vel_bridge', qos)

        self.get_logger().info(
            '🔄 Twist Relay: /cmd_vel (TwistStamped) → /cmd_vel_bridge (Twist)'
        )

    def _relay_cb(self, msg: TwistStamped):
        out = Twist()
        out.linear = msg.twist.linear
        out.angular = msg.twist.angular
        self.bridge_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
