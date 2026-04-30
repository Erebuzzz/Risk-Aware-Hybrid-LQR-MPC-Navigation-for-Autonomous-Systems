#!/usr/bin/env python3
"""
Minimal Drive Test for TurtleBot3
==================================
Tests that cmd_vel (TwistStamped) actually drives the robot.

Phase 1 (0-5s): Drive straight forward at 0.1 m/s
Phase 2 (5-15s): Drive in a circle (v=0.1, omega=0.3)
Phase 3 (15s+): Stop

If the robot does NOT move: the cmd_vel bridge is broken.
If the robot DOES move: the controller algorithm is the issue.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math


class DriveTestNode(Node):
    def __init__(self):
        super().__init__('drive_test')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', qos)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos)

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz
        self.start_time = None
        self.odom_count = 0
        self.x = 0.0
        self.y = 0.0
        self.get_logger().info('🧪 Drive test started. Waiting for odom...')

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.odom_count += 1
        if self.odom_count == 1:
            self.start_time = self.get_clock().now()
            self.get_logger().info(f'📡 First odom at ({self.x:.3f}, {self.y:.3f})')

    def tick(self):
        if self.start_time is None:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if elapsed < 5.0:
            # Phase 1: Drive straight
            cmd.twist.linear.x = 0.1
            cmd.twist.angular.z = 0.0
            phase = 'STRAIGHT'
        elif elapsed < 15.0:
            # Phase 2: Circle
            cmd.twist.linear.x = 0.1
            cmd.twist.angular.z = 0.3
            phase = 'CIRCLE'
        else:
            # Phase 3: Stop
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            phase = 'STOP'

        self.cmd_pub.publish(cmd)

        if int(elapsed * 20) % 20 == 0:  # Log every ~1s
            dist = math.sqrt(self.x**2 + self.y**2)
            self.get_logger().info(
                f't={elapsed:.1f}s | {phase} | '
                f'pos=({self.x:.3f}, {self.y:.3f}) dist={dist:.3f}m'
            )


def main(args=None):
    rclpy.init(args=args)
    node = DriveTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cmd = TwistStamped()
        cmd.header.stamp = node.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        node.cmd_pub.publish(cmd)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
