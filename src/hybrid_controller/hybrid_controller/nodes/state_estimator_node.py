#!/usr/bin/env python3
"""
State Estimator Node
====================

ROS2 node that processes odometry and provides state estimates.

Subscribers:
    /odom (nav_msgs/Odometry): Robot odometry from Gazebo/sensors

Publishers:
    /robot_state (std_msgs/Float32MultiArray): Filtered state [px, py, theta]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

import numpy as np
from tf_transformations import euler_from_quaternion


class StateEstimatorNode(Node):
    """
    ROS2 node for state estimation from odometry.
    
    Currently implements direct pass-through from odometry.
    Can be extended with Kalman filtering if needed.
    """
    
    def __init__(self):
        super().__init__('state_estimator')
        
        # State storage
        self.state = np.zeros(3)  # [px, py, theta]
        self.velocity = np.zeros(2)  # [v, omega]
        
        # QoS
        qos = QoSProfile(depth=10)
        
        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos
        )
        
        # Publisher
        self.state_pub = self.create_publisher(
            Float32MultiArray,
            '/robot_state',
            qos
        )
        
        self.get_logger().info("State estimator initialized")
    
    def odom_callback(self, msg: Odometry):
        """Process odometry message and publish state."""
        # Extract position
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        
        # Extract orientation (quaternion to euler)
        q = msg.pose.pose.orientation
        try:
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.state[2] = yaw
        except:
            # Fallback: compute yaw from quaternion manually
            self.state[2] = np.arctan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
        
        # Extract velocity
        self.velocity[0] = msg.twist.twist.linear.x
        self.velocity[1] = msg.twist.twist.angular.z
        
        # Publish state
        state_msg = Float32MultiArray()
        state_msg.data = [float(s) for s in self.state]
        self.state_pub.publish(state_msg)
    
    def get_state(self) -> np.ndarray:
        """Return current state estimate."""
        return self.state.copy()
    
    def get_velocity(self) -> np.ndarray:
        """Return current velocity estimate."""
        return self.velocity.copy()


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
