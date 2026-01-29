#!/usr/bin/env python3
"""
LQR Controller Node
===================

ROS2 node that implements LQR trajectory tracking control.

Subscribers:
    /odom (nav_msgs/Odometry): Robot state from odometry
    /current_reference (geometry_msgs/PoseStamped): Current reference pose
    /reference_velocity (geometry_msgs/Twist): Reference velocity

Publishers:
    /cmd_vel (geometry_msgs/Twist): Velocity commands
    /lqr/tracking_error (std_msgs/Float32MultiArray): Tracking error

Parameters:
    Q_diag: State error weights [q_x, q_y, q_theta]
    R_diag: Control effort weights [r_v, r_omega]
    control_rate: Control loop rate (Hz)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32MultiArray

import numpy as np

from ..controllers.lqr_controller import LQRController
from ..logging.simulation_logger import SimulationLogger


class LQRControllerNode(Node):
    """
    ROS2 node for LQR trajectory tracking control.
    """
    
    def __init__(self):
        super().__init__('lqr_controller')
        
        # Declare parameters
        self.declare_parameter('Q_diag', [10.0, 10.0, 1.0])
        self.declare_parameter('R_diag', [0.1, 0.1])
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('v_max', 1.0)
        self.declare_parameter('omega_max', 1.5)
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('log_level', 'INFO')
        
        # Get parameters
        Q_diag = self.get_parameter('Q_diag').value
        R_diag = self.get_parameter('R_diag').value
        control_rate = self.get_parameter('control_rate').value
        v_max = self.get_parameter('v_max').value
        omega_max = self.get_parameter('omega_max').value
        dt = self.get_parameter('dt').value
        log_level = self.get_parameter('log_level').value
        
        # Initialize LQR controller
        self.lqr = LQRController(
            Q_diag=Q_diag,
            R_diag=R_diag,
            dt=dt,
            v_max=v_max,
            omega_max=omega_max
        )
        
        # Initialize logger
        self.logger = SimulationLogger(
            log_dir='logs',
            log_level=log_level,
            node_name='lqr_controller'
        )
        
        # State storage
        self.current_state = np.zeros(3)
        self.current_ref_state = np.zeros(3)
        self.current_ref_control = np.zeros(2)
        self.timestep = 0
        
        # Flags
        self.state_received = False
        self.ref_received = False
        
        # QoS
        qos = QoSProfile(depth=10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos)
        
        self.ref_pose_sub = self.create_subscription(
            PoseStamped, '/current_reference', self.ref_pose_callback, qos)
        
        self.ref_vel_sub = self.create_subscription(
            Twist, '/reference_velocity', self.ref_vel_callback, qos)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.error_pub = self.create_publisher(Float32MultiArray, '/lqr/tracking_error', qos)
        
        # Control timer
        self.control_timer = self.create_timer(1.0 / control_rate, self.control_callback)
        
        self.get_logger().info(
            f"LQR controller initialized: Q={Q_diag}, R={R_diag}, rate={control_rate}Hz"
        )
    
    def odom_callback(self, msg: Odometry):
        """Update current robot state from odometry."""
        self.current_state[0] = msg.pose.pose.position.x
        self.current_state[1] = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.current_state[2] = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        
        self.state_received = True
    
    def ref_pose_callback(self, msg: PoseStamped):
        """Update reference pose."""
        self.current_ref_state[0] = msg.pose.position.x
        self.current_ref_state[1] = msg.pose.position.y
        
        q = msg.pose.orientation
        self.current_ref_state[2] = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        
        self.ref_received = True
    
    def ref_vel_callback(self, msg: Twist):
        """Update reference velocity."""
        self.current_ref_control[0] = msg.linear.x
        self.current_ref_control[1] = msg.angular.z
    
    def control_callback(self):
        """Execute LQR control loop."""
        if not (self.state_received and self.ref_received):
            return
        
        # Compute LQR control
        u, error = self.lqr.compute_control_at_operating_point(
            self.current_state,
            self.current_ref_state,
            self.current_ref_control
        )
        
        # Publish velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = float(u[0])
        cmd_msg.angular.z = float(u[1])
        self.cmd_pub.publish(cmd_msg)
        
        # Publish tracking error
        error_msg = Float32MultiArray()
        error_msg.data = [float(e) for e in error]
        self.error_pub.publish(error_msg)
        
        # Log state and control
        self.logger.log_state(
            timestep=self.timestep,
            state=self.current_state,
            state_ref=self.current_ref_state,
            error=error
        )
        
        self.logger.log_control(
            timestep=self.timestep,
            control=u,
            controller_type='LQR'
        )
        
        self.timestep += 1


def main(args=None):
    rclpy.init(args=args)
    node = LQRControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Export logs
        node.logger.finalize()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
