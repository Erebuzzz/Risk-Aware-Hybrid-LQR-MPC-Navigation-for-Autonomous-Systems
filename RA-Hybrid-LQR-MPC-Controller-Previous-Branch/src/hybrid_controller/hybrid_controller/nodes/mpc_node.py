#!/usr/bin/env python3
"""
MPC Controller Node
===================

ROS2 node that implements MPC with obstacle avoidance.

Subscribers:
    /odom (nav_msgs/Odometry): Robot state from odometry
    /current_reference (geometry_msgs/PoseStamped): Current reference pose
    /reference_velocity (geometry_msgs/Twist): Reference velocity
    /mpc_obstacles (std_msgs/Float32MultiArray): Obstacle positions [x1,y1,r1, x2,y2,r2,...]

Publishers:
    /cmd_vel (geometry_msgs/Twist): Velocity commands
    /mpc/solve_time (std_msgs/Float32): Solver time in ms
    /mpc/predicted_path (nav_msgs/Path): MPC predicted trajectory

Parameters:
    horizon: Prediction horizon N
    d_safe: Safety distance from obstacles
    control_rate: Control loop rate (Hz)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32MultiArray, Float32

import numpy as np

from ..controllers.mpc_controller import MPCController, Obstacle
from ..trajectory.reference_generator import ReferenceTrajectoryGenerator
from ..logging.simulation_logger import SimulationLogger


class MPCControllerNode(Node):
    """
    ROS2 node for MPC control with obstacle avoidance.
    """
    
    def __init__(self):
        super().__init__('mpc_controller')
        
        # Declare parameters
        self.declare_parameter('horizon', 10)
        self.declare_parameter('Q_diag', [10.0, 10.0, 1.0])
        self.declare_parameter('R_diag', [0.1, 0.1])
        self.declare_parameter('P_diag', [20.0, 20.0, 2.0])
        self.declare_parameter('d_safe', 0.3)
        self.declare_parameter('slack_penalty', 1000.0)
        self.declare_parameter('v_max', 1.0)
        self.declare_parameter('omega_max', 1.5)
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('solver', 'ECOS')
        self.declare_parameter('log_level', 'INFO')
        
        # Get parameters
        horizon = self.get_parameter('horizon').value
        Q_diag = self.get_parameter('Q_diag').value
        R_diag = self.get_parameter('R_diag').value
        P_diag = self.get_parameter('P_diag').value
        d_safe = self.get_parameter('d_safe').value
        slack_penalty = self.get_parameter('slack_penalty').value
        v_max = self.get_parameter('v_max').value
        omega_max = self.get_parameter('omega_max').value
        dt = self.get_parameter('dt').value
        control_rate = self.get_parameter('control_rate').value
        solver = self.get_parameter('solver').value
        log_level = self.get_parameter('log_level').value
        
        self.horizon = horizon
        self.d_safe = d_safe
        
        # Initialize MPC controller
        self.mpc = MPCController(
            horizon=horizon,
            Q_diag=Q_diag,
            R_diag=R_diag,
            P_diag=P_diag,
            d_safe=d_safe,
            slack_penalty=slack_penalty,
            v_max=v_max,
            omega_max=omega_max,
            dt=dt,
            solver=solver
        )
        
        # Initialize trajectory generator for reference lookahead
        self.traj_gen = ReferenceTrajectoryGenerator(A=2.0, a=0.5, dt=dt)
        self.traj_gen.generate(duration=30.0)
        
        # Initialize logger
        self.logger = SimulationLogger(
            log_dir='logs',
            log_level=log_level,
            node_name='mpc_controller'
        )
        
        # State storage
        self.current_state = np.zeros(3)
        self.obstacles: list = []
        self.timestep = 0
        self.current_traj_idx = 0
        
        # Flags
        self.state_received = False
        
        # QoS
        qos = QoSProfile(depth=10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos)
        
        self.obstacle_sub = self.create_subscription(
            Float32MultiArray, '/mpc_obstacles', self.obstacle_callback, qos)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.solve_time_pub = self.create_publisher(Float32, '/mpc/solve_time', qos)
        self.predicted_path_pub = self.create_publisher(Path, '/mpc/predicted_path', qos)
        
        # Control timer
        self.control_timer = self.create_timer(1.0 / control_rate, self.control_callback)
        
        self.get_logger().info(
            f"MPC controller initialized: N={horizon}, d_safe={d_safe}, rate={control_rate}Hz"
        )
    
    def odom_callback(self, msg: Odometry):
        """Update current robot state from odometry."""
        self.current_state[0] = msg.pose.pose.position.x
        self.current_state[1] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        self.current_state[2] = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        
        self.state_received = True
    
    def obstacle_callback(self, msg: Float32MultiArray):
        """Update obstacle list from obstacle message."""
        # Format: [x1, y1, r1, x2, y2, r2, ...]
        data = msg.data
        self.obstacles = []
        
        for i in range(0, len(data), 3):
            if i + 2 < len(data):
                self.obstacles.append(Obstacle(
                    x=data[i],
                    y=data[i + 1],
                    radius=data[i + 2]
                ))
        
        self.get_logger().debug(f"Updated {len(self.obstacles)} obstacles")
    
    def control_callback(self):
        """Execute MPC control loop."""
        if not self.state_received:
            return
        
        # Get reference trajectory segment
        x_refs, u_refs = self.traj_gen.get_trajectory_segment(
            self.current_traj_idx,
            self.horizon + 1
        )
        
        # Solve MPC
        solution = self.mpc.solve(
            x0=self.current_state,
            x_refs=x_refs,
            u_refs=u_refs,
            obstacles=self.obstacles,
            use_soft_constraints=True
        )
        
        # Publish velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = float(solution.optimal_control[0])
        cmd_msg.angular.z = float(solution.optimal_control[1])
        self.cmd_pub.publish(cmd_msg)
        
        # Publish solve time
        solve_time_msg = Float32()
        solve_time_msg.data = float(solution.solve_time_ms)
        self.solve_time_pub.publish(solve_time_msg)
        
        # Publish predicted path
        self.publish_predicted_path(solution.predicted_states)
        
        # Log
        error = self.current_state - x_refs[0]
        error[2] = np.arctan2(np.sin(error[2]), np.cos(error[2]))
        
        self.logger.log_state(
            timestep=self.timestep,
            state=self.current_state,
            state_ref=x_refs[0],
            error=error
        )
        
        self.logger.log_control(
            timestep=self.timestep,
            control=solution.optimal_control,
            controller_type='MPC',
            solve_time=solution.solve_time_ms
        )
        
        if solution.slack_used:
            self.logger.log_constraint_event(
                timestep=self.timestep,
                constraint_type='slack_activated',
                details={'reason': 'obstacle_proximity'}
            )
        
        # Log obstacle proximity warnings
        for i, obs in enumerate(self.obstacles):
            dist = np.sqrt(
                (self.current_state[0] - obs.x)**2 + 
                (self.current_state[1] - obs.y)**2
            ) - obs.radius
            
            if dist < self.d_safe * 1.5:
                self.logger.log_obstacle_proximity(
                    timestep=self.timestep,
                    obstacle_id=i,
                    distance=dist,
                    warning_threshold=self.d_safe * 1.5
                )
        
        self.timestep += 1
        self.current_traj_idx += 1
    
    def publish_predicted_path(self, predicted_states: np.ndarray):
        """Publish MPC predicted trajectory for visualization."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        
        for state in predicted_states:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = state[0]
            pose.pose.position.y = state[1]
            pose.pose.orientation.z = np.sin(state[2] / 2)
            pose.pose.orientation.w = np.cos(state[2] / 2)
            path_msg.poses.append(pose)
        
        self.predicted_path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.logger.finalize()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
