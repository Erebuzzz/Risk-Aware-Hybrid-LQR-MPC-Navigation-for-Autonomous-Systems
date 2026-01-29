#!/usr/bin/env python3
"""
MPC Obstacle Avoidance Launch File
==================================

Launch file for Phase 2: MPC with obstacle avoidance demonstration.

This launches:
- Trajectory publisher node (Figure-8 reference)
- State estimator node (odometry processing)  
- MPC controller node (obstacle avoidance)
- Obstacle publisher node (publishes static obstacles)

Usage:
    ros2 launch hybrid_controller mpc_obstacle.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('hybrid_controller')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Trajectory publisher node
    trajectory_node = Node(
        package='hybrid_controller',
        executable='trajectory_node',
        name='trajectory_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'amplitude': 2.0,
            'frequency': 0.5,
            'duration': 30.0,
            'dt': 0.02,
        }]
    )
    
    # State estimator node
    state_estimator_node = Node(
        package='hybrid_controller',
        executable='state_estimator_node',
        name='state_estimator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    # MPC controller node
    mpc_node = Node(
        package='hybrid_controller',
        executable='mpc_node',
        name='mpc_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'horizon': 10,
            'Q_diag': [10.0, 10.0, 1.0],
            'R_diag': [0.1, 0.1],
            'P_diag': [20.0, 20.0, 2.0],
            'd_safe': 0.3,
            'slack_penalty': 1000.0,
            'v_max': 1.0,
            'omega_max': 1.5,
            'dt': 0.02,
            'control_rate': 20.0,
            'solver': 'ECOS',
            'log_level': 'INFO',
        }]
    )
    
    # Static obstacle publisher (publishes obstacle positions)
    # Format: [x1, y1, radius1, x2, y2, radius2, ...]
    obstacle_publisher = Node(
        package='ros2topic',
        executable='ros2topic',
        name='obstacle_publisher',
        output='screen',
        arguments=[
            'pub', '-r', '1.0',  # Publish at 1 Hz
            '/mpc_obstacles', 
            'std_msgs/msg/Float32MultiArray',
            '{data: [1.0, 0.5, 0.2, -0.5, -1.0, 0.25, 1.5, -0.3, 0.15]}'
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        trajectory_node,
        state_estimator_node,
        # Delay MPC start to allow trajectory to initialize
        TimerAction(
            period=1.0,
            actions=[mpc_node]
        ),
        obstacle_publisher,
    ])
