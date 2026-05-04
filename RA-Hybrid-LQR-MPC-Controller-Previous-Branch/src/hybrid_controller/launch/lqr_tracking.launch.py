#!/usr/bin/env python3
"""
LQR Tracking Launch File
========================

Launch file for Phase 1: LQR trajectory tracking demonstration.

This launches:
- Trajectory publisher node (Figure-8 reference)
- State estimator node (odometry processing)
- LQR controller node (trajectory tracking)

Usage:
    ros2 launch hybrid_controller lqr_tracking.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('hybrid_controller')
    
    # Paths
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')
    
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
            'duration': 20.0,
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
    
    # LQR controller node
    lqr_node = Node(
        package='hybrid_controller',
        executable='lqr_node',
        name='lqr_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'Q_diag': [10.0, 10.0, 1.0],
            'R_diag': [0.1, 0.1],
            'control_rate': 50.0,
            'v_max': 1.0,
            'omega_max': 1.5,
            'dt': 0.02,
            'log_level': 'INFO',
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        trajectory_node,
        state_estimator_node,
        lqr_node,
    ])
