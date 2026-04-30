#!/usr/bin/env python3
"""
Launch: TurtleBot3 + Pure LQR Controller + RViz
================================================

Launches the pure LQR controller (no MPC, no hybrid logic) for baseline
trajectory tracking with TurtleBot3 in GZ Sim.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hybrid_nav_dir = get_package_share_directory('hybrid_nav')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    tb3_launch_dir = os.path.join(tb3_gazebo_dir, 'launch')

    params_yaml = os.path.join(hybrid_nav_dir, 'config', 'params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # LQR doesn't need obstacle world, but use it for consistency
    custom_world = os.path.join(hybrid_nav_dir, 'worlds', 'hybrid_obstacle_world.sdf')
    fallback_world = os.path.join(tb3_gazebo_dir, 'worlds', 'empty_world.world')
    world = custom_world if os.path.exists(custom_world) else fallback_world

    set_gz_model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_gazebo_dir, 'models'),
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v1 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v1 '}.items(),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items(),
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/cmd_vel_bridge@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[('/cmd_vel_bridge', '/cmd_vel')],
    )

    twist_relay = Node(
        package='hybrid_nav',
        executable='twist_relay_node',
        name='twist_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Pure LQR Controller Node
    lqr_controller = Node(
        package='hybrid_nav',
        executable='lqr_controller_node',
        name='lqr_controller_node',
        output='screen',
        parameters=[params_yaml],
    )

    rviz_config = os.path.join(hybrid_nav_dir, 'rviz', 'hybrid_nav.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'))
    ld.add_action(DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 visualization'))
    ld.add_action(DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Robot initial X position'))
    ld.add_action(DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Robot initial Y position'))

    ld.add_action(set_gz_model_path)
    ld.add_action(gz_server)
    ld.add_action(gz_client)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_turtlebot)
    ld.add_action(ros_gz_bridge)
    ld.add_action(twist_relay)
    ld.add_action(lqr_controller)
    ld.add_action(rviz)

    return ld
