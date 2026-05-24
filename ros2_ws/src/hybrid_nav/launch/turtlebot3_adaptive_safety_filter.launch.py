#!/usr/bin/env python3
"""
Launch: TurtleBot3 + Adaptive Safety Filter + Option B Experiments
==================================================================

Runs the research-paper controller with actuation mismatch and moving obstacles.
The controller publishes /cmd_vel_raw, the distortion node publishes /cmd_vel,
and the relay converts TwistStamped to Twist for Gazebo.
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
    hybrid_nav_dir = get_package_share_directory("hybrid_nav")
    tb3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    tb3_launch_dir = os.path.join(tb3_gazebo_dir, "launch")

    params_yaml = os.path.join(hybrid_nav_dir, "config", "params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")
    use_actuation_mismatch = LaunchConfiguration(
        "use_actuation_mismatch", default="true"
    )
    obstacle_scenario = LaunchConfiguration("obstacle_scenario", default="crossing")
    obstacle_format = LaunchConfiguration("obstacle_format", default="quintuple")

    world = os.path.join(hybrid_nav_dir, "worlds", "hybrid_obstacle_world.sdf")

    set_gz_model_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(tb3_gazebo_dir, "models"),
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -s -v1 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-g -v1 "}.items(),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        remappings=[("/cmd_vel", "/cmd_vel_bridge")],
    )

    twist_relay = Node(
        package="hybrid_nav",
        executable="twist_relay_node",
        name="twist_relay",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    adaptive_safety_filter = Node(
        package="hybrid_nav",
        executable="adaptive_safety_filter_node",
        name="adaptive_safety_filter_node",
        output="screen",
        parameters=[params_yaml],
        remappings=[("/cmd_vel", "/cmd_vel_raw")],
    )

    command_distortion = Node(
        package="hybrid_nav",
        executable="command_distortion_node",
        name="command_distortion_node",
        output="screen",
        parameters=[params_yaml, {"enabled": use_actuation_mismatch}],
    )

    dynamic_obstacles = Node(
        package="hybrid_nav",
        executable="dynamic_obstacle_publisher_node",
        name="dynamic_obstacle_publisher_node",
        output="screen",
        parameters=[
            params_yaml,
            {
                "scenario": obstacle_scenario,
                "message_format": obstacle_format,
            },
        ],
    )

    rviz_config = os.path.join(hybrid_nav_dir, "rviz", "hybrid_nav.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="true"))
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))
    ld.add_action(DeclareLaunchArgument("x_pose", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("y_pose", default_value="0.0"))
    ld.add_action(
        DeclareLaunchArgument("use_actuation_mismatch", default_value="true")
    )
    ld.add_action(DeclareLaunchArgument("obstacle_scenario", default_value="crossing"))
    ld.add_action(DeclareLaunchArgument("obstacle_format", default_value="quintuple"))

    ld.add_action(set_gz_model_path)
    ld.add_action(gz_server)
    ld.add_action(gz_client)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_turtlebot)
    ld.add_action(ros_gz_bridge)
    ld.add_action(twist_relay)
    ld.add_action(adaptive_safety_filter)
    ld.add_action(command_distortion)
    ld.add_action(dynamic_obstacles)
    ld.add_action(rviz)

    return ld
