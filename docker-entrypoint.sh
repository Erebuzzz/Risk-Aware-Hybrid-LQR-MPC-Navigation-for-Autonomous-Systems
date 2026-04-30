#!/bin/bash
# =============================================================================
# Docker entrypoint: source ROS2 + workspace, then exec command
# =============================================================================
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}

exec "$@"
