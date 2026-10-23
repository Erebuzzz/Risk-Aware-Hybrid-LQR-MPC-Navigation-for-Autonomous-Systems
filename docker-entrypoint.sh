#!/bin/bash
# =============================================================================
# Docker entrypoint: source ROS2 + workspace, then exec command
# =============================================================================
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

export VIRTUAL_ENV=/opt/venv
export PATH="/opt/venv/bin:${PATH}"
VENV_SITE_PACKAGES=$(/opt/venv/bin/python -c 'import site; print(site.getsitepackages()[0])')
export PYTHONPATH="${VENV_SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}

exec "$@"
