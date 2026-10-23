#!/usr/bin/env bash
# Run from /ros2_ws after the image entrypoint sources ROS and the overlay.
set -Eeuo pipefail

python -c 'import cvxpy; print("cvxpy", cvxpy.__version__)'
colcon test --event-handlers console_direct+
colcon test-result --verbose
ros2 pkg prefix hybrid_controller
ros2 pkg prefix hybrid_nav
