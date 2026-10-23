# syntax=docker/dockerfile:1
# Risk-aware hybrid LQR-MPC navigation image.
#
# Build:
#   make docker-build

ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full

# Re-declare after FROM so it's available in RUN
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ENV TURTLEBOT3_MODEL=burger
SHELL ["/bin/bash", "-c"]

# System dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-pip \
    python3-venv \
    ros-${ROS_DISTRO}-turtlebot3-gazebo \
    ros-${ROS_DISTRO}-turtlebot3-description \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies
RUN python3 -m venv --system-site-packages /opt/venv
ENV PATH="/opt/venv/bin:$PATH"
ENV PYTHONUNBUFFERED=1

COPY requirements.txt /tmp/requirements.txt
RUN python -m pip install --no-cache-dir --upgrade pip \
    && python -m pip install --no-cache-dir -r /tmp/requirements.txt \
    && python -m pip install --no-cache-dir --ignore-installed colcon-common-extensions \
    && python -c "import cvxpy; print('cvxpy', cvxpy.__version__)"

# Copy the two active packages into one colcon workspace.
COPY ros2_ws /ros2_ws
COPY src /ros2_ws/src
WORKDIR /ros2_ws

# Install the controller library before building the ROS package.
RUN python -m pip install --no-cache-dir -e src/hybrid_controller

# Build the ROS 2 workspace.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    /opt/venv/bin/colcon build --symlink-install --event-handlers console_direct+

# Entrypoint and the bounded, headless experiment command used by Compose/CI.
COPY docker-entrypoint.sh /docker-entrypoint.sh
COPY scripts/docker_ros2_checks.sh /usr/local/bin/docker-ros2-checks
COPY scripts/docker_headless_experiment.sh /usr/local/bin/docker-headless-experiment
COPY benchmarks/generate_ros_validation_report.py /opt/gate1/generate_ros_validation_report.py
RUN sed -i 's/\r$//' /docker-entrypoint.sh /usr/local/bin/docker-ros2-checks \
        /usr/local/bin/docker-headless-experiment && \
    chmod +x /docker-entrypoint.sh /usr/local/bin/docker-ros2-checks \
        /usr/local/bin/docker-headless-experiment
ENTRYPOINT ["/bin/bash", "/docker-entrypoint.sh"]

# Default to an interactive shell. Compose and CI provide explicit commands.
CMD ["bash"]
