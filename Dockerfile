# =============================================================================
# Risk-Aware Hybrid LQR-MPC Navigation — Dockerfile
# =============================================================================
# Build for Jazzy (default) or Humble:
#   docker build --build-arg ROS_DISTRO=jazzy -t hybrid_nav:jazzy .
#   docker build --build-arg ROS_DISTRO=humble -t hybrid_nav:humble .
# =============================================================================

ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full

# Re-declare after FROM so it's available in RUN
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ENV TURTLEBOT3_MODEL=burger
SHELL ["/bin/bash", "-c"]

# ── System dependencies ─────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-${ROS_DISTRO}-turtlebot3-gazebo \
    ros-${ROS_DISTRO}-turtlebot3-description \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    && rm -rf /var/lib/apt/lists/*

# ── Python dependencies ─────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y python3-venv \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt

# ── Copy workspace ──────────────────────────────────────────────────────────
COPY ros2_ws /ros2_ws
COPY src /ros2_ws/src
WORKDIR /ros2_ws

# ── Install hybrid_controller as editable package ───────────────────────────
RUN pip install -e src/hybrid_controller

# ── Build ROS2 workspace ────────────────────────────────────────────────────
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    python3 /usr/bin/colcon build --symlink-install

# ── Entrypoint ──────────────────────────────────────────────────────────────
# CRLF from Windows checkouts breaks kernel shebang parsing ("no such file or directory").
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN sed -i 's/\r$//' /docker-entrypoint.sh && chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/bin/bash", "/docker-entrypoint.sh"]

# Default: launch hybrid controller
CMD ["ros2", "launch", "hybrid_nav", "turtlebot3_hybrid.launch.py"]
