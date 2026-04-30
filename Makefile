.PHONY: setup build run run-mpc run-adaptive run-lqr sim test clean help

# ==============================================================================
# Risk-Aware Hybrid LQR-MPC Navigation — Build System
# ==============================================================================
# Targets:
#   make setup     — Install system + Python dependencies
#   make build     — colcon build (symlink-install)
#   make run       — Launch Hybrid controller in Gazebo
#   make run-mpc   — Launch pure MPC controller
#   make run-adaptive — Launch adaptive MPC controller
#   make run-lqr   — Launch pure LQR controller
#   make sim       — Standalone simulation (no ROS2/Gazebo)
#   make test      — Run unit tests
#   make clean     — Remove build artifacts
# ==============================================================================

SHELL := /bin/bash
ROS2_WS := ros2_ws

help:
	@echo "Available targets:"
	@echo "  make setup       — Install dependencies"
	@echo "  make build       — colcon build"
	@echo "  make run         — Launch Hybrid (Gazebo + RViz)"
	@echo "  make run-mpc     — Launch pure MPC"
	@echo "  make run-adaptive — Launch adaptive MPC"
	@echo "  make run-lqr     — Launch pure LQR"
	@echo "  make sim         — Standalone simulation (no ROS2)"
	@echo "  make test        — Run unit tests"
	@echo "  make clean       — Remove build artifacts"

# ── Dependencies ─────────────────────────────────────────────────────────────
setup:
	rosdep install --from-paths $(ROS2_WS)/src --ignore-src -r -y
	pip install -r requirements.txt
	pip install -e $(ROS2_WS)/src/hybrid_controller

# ── Build ────────────────────────────────────────────────────────────────────
build:
	cd $(ROS2_WS) && colcon build --symlink-install

# ── Launch (require source install/setup.bash first) ─────────────────────────
run:
	@echo "Launching Hybrid LQR-MPC Controller..."
	source $(ROS2_WS)/install/setup.bash && \
		ros2 launch hybrid_nav turtlebot3_hybrid.launch.py

run-mpc:
	@echo "Launching Pure MPC Controller..."
	source $(ROS2_WS)/install/setup.bash && \
		ros2 launch hybrid_nav turtlebot3_mpc.launch.py

run-adaptive:
	@echo "Launching Adaptive MPC Controller..."
	source $(ROS2_WS)/install/setup.bash && \
		ros2 launch hybrid_nav turtlebot3_adaptive_mpc.launch.py

run-lqr:
	@echo "Launching Pure LQR Controller..."
	source $(ROS2_WS)/install/setup.bash && \
		ros2 launch hybrid_nav turtlebot3_lqr.launch.py

# ── Standalone simulation (no ROS2/Gazebo needed) ────────────────────────────
sim:
	python run_simulation.py --mode hybrid

sim-compare:
	python run_simulation.py --mode compare

# ── Tests ────────────────────────────────────────────────────────────────────
test:
	python -m pytest tests/ -v
	@echo "--- Import smoke test ---"
	python -c "from hybrid_controller.controllers.mpc_controller import MPCController, MPCSolution; print('✅ MPC import OK')"
	python -c "from hybrid_controller.controllers.adaptive_mpc_controller import AdaptiveMPCController; print('✅ Adaptive MPC import OK')"
	python -c "from hybrid_controller.controllers.lqr_controller import LQRController; print('✅ LQR import OK')"

# ── Clean ────────────────────────────────────────────────────────────────────
clean:
	rm -rf $(ROS2_WS)/build $(ROS2_WS)/install $(ROS2_WS)/log
