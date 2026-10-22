.PHONY: help setup test compile recoverability docker-build docker-test ros2-build ros2-test clean

SHELL := /bin/bash
PYTHON ?= python
ROS_DISTRO ?= jazzy

help:
	@echo "Available targets:"
	@echo "  make setup             Install local Python dependencies"
	@echo "  make test              Run the Python test suite"
	@echo "  make compile           Compile Python sources"
	@echo "  make recoverability    Run the deterministic Stage A grid"
	@echo "  make docker-build      Build the ROS 2 Jazzy image"
	@echo "  make docker-test       Run the containerized ROS 2 checks"
	@echo "  make ros2-build        Build the ROS 2 workspace on the host"
	@echo "  make ros2-test         Test the ROS 2 workspace on the host"
	@echo "  make clean             Remove ROS 2 build artifacts"

setup:
	$(PYTHON) -m pip install -r requirements.txt
	$(PYTHON) -m pip install -e src/hybrid_controller

test:
	$(PYTHON) -m pytest -q

compile:
	$(PYTHON) -m compileall -q benchmarks src tests ros2_ws/src/hybrid_nav

recoverability:
	$(PYTHON) -m benchmarks.validate_recoverability_gap \
		--output Output/Plots/Paper/recoverability_gap_validation

docker-build:
	docker compose build jazzy

docker-test:
	docker compose --profile ci run --rm ci

ros2-build:
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
		colcon build --symlink-install \
			--base-paths ros2_ws/src src \
			--build-base ros2_ws/build \
			--install-base ros2_ws/install \
			--log-base ros2_ws/log

ros2-test:
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
		source ros2_ws/install/setup.bash && \
		colcon test \
			--base-paths ros2_ws/src src \
			--build-base ros2_ws/build \
			--install-base ros2_ws/install \
			--log-base ros2_ws/log \
			--event-handlers console_direct+ && \
		colcon test-result --verbose

clean:
	rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
