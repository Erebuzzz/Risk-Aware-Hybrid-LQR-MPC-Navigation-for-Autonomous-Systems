# Controller package

The active Python package is `src/hybrid_controller`. It is installed as an
ament Python package for ROS 2 and can also be imported directly from the
source tree by the local test and benchmark commands.

```text
src/hybrid_controller/
├── hybrid_controller/controllers/   LQR, MPC, risk, backup, and ESO logic
├── hybrid_controller/models/        Kinematics and actuator-aware dynamics
├── hybrid_controller/trajectory/    References and progress utilities
├── hybrid_controller/logging/       Simulation telemetry
├── package.xml                      ROS 2 package metadata
└── setup.py                         Python package metadata
```

The ROS 2 nodes live separately in `ros2_ws/src/hybrid_nav`. Docker copies both
packages into one colcon workspace, so local development does not require
symlinking controller modules into the ROS 2 package.
