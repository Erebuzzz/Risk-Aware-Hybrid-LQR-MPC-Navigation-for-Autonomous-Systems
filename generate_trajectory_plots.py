import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'ros2_ws', 'src', 'hybrid_controller')))

from hybrid_controller.trajectory.trajectory_factory import TrajectoryFactory
from hybrid_controller.utils.visualization import Visualizer

PLOTS_TRAJECTORIES = os.path.join("Output", "Plots", "Trajectories")
trajectory_types = ["figure8", "clover3", "rose4", "spiral", "random_wp"]


def generate_trajectory_plots():
    factory = TrajectoryFactory()

    print("Generating plotting artifacts for trajectory geometries...")

    for traj_type in trajectory_types:
        try:
            subdir = os.path.join(PLOTS_TRAJECTORIES, traj_type)
            os.makedirs(subdir, exist_ok=True)
            viz = Visualizer(output_dir=subdir)

            # Generate sample paths of 20 seconds
            traj = factory.generate(
                traj_type,
                duration=20.0,
                dt=0.02,
                A=2.0,
                seed=42 if traj_type == "random_wp" else None,
            )

            # Traj array contains: [t, px, py, theta, v, omega]
            states = traj[:, 1:3]  # Using generated geometric paths as perfectly tracked
            reference = traj[:, 1:4]

            # Standard plotter expects (T, 3) arrays, so pad states with dummy angles if required by plotter
            import numpy as np

            padded_states = np.zeros((traj.shape[0], 3))
            padded_states[:, :2] = states
            padded_states[:, 2] = reference[:, 2]

            out_path = os.path.join(subdir, f"{traj_type}_trajectory.png")
            viz.plot_trajectory(
                states=padded_states,
                reference=reference,
                title=f"{traj_type.capitalize()} Trajectory Profile",
                save_path=out_path,
            )
            print(f" -> Exported {out_path}")

        except Exception as e:
            print(f"Failed to generate {traj_type}: {e}")

if __name__ == '__main__':
    generate_trajectory_plots()
