import sys
import os
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'ros2_ws', 'src', 'hybrid_controller')))

from hybrid_controller.trajectory.trajectory_factory import TrajectoryFactory
from hybrid_controller.utils.visualization import Visualizer

def generate_trajectory_plots():
    os.makedirs('Output/Plots/Trajectories', exist_ok=True)
    factory = TrajectoryFactory()
    viz = Visualizer(output_dir='Output/Plots/Trajectories')
    
    trajectory_types = ['figure8', 'clover3', 'rose4', 'spiral', 'random_wp']
    
    print("Generating plotting artifacts for trajectory geometries...")
    
    for traj_type in trajectory_types:
        try:
            # Generate sample paths of 20 seconds
            traj = factory.generate(traj_type, duration=20.0, dt=0.02, A=2.0)
            
            # Traj array contains: [t, px, py, theta, v, omega]
            states = traj[:, 1:3] # Using generated geometric paths as perfectly tracked
            reference = traj[:, 1:4] 
            
            # Standard plotter expects (T, 3) arrays, so pad states with dummy angles if required by plotter
            import numpy as np
            padded_states = np.zeros((traj.shape[0], 3))
            padded_states[:, :2] = states
            padded_states[:, 2] = reference[:, 2]
            
            viz.plot_trajectory(
                states=padded_states, 
                reference=reference, 
                title=f"{traj_type.capitalize()} Trajectory Profile", 
                save_path=f"Output/Plots/Trajectories/{traj_type}_trajectory.png"
            )
            print(f" -> Exported Output/Plots/Trajectories/{traj_type}_trajectory.png")
            
        except Exception as e:
            print(f"Failed to generate {traj_type}: {e}")

if __name__ == '__main__':
    generate_trajectory_plots()
