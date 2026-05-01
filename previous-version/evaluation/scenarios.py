
import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
from hybrid_controller.controllers.mpc_controller import Obstacle

@dataclass
class ObstacleConfig:
    """Obstacle layout configuration."""
    obstacles: List[Dict[str, float]]
    seed: int
    name: str = "random"
    
    def to_obstacle_list(self) -> List[Obstacle]:
        return [Obstacle(x=o['x'], y=o['y'], radius=o['radius']) for o in self.obstacles]

class ScenarioGenerator:
    """Base class for scenario generation."""
    def generate(self, n_configs: int, base_seed: int) -> List[ObstacleConfig]:
        raise NotImplementedError

class RandomScenario(ScenarioGenerator):
    """Original randomized scatter."""
    def generate(self, n_configs: int, base_seed: int) -> List[ObstacleConfig]:
        configs = []
        for i in range(n_configs):
            rng = np.random.RandomState(base_seed + i)
            n_obs = rng.randint(2, 6)
            obstacles = []
            attempts = 0
            while len(obstacles) < n_obs and attempts < 100:
                x = rng.uniform(-2.0, 2.0)
                y = rng.uniform(-1.5, 1.5)
                r = rng.uniform(0.1, 0.3)
                
                # Simple check against origin
                if x**2 + y**2 > 0.6**2: 
                     obstacles.append({'x': float(x), 'y': float(y), 'radius': float(r)})
                attempts += 1
            configs.append(ObstacleConfig(obstacles=obstacles, seed=base_seed+i, name=f"random_{i}"))
        return configs

class CorridorScenario(ScenarioGenerator):
    """
    Narrow corridor formed by two walls of obstacles.
    Tests simultaneous constraint handling from both sides.
    """
    def generate(self, n_configs: int, base_seed: int) -> List[ObstacleConfig]:
        configs = []
        for i in range(n_configs):
            rng = np.random.RandomState(base_seed + i)
            obstacles = []
            
            # Corridor width varies slightly
            width = rng.uniform(0.6, 0.9)  
            length = 3.0
            
            # Left wall
            for x in np.linspace(-1.0, 1.0, 5):
                y = width/2 + rng.uniform(-0.05, 0.05)
                obstacles.append({'x': float(x), 'y': float(y), 'radius': 0.15})
                
            # Right wall
            for x in np.linspace(-1.0, 1.0, 5):
                y = -width/2 + rng.uniform(-0.05, 0.05)
                obstacles.append({'x': float(x), 'y': float(y), 'radius': 0.15})
                
            configs.append(ObstacleConfig(obstacles=obstacles, seed=base_seed+i, name=f"corridor_{i}"))
        return configs

class BugTrapScenario(ScenarioGenerator):
    """
    U-shaped obstacle configuration.
    Tests local minima and ability to navigate around complex geometry.
    Note: Standard MPC often fails this without a global planner or very long horizon.
    """
    def generate(self, n_configs: int, base_seed: int) -> List[ObstacleConfig]:
        configs = []
        for i in range(n_configs):
            rng = np.random.RandomState(base_seed + i)
            obstacles = []
            
            # U-shape parameters
            center_x = rng.uniform(0.5, 1.5)
            center_y = rng.uniform(-0.5, 0.5)
            r = 0.15
            
            # Back wall
            obstacles.append({'x': center_x, 'y': center_y, 'radius': r})
            obstacles.append({'x': center_x, 'y': center_y + 0.25, 'radius': r})
            obstacles.append({'x': center_x, 'y': center_y - 0.25, 'radius': r})
            
            # Side walls (forming the 'hook')
            obstacles.append({'x': center_x - 0.25, 'y': center_y + 0.25, 'radius': r})
            obstacles.append({'x': center_x - 0.25, 'y': center_y - 0.25, 'radius': r})
            
            configs.append(ObstacleConfig(obstacles=obstacles, seed=base_seed+i, name=f"bugtrap_{i}"))
        return configs

class DenseClutterScenario(ScenarioGenerator):
    """
    High-density field forcing almost continuous avoidance.
    Stress tests solver speed and feasibility logic.
    """
    def generate(self, n_configs: int, base_seed: int) -> List[ObstacleConfig]:
        configs = []
        for i in range(n_configs):
            rng = np.random.RandomState(base_seed + i)
            obstacles = []
            
            # Grid-like high density
            n_obs = rng.randint(8, 15)
            for _ in range(n_obs):
                x = rng.uniform(-1.5, 1.5)
                y = rng.uniform(-1.0, 1.0)
                
                # Clear start/goal
                if x**2 + y**2 > 0.8**2:
                    obstacles.append({'x': float(x), 'y': float(y), 'radius': rng.uniform(0.1, 0.2)})
            
            configs.append(ObstacleConfig(obstacles=obstacles, seed=base_seed+i, name=f"dense_{i}"))
        return configs

def get_generator(name: str) -> ScenarioGenerator:
    if name == 'corridor': return CorridorScenario()
    if name == 'bugtrap': return BugTrapScenario()
    if name == 'dense': return DenseClutterScenario()
    return RandomScenario()
