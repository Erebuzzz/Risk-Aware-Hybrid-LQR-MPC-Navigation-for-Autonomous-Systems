
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import Tuple, Deque

@dataclass
class ActuatorParams:
    """Configuration for actuator dynamics and sensor noise."""
    tau_v: float = 0.1          # Velocity time constant (sec)
    tau_omega: float = 0.1      # Angular velocity time constant (sec)
    noise_v_std: float = 0.0    # Velocity command noise std
    noise_omega_std: float = 0.0 # Angular velocity command noise std
    delay_steps: int = 0        # Control pipeline delay (steps)

class ActuatorDynamics:
    """
    Simulates low-level actuator dynamics, delays, and noise.
    
    Model:
        tau * du/dt + u = u_cmd
        u_actual = u + noise
        
    Also handles discrete pipeline delay buffer.
    """
    def __init__(self, params: ActuatorParams, dt: float):
        self.params = params
        self.dt = dt
        
        # Internal state (actual output)
        self.v_actual = 0.0
        self.omega_actual = 0.0
        
        # Delay buffer
        self._buffer: Deque[Tuple[float, float]] = deque(maxlen=max(1, params.delay_steps + 1))
        # Fill buffer with zeros initially
        for _ in range(max(1, params.delay_steps + 1)):
            self._buffer.append((0.0, 0.0))
            
    def reset(self):
        """Reset internal state."""
        self.v_actual = 0.0
        self.omega_actual = 0.0
        self._buffer.clear()
        for _ in range(max(1, self.params.delay_steps + 1)):
            self._buffer.append((0.0, 0.0))
            
    def update(self, v_cmd: float, omega_cmd: float) -> Tuple[float, float]:
        """
        Update actuator state and return realized control inputs.
        
        Args:
            v_cmd: Commanded linear velocity
            omega_cmd: Commanded angular velocity
            
        Returns:
            Tuple (v_realized, omega_realized) applied to the robot
        """
        # 1. Apply pipeline delay
        self._buffer.append((v_cmd, omega_cmd))
        
        # With delay=d, we pull the command from d steps ago
        # If len is d+1, buffer[0] is the delayed command
        if self.params.delay_steps > 0:
            # If buffer full, pop delayed command. The newly appended one is at the end.
            # actually deque automatically pops if maxlen is reached, wait.
            # No, deque auto-discards from left if we append to right and it's full.
            # So buffer[-1] is newest, buffer[0] is oldest.
            # If maxlen = delay+1, then buffer[0] is exactly (delay) steps old.
            v_delayed, omega_delayed = self._buffer[0]
        else:
            v_delayed, omega_delayed = v_cmd, omega_cmd
            
        # 2. First-order lag dynamics
        # v[k+1] = v[k] + dt/tau * (v_cmd - v[k])
        # Safe-guard against tau=0
        if self.params.tau_v > 1e-4:
            alpha_v = self.dt / self.params.tau_v
            self.v_actual += alpha_v * (v_delayed - self.v_actual)
        else:
            self.v_actual = v_delayed
            
        if self.params.tau_omega > 1e-4:
            alpha_w = self.dt / self.params.tau_omega
            self.omega_actual += alpha_w * (omega_delayed - self.omega_actual)
        else:
            self.omega_actual = omega_delayed
            
        # 3. Add actuator noise (optional)
        v_out = self.v_actual
        omega_out = self.omega_actual
        
        if self.params.noise_v_std > 0:
            v_out += np.random.normal(0, self.params.noise_v_std)
            
        if self.params.noise_omega_std > 0:
            omega_out += np.random.normal(0, self.params.noise_omega_std)
            
        return v_out, omega_out
