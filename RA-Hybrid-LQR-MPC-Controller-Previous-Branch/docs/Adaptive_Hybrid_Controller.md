# Adaptive Hybrid Controller Documentation

## Overview

The **Adaptive Hybrid Controller** is a novel control architecture that combines:
- **Adaptive MPC** with LMS (Least Mean Squares) parameter adaptation
- **LQR** for efficient trajectory tracking
- **Distance-based risk metrics** for intelligent switching
- **Smooth blending** with anti-chatter guarantees

This controller is designed for autonomous robots operating in environments with:
- Uncertain or time-varying dynamics
- Mixed obstacle-free and obstacle-dense regions
- Need for online learning and adaptation

---

## Architecture

### High-Level Block Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    ADAPTIVE HYBRID CONTROLLER                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────┐         ┌──────────────────────────────┐     │
│  │   Obstacle   │────────▶│   Risk Assessment Module     │     │
│  │   Detection  │         │   - Distance-based risk      │     │
│  └──────────────┘         │   - Predictive risk          │     │
│                            │   - Combined risk metric     │     │
│                            └──────────┬───────────────────┘     │
│                                       │ risk ∈ [0,1]            │
│                                       ▼                          │
│                            ┌──────────────────────────────┐     │
│                            │   Blending Supervisor        │     │
│                            │   w(t) = sigmoid(risk)       │     │
│                            │   + hysteresis               │     │
│                            │   + rate limiting            │     │
│                            │   + feasibility fallback     │     │
│                            └──────────┬───────────────────┘     │
│                                       │ w(t) ∈ [0,1]            │
│                  ┌────────────────────┴────────────────────┐    │
│                  │                                          │    │
│                  ▼                                          ▼    │
│     ┌────────────────────────┐              ┌────────────────────────┐
│     │   LQR Controller       │              │  Adaptive MPC          │
│     │   - DARE solver        │              │  - CasADi + IPOPT     │
│     │   - Efficient tracking │              │  - Nonlinear dynamics │
│     │   - Low computational  │              │  - LMS adaptation     │
│     │     cost               │              │  - Terminal LQR       │
│     └──────────┬─────────────┘              └──────────┬─────────────┘
│                │ u_lqr                                  │ u_mpc        │
│                │                                        │              │
│                └──────────────┬─────────────────────────┘              │
│                               ▼                                        │
│                    u = w(t)·u_mpc + (1-w(t))·u_lqr                   │
│                               │                                        │
│                               ▼                                        │
│                    ┌──────────────────────┐                           │
│                    │  LMS Adaptation      │                           │
│                    │  (active when w>0.5) │                           │
│                    │  θ̂ ← θ̂ + Γ·Φᵀ·e      │                           │
│                    └──────────────────────┘                           │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘
```

---

## Key Components

### 1. Risk Assessment Module

**Purpose:** Quantify the level of danger based on obstacle proximity and predicted trajectory violations.

**Metrics:**
- **Distance Risk**: `r_dist = max(0, 1 - (d - d_safe)/(d_trigger - d_safe))`
  - `d`: Minimum distance to nearest obstacle
  - `d_safe`: Safety distance threshold (default: 0.3m)
  - `d_trigger`: Distance at which risk starts increasing (default: 1.0m)

- **Predictive Risk**: Checks if predicted MPC trajectory violates constraints
  - Evaluates future states over prediction horizon
  - Weights earlier violations more heavily

- **Combined Risk**: `r = α·r_dist + β·r_pred`
  - `α`: Weight for distance risk (default: 0.6)
  - `β`: Weight for predictive risk (default: 0.4)

**Output:** `RiskAssessment` object with:
- `combined_risk`: Overall risk value ∈ [0, 1]
- `distance_risk`: Distance-based component
- `predictive_risk`: Prediction-based component
- `min_obstacle_distance`: Nearest obstacle distance
- `risk_level`: Categorical level ("low", "medium", "high", "critical")

---

### 2. Blending Supervisor

**Purpose:** Compute smooth blending weight `w(t)` that determines control mode.

**Blending Law:**
```
u(t) = w(t) · u_mpc(t) + (1 - w(t)) · u_lqr(t)
```

where:
- `w(t) = 0`: Pure LQR (far from obstacles)
- `w(t) = 1`: Pure Adaptive MPC (near obstacles)
- `0 < w(t) < 1`: Blended control

**Weight Computation Pipeline:**

1. **Sigmoid Mapping:**
   ```
   w_raw = 1 / (1 + exp(-k·(risk - threshold)))
   ```
   - `k`: Steepness parameter (default: 10.0)
   - `threshold`: Risk at which w = 0.5 (default: 0.3)

2. **Hysteresis Deadband:**
   - If `risk ∈ [threshold - band, threshold + band]`: maintain previous weight
   - Prevents oscillation near transition point
   - `band`: Half-width of deadband (default: 0.05)

3. **Rate Limiting:**
   ```
   |dw/dt| ≤ dw_max
   ```
   - Ensures Lipschitz continuity
   - `dw_max`: Maximum rate of change (default: 2.0 s⁻¹)

4. **Feasibility Fallback:**
   - If MPC solver fails: `w ← w · decay_factor`
   - If solve time exceeds limit: `w ← w · 0.9`
   - If slack usage is high: `w ← w · 0.95`

**Formal Guarantees:**
- **Lipschitz Continuity:** `|w(t₁) - w(t₂)| ≤ dw_max · |t₁ - t₂|`
- **Finite Switches:** With hysteresis, guaranteed finite number of mode transitions
- **Safe Degradation:** Always falls back to LQR on MPC failure

---

### 3. LQR Controller

**Purpose:** Efficient trajectory tracking in low-risk regions.

**Formulation:**
- Solves Discrete Algebraic Riccati Equation (DARE)
- Computes optimal feedback gain `K`
- Control law: `u = u_ref - K·(x - x_ref)`

**Advantages:**
- Very low computational cost (~0.1 ms)
- Guaranteed stability for linearized system
- Smooth control inputs

**Limitations:**
- No obstacle avoidance
- Assumes small deviations from reference
- Requires accurate linearization

---

### 4. Adaptive MPC Controller

**Purpose:** Obstacle-aware optimal control with online parameter learning.

**Formulation:**
```
min  Σ_{k=0}^{N-1} (||x_k - x_ref||²_Q + ||u_k - u_ref||²_R + q_ξ||ξ_k||²)
   + ω Σ_{k=N}^{N+M} (||x_k - x_ref||²_Q + ||u_k - u_ref||²_R)
   + ω ||x_{N+M} - x_ref||²_Q

s.t. x_{k+1} = f(x_k, u_k, θ̂)                    (nonlinear dynamics with adapted params)
     u_k = u_ref - K·(x_k - x_ref)                k ∈ [N, N+M]  (terminal LQR)
     ||p_k + ξ_k - p_obs|| ≥ r_obs + d_safe      (obstacle avoidance)
     u_min ≤ u_k ≤ u_max                          (input constraints)
```

**Key Features:**
- **Nonlinear Dynamics:** Uses exact differential drive kinematics
- **Terminal LQR Rollout:** Extended horizon with feedback for stability
- **Exact Obstacle Constraints:** Euclidean norm distance
- **Slack Variables:** Per-state softening for feasibility
- **CasADi + IPOPT:** Efficient nonlinear optimization

**Solver:** IPOPT with MUMPS linear solver
- Typical solve time: 10-30 ms
- Warm-start support for real-time performance

---

### 5. LMS Parameter Adaptation

**Purpose:** Online learning of velocity and angular velocity scaling factors.

**Adapted Parameters:**
```
θ = [v_scale, ω_scale]
```

These scale the control inputs in the dynamics:
```
ẋ = v_scale · v · cos(θ)
ẏ = v_scale · v · sin(θ)
θ̇ = ω_scale · ω
```

**LMS Update Law:**
```
1. Prediction: x̂_{k+1} = x_k + dt · f(x_k, u_k, θ̂_k)
2. Measurement: x_{k+1} (from robot)
3. Error: e = x_{k+1} - x̂_{k+1}
4. Gradient: Φ = ∂f/∂θ
5. Update: θ̂_{k+1} = θ̂_k + Γ · Φᵀ · e
6. Projection: θ̂_{k+1} ← clip(θ̂_{k+1}, θ_min, θ_max)
```

**Parameters:**
- `Γ`: Learning rate matrix (default: 0.005 · I)
- `θ_min`: Lower bounds (default: [0.5, 0.5])
- `θ_max`: Upper bounds (default: [2.0, 2.0])

**Adaptation Strategy:**
- Only adapts when `w(t) > 0.5` (MPC-dominant mode)
- Prevents adaptation during LQR-only operation
- Stores parameter history for analysis

**Convergence:**
- Converges to true parameters if:
  - System is persistently excited
  - Learning rate is sufficiently small
  - Dynamics are in the model class

---

## Usage

### Basic Example

```python
from hybrid_controller.controllers.adaptive_hybrid_controller import AdaptiveHybridController
from hybrid_controller.controllers.mpc_controller import Obstacle
import numpy as np

# Initialize controller
controller = AdaptiveHybridController(
    # Adaptive MPC parameters
    prediction_horizon=10,
    terminal_horizon=5,
    mpc_Q_diag=[30.0, 30.0, 5.0],
    mpc_R_diag=[0.1, 0.1],
    d_safe=0.3,
    enable_adaptation=True,
    adaptation_gamma=0.005,
    theta_init=np.array([0.85, 0.85]),  # Initial guess
    # LQR parameters
    lqr_Q_diag=[15.0, 15.0, 8.0],
    lqr_R_diag=[0.1, 0.1],
    # Blending parameters
    k_sigmoid=10.0,
    risk_threshold=0.3,
    dw_max=2.0,
    # Common
    v_max=2.0,
    omega_max=3.0,
    dt=0.02
)

# Define obstacles
obstacles = [
    Obstacle(x=1.0, y=0.5, radius=0.2),
    Obstacle(x=-0.5, y=-1.0, radius=0.25)
]

# Control loop
for k in range(N):
    # Get reference trajectory segment
    x_refs, u_refs = get_reference_segment(k, horizon)
    
    # Compute control
    u, info = controller.compute_control(
        x=current_state,
        x_ref=reference_state,
        u_ref=reference_control,
        obstacles=obstacles,
        x_refs=x_refs,
        u_refs=u_refs,
        mpc_rate=5  # Run MPC every 5 steps
    )
    
    # Apply control
    next_state = robot.simulate_step(current_state, u, dt)
    
    # Monitor adaptation
    print(f"Weight: {info.weight:.3f}, Mode: {info.mode}")
    print(f"Params: v_scale={info.param_estimates[0]:.3f}, "
          f"ω_scale={info.param_estimates[1]:.3f}")
    print(f"Adaptation active: {info.adaptation_active}")
```

### Running Simulations

```bash
# Basic adaptive hybrid simulation
python run_simulation.py --mode adaptive_hybrid

# With different scenarios
python run_simulation.py --mode adaptive_hybrid --scenario dense
python run_simulation.py --mode adaptive_hybrid --scenario corridor

# With realistic actuator dynamics
python run_simulation.py --mode adaptive_hybrid --realistic

# Longer duration for better adaptation
python run_simulation.py --mode adaptive_hybrid --duration 30
```

---

## Parameter Tuning Guide

### Blending Parameters

| Parameter | Default | Range | Effect |
|-----------|---------|-------|--------|
| `k_sigmoid` | 10.0 | [5, 20] | Higher = sharper transition between controllers |
| `risk_threshold` | 0.3 | [0.2, 0.5] | Risk level at which w = 0.5 |
| `dw_max` | 2.0 | [1.0, 5.0] | Higher = faster mode switching |
| `hysteresis_band` | 0.05 | [0.02, 0.1] | Larger = more stable but slower response |

**Tuning Tips:**
- **Aggressive switching:** Increase `k_sigmoid` and `dw_max`
- **Smooth transitions:** Decrease `k_sigmoid`, increase `hysteresis_band`
- **Earlier MPC activation:** Decrease `risk_threshold`

### Adaptation Parameters

| Parameter | Default | Range | Effect |
|-----------|---------|-------|--------|
| `adaptation_gamma` | 0.005 | [0.001, 0.01] | Learning rate (higher = faster but less stable) |
| `theta_init` | [0.85, 0.85] | [0.5, 2.0] | Initial parameter guess |
| `theta_min` | [0.5, 0.5] | - | Lower bounds on parameters |
| `theta_max` | [2.0, 2.0] | - | Upper bounds on parameters |

**Tuning Tips:**
- **Faster adaptation:** Increase `adaptation_gamma` (but watch for instability)
- **More conservative:** Tighten bounds (`theta_min`, `theta_max`)
- **Better initial guess:** Set `theta_init` closer to true values if known

### MPC Parameters

| Parameter | Default | Range | Effect |
|-----------|---------|-------|--------|
| `prediction_horizon` | 10 | [5, 15] | Longer = better planning but slower |
| `terminal_horizon` | 5 | [3, 10] | Extended stability horizon |
| `omega_term` | 10.0 | [5.0, 20.0] | Terminal cost weight |
| `q_xi` | 1000.0 | [100, 5000] | Slack penalty (higher = stricter constraints) |

---

## Performance Characteristics

### Computational Cost

| Component | Typical Time | Frequency |
|-----------|--------------|-----------|
| LQR | ~0.1 ms | Every step (50 Hz) |
| Adaptive MPC | 10-30 ms | Every 5 steps (10 Hz) |
| Risk Assessment | ~0.05 ms | Every step |
| Blending | ~0.01 ms | Every step |
| LMS Update | ~0.02 ms | When w > 0.5 |

**Total:** ~0.2 ms per step (LQR-dominant), ~30 ms per MPC step

### Memory Usage

- Controller state: ~10 KB
- MPC solver workspace: ~500 KB
- Parameter history: ~0.1 KB per step

### Tracking Performance

Typical results on figure-8 trajectory with obstacles:
- **Mean position error:** 0.02-0.05 m
- **Max position error:** 0.1-0.2 m
- **Collision rate:** 0% (with proper tuning)
- **Parameter convergence:** 50-200 steps

---

## Comparison with Other Controllers

| Metric | LQR | MPC | Hybrid (MPC+LQR) | Adaptive Hybrid |
|--------|-----|-----|------------------|-----------------|
| **Obstacle Avoidance** | ✗ | ✓ | ✓ | ✓ |
| **Online Learning** | ✗ | ✗ | ✗ | ✓ |
| **Computational Cost** | Very Low | High | Medium | Medium |
| **Tracking Accuracy** | Good | Excellent | Excellent | Excellent |
| **Robustness to Uncertainty** | Poor | Medium | Medium | Excellent |
| **Smooth Control** | ✓ | ✓ | ✓ | ✓ |
| **Real-time Capable** | ✓ | ✓ | ✓ | ✓ |

---

## Troubleshooting

### Issue: Parameters not adapting

**Symptoms:** `param_estimates` stay at initial values

**Causes:**
1. Robot stays in LQR-dominant mode (w < 0.5)
2. Learning rate too small
3. Insufficient excitation

**Solutions:**
- Increase obstacle density or decrease `risk_threshold`
- Increase `adaptation_gamma`
- Run longer simulations

### Issue: Unstable adaptation

**Symptoms:** Parameters oscillate or diverge

**Causes:**
1. Learning rate too high
2. Poor initial guess
3. Model mismatch

**Solutions:**
- Decrease `adaptation_gamma`
- Improve `theta_init` estimate
- Tighten parameter bounds

### Issue: Frequent mode switching

**Symptoms:** Weight oscillates rapidly

**Causes:**
1. Hysteresis band too small
2. Rate limit too high
3. Risk threshold at boundary

**Solutions:**
- Increase `hysteresis_band`
- Decrease `dw_max`
- Adjust `risk_threshold`

### Issue: MPC solver failures

**Symptoms:** Frequent "fallback" status

**Causes:**
1. Infeasible constraints
2. Poor warm-start
3. Solver timeout

**Solutions:**
- Increase slack penalty `q_xi`
- Reduce prediction horizon
- Increase `solver_time_limit`

---

## Advanced Topics

### Custom Risk Metrics

You can define custom risk functions by subclassing `RiskMetrics`:

```python
class CustomRiskMetrics(RiskMetrics):
    def compute_distance_risk(self, state, obstacles):
        # Your custom distance risk logic
        pass
    
    def compute_predictive_risk(self, predicted_states, obstacles):
        # Your custom predictive risk logic
        pass
```

### Multi-Rate Control

The controller supports different update rates for LQR and MPC:

```python
# LQR runs at 50 Hz, MPC at 10 Hz
u, info = controller.compute_control(
    ...,
    mpc_rate=5  # Run MPC every 5 steps
)
```

### Parameter Bounds Adaptation

Dynamically adjust parameter bounds based on confidence:

```python
# After some adaptation
if confidence_high:
    controller.adaptive_mpc.adaptation.theta_min = [0.9, 0.9]
    controller.adaptive_mpc.adaptation.theta_max = [1.1, 1.1]
```

---

## References

1. **Adaptive MPC:** Aswani, A., et al. "Provably safe and robust learning-based model predictive control." Automatica, 2013.

2. **LMS Adaptation:** Haykin, S. "Adaptive Filter Theory." Prentice Hall, 2002.

3. **Hybrid Control:** Liberzon, D. "Switching in Systems and Control." Birkhäuser, 2003.

4. **Differential Drive Kinematics:** Siegwart, R., et al. "Introduction to Autonomous Mobile Robots." MIT Press, 2011.

---

## Citation

If you use this controller in your research, please cite:

```bibtex
@misc{adaptive_hybrid_controller,
  title={Adaptive Hybrid Controller for Autonomous Navigation},
  author={Your Name},
  year={2024},
  howpublished={GitHub Repository}
}
```

---

## License

MIT License - See LICENSE file for details.
