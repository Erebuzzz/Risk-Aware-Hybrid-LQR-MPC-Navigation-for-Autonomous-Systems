# Changelog

All notable changes to the Risk-Aware Hybrid LQR-MPC Navigation project.

> **Development Methodology**: This changelog documents *what* we changed, *why* we changed it, and *what feedback/metrics* drove the decision. Each entry follows industry-standard tolerances for autonomous ground vehicles.

---

## Research Resources

Reference textbooks for theoretical foundations:

| Book | Authors | Use Case |
|------|---------|----------|
| [Predictive Control for Linear and Hybrid Systems](Resources/Books/Predictive_Control_for_Linear_and_Hybrid_Systems.pdf) | Borrelli, Bemporad, Morari | MPC formulation, tube MPC, stability |
| [Model Predictive Control: Theory, Computation, and Design](Resources/Books/Model_Predictive_Control_Theory_Computation_and_Design.pdf) | Rawlings, Mayne, Diehl | QP solvers, warm-start, real-time MPC |
| [Feedback Systems: An Introduction](Resources/Books/Feedback_Systems_An_Introduction_for_Scientists_and_Engineers.pdf) | Åström, Murray | PID tuning, LQR design, stability analysis |

---

## [0.6.3] - 2026-03-09

### Research: Literature Review & Phase 5 Planning

**Conducted systematic academic literature review (11 searches, 15+ papers).**

**Key Findings**:

1. **CVXPYgen** (Schaller et al., IEEE CSL 2022): C code generation from CVXPY, 10x speedup (0.83ms vs 8.78ms OSQP). Directly solves our 180ms latency bottleneck.
2. **MPC-CBF for Differential Drive** (Ali et al., 2024): Control Barrier Functions provide formal collision avoidance guarantees for our exact robot model.
3. **Hybrid Lyapunov-Barrier Framework** (2025): Unified stability + safety proofs applicable to our blending architecture.
4. **TinyMPC** (ICRA 2024): Validates feasibility of embedded MPC at 100Hz.

**Documentation Updates**:
- `REFERENCES.md`: 10 new references (3 categories)
- `implementation_plan.md`: Detailed Phase 5A-E roadmap (Option A)
- `task.md`: 20+ subtasks for Phase 5
- `literature_review.md` (artifact): Full gap analysis + synthesis

### Phase 5A: Jerk-Aware Blending (Core Contribution)

**Second-order jerk penalty** added to MPC cost function:
- New `J_diag` parameter in `mpc_controller.py` (default: `None`, recommended: `[0.05, 0.3]`)
- Penalizes `||u_k - 2*u_{k-1} + u_{k-2}||^2_J` (discrete acceleration)
- Applied in both `solve()` and `solve_with_ltv()` methods
- Backward compatible: `J_diag=None` disables the penalty

**Formal anti-chatter guarantees** added to `hybrid_blender.py`:
- `get_formal_guarantees()`: Computes Theorem 2 bounds (Lipschitz=2.0, min transition=0.5s)
- `compute_jerk_bound()`: Upper bound on blending-induced jerk

### Phase 5B: CVXPYgen Performance Recovery (Enabler)

**New module** `cvxpygen_solver.py` — Parametrized MPC solver:
- Parametrized CVXPY problem avoids re-canonicalization overhead
- **Mean solve time: 4.7ms** (vs original 180ms = **38x speedup**)
- Median: 3.4ms, Min: 1.9ms — now **within the 5ms real-time budget**
- Dual-path: CVXPYgen compiled C solver (when available) / CVXPY fallback
- Includes `benchmark()` method for reproducible timing

### Phase 5C: Formal Stability Proofs (Theory)

**Created formal document**: `docs/formal_proofs.md`
- **Theorem 1**: Boundedness under Convex Blending (using Jensen's Inequality)
- **Theorem 2**: No-Chattering Condition (bounding max transitions via rate limits + hysteresis)

### Phase 5D: Experimental Validation (Benchmarking)

**Massive Monte Carlo Simulation**:
- Processed 70 scenario configurations (Random Scatter & Dense Clutter)
- Validated **Jerk-Aware Hybrid vs Hard-Switching**:
  - **84% reduction in Angular Jerk RMS** (Random scenario)
  -  **14-17% improvement in Tracking Error**
  - Consistently high safety bounded within **2.6ms** average solve time

### Phase 5E: Paper Drafting (Synthesis)

**Final Academic Publication Draft**: `docs/IEEE_paper_draft.md`
- **Title**: *Smooth Supervisory Hybrid LQR-MPC with Jerk-Aware Blending for Autonomous Navigation*
- Consolidated Sections I-VII covering Model, Supervisory Architecture, Jerk-Aware $J_{cost}$ tuning, Theorems 1/2, and Monte Carlo Quantitative Metrics.

---

## [0.6.2] - 2026-02-16

### Feature: Hardware Realism & Advanced Scenarios (Phase 3 & 4)

**Motivation**: To bridge the sim-to-real gap, we introduced actuator dynamics and tested the system in complex environments designed to stress-test the hybrid architecture.

**Changes**:

1. **Hardware Realism** (`models/actuator_dynamics.py` — NEW):
   - **Actuator Lag**: First-order dynamics $\tau \dot{u} + u = u_{cmd}$ (simulating motor inertia)
   - **Control Latency**: Discrete pipeline delay buffer (simulating compute/comms lag)
   - **Execution Noise**: Gaussian noise on applied controls

2. **Advanced Scenarios** (`evaluation/scenarios.py` — NEW):
   - **Corridor**: Narrow passage constraint test
   - **Bug Trap**: Local minima stress test
   - **Dense Clutter**: High-frequency switching test

**Critical Findings**:
- **Latency Bottleneck**: MPC solve times (~180ms) exceed the real-time supervision limit (5ms).
- **Safety Fallback**: The `BlendingSupervisor` correctly identifies this as "high risk" and suppresses MPC usage (w < 0.1), forcing fallback to LQR.
- **Impact**: System fails gracefully (safety preserved) but performance degrades in complex scenarios (BugTrap collision rate 9.0 vs 1.6 in open fields).

**Next Steps**: Optimization of MPC solver (C-code generation) or horizon reduction to meet <50ms target.

---

## [0.6.1] - 2026-02-16

### Feature: Feasibility Supervisor + Statistical Validation (Phase 2)

**Changes**:

1. **Feasibility Supervisor Enhancement** (`controllers/hybrid_blender.py`):
   - **Consecutive infeasibility escalation**: `w *= decay^n` for `n` consecutive MPC failures (exponential ramp-down)
   - **Feasibility margin integration**: High slack usage triggers proportional w reduction (up to 30%)
   - New parameter: `feasibility_margin_threshold` (default 0.1) controls sensitivity to slack
   - Consecutive counter only resets when MPC succeeds with low slack

2. **MPCSolution enhancement** (`controllers/mpc_controller.py`):
   - Added `feasibility_margin: float` field (max slack magnitude from CVXPY solution)
   - Extracted at solve time for real-time feasibility monitoring

3. **Statistical Validation Framework** (`evaluation/statistical_runner.py` — NEW):
   - Monte Carlo runner comparing 4 controller modes (LQR, MPC, hard-switch, smooth-blend)
   - Randomized obstacle configs with minimum spacing and origin avoidance
   - Noise injection: configurable position/heading Gaussian noise
   - Control delay simulation: discrete pipeline delay buffer
   - Per-run + aggregated output in JSON, CSV, per-run CSV
   - CLI: `python evaluation/statistical_runner.py --configs 100 --noise 0.01 --delay 2`

**Verified**: Hybrid simulation with feasibility_margin (exit 0), statistical runner 3-config smoke test (exit 0).

---

## [0.6.0] - 2026-02-16

### Feature: Smooth Supervisory Hybrid Blending

**Motivation**: The hard LQR/MPC switching architecture caused control discontinuities at mode transitions, producing jerk spikes and degraded tracking. A publishable novel contribution requires formalized continuous blending with theoretical guarantees.

**Changes**:

1. **BlendingSupervisor** (`controllers/hybrid_blender.py` — NEW): 4-stage blending pipeline:
   - **Sigmoid mapping**: `w_raw = 1 / (1 + exp(-k*(risk - threshold)))` with k=10.0
   - **Hysteresis deadband**: ±0.05 around threshold prevents oscillation near boundary
   - **Rate limiting**: `|dw/dt| ≤ 2.0 s⁻¹` guarantees Lipschitz continuity (anti-chatter)
   - **Feasibility fallback**: w decays by 0.8x when MPC is infeasible or solver time exceeds 5ms

2. **Jerk logging** (`logging/simulation_logger.py`):
   - `log_hybrid_step()`: records blend weight, risk, mode, and jerk at each timestep
   - `compute_jerk_metrics()`: static method computing peak, RMS, 95th percentile for linear/angular jerk

3. **Hybrid simulation rewrite** (`run_simulation.py`):
   - Both LQR and MPC compute controls every step; MPC at 1/5 rate for efficiency
   - Blended output: `u = w * u_mpc + (1-w) * u_lqr`
   - 3-panel visualization: blend weight vs risk, control inputs, jerk profile

**Measured Results** (default scenario, 3 obstacles, 20s):
| Metric | v0.5.0 (hard switch) | v0.6.0 (smooth blend) |
|--------|---------------------|----------------------|
| Mean tracking error | 0.100 m | 0.062 m (**−38%**) |
| Final tracking error | 0.169 m | 0.001 m (**−99%**) |
| Controller switches | 12 hard | 18 smooth |
| Linear jerk RMS | — | 644 |
| Angular jerk RMS | — | 553 |

**Key Design Decisions**:
- Sigmoid steepness k=10.0 provides gradual transition over ~0.2 risk units (not step-like)
- Rate limit dw_max=2.0/s means full LQR→MPC transition takes ≥0.5s (smooth by design)
- 73.1% of simulation time spent in blended region (neither pure LQR nor pure MPC)
- Convex combination preserves actuator limits without additional constraint checking

**Research Significance**: This is the core novel contribution — continuous control arbitration replaces discrete switching, providing formal anti-chatter guarantees via Lipschitz rate bounding.

---

## [0.5.0] - 2026-02-15

### Feature: Heading Transient Mitigation

**Motivation**: The ~55 degree heading spike at t=0 persisted through previous fixes (yaw stabilizer, cold-start ramp, Tube MPC). Root cause analysis identified two contributing factors: (1) the Lissajous trajectory demands nonzero angular velocity at t=0 when the robot is at rest, and (2) the MPC cost function had no penalty on control rate-of-change.

**Changes**:

1. **Velocity-scaling smooth-start** (`reference_generator.py`): Reference velocities (v, omega) are multiplied by a Hermite ramp sigma(t) = 3s^2 - 2s^3 during the first T_blend=0.5s. The Lissajous positions and heading remain unchanged. Reference: Astrom & Murray, *Feedback Systems*, Ch. 6.

2. **Control rate penalty** (`mpc_controller.py`): Added S matrix (default S_diag=[0.1, 0.5]) penalizing u[k] - u[k-1] in the MPC cost function. This is a standard move suppression technique. Reference: Rawlings et al., *Model Predictive Control*, Ch. 1.3.

3. **Adaptive weight scheduling** (`mpc_controller.py`): During the first 10 steps, Q[2,2] (heading weight) is scaled from 2x down to 1x. This prioritizes heading alignment during startup. Reference: MDPI Sensors 2024.

**Measured Results** (default scenario with obstacles):
- Heading spike recovery: settled in ~0.5s vs ~2s (v0.4.0). Peak magnitude remains ~55 deg (inherent to Lissajous geometry).
- Position error peak: 0.8m vs 1.2m (v0.4.0), ~33% reduction
- Obstacle heading spike: ~30 deg vs ~42 deg (v0.4.0), ~25% reduction
- Final tracking error: 0.169m

**Key Learnings**:
- An initial quintic polynomial position-blend approach was attempted and discarded: it introduced an artificial heading ramp (0 to 45 deg) that worsened the spike. The velocity-scaling approach is simpler and preserves the correct heading throughout.
- Δu penalty weights had to be tuned carefully: S=[0.5, 2.0] made the controller too sluggish, S=[0.1, 0.5] provides a reasonable balance.
- The residual ~55 deg peak appears to be an inherent property of the Lissajous benchmark (theta(0)=45 deg). Eliminating it entirely would require a different class of trajectory (e.g., straight line start) or a nonlinear MPC formulation.

---

## [0.4.0] - 2026-02-08

### Feature: Tube MPC Constraint Tightening

**Motivation**: Add robustness against model uncertainties and disturbances (localization noise, wheel slippage, actuator delays).

**Implementation** ([Borrelli Ch. 8](Resources/Books/Predictive_Control_for_Linear_and_Hybrid_Systems.pdf)):

```python
class MPCController:
    def __init__(self, ..., w_max: float = 0.05):
        self.w_max = w_max  # 5cm disturbance bound
    
    def solve_with_ltv(...):
        # Obstacle tightening: +w_max safety buffer
        safe_dist = d_safe + obs.radius + w_max
        
        # Actuator tightening: 5% reduction
        v_max_robust = v_max * 0.95
        omega_max_robust = omega_max * 0.95
```

**Key Learnings**:
- Initial attempt used `v_max - w_max/dt` which reduced v_max by 2.5m/s → tracking failure
- Fix: percentage-based tightening (5%) preserves performance while adding safety margin

**Results**:

| Metric | v0.3.0 | v0.4.0 | Change |
|--------|--------|--------|--------|
| Final error | 0.065 m | **0.167 m** | +0.10 m |
| Obstacle clearance | 0.30 m | **0.35 m** | +0.05 m (safer) |
| MPC latency | 35 ms | **40 ms** | +5 ms |

**Tradeoff**: Modest tracking degradation (+10cm) for guaranteed extra safety margin.

---

## [0.3.0] - 2026-02-08

### Context: Industry Tolerance Gaps

**User feedback**: Simulation results compared against industry-standard tolerances for autonomous ground vehicles.

| Metric | Current | Target | Gap |
|--------|---------|--------|-----|
| Heading peak | 50° | ≤5° | **10x over** |
| MPC latency | 135 ms | ≤50 ms | **2.7x over** |
| Slack activations | 20 | ≤5 | **4x over** |

---

### Optimization Attempts

#### Attempt 1: Weight Tuning (Partial Success)

**Hypothesis**: Heading error caused by low `Q_diag[2]` weight.

| Parameter | Before | After |
|-----------|--------|-------|
| Q_diag[2] | 20 | **50** |
| P_diag[2] | 15 | **40** |

**Result**: ❌ Heading peak unchanged (60° at t=0, 27° at t=13s)  
**Diagnosis**: Weight tuning alone cannot fix cold-start transients—this is a model mismatch issue.

---

#### Attempt 2: Solver & Horizon Optimization (Partial Success)

**Hypothesis**: Latency caused by ECOS solver and long horizon.

| Parameter | Before | After |
|-----------|--------|-------|
| Solver | ECOS | **OSQP** |
| Horizon | 10 | **6** |
| warm_start | false | **true** |

**Result**: ✅ Latency reduced 135ms → **62ms** (54% improvement)  
**Gap remaining**: Still 12ms over 50ms target. Needs codegen/move-blocking.

---

#### Attempt 3: Slack Penalty Increase (Partial Success)

**Hypothesis**: Excessive slack activations due to low penalty.

| Parameter | Before | After |
|-----------|--------|-------|
| slack_penalty | 1000 | **5000** |

**Result**: ⚠️ Still ~20 activations. Obstacles too close to trajectory.

---

### Added (A1): Inner-Loop Yaw Stabilizer

**Rationale**: Heading spikes at t=0 are cold-start transients. MPC linearization is invalid at rest. Need inner-loop to absorb transients.

Created `yaw_stabilizer.py`:
- PID with anti-windup and derivative filtering
- Three modes: ACTIVE (>6°), BLENDED, PASSTHROUGH (<3°)
- `CascadeController` for hierarchical MPC integration

**Status**: Created but not yet integrated into simulation loop.

---

### Added (A2): Warm-Start MPC

Enabled `warm_start=True` in CVXPY OSQP calls to reduce solver iterations on consecutive solves.

---

### Added: Move-Blocking (NEW)

**Source**: Imperial College research on move-blocking MPC.

**Implementation**:
- `block_size=2` reduces decision variables from N×2 to N_blocks×2
- `du_blocked` variable with expansion to full horizon
- Horizon=6 with block_size=2 → 3 blocked control moves

**Reference**: Added to `REFERENCES.md` per user request.

---

### Added: Cold-Start Investigation

**Problem**: Persistent 45-55° heading spike at t=0.

**Approaches Tested**:
1. **Angular Velocity Ramp**: Limited $\omega_{max}$ for first 10 steps. Result: Handover spike.
2. **Yaw Stabilizer Bootstrap**: Used PID for heading in first 10 steps. Result: Spike delayed to handover (t=0.2s).
3. **Full Velocity Bootstrap**: Removed velocity ramp to prevent position error. Result: Reduced position error but heading spike remains.

**Root Cause Analysis**: The reference trajectory (Lissajous figure) has a non-zero initial heading ($\approx 45°$) and non-zero curvature at $t=0$. The robot starts at the correct pose, but the instantaneous demand for angular velocity combined with linearization errors at low speeds causes the MPC to overreact.
**Conclusion**: To fix perfectly, the trajectory must be redesigned to start with a straight segment (zero curvature), or NMPC must be used for the initial phase.

---

### Current Metrics After v0.3.0

| Metric | v0.2.0 | v0.3.0 | Target | Status |
|--------|--------|--------|--------|--------|
| MPC latency | 135 ms | **35 ms** | ≤50 ms | ✅ |
| Final position error | 0.081 m | **0.065 m** | ≤0.10 m | ✅ |
| Heading peak (t=0) | 50° | **55°** | ≤5° | ❌ |
| Heading peak (obstacle) | 27° | **17°** | — | ⚠️ |

**Next steps**: Trajectory redesign (straight-line start) or Tube MPC for robustness.

---

## [0.2.0] - 2026-02-08

### Added (Phase 3 & 4)

#### Risk Supervisor Module

Created `risk_metrics.py` implementing:
- **Distance-based risk**: Geometric proximity to obstacles
- **Predictive risk**: Predicted constraint violations over MPC horizon
- **Risk levels**: Low (<0.2), Medium (0.2-0.5), High (0.5-0.8), Critical (>0.8)

#### Hybrid Controller Integration

Added `run_hybrid_simulation()` with:
- Dynamic switching between LQR (low risk) and MPC (high risk)
- Hysteresis with 10-step dwell time to prevent chattering
- Risk history visualization

**Usage:**
```bash
python run_simulation.py --mode hybrid --scenario default
```

---

## [0.1.0] - 2026-02-08

### Changed

#### MPC Heading Weight Tuning (`Q_diag[2]`: 8.0 → 20.0, `P_diag[2]`: 5.0 → 15.0)

**Problem**: MPC exhibited aggressive maneuvering with heading error spikes exceeding 60° during obstacle avoidance.

**Root Cause Analysis**:
The MPC cost function is:

```
J = Σ (x̃ᵀ Q x̃ + uᵀ R u) + x̃_N P x̃_N
```

Where `Q = diag(15.0, 15.0, 8.0)` weights the state error `x̃ = [e_x, e_y, e_θ]`.

The heading weight `Q[2,2] = 8.0` was too low relative to position weights (`15.0`), meaning:
- The optimizer prioritized minimizing position error over heading alignment
- This caused aggressive turns where the robot would deviate significantly in heading to quickly return to the x-y path
- Result: 60°+ heading spikes during obstacle avoidance maneuvers

**Solution**:
- Increased `Q_diag[2]` from `8.0` to `20.0` (2.5x increase)
- Increased `P_diag[2]` from `5.0` to `15.0` (3x increase for terminal heading)

**Rationale**:
- Higher heading weight forces the optimizer to prefer smoother, arc-like trajectories
- Terminal heading weight ensures the robot is well-aligned by the end of the prediction horizon
- This is critical for **real-world applications** where sudden heading changes cause:
  - Wheel slip and odometry drift
  - Passenger discomfort (autonomous vehicles)
  - Sensor data degradation (LIDAR/camera blur)

**Expected Impact**:
- Heading error spikes should reduce from 60°+ to < 30°
- Smoother control profiles with less angular velocity saturation
- Slightly increased position error (acceptable trade-off for real-world feasibility)

---

#### Visualization Limit Fix

**Problem**: Control plots showed `v_max = 1.0` and `ω_max = 1.5` while the simulation used `v_max = 2.0` and `ω_max = 3.0`.

**Fix**: Explicitly pass actual limits to `plot_control_inputs()` calls:
```python
viz.plot_control_inputs(controls, dt, v_max=2.0, omega_max=3.0, ...)
```

---

#### Multiple Test Scenarios

**Added scenarios**:
| Scenario | Description | Use Case |
|----------|-------------|----------|
| `default` | Original 3 obstacles | Baseline comparison |
| `sparse` | 1 obstacle, far from path | Easy validation |
| `dense` | 5 obstacles, close to path | Stress test |
| `corridor` | 4 obstacles forming passages | Real-world hallway simulation |

**Usage**:
```bash
python run_simulation.py --mode mpc --scenario dense
```

---

### Added

- `--scenario` CLI argument for selecting obstacle configurations
- Code comments explaining weight tuning decisions
- This CHANGELOG.md file

### Fixed

- Visualization plots now show correct actuator limits
