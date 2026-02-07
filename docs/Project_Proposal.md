# Project Proposal

## Risk-Aware Hybrid LQR–MPC Navigation for Autonomous Systems

---

## 1. Introduction and Motivation

Autonomous robots must navigate safely and efficiently in environments that may vary significantly in complexity. In open, obstacle-free regions, simple feedback controllers are sufficient due to their low computational cost. However, in cluttered or dynamic environments, controllers must explicitly reason about constraints and future system behavior to avoid collisions.

**Linear Quadratic Regulator (LQR)** and **Model Predictive Control (MPC)** represent two fundamentally different approaches:

| Controller | Strengths | Weaknesses |
|------------|-----------|------------|
| **LQR** | Computational efficiency, strong local stability | Ignores constraints, unsafe near obstacles |
| **MPC** | Explicit constraint handling, predictive planning | High computational cost, resource intensive |

Running MPC continuously is often unnecessary and inefficient. Conversely, relying solely on LQR compromises safety. This motivates a **hybrid control strategy** that combines the strengths of both methods.

---

## 2. Problem Statement

Design a **risk-aware hybrid navigation controller** such that:

- ✅ LQR is used for nominal navigation in low-risk environments
- ✅ MPC is activated only when navigation risk becomes significant
- ✅ Switching between controllers is governed by explicit, measurable risk metrics

### Risk Quantification

Risk is measured using:
1. **Geometric proximity** to obstacles
2. **Predicted constraint violations** over a finite horizon
3. **Estimation or sensing uncertainty**

### Goal

Achieve safety comparable to always-on MPC while maintaining efficiency close to LQR and significantly reducing average computational cost.

---

## 3. Literature Review

### 3.1 LQR-Based Navigation
LQR is widely used for trajectory tracking and stabilization due to its simplicity and optimality for linear systems. However, it fundamentally lacks a mechanism to enforce state or input constraints.

### 3.2 MPC-Based Navigation
MPC is the standard tool for constrained navigation and obstacle avoidance. Despite its strengths, MPC suffers from high computational cost, sensitivity to horizon length, and solver dependence.

### 3.3 Hybrid and Compositional Control
Recent works have explored combining local optimal controllers with receding-horizon planners. However, most existing approaches:
- Are domain-specific (e.g., legged locomotion, contact tasks)
- Use simple distance-based switching
- Do not quantify computational savings versus safety loss

---

## 4. Research Gap

A **clear and general navigation-oriented framework** is missing. Specifically:

- Switching is often reactive rather than predictive
- Risk is rarely quantified beyond raw distance thresholds
- Stability during switching is insufficiently addressed
- Little empirical guidance on tuning switching thresholds

**This project addresses these gaps by introducing predictive, risk-aware switching and systematic experimental evaluation.**

---

## 5. Technical Architecture

### 5.1 Control Stack

```
┌─────────────────────────────────────────────┐
│        Risk-Aware Supervisory Layer         │
│   (Evaluates risk, decides active controller)│
├─────────────────┬───────────────────────────┤
│  LQR Controller │       MPC Controller      │
│   (Nominal)     │    (Safety-Critical)      │
├─────────────────┴───────────────────────────┤
│           System Modeling Layer             │
│    (Differential drive kinematics)          │
└─────────────────────────────────────────────┘
```

### 5.2 Layer Responsibilities

| Layer | Responsibility |
|-------|----------------|
| **System Modeling** | Discrete-time state-space representation |
| **LQR Controller** | Fast response, low overhead, default mode |
| **MPC Controller** | Constraint enforcement, obstacle avoidance |
| **Supervisor** | Risk evaluation, switching logic |

---

## 6. Platform Choice

### Primary Platform: Single Ground Robot

| Aspect | Details |
|--------|---------|
| **Model** | Differential-drive kinematic model |
| **Dynamics** | Slow, well-behaved, linearization valid |
| **Obstacle Avoidance** | Predominantly planar |
| **Risk Level** | Minimal simulation/hardware risks |

### Optional Extensions
- **UAVs**: Faster dynamics, tighter actuator constraints
- **Multi-Robot Systems**: Distributed decision-making, formation control

---

## 7. Engineering Phases

### Phase 1: Foundations ✅
- [x] Select robot model and simulation environment
- [x] Derive linearized dynamics
- [x] Implement LQR baseline

### Phase 2: Safety Controller ✅
- [x] Formulate MPC optimization problem
- [x] Select solver (CVXPY) and horizon
- [x] Implement warm-start strategy

### Phase 3: Risk Supervisor (Future)
- [ ] Implement distance-based risk metric
- [ ] Implement predictive constraint violation check
- [ ] Add uncertainty-aware risk metric

### Phase 4: Hybrid Integration (Future)
- [ ] Implement switching logic
- [ ] Add hysteresis and dwell-time
- [ ] Validate stability in long simulations

### Phase 5: Experiments (Future)
- [ ] Open-space navigation
- [ ] Static cluttered environment
- [ ] Dynamic obstacle scenarios

---

## 8. Performance Metrics

| Metric | Description |
|--------|-------------|
| **Collision Rate** | Frequency of collisions/near-collisions |
| **Tracking Error** | Deviation from reference trajectory |
| **Control Effort** | Actuator usage and energy consumption |
| **Computation Time** | Time per control step (real-time feasibility) |
| **Switching Statistics** | MPC activation frequency and duration |

---

## 9. Implementation Environment

| Component | Choice |
|-----------|--------|
| **Framework** | ROS 2 (Humble) |
| **Simulation** | Gazebo |
| **Language** | Python |
| **MPC Solver** | CVXPY (with ECOS/OSQP backend) |
| **Prototyping** | Standalone Python simulation |

---

## 10. Key References

1. **Kong et al.** - *Hybrid iLQR Model Predictive Control for Contact-Implicit Stabilization on Legged Robots*, IEEE T-RO 2023

2. **Le Cleac'h et al.** - *Fast Contact-Implicit Model Predictive Control*, IEEE T-RO 2024

3. **Wu et al.** - *Composing MPC with LQR and Neural Networks for Real-Time Hybrid Control*, arXiv 2021

4. **Awad et al.** - *Model Predictive Control with Fuzzy Logic Switching for Path Tracking of Autonomous Vehicles*, 2022

---

## 11. Conclusion

This project proposes a principled, risk-aware hybrid LQR–MPC navigation framework that balances safety and efficiency. By explicitly quantifying risk and systematically evaluating performance trade-offs, the work provides both practical insights and research contributions suitable for academic dissemination.

---

## Contact

| Name | GitHub | Email |
|------|--------|-------|
| Kshitiz | [@Erebuzzz](https://github.com/Erebuzzz) | kshitiz23@iiserb.ac.in |
| Agolika | [@Agolika413](https://github.com/Agolika413) | agolika23@iiserb.ac.in |
