# References

Academic references for theoretical foundations and implementation decisions.

----

## Books

### Model Predictive Control

| Citation | Location | Used For |
|----------|----------|----------|
| Rawlings, J.B., Mayne, D.Q., Diehl, M. (2017). *Model Predictive Control: Theory, Computation, and Design* (2nd ed.). Nob Hill Publishing. | [Local PDF](Resources/Books/Model_Predictive_Control_Theory_Computation_and_Design.pdf) | QP formulation, warm-start, move-blocking |
| Borrelli, F., Bemporad, A., Morari, M. (2017). *Predictive Control for Linear and Hybrid Systems*. Cambridge University Press. | [Local PDF](Resources/Books/Predictive_Control_for_Linear_and_Hybrid_Systems.pdf) | Tube MPC, constraint tightening, stability |

### Control Theory

| Citation | Location | Used For |
|----------|----------|----------|
| Åström, K.J., Murray, R.M. (2008). *Feedback Systems: An Introduction for Scientists and Engineers*. Princeton University Press. | [Local PDF](Resources/Books/Feedback_Systems_An_Introduction_for_Scientists_and_Engineers.pdf) | PID tuning, LQR design, stability analysis |

----

## Research Papers

### Tube MPC & Robustness

| Citation | DOI/Link | Used For |
|----------|----------|----------|
| Mayne, D.Q. et al. (2005). "Robust model predictive control of constrained linear systems with bounded disturbances." | Automatica 41(2) | Invariant tubes, constraint tightening |
| TU Munich (2024). "Tube-based MPC for autonomous vehicle path tracking." | [tum.de](https://www.tum.de) | Disturbance observer integration |

### Move-Blocking

| Citation | DOI/Link | Used For |
|----------|----------|----------|
| Imperial College (2023). "Move-blocking strategies for computationally efficient MPC." | [imperial.ac.uk](https://www.imperial.ac.uk) | Decision variable reduction |
| Cagienard et al. (2004). "Move blocking strategies in receding horizon control." | IEEE CDC | Block parameterization |

### Differential-Drive MPC

| Citation | DOI/Link | Used For |
|----------|----------|----------|
| SCS Europe (2023). "MPC for trajectory tracking in differential drive robots." | [scs-europe.net](https://scs-europe.net) | Kinematic model, cold-start handling |
| CEUR-WS (2023). "Nonlinear MPC for mobile robots: orientation error analysis." | [ceur-ws.org](https://ceur-ws.org) | Heading error minimization |

----

## APIs & Datasets

| Resource | Access | Used For |
|----------|--------|----------|
| arXiv API | [api.arxiv.org](https://info.arxiv.org/help/api/index.html) | Paper search |
| Elsevier ScienceDirect | API Key: `[REDACTED]` | Journal access |
| OpenAlex | API Key: `[REDACTED]` | Academic metadata |

----

## Code References

| Library | Version | Used For |
|---------|---------|----------|
| CVXPY | ≥1.4 | Convex optimization, MPC formulation |
| OSQP | ≥0.6 | QP solver, warm-start support |
| NumPy | ≥1.24 | Array operations, linearization |
| Matplotlib | ≥3.7 | Visualization |

----

*Last updated: 2026-02-08*