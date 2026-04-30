# References

Core sources that directly support the theory and implementation in this repository. For a longer annotated list (pillars, code-to-paper map), see [docs/REFERENCES.md](docs/REFERENCES.md).

---

## Textbooks

| Citation | Notes |
|----------|--------|
| J. B. Rawlings, D. Q. Mayne, M. Diehl, *Model Predictive Control: Theory, Computation, and Design*, 2nd ed., Nob Hill, 2017. | [Local PDF](Resources/Books/Model_Predictive_Control_Theory_Computation_and_Design.pdf). MPC formulation, terminal cost, Δu regularization, stability. |
| F. Borrelli, A. Bemporad, M. Morari, *Predictive Control for Linear and Hybrid Systems*, Cambridge, 2017. | [Local PDF](Resources/Books/Predictive_Control_for_Linear_and_Hybrid_Systems.pdf). Hybrid / supervisory MPC; complements the LQR–MPC blend. |

---

## Papers (project anchor)

| Citation | Why it is here |
|----------|----------------|
| N. J. Kong, C. Li, G. Council, and A. M. Johnson, “Hybrid iLQR Model Predictive Control for Contact Implicit Stabilization on Legged Robots,” *IEEE Transactions on Robotics*, vol. 39, no. 6, pp. 4712–4727, Dec. 2023. | **Primary inspiration for the hybrid controller idea and its extension** to this work: HiLQR MPC combines a fast iLQR-style mode with MPC for hybrid/contact dynamics on legged systems (saltation matrix, reference extension, mode mismatch). This repository adapts that *supervisory hybrid* concept to **risk-aware differential-drive navigation** (TurtleBot3, reference tracking, obstacle-aware sigmoid blending), not legged contact-implicit whole-body planning. DOI: [10.1109/TRO.2023.3308773](https://doi.org/10.1109/TRO.2023.3308773). [IEEE Xplore](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10252162&isnumber=10352149). |
| F. Wu *et al.*, “Composing MPC with LQR and Neural Networks for Amortized Efficiency and Stable Control,” arXiv:2112.07238, 2021. | Close analog to **composing LQR with MPC** with stability in mind; motivates the hybrid architecture (we use sigmoid blending instead of a NN mode). |
| D. Q. Mayne, J. B. Rawlings, C. V. Rao, P. O. M. Scokaert, “Constrained model predictive control: Stability and optimality,” *Automatica*, 36(6):789–814, 2000. | Canonical **constrained MPC stability** reference (terminal ingredients, Lyapunov viewpoint). DOI: [10.1016/S0005-1098(99)00214-9](https://doi.org/10.1016/S0005-1098(99)00214-9). |

---

## Software and reference code

| Resource | Role in this project |
|----------|----------------------|
| [KohlerJohannes/Adaptive](https://github.com/KohlerJohannes/Adaptive) | Köhler, *Certainty-equivalent adaptive MPC for uncertain nonlinear systems* (2026): theory and MATLAB reference for **adaptive MPC**; our `AdaptiveMPCController` is Python/CasADi and **not** a port. |
| [CVXPY](https://www.cvxpy.org/) | Convex modeling for the QP-based MPC. |
| [OSQP](https://osqp.org/) | QP solver used with CVXPY. |
| [CasADi](https://web.casadi.org/) | Symbolic NLP for **Adaptive MPC** (`adaptive_mpc_controller.py`). |

---

*Last updated: 2026-04-30*
