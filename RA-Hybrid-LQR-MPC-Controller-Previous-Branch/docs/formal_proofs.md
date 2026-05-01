# Appendix: Formal proofs for Risk-Aware Hybrid Navigation Framework

This document contains the formal mathematical proofs supporting the theoretical guarantees of the Smooth Supervisory Hybrid LQR-MPC architecture. These proofs form the theoretical foundation of the associated research paper.

## Theorem 1: Boundedness under Convex Blending

**Statement:**
Consider a discrete-time nonlinear system $x_{k+1} = f(x_k, u_k)$. Let $u^{MPC}(x)$ and $u^{LQR}(x)$ be two stabilizing control laws with associated Lyapunov functions $V_{MPC}(x)$ and $V_{LQR}(x)$ such that:
1. $V_{MPC}(f(x, u^{MPC})) - V_{MPC}(x) \leq -\alpha_{MPC}(||x||)$
2. $V_{LQR}(f(x, u^{LQR})) - V_{LQR}(x) \leq -\alpha_{LQR}(||x||)$

Under the continuous convex blending law $u(x, t) = w(t)u^{MPC}(x) + (1-w(t))u^{LQR}(x)$ where $w(t) \in [0, 1]$, the closed-loop system state remains bounded, and the unified Lyapunov function candidate $V(x) = w V_{MPC}(x) + (1-w) V_{LQR}(x)$ decreases strictly outside a bounded region $\mathcal{B}$.

**Proof:**
By definition of the blending law, $w(t) \in [0, 1] \implies (1-w(t)) \in [0, 1]$.
The control input is a convex combination of two admissible control inputs. Assuming the input constraint set $\mathcal{U}$ is convex, $u_{blend} \in \mathcal{U}$ is strictly satisfied.

Consider the unified Lyapunov candidate:
$V(x, t) = w(t) V_{MPC}(x) + (1-w(t)) V_{LQR}(x)$

Computing the difference along the system trajectories:
$\Delta V = V(x_{k+1}, t_{k+1}) - V(x_k, t_k)$
$\Delta V = w_{k+1}V_{MPC}(x_{k+1}) + (1-w_{k+1})V_{LQR}(x_{k+1}) - [w_k V_{MPC}(x_k) + (1-w_k) V_{LQR}(x_k)]$

Let $w_{k+1} = w_k + \Delta w$. Substituting and rearranging:
$\Delta V = w_k [V_{MPC}(x_{k+1}) - V_{MPC}(x_k)] + (1-w_k) [V_{LQR}(x_{k+1}) - V_{LQR}(x_k)] + \Delta w [V_{MPC}(x_{k+1}) - V_{LQR}(x_{k+1})]$

Because $u_{blend}$ is a convex combination, the system dynamics $f(x, u_{blend})$ can be approximated via Taylor expansion around the individual control inputs. For control-affine systems $f(x,u) = g(x) + h(x)u$, the dynamics are exactly linear in $u$:
$f(x, u_{blend}) = w f(x, u^{MPC}) + (1-w) f(x, u^{LQR})$

Assuming $V_{MPC}$ and $V_{LQR}$ are convex functions, by Jensen's Inequality:
$V_i(f(x, u_{blend})) \leq w V_i(f(x, u^{MPC})) + (1-w) V_i(f(x, u^{LQR}))$ for $i \in \{MPC, LQR\}$

Applying the individual stability conditions:
$\Delta V \leq -w_k \alpha_{MPC}(||x||) - (1-w_k) \alpha_{LQR}(||x||) + L_{blend} ||\Delta w||$
where $L_{blend}$ is a bound on the difference $|V_{MPC}(x_{k+1}) - V_{LQR}(x_{k+1})|$.

Since the blending supervisor enforces a strict rate limit $|\Delta w| \leq \dot{w}_{max} \Delta t$, the perturbation term is strictly bounded by $L_{blend} \dot{w}_{max} \Delta t$.

For sufficiently large $||x||$, the negative definite terms dominate:
$w_k \alpha_{MPC}(||x||) + (1-w_k) \alpha_{LQR}(||x||) > L_{blend} \dot{w}_{max} \Delta t$
ensuring $\Delta V < 0$.

Therefore, the state $x_k$ ultimately converges to and remains within a bounded set $\mathcal{B}$ whose size is proportional to the maximum blending rate $\dot{w}_{max}$. $\blacksquare$

---

## Theorem 2: No-Chattering Condition

**Statement:**
For the continuous blending supervisor defined by:
$w_{target}(t) = \sigma(r(t)) = \frac{1}{1 + e^{-k(r(t) - r_{thresh})}}$
where $r(t) \in [0, 1]$ is the scalar risk metric.

Given a hysteresis deadband $h > 0$ and a strict derivative bound $|\dot{w}(t)| \leq \dot{w}_{max}$, the closed-loop system is strictly free of Zeno behavior (infinite switching in finite time) and the maximum frequency of control mode transitions (crossing $w=0.5$) is bounded by $f_{max} \leq \frac{\dot{w}_{max}}{2}$.

**Proof:**
A "switch" or "transition" is defined as the blending weight $w(t)$ crossing the midpoint threshold $w=0.5$ from one stable domain (e.g., $w < 0.1$) to the other (e.g., $w > 0.9$).

To complete one full switching cycle (LQR-dominant $\to$ MPC-dominant $\to$ LQR-dominant), the blending weight must traverse from $w=0$ to $w=1$ and back to $w=0$.
The total variation required for this cycle is $\Delta w_{total} = 2.0$.

By construction in the `BlendingSupervisor` class, the weight derivative is strictly enforced:
$|\dot{w}(t)| \leq \dot{w}_{max} \quad \forall t$

Therefore, the minimum time $T_{cycle}$ required to complete one full switching cycle is constrained by the maximum rate of change:
$T_{cycle} \geq \frac{\Delta w_{total}}{\dot{w}_{max}} = \frac{2.0}{\dot{w}_{max}}$

The maximum switching frequency is the inverse of the minimum cycle time:
$f_{max} = \frac{1}{T_{cycle}} \leq \frac{\dot{w}_{max}}{2}$

Furthermore, the hysteresis mechanism defines a strict deadband $(r_{thresh} - h, r_{thresh} + h)$. While the risk metric $r(t)$ remains within this deadband, $\dot{w}_{target} = 0$.

For a chattering event to occur, the external environment or sensor noise must drive the risk metric $r(t)$ across the entire deadband width $\Delta r = 2h$. Assuming the risk metric itself has bounded variations (e.g., bounded obstacle relative velocity), the time required to traverse the deadband strictly separates switching events.

Since the minimum time between transitions is lower-bounded by a strictly positive constant $T \geq \min(\frac{1}{\dot{w}_{max}}, \frac{2h}{\dot{r}_{max}}) > 0$, Zeno behavior (where time between switches approaches 0) is mathematically impossible. $\blacksquare$
