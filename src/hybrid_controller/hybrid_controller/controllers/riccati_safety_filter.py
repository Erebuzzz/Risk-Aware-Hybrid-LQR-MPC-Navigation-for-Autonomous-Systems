"""Riccati-backed tangent-plane QP safety filter.

This Python implementation is the reference formulation used for parity with
the Rust core.  The QP workspace is parameterized and reused across calls.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence
import time

import cvxpy as cp
import numpy as np
from scipy.linalg import solve_discrete_are

from ..models.augmented_dynamics import AugmentedDifferentialDriveModel
from .cvar_gate import CVaRRiskGate, ObstacleKinematics
from .rust_core import RustHorizonQpWorkspace


@dataclass(frozen=True)
class SafetyFilterConfig:
    """Reference QP configuration."""

    horizon: int = 8
    max_obstacles: int = 8
    dt: float = 0.05
    d_safe: float = 0.30
    robot_radius: float = 0.105
    alpha: float = 0.95
    v_min: float = -1.0
    v_max: float = 1.0
    omega_min: float = -2.0
    omega_max: float = 2.0
    q_diag: tuple[float, ...] = (20.0, 20.0, 8.0, 2.0, 1.0)
    r_diag: tuple[float, ...] = (0.2, 0.4)
    r_filter_diag: tuple[float, ...] = (2.0, 4.0)
    rate_penalty_diag: tuple[float, ...] = (0.1, 0.5)
    max_control_delta: tuple[float, ...] = (0.15, 0.15)
    slack_penalty: float = 10000.0
    kappa_slip: float = 1.0
    backend: str = "python"

    def __post_init__(self) -> None:
        if self.horizon < 1 or self.max_obstacles < 1:
            raise ValueError("horizon and max_obstacles must be positive")
        if (
            len(self.q_diag) != 5
            or len(self.r_diag) != 2
            or len(self.r_filter_diag) != 2
            or len(self.rate_penalty_diag) != 2
            or len(self.max_control_delta) != 2
        ):
            raise ValueError("weight dimensions must match the model")
        if self.slack_penalty <= 0.0:
            raise ValueError("slack_penalty must be positive")
        if any(value <= 0.0 for value in self.max_control_delta):
            raise ValueError("max_control_delta must be positive")
        if self.backend not in {"python", "rust"}:
            raise ValueError("backend must be 'python' or 'rust'")


@dataclass
class SafetyFilterSolution:
    """QP result and diagnostics."""

    status: str
    optimal_control: np.ndarray
    control_sequence: np.ndarray
    predicted_states: np.ndarray
    solve_time_ms: float
    intervention_norm: float
    slack_max: float
    feasible: bool

    @property
    def safety_satisfied(self) -> bool:
        """Whether the solved QP satisfied the soft obstacle constraints.

        ``feasible`` describes solver termination.  A solved QP can still use
        positive obstacle slack, so callers that need a safety-preserving
        command must check this stricter predicate and select a backup policy
        when it is false.
        """
        return bool(
            self.feasible
            and np.isfinite(self.slack_max)
            and self.slack_max <= 1.0e-6
        )


class RiccatiSafetyFilter:
    """Reusable tangent-plane OSQP safety filter around an LQR command."""

    def __init__(
        self,
        model: AugmentedDifferentialDriveModel | None = None,
        config: SafetyFilterConfig | None = None,
    ) -> None:
        self.config = config or SafetyFilterConfig()
        self.model = model or AugmentedDifferentialDriveModel()
        if abs(self.model.dt - self.config.dt) > 1e-12:
            raise ValueError("model dt and safety filter dt must match")
        self.nx = 5
        self.nu = 2
        self.N = self.config.horizon
        self.max_obstacles = self.config.max_obstacles
        self.Q = np.diag(self.config.q_diag)
        self.R = np.diag(self.config.r_diag)
        self.R_filter = np.diag(self.config.r_filter_diag)
        self.R_rate = np.diag(self.config.rate_penalty_diag)
        self.P = self._compute_dare_terminal_cost()
        self.K = self._compute_lqr_gain()
        self.risk_gate = CVaRRiskGate(
            alpha=self.config.alpha,
            d_safe=self.config.d_safe,
            robot_radius=self.config.robot_radius,
            dt=self.config.dt,
        )
        self._rust_workspace: RustHorizonQpWorkspace | None = None
        if self.config.backend == "rust":
            self._rust_workspace = RustHorizonQpWorkspace(
                horizon=self.N,
                max_obstacles=self.max_obstacles,
                q_diag=self.config.q_diag,
                r_diag=self.config.r_diag,
                terminal_cost=self.P,
                r_filter_diag=self.config.r_filter_diag,
                rate_penalty_diag=self.config.rate_penalty_diag,
                max_control_delta=self.config.max_control_delta,
                slack_penalty=self.config.slack_penalty,
                v_min=self.config.v_min,
                v_max=self.config.v_max,
                omega_min=self.config.omega_min,
                omega_max=self.config.omega_max,
            )
        else:
            self._build_problem()
        self._previous_dx: np.ndarray | None = None
        self._previous_du: np.ndarray | None = None
        self._previous_slack: np.ndarray | None = None
        self._previous_command: np.ndarray | None = None

    def _nominal_linearization(self) -> tuple[np.ndarray, np.ndarray]:
        state = np.array([0.0, 0.0, 0.0, 0.4, 0.0], dtype=float)
        control = np.array([0.4, 0.0], dtype=float)
        return self.model.jacobians(state, control, np.zeros(3))

    def _compute_dare_terminal_cost(self) -> np.ndarray:
        a_nom, b_nom = self._nominal_linearization()
        try:
            result = solve_discrete_are(a_nom, b_nom, self.Q, self.R)
            if np.all(np.linalg.eigvalsh(result) > 0.0):
                return 0.5 * (result + result.T)
        except Exception:
            pass
        return 10.0 * self.Q

    def _compute_lqr_gain(self) -> np.ndarray:
        a_nom, b_nom = self._nominal_linearization()
        try:
            return np.linalg.solve(
                self.R + b_nom.T @ self.P @ b_nom,
                b_nom.T @ self.P @ a_nom,
            )
        except np.linalg.LinAlgError:
            return np.zeros((self.nu, self.nx), dtype=float)

    def _build_problem(self) -> None:
        N = self.N
        M = self.max_obstacles
        self.dx = cp.Variable((N + 1, self.nx), name="state_error")
        self.du = cp.Variable((N, self.nu), name="control_perturbation")
        self.slack = cp.Variable((M, N), nonneg=True, name="obstacle_slack")
        self.x0_error = cp.Parameter(self.nx, name="x0_error")
        self.u_reference = cp.Parameter((N, self.nu), name="u_reference")
        self.u_previous = cp.Parameter(self.nu, name="u_previous")
        self.rate_linear = cp.Parameter(self.nu, name="rate_linear")
        self.a_parameters = [cp.Parameter((self.nx, self.nx), name=f"A_{i}") for i in range(N)]
        self.b_parameters = [cp.Parameter((self.nx, self.nu), name=f"B_{i}") for i in range(N)]
        self.obstacle_normals = [cp.Parameter((M, 2), name=f"obstacle_normals_{i}") for i in range(N)]
        self.obstacle_rhs = [cp.Parameter(M, name=f"obstacle_rhs_{i}") for i in range(N)]

        constraints = [self.dx[0] == self.x0_error]
        objective = cp.quad_form(self.dx[N], self.P)
        for i in range(N):
            constraints.append(
                self.dx[i + 1] == self.a_parameters[i] @ self.dx[i] + self.b_parameters[i] @ self.du[i]
            )
            total_u = self.u_reference[i] + self.du[i]
            constraints.extend(
                [
                    total_u[0] >= self.config.v_min,
                    total_u[0] <= self.config.v_max,
                    total_u[1] >= self.config.omega_min,
                    total_u[1] <= self.config.omega_max,
                ]
            )
            if i == 0:
                constraints.extend(
                    [
                        total_u[0]
                        >= self.u_previous[0] - self.config.max_control_delta[0],
                        total_u[0]
                        <= self.u_previous[0] + self.config.max_control_delta[0],
                        total_u[1]
                        >= self.u_previous[1] - self.config.max_control_delta[1],
                        total_u[1]
                        <= self.u_previous[1] + self.config.max_control_delta[1],
                    ]
                )
            objective += cp.quad_form(self.dx[i], self.Q)
            objective += cp.quad_form(self.du[i], self.R)
            if i == 0:
                objective += cp.quad_form(self.du[i], self.R_filter)
                objective += cp.quad_form(self.du[i], self.R_rate)
                objective += self.rate_linear @ self.du[i]
            for obstacle_index in range(M):
                normal = self.obstacle_normals[i][obstacle_index]
                lhs = cp.sum(cp.multiply(normal, self.dx[i, :2]))
                constraints.append(lhs + self.slack[obstacle_index, i] >= self.obstacle_rhs[i][obstacle_index])
            objective += self.config.slack_penalty * cp.sum_squares(self.slack[:, i])

        self.problem = cp.Problem(cp.Minimize(objective), constraints)

    def nominal_lqr_control(self, state: np.ndarray, reference: np.ndarray, reference_control: np.ndarray) -> np.ndarray:
        error = np.asarray(state, dtype=float).reshape(5) - np.asarray(reference, dtype=float).reshape(5)
        error[2] = self.model.normalize_angle(error[2])
        command = np.asarray(reference_control, dtype=float).reshape(2) - self.K @ error
        return self._clip_control(command)

    def set_previous_command(self, command: np.ndarray) -> None:
        """Set the command used by the step-zero slew and rate penalties."""
        self._previous_command = self._clip_control(command)

    def limit_command(
        self, command: np.ndarray, previous_command: np.ndarray | None = None
    ) -> np.ndarray:
        """Apply the configured command slew limit outside an active QP solve."""
        previous = (
            self._previous_command
            if previous_command is None
            else self._clip_control(previous_command)
        )
        if previous is None:
            return self._clip_control(command)
        clipped = self._clip_control(command)
        delta = np.clip(
            clipped - previous,
            -np.asarray(self.config.max_control_delta),
            np.asarray(self.config.max_control_delta),
        )
        limited = self._clip_control(previous + delta)
        self._previous_command = limited.copy()
        return limited

    def solve(
        self,
        state: np.ndarray,
        references: np.ndarray,
        reference_controls: np.ndarray,
        obstacles: Sequence[Any] = (),
        disturbance: np.ndarray | None = None,
        nominal_states: np.ndarray | None = None,
        covariance_sequence: np.ndarray | None = None,
    ) -> SafetyFilterSolution:
        """Solve the parameterized QP and return the first safe command."""
        x0 = np.asarray(state, dtype=float).reshape(5)
        x_refs = self._pad_states(references)
        u_refs = self._pad_controls(reference_controls)
        d_hat = np.zeros(3, dtype=float) if disturbance is None else np.asarray(disturbance, dtype=float).reshape(3)
        if nominal_states is None:
            nominal = np.zeros((self.N + 1, self.nx), dtype=float)
            nominal[0] = x0
            for i in range(self.N):
                nominal[i + 1] = self.model.step(nominal[i], u_refs[i], d_hat)
        else:
            nominal = self._pad_states(nominal_states)

        a_sequence = []
        b_sequence = []
        for i in range(self.N):
            a_i, b_i = self.model.jacobians(nominal[i], u_refs[i], d_hat)
            a_sequence.append(a_i)
            b_sequence.append(b_i)

        if covariance_sequence is None:
            covariance = np.repeat(np.eye(self.nx)[None, :, :] * 1e-5, self.N + 1, axis=0)
        else:
            covariance = np.asarray(covariance_sequence, dtype=float)
            if covariance.shape != (self.N + 1, self.nx, self.nx):
                raise ValueError("covariance_sequence must have shape (N+1, 5, 5)")

        if self.config.backend == "rust":
            return self._solve_rust_backend(
                x0,
                x_refs,
                u_refs,
                nominal,
                a_sequence,
                b_sequence,
                covariance,
                obstacles,
            )

        self.x0_error.value = x0 - x_refs[0]
        self.u_reference.value = u_refs
        self.u_previous.value = (
            u_refs[0]
            if self._previous_command is None
            else self._previous_command
        )
        self.rate_linear.value = 2.0 * self.R_rate @ (
            u_refs[0] - self.u_previous.value
        )
        for i in range(self.N):
            self.a_parameters[i].value = a_sequence[i]
            self.b_parameters[i].value = b_sequence[i]
            normals, rhs = self._obstacle_planes(
                nominal[i], x_refs[i, :2], covariance[i], obstacles, i
            )
            self.obstacle_normals[i].value = normals
            self.obstacle_rhs[i].value = rhs

        if self._previous_dx is not None:
            self.dx.value = self._previous_dx
        if self._previous_du is not None:
            self.du.value = self._previous_du
        if self._previous_slack is not None:
            self.slack.value = self._previous_slack

        start = time.perf_counter()
        try:
            self.problem.solve(
                solver=cp.OSQP,
                warm_start=True,
                verbose=False,
                eps_abs=1e-5,
                eps_rel=1e-5,
                max_iter=10000,
                polishing=False,
            )
            if self.problem.status == cp.USER_LIMIT:
                self.problem.solve(
                    solver=cp.OSQP,
                    warm_start=True,
                    verbose=False,
                    eps_abs=2e-5,
                    eps_rel=2e-5,
                    max_iter=20000,
                    polishing=True,
                )
        except Exception:
            elapsed = (time.perf_counter() - start) * 1000.0
            return self._fallback_solution(nominal, u_refs, elapsed, "solver_error")
        elapsed = (time.perf_counter() - start) * 1000.0

        if self.problem.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
            return self._fallback_solution(nominal, u_refs, elapsed, str(self.problem.status))

        dx_value = np.asarray(self.dx.value, dtype=float)
        du_value = np.asarray(self.du.value, dtype=float)
        slack_value = np.asarray(self.slack.value, dtype=float)
        self._previous_dx = dx_value.copy()
        self._previous_du = du_value.copy()
        self._previous_slack = slack_value.copy()
        predicted_states = x_refs + dx_value
        controls = self._clip_sequence(u_refs + du_value)
        self._previous_command = controls[0].copy()
        intervention = float(np.linalg.norm(controls[0] - u_refs[0]))
        return SafetyFilterSolution(
            status="optimal",
            optimal_control=controls[0].copy(),
            control_sequence=controls,
            predicted_states=predicted_states,
            solve_time_ms=elapsed,
            intervention_norm=intervention,
            slack_max=float(np.max(slack_value)) if slack_value.size else 0.0,
            feasible=True,
        )

    def reset(self) -> None:
        self._previous_dx = None
        self._previous_du = None
        self._previous_slack = None
        self._previous_command = None
        if self._rust_workspace is not None:
            self._rust_workspace.reset()
        else:
            self.dx.value = None
            self.du.value = None
            self.slack.value = None

    def _solve_rust_backend(
        self,
        state: np.ndarray,
        references: np.ndarray,
        reference_controls: np.ndarray,
        nominal_states: np.ndarray,
        a_sequence: Sequence[np.ndarray],
        b_sequence: Sequence[np.ndarray],
        covariance: np.ndarray,
        obstacles: Sequence[Any],
    ) -> SafetyFilterSolution:
        if self._rust_workspace is None:
            raise RuntimeError("Rust safety-filter workspace was not initialized")

        normals_sequence = []
        rhs_sequence = []
        for i in range(self.N):
            normals, rhs = self._obstacle_planes(
                nominal_states[i], references[i, :2], covariance[i], obstacles, i
            )
            normals_sequence.append(normals)
            rhs_sequence.append(rhs)

        start = time.perf_counter()
        try:
            values, status, solve_time_ms = self._rust_workspace.solve(
                state - references[0],
                reference_controls[0]
                if self._previous_command is None
                else self._previous_command,
                reference_controls,
                a_sequence,
                b_sequence,
                normals_sequence,
                rhs_sequence,
            )
        except Exception:
            elapsed = (time.perf_counter() - start) * 1000.0
            return self._fallback_solution(
                nominal_states,
                reference_controls,
                elapsed,
                "rust_solver_error",
            )

        expected = (self.N + 1) * self.nx + self.N * self.nu + self.N * self.max_obstacles
        if len(values) != expected:
            elapsed = (time.perf_counter() - start) * 1000.0
            return self._fallback_solution(
                nominal_states,
                reference_controls,
                elapsed,
                "rust_invalid_solution",
            )

        dx_count = (self.N + 1) * self.nx
        du_count = self.N * self.nu
        dx_value = np.asarray(values[:dx_count], dtype=float).reshape(self.N + 1, self.nx)
        du_value = np.asarray(values[dx_count : dx_count + du_count], dtype=float).reshape(
            self.N, self.nu
        )
        slack_value = np.asarray(values[dx_count + du_count :], dtype=float).reshape(
            self.N, self.max_obstacles
        )
        if not (
            np.all(np.isfinite(dx_value))
            and np.all(np.isfinite(du_value))
            and np.all(np.isfinite(slack_value))
        ):
            return self._fallback_solution(
                nominal_states,
                reference_controls,
                float(solve_time_ms),
                "rust_nonfinite_solution",
            )

        feasible = status in {"solved", "solved_inaccurate"}
        if not feasible:
            return self._fallback_solution(
                nominal_states,
                reference_controls,
                float(solve_time_ms),
                f"rust_{status}",
            )

        controls = self._clip_sequence(reference_controls + du_value)
        self._previous_command = controls[0].copy()
        return SafetyFilterSolution(
            status=f"rust_{status}",
            optimal_control=controls[0].copy(),
            control_sequence=controls,
            predicted_states=references + dx_value,
            solve_time_ms=float(solve_time_ms),
            intervention_norm=float(np.linalg.norm(controls[0] - reference_controls[0])),
            slack_max=float(np.max(slack_value)) if slack_value.size else 0.0,
            feasible=True,
        )

    def _obstacle_planes(
        self,
        nominal_state: np.ndarray,
        reference_position: np.ndarray,
        covariance: np.ndarray,
        obstacles: Sequence[Any],
        step: int,
    ) -> tuple[np.ndarray, np.ndarray]:
        normals = np.tile(np.array([[1.0, 0.0]]), (self.max_obstacles, 1))
        rhs_values = np.full(self.max_obstacles, -1e3, dtype=float)
        for index, raw_obstacle in enumerate(obstacles[: self.max_obstacles]):
            obstacle = self._obstacle(raw_obstacle)
            position = obstacle.position_at(step * self.config.dt)
            offset = nominal_state[:2] - position
            distance = float(np.linalg.norm(offset))
            normal = np.array([1.0, 0.0], dtype=float) if distance < 1e-10 else offset / distance
            sigma = float(np.sqrt(max(0.0, normal @ covariance[:2, :2] @ normal)))
            normals[index] = normal
            safe_distance = (
                obstacle.radius
                + self.config.robot_radius
                + self.config.d_safe
                + self.risk_gate.c_alpha * sigma
            )
            rhs_values[index] = safe_distance - float(normal @ (reference_position - position))
        return normals, rhs_values

    def _fallback_solution(
        self,
        nominal_states: np.ndarray,
        reference_controls: np.ndarray,
        elapsed: float,
        status: str,
    ) -> SafetyFilterSolution:
        controls = np.zeros_like(reference_controls)
        return SafetyFilterSolution(
            status=status,
            optimal_control=np.zeros(2, dtype=float),
            control_sequence=controls,
            predicted_states=nominal_states,
            solve_time_ms=elapsed,
            intervention_norm=float(np.linalg.norm(reference_controls[0])),
            slack_max=float("inf"),
            feasible=False,
        )

    def _pad_states(self, values: np.ndarray) -> np.ndarray:
        array = np.asarray(values, dtype=float)
        if array.ndim == 1:
            array = array.reshape(1, -1)
        if array.shape[1] != self.nx:
            raise ValueError("state references must have five columns")
        if len(array) < self.N + 1:
            array = np.vstack([array, np.repeat(array[-1:], self.N + 1 - len(array), axis=0)])
        return array[: self.N + 1].copy()

    def _pad_controls(self, values: np.ndarray) -> np.ndarray:
        array = np.asarray(values, dtype=float)
        if array.ndim == 1:
            array = array.reshape(1, -1)
        if array.shape[1] != self.nu:
            raise ValueError("control references must have two columns")
        if len(array) < self.N:
            array = np.vstack([array, np.repeat(array[-1:], self.N - len(array), axis=0)])
        return self._clip_sequence(array[: self.N].copy())

    def _clip_control(self, value: np.ndarray) -> np.ndarray:
        result = np.asarray(value, dtype=float).reshape(2).copy()
        result[0] = np.clip(result[0], self.config.v_min, self.config.v_max)
        result[1] = np.clip(result[1], self.config.omega_min, self.config.omega_max)
        return result

    def _clip_sequence(self, values: np.ndarray) -> np.ndarray:
        return np.vstack([self._clip_control(row) for row in np.asarray(values)])

    @staticmethod
    def _obstacle(value: Any) -> ObstacleKinematics:
        if isinstance(value, ObstacleKinematics):
            return value
        if isinstance(value, dict):
            return ObstacleKinematics(
                x=float(value.get("x", 0.0)),
                y=float(value.get("y", 0.0)),
                radius=float(value.get("radius", 0.0)),
                vx=float(value.get("vx", 0.0)),
                vy=float(value.get("vy", 0.0)),
            )
        return ObstacleKinematics(
            x=float(getattr(value, "x", 0.0)),
            y=float(getattr(value, "y", 0.0)),
            radius=float(getattr(value, "radius", 0.0)),
            vx=float(getattr(value, "vx", 0.0)),
            vy=float(getattr(value, "vy", 0.0)),
        )
