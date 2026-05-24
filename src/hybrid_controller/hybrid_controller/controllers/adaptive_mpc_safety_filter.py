"""
Risk-triggered adaptive MPC safety filter with LQR nominal tracking.

This is the Option A implementation path:

    LQR nominal command
        -> rollout risk prediction
        -> adaptive MPC safety filter when unsafe
        -> brake/turn backup if MPC is unavailable

The existing blended hybrid controller is left untouched for baseline
comparison.
"""

from dataclasses import dataclass
from typing import Any, Optional, Sequence, Tuple

import numpy as np

from .adaptive_mpc_controller import AdaptiveMPCController
from .backup_safety import BackupCommand, BackupSafetyController
from .lqr_controller import LQRController
from .mpc_controller import MPCSolution
from .risk_predictor import LQRRiskPredictor, RolloutRisk


@dataclass
class SafetyFilterResult:
    """Controller output and diagnostic metadata for one timestep."""

    mode: str
    optimal_control: np.ndarray
    lqr_control: np.ndarray
    risk: RolloutRisk
    theta_hat: np.ndarray
    solver_status: str
    solver_time_ms: float
    predicted_states: np.ndarray
    mpc_solution: Optional[MPCSolution] = None
    backup: Optional[BackupCommand] = None

    @property
    def used_mpc(self) -> bool:
        return self.mode == "ADAPTIVE_MPC_FILTER"

    @property
    def used_backup(self) -> bool:
        return self.mode == "BACKUP"


class AdaptiveMPCSafetyFilter:
    """
    Least-invasive adaptive MPC safety filter around LQR.

    The adaptive MPC receives an LQR-generated nominal control sequence as its
    input reference. Therefore, when safety constraints are active, the solver
    modifies LQR as little as possible instead of replacing it with an unrelated
    MPC tracking command.
    """

    def __init__(
        self,
        prediction_horizon: int = 10,
        terminal_horizon: int = 5,
        Q_diag: Optional[list] = None,
        R_diag: Optional[list] = None,
        d_safe: float = 0.3,
        dt: float = 0.05,
        v_max: float = 0.22,
        omega_max: float = 2.84,
        adaptation_gamma: float = 0.005,
        mpc_time_limit_ms: float = 250.0,
        robot_radius: float = 0.105,
    ):
        self.dt = float(dt)
        self.v_max = float(v_max)
        self.omega_max = float(omega_max)
        self.mpc_time_limit_ms = float(mpc_time_limit_ms)

        if Q_diag is None:
            Q_diag = [30.0, 30.0, 5.0]
        if R_diag is None:
            R_diag = [0.1, 0.1]

        self.lqr = LQRController(
            Q_diag=Q_diag,
            R_diag=R_diag,
            dt=dt,
            v_max=v_max,
            omega_max=omega_max,
        )
        self.adaptive_mpc = AdaptiveMPCController(
            prediction_horizon=prediction_horizon,
            terminal_horizon=terminal_horizon,
            Q_diag=Q_diag,
            R_diag=R_diag,
            d_safe=d_safe,
            v_max=v_max,
            omega_max=omega_max,
            dt=dt,
            enable_adaptation=True,
            adaptation_gamma=adaptation_gamma,
        )
        self.predictor = LQRRiskPredictor(
            horizon=prediction_horizon,
            dt=dt,
            d_safe=d_safe,
            robot_radius=robot_radius,
        )
        self.backup_controller = BackupSafetyController(
            v_max=v_max,
            omega_max=omega_max,
        )

    @property
    def required_state_horizon(self) -> int:
        return self.adaptive_mpc.N_ext + 1

    @property
    def required_control_horizon(self) -> int:
        return self.adaptive_mpc.N_ext

    @property
    def param_estimates(self) -> np.ndarray:
        return self.adaptive_mpc.param_estimates

    def adapt_parameters(
        self,
        x_measured: np.ndarray,
        x_prev: np.ndarray,
        u_prev: np.ndarray,
    ) -> None:
        self.adaptive_mpc.adapt_parameters(x_measured, x_prev, u_prev)

    def compute_control(
        self,
        x0: np.ndarray,
        x_refs: np.ndarray,
        u_refs: np.ndarray,
        obstacles: Sequence[Any] = (),
    ) -> SafetyFilterResult:
        """Compute one control command using LQR, adaptive MPC, or backup."""
        x0 = np.asarray(x0, dtype=float).reshape(3)
        x_refs, u_refs = self._pad_reference(x_refs, u_refs)

        lqr_sequence = self._build_lqr_nominal_sequence(x0, x_refs, u_refs)
        lqr_control = lqr_sequence[0].copy()
        theta_hat = self.param_estimates

        risk = self.predictor.assess(
            x0=x0,
            controls=lqr_sequence[: self.predictor.horizon],
            theta_hat=theta_hat,
            obstacles=obstacles,
        )

        if risk.safe:
            return SafetyFilterResult(
                mode="LQR",
                optimal_control=self._clip_control(lqr_control),
                lqr_control=lqr_control,
                risk=risk,
                theta_hat=theta_hat,
                solver_status="not_triggered",
                solver_time_ms=0.0,
                predicted_states=risk.predicted_states,
            )

        mpc_solution = self.adaptive_mpc.solve_tracking(
            x0=x0,
            x_refs=x_refs,
            u_refs=lqr_sequence,
            obstacles=list(obstacles),
        )

        solver_time = float(getattr(mpc_solution, "solve_time_ms", 0.0))
        solver_status = str(getattr(mpc_solution, "status", "unknown"))
        usable_mpc = (
            solver_status == "optimal"
            and solver_time <= self.mpc_time_limit_ms
            and mpc_solution.optimal_control is not None
        )

        if usable_mpc:
            return SafetyFilterResult(
                mode="ADAPTIVE_MPC_FILTER",
                optimal_control=self._clip_control(mpc_solution.optimal_control),
                lqr_control=lqr_control,
                risk=risk,
                theta_hat=theta_hat,
                solver_status=solver_status,
                solver_time_ms=solver_time,
                predicted_states=mpc_solution.predicted_states,
                mpc_solution=mpc_solution,
            )

        reason = (
            "mpc_slow"
            if solver_status == "optimal"
            else f"mpc_{solver_status}"
        )
        backup = self.backup_controller.compute(x0, obstacles, reason=reason)
        return SafetyFilterResult(
            mode="BACKUP",
            optimal_control=self._clip_control(backup.control),
            lqr_control=lqr_control,
            risk=risk,
            theta_hat=theta_hat,
            solver_status=solver_status,
            solver_time_ms=solver_time,
            predicted_states=risk.predicted_states,
            mpc_solution=mpc_solution,
            backup=backup,
        )

    def _build_lqr_nominal_sequence(
        self,
        x0: np.ndarray,
        x_refs: np.ndarray,
        u_refs: np.ndarray,
    ) -> np.ndarray:
        controls = np.zeros((self.required_control_horizon, 2), dtype=float)
        x_rollout = np.asarray(x0, dtype=float).reshape(3).copy()
        theta_hat = self.param_estimates

        for k in range(self.required_control_horizon):
            u_lqr, _ = self.lqr.compute_control_at_operating_point(
                x_rollout,
                x_refs[min(k, len(x_refs) - 1)],
                u_refs[min(k, len(u_refs) - 1)],
            )
            controls[k] = self._clip_control(u_lqr)
            x_rollout = self.predictor.propagate_state(
                x_rollout,
                controls[k],
                theta_hat,
                self.dt,
            )

        return controls

    def _pad_reference(
        self,
        x_refs: np.ndarray,
        u_refs: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        x_refs = np.asarray(x_refs, dtype=float)
        u_refs = np.asarray(u_refs, dtype=float)
        if x_refs.ndim == 1:
            x_refs = x_refs.reshape(1, 3)
        if u_refs.ndim == 1:
            u_refs = u_refs.reshape(1, 2)

        if len(x_refs) < self.required_state_horizon:
            pad = np.repeat(x_refs[-1:], self.required_state_horizon - len(x_refs), axis=0)
            x_refs = np.vstack([x_refs, pad])
        if len(u_refs) < self.required_control_horizon:
            pad = np.repeat(u_refs[-1:], self.required_control_horizon - len(u_refs), axis=0)
            u_refs = np.vstack([u_refs, pad])

        return x_refs[: self.required_state_horizon], u_refs[: self.required_control_horizon]

    def _clip_control(self, u: np.ndarray) -> np.ndarray:
        u = np.asarray(u, dtype=float).reshape(2)
        return np.array(
            [
                np.clip(u[0], -self.v_max, self.v_max),
                np.clip(u[1], -self.omega_max, self.omega_max),
            ],
            dtype=float,
        )
