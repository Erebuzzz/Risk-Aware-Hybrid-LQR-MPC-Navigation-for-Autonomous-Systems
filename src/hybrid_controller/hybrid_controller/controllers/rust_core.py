"""Explicit access to the compiled Rust numerical core."""

from __future__ import annotations

try:
    import ra_psf_rs as _rust_core
except ImportError as exc:  # pragma: no cover
    _rust_core = None
    _import_error = exc
else:
    _import_error = None


def rust_core_available() -> bool:
    """Return whether the compiled extension is importable."""
    return _rust_core is not None


def require_rust_core():
    """Return the extension or raise an actionable build error."""
    if _rust_core is None:
        raise RuntimeError(
            "ra_psf_rs is unavailable. Build it with maturin develop --release "
            "from crates/ra_psf_core."
        ) from _import_error
    return _rust_core


class RustHorizonQpWorkspace:
    """Python adapter for the stateful Rust horizon QP workspace."""

    def __init__(
        self,
        horizon: int,
        max_obstacles: int,
        q_diag,
        r_diag,
        terminal_cost,
        r_filter_diag,
        rate_penalty_diag,
        max_control_delta,
        slack_penalty: float,
        v_min: float,
        v_max: float,
        omega_min: float,
        omega_max: float,
    ) -> None:
        self.horizon = int(horizon)
        self.max_obstacles = int(max_obstacles)
        core = require_rust_core()
        self._workspace = core.HorizonQpWorkspace(
            self.horizon,
            self.max_obstacles,
            list(q_diag),
            list(r_diag),
            [list(row) for row in terminal_cost],
            list(r_filter_diag),
            list(rate_penalty_diag),
            list(max_control_delta),
            float(slack_penalty),
            float(v_min),
            float(v_max),
            float(omega_min),
            float(omega_max),
        )

    def solve(
        self,
        x0_error,
        previous_control,
        u_reference,
        a_sequence,
        b_sequence,
        obstacle_normals,
        obstacle_rhs,
    ):
        return self._workspace.solve(
            list(x0_error),
            list(previous_control),
            [[float(value) for value in row] for row in u_reference],
            [[[float(value) for value in row] for row in matrix] for matrix in a_sequence],
            [[[float(value) for value in row] for row in matrix] for matrix in b_sequence],
            [
                [[float(value) for value in row] for row in matrix]
                for matrix in obstacle_normals
            ],
            [[float(value) for value in row] for row in obstacle_rhs],
        )

    def reset(self) -> None:
        self._workspace.reset()
