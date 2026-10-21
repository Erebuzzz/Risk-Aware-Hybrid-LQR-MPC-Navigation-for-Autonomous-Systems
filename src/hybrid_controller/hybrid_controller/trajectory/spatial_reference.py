"""Spatial arc-length references and progress projection."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np
from scipy.interpolate import CubicSpline


@dataclass(frozen=True)
class SpatialReferencePoint:
    """Reference data at one arc-length coordinate."""

    s: float
    position: np.ndarray
    tangent: np.ndarray
    heading: float


class SpatialFigure8Reference:
    """Closed Figure 8 reference parameterized by arc length."""

    def __init__(self, amplitude: float = 2.0, samples: int = 20000) -> None:
        if amplitude <= 0.0:
            raise ValueError("amplitude must be positive")
        if samples < 100:
            raise ValueError("samples must be at least 100")
        self.amplitude = float(amplitude)
        self.samples = int(samples)
        self._q_grid = np.linspace(0.0, 2.0 * np.pi, self.samples + 1)
        px, py = self._position_from_q(self._q_grid)
        ds = np.hypot(np.diff(px), np.diff(py))
        self._arc_grid = np.concatenate(([0.0], np.cumsum(ds)))
        self.total_length = float(self._arc_grid[-1])

    def position(self, s: float) -> np.ndarray:
        q = self._q_from_s(s)
        px, py = self._position_from_q(q)
        return np.array([float(px), float(py)], dtype=float)

    def tangent(self, s: float) -> np.ndarray:
        q = self._q_from_s(s)
        dx_dq = self.amplitude * np.cos(q)
        dy_dq = self.amplitude * np.cos(2.0 * q)
        tangent = np.array([dx_dq, dy_dq], dtype=float)
        norm = np.linalg.norm(tangent)
        if norm < 1e-12:
            return np.array([1.0, 0.0], dtype=float)
        return tangent / norm

    def evaluate(self, s: float) -> SpatialReferencePoint:
        tangent = self.tangent(s)
        return SpatialReferencePoint(
            s=self._normalize_s(s),
            position=self.position(s),
            tangent=tangent,
            heading=float(np.arctan2(tangent[1], tangent[0])),
        )

    def project(
        self,
        position: np.ndarray,
        s_previous: float,
        forward_window: float = 0.75,
        backward_window: float = 0.20,
        samples: int = 121,
    ) -> float:
        """Project a position using a local forward-biased arc window."""
        p = np.asarray(position, dtype=float).reshape(-1)
        if p.size != 2 or not np.all(np.isfinite(p)):
            raise ValueError("position must be a finite vector of length 2")
        if forward_window < 0.0 or backward_window < 0.0:
            raise ValueError("projection windows must be nonnegative")
        offsets = np.unique(
            np.concatenate(
                [
                    np.linspace(-backward_window, forward_window, max(3, samples)),
                    np.array([0.0]),
                ]
            )
        )
        candidates = np.mod(self._normalize_s(s_previous) + offsets, self.total_length)
        points = np.vstack([self.position(s) for s in candidates])
        distances = np.linalg.norm(points - p[None, :], axis=1)
        minimum = float(np.min(distances))
        close = np.flatnonzero(distances <= minimum + 1e-9)
        if close.size == 0:
            return self._normalize_s(s_previous)
        selected = close[np.argmin(np.abs(offsets[close]))]
        return float(candidates[selected])

    def cross_track_error(self, position: np.ndarray, s: float) -> float:
        p = np.asarray(position, dtype=float).reshape(2)
        return float(np.linalg.norm(p - self.position(s)))

    def update_progress(
        self,
        position: np.ndarray,
        s_previous: float,
        forward_window: float = 0.75,
        backward_window: float = 0.20,
    ) -> Tuple[float, float]:
        s = self.project(position, s_previous, forward_window, backward_window)
        return s, self.cross_track_error(position, s)

    def _q_from_s(self, s: float) -> float:
        s_normalized = self._normalize_s(s)
        return float(np.interp(s_normalized, self._arc_grid, self._q_grid))

    def _normalize_s(self, s: float) -> float:
        return float(s % self.total_length)

    def _position_from_q(self, q: np.ndarray | float) -> Tuple[np.ndarray, np.ndarray]:
        q = np.asarray(q, dtype=float)
        return (
            self.amplitude * np.sin(q),
            self.amplitude * np.sin(q) * np.cos(q),
        )


class CubicSpline2D:
    """Arc-length parameterized cubic spline for arbitrary planar paths."""

    def __init__(self, waypoints: np.ndarray, closed: bool = False, samples: int = 20000) -> None:
        points = np.asarray(waypoints, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3:
            raise ValueError("waypoints must have shape (N, 2) with N >= 3")
        self.closed = bool(closed)
        self.samples = int(samples)
        if self.samples < 100:
            raise ValueError("samples must be at least 100")

        if self.closed and not np.allclose(points[0], points[-1]):
            points = np.vstack([points, points[0]])
        chord = np.linalg.norm(np.diff(points, axis=0), axis=1)
        if np.any(chord <= 1e-10):
            raise ValueError("waypoints must not contain duplicate neighbors")
        parameter = np.concatenate(([0.0], np.cumsum(chord)))
        self._parameter_end = float(parameter[-1])
        bc_type = "periodic" if self.closed else "natural"
        self._spline_x = CubicSpline(parameter, points[:, 0], bc_type=bc_type)
        self._spline_y = CubicSpline(parameter, points[:, 1], bc_type=bc_type)
        self._parameter_grid = np.linspace(0.0, self._parameter_end, self.samples + 1)
        sample_x = self._spline_x(self._parameter_grid)
        sample_y = self._spline_y(self._parameter_grid)
        sample_ds = np.hypot(np.diff(sample_x), np.diff(sample_y))
        self._arc_grid = np.concatenate(([0.0], np.cumsum(sample_ds)))
        self.total_length = float(self._arc_grid[-1])

    def evaluate(self, s: float) -> SpatialReferencePoint:
        s_norm = s % self.total_length if self.closed else float(np.clip(s, 0.0, self.total_length))
        parameter = float(np.interp(s_norm, self._arc_grid, self._parameter_grid))
        position = np.array([self._spline_x(parameter), self._spline_y(parameter)], dtype=float)
        tangent = np.array([self._spline_x(parameter, 1), self._spline_y(parameter, 1)], dtype=float)
        norm = np.linalg.norm(tangent)
        if norm < 1e-12:
            tangent = np.array([1.0, 0.0], dtype=float)
        else:
            tangent /= norm
        return SpatialReferencePoint(
            s=s_norm,
            position=position,
            tangent=tangent,
            heading=float(np.arctan2(tangent[1], tangent[0])),
        )

    def project(
        self,
        position: np.ndarray,
        s_previous: float,
        forward_window: float = 1.0,
        backward_window: float = 0.25,
        samples: int = 121,
    ) -> float:
        p = np.asarray(position, dtype=float).reshape(2)
        offsets = np.unique(
            np.concatenate(
                [
                    np.linspace(-backward_window, forward_window, max(3, samples)),
                    np.array([0.0]),
                ]
            )
        )
        if self.closed:
            candidates = np.mod(s_previous + offsets, self.total_length)
        else:
            candidates = np.clip(s_previous + offsets, 0.0, self.total_length)
        points = np.vstack([self.evaluate(s).position for s in candidates])
        distances = np.linalg.norm(points - p[None, :], axis=1)
        minimum = np.min(distances)
        close = np.flatnonzero(distances <= minimum + 1e-9)
        selected = close[np.argmin(np.abs(offsets[close]))]
        return float(candidates[selected])
