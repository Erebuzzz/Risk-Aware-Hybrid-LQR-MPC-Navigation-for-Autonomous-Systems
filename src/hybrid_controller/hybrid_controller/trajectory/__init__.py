"""Trajectory generation and spatial reference utilities."""

from .reference_generator import ReferenceTrajectoryGenerator
from .spatial_reference import CubicSpline2D, SpatialFigure8Reference

__all__ = [
    'ReferenceTrajectoryGenerator',
    'CubicSpline2D',
    'SpatialFigure8Reference',
]
