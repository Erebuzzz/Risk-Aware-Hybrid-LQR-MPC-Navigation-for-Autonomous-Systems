"""Controllers for LQR, MPC, legacy blending, and adaptive safety filtering.

The blended hybrid controller is kept for baseline comparison. The current
research direction is the risk-triggered adaptive MPC safety filter.
"""

from .lqr_controller import LQRController
from .mpc_controller import MPCController
from .hybrid_blender import BlendingSupervisor
from .risk_predictor import LQRRiskPredictor
from .backup_safety import BackupSafetyController

__all__ = [
    'LQRController',
    'MPCController',
    'BlendingSupervisor',
    'LQRRiskPredictor',
    'BackupSafetyController',
]
