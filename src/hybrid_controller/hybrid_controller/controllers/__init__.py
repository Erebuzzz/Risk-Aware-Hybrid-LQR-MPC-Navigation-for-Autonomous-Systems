"""Controllers for LQR, MPC, legacy blending, and adaptive safety filtering.

The blended hybrid controller is kept for baseline comparison. The current
research direction is the risk-triggered adaptive MPC safety filter.
"""

from .lqr_controller import LQRController
from .mpc_controller import MPCController
from .hybrid_blender import BlendingSupervisor
from .risk_predictor import LQRRiskPredictor
from .backup_safety import BackupSafetyController
from .eso import ESOConfig, ExtendedStateObserver3D
from .cvar_gate import CVaRRiskGate, RiskHysteresis
from .riccati_safety_filter import RiccatiSafetyFilter, SafetyFilterConfig
from .rust_core import RustHorizonQpWorkspace, require_rust_core, rust_core_available

__all__ = [
    'LQRController',
    'MPCController',
    'BlendingSupervisor',
    'LQRRiskPredictor',
    'BackupSafetyController',
    'ESOConfig',
    'ExtendedStateObserver3D',
    'CVaRRiskGate',
    'RiskHysteresis',
    'RiccatiSafetyFilter',
    'SafetyFilterConfig',
    'require_rust_core',
    'rust_core_available',
    'RustHorizonQpWorkspace',
]
