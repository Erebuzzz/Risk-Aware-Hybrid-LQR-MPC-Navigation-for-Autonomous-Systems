"""
Simulation Logger
=================

Comprehensive logging system for tracking simulation processes including:
- State changes (position, orientation, errors)
- Parameter updates (LQR/MPC parameters)
- Control actions (inputs, controller type, solve time)
- Error states with process identification
- Constraint events (obstacle proximity, violations)

Supports export to CSV and JSON formats for post-analysis.
"""

import logging
import json
import csv
import os
from datetime import datetime
from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field, asdict
from enum import Enum
import numpy as np


class LogEventType(Enum):
    """Types of loggable events."""
    STATE_UPDATE = "state_update"
    CONTROL_ACTION = "control_action"
    PARAMETER_CHANGE = "parameter_change"
    ERROR = "error"
    CONSTRAINT_EVENT = "constraint_event"
    SIMULATION_EVENT = "simulation_event"


@dataclass
class LogEntry:
    """Structured log entry with metadata."""
    timestamp: str
    level: str
    process: str
    event_type: str
    data: Dict[str, Any]
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class SimulationLogger:
    """
    Comprehensive logging for simulation processes.
    
    Provides structured logging with multiple output formats:
    - Console output with color coding
    - Log files with timestamps
    - JSON export for programmatic analysis
    - CSV export for state/control history
    
    Example:
        logger = SimulationLogger(log_dir="logs", log_level="INFO")
        logger.log_state(timestep=0, state=np.array([0, 0, 0]), 
                        state_ref=np.array([0, 0, 1.11]),
                        error=np.array([0, 0, -1.11]))
        logger.log_control(timestep=0, control=np.array([0.5, 0.35]),
                          controller_type="LQR")
    """
    
    def __init__(self, log_dir: str = "logs", log_level: str = "INFO", 
                 node_name: str = "simulation"):
        """
        Initialize logger with file and console handlers.
        
        Args:
            log_dir: Directory to store log files
            log_level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
            node_name: Name identifier for this logger instance
        """
        self.log_dir = log_dir
        self.node_name = node_name
        self.log_entries: List[LogEntry] = []
        self.state_history: List[Dict[str, Any]] = []
        self.control_history: List[Dict[str, Any]] = []
        
        # Create log directory if it doesn't exist
        os.makedirs(log_dir, exist_ok=True)
        
        # Generate timestamp for log files
        self.session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Configure Python logger
        self.logger = logging.getLogger(node_name)
        self.logger.setLevel(getattr(logging, log_level.upper()))
        
        # Clear existing handlers
        self.logger.handlers.clear()
        
        # Console handler with formatting
        console_handler = logging.StreamHandler()
        console_handler.setLevel(getattr(logging, log_level.upper()))
        console_format = logging.Formatter(
            '[%(asctime)s.%(msecs)03d] [%(levelname)s] [%(name)s] %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        console_handler.setFormatter(console_format)
        self.logger.addHandler(console_handler)
        
        # File handler
        log_file = os.path.join(log_dir, f"simulation_{self.session_timestamp}.log")
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.DEBUG)  # Always log everything to file
        file_handler.setFormatter(console_format)
        self.logger.addHandler(file_handler)
        
        self.logger.info(f"SimulationLogger initialized. Log file: {log_file}")
    
    def _create_entry(self, level: str, process: str, 
                      event_type: LogEventType, data: Dict[str, Any]) -> LogEntry:
        """Create a structured log entry."""
        entry = LogEntry(
            timestamp=datetime.now().isoformat(),
            level=level,
            process=process,
            event_type=event_type.value,
            data=data
        )
        self.log_entries.append(entry)
        return entry
    
    def _array_to_list(self, arr: np.ndarray) -> List[float]:
        """Convert numpy array to list for JSON serialization."""
        if isinstance(arr, np.ndarray):
            return arr.tolist()
        return arr
    
    def log_state(self, timestep: int, state: np.ndarray, 
                  state_ref: np.ndarray, error: np.ndarray,
                  error_norm: Optional[float] = None) -> None:
        """
        Log robot state, reference, and tracking error.
        
        Args:
            timestep: Current simulation timestep
            state: Robot state [px, py, theta]
            state_ref: Reference state [px_ref, py_ref, theta_ref]
            error: Tracking error state - state_ref
            error_norm: Optional precomputed error norm
        """
        if error_norm is None:
            error_norm = float(np.linalg.norm(error))
        
        data = {
            "timestep": timestep,
            "state": {
                "px": float(state[0]),
                "py": float(state[1]),
                "theta": float(state[2])
            },
            "reference": {
                "px": float(state_ref[0]),
                "py": float(state_ref[1]),
                "theta": float(state_ref[2])
            },
            "error": {
                "px": float(error[0]),
                "py": float(error[1]),
                "theta": float(error[2])
            },
            "error_norm": error_norm
        }
        
        self._create_entry("DEBUG", "state", LogEventType.STATE_UPDATE, data)
        
        # Store in state history for CSV export
        self.state_history.append({
            "timestep": timestep,
            "px": float(state[0]),
            "py": float(state[1]),
            "theta": float(state[2]),
            "px_ref": float(state_ref[0]),
            "py_ref": float(state_ref[1]),
            "theta_ref": float(state_ref[2]),
            "error_px": float(error[0]),
            "error_py": float(error[1]),
            "error_theta": float(error[2]),
            "error_norm": error_norm
        })
        
        self.logger.debug(
            f"k={timestep} | x=[{state[0]:.2f}, {state[1]:.2f}, {state[2]:.2f}] | "
            f"x_ref=[{state_ref[0]:.2f}, {state_ref[1]:.2f}, {state_ref[2]:.2f}] | "
            f"err_norm={error_norm:.4f}"
        )
    
    def log_control(self, timestep: int, control: np.ndarray,
                    controller_type: str, solve_time: Optional[float] = None,
                    iterations: Optional[int] = None) -> None:
        """
        Log control action with controller metadata.
        
        Args:
            timestep: Current simulation timestep
            control: Control input [v, omega]
            controller_type: "LQR" or "MPC"
            solve_time: Optional solver time in milliseconds
            iterations: Optional number of solver iterations
        """
        data = {
            "timestep": timestep,
            "control": {
                "v": float(control[0]),
                "omega": float(control[1])
            },
            "controller": controller_type,
            "solve_time_ms": solve_time,
            "iterations": iterations
        }
        
        self._create_entry("DEBUG", "control", LogEventType.CONTROL_ACTION, data)
        
        # Store in control history
        self.control_history.append({
            "timestep": timestep,
            "v": float(control[0]),
            "omega": float(control[1]),
            "controller": controller_type,
            "solve_time_ms": solve_time
        })
        
        msg = f"k={timestep} | u=[{control[0]:.3f}, {control[1]:.3f}] | controller={controller_type}"
        if solve_time is not None:
            msg += f" | solve_time={solve_time:.2f}ms"
        if iterations is not None:
            msg += f" | iterations={iterations}"
        
        self.logger.debug(msg)
    
    def log_parameter_change(self, param_name: str, 
                             old_value: Any, new_value: Any,
                             source: str = "runtime") -> None:
        """
        Log parameter modifications.
        
        Args:
            param_name: Name of the parameter being changed
            old_value: Previous parameter value
            new_value: New parameter value
            source: Source of the change (e.g., "runtime", "config", "user")
        """
        data = {
            "parameter": param_name,
            "old_value": self._array_to_list(old_value) if isinstance(old_value, np.ndarray) else old_value,
            "new_value": self._array_to_list(new_value) if isinstance(new_value, np.ndarray) else new_value,
            "source": source
        }
        
        self._create_entry("INFO", "parameter", LogEventType.PARAMETER_CHANGE, data)
        
        self.logger.info(
            f"Parameter '{param_name}' changed: {old_value} -> {new_value} (source: {source})"
        )
    
    def log_error(self, process_name: str, error_type: str,
                  message: str, exception: Optional[Exception] = None,
                  recovery_action: Optional[str] = None) -> None:
        """
        Log errors with process identification.
        
        Args:
            process_name: Name of the process/module that encountered the error
            error_type: Category of error (e.g., "SolverError", "ConstraintViolation")
            message: Detailed error message
            exception: Optional Python exception object
            recovery_action: Optional description of recovery action taken
        """
        data = {
            "process": process_name,
            "error_type": error_type,
            "message": message,
            "exception": str(exception) if exception else None,
            "traceback": None,  # Could add traceback.format_exc() if needed
            "recovery_action": recovery_action
        }
        
        self._create_entry("ERROR", process_name, LogEventType.ERROR, data)
        
        log_msg = f"Process: {process_name} | Error: {error_type} | {message}"
        if recovery_action:
            log_msg += f" | Recovery: {recovery_action}"
        
        self.logger.error(log_msg)
    
    def log_constraint_event(self, timestep: int, 
                             constraint_type: str, details: Dict[str, Any],
                             is_violation: bool = False) -> None:
        """
        Log constraint violations or activations.
        
        Args:
            timestep: Current simulation timestep
            constraint_type: Type of constraint (e.g., "obstacle", "actuator", "slack")
            details: Dictionary with constraint-specific details
            is_violation: Whether this is a violation (True) or just activation (False)
        """
        data = {
            "timestep": timestep,
            "constraint_type": constraint_type,
            "details": details,
            "is_violation": is_violation
        }
        
        level = "WARNING" if is_violation else "INFO"
        self._create_entry(level, "constraint", LogEventType.CONSTRAINT_EVENT, data)
        
        msg = f"k={timestep} | Constraint: {constraint_type}"
        for key, value in details.items():
            msg += f" | {key}={value}"
        
        if is_violation:
            self.logger.warning(msg)
        else:
            self.logger.info(msg)
    
    def log_simulation_event(self, event: str, details: Optional[Dict[str, Any]] = None) -> None:
        """
        Log general simulation events.
        
        Args:
            event: Description of the event
            details: Optional dictionary with event-specific details
        """
        data = {
            "event": event,
            "details": details or {}
        }
        
        self._create_entry("INFO", "simulation", LogEventType.SIMULATION_EVENT, data)
        
        msg = event
        if details:
            for key, value in details.items():
                msg += f" | {key}={value}"
        
        self.logger.info(msg)
    
    def log_mpc_solve(self, timestep: int, solve_time_ms: float,
                      iterations: int, status: str,
                      slack_used: bool = False) -> None:
        """
        Log MPC solver results.
        
        Args:
            timestep: Current simulation timestep
            solve_time_ms: Solver time in milliseconds
            iterations: Number of solver iterations
            status: Solver status (e.g., "optimal", "infeasible")
            slack_used: Whether slack variables were activated
        """
        level = "INFO" if status == "optimal" else "WARNING"
        
        data = {
            "timestep": timestep,
            "solve_time_ms": solve_time_ms,
            "iterations": iterations,
            "status": status,
            "slack_used": slack_used
        }
        
        self._create_entry(level, "mpc.solver", LogEventType.CONTROL_ACTION, data)
        
        msg = f"MPC solve completed: {iterations} iterations, {solve_time_ms:.2f}ms, status={status}"
        if slack_used:
            msg += " (slack activated)"
        
        if level == "INFO":
            self.logger.info(msg)
        else:
            self.logger.warning(msg)
    
    def log_obstacle_proximity(self, timestep: int, obstacle_id: int,
                               distance: float, warning_threshold: float) -> None:
        """
        Log obstacle proximity warnings.
        
        Args:
            timestep: Current simulation timestep
            obstacle_id: Identifier of the obstacle
            distance: Distance to obstacle in meters
            warning_threshold: Warning threshold distance
        """
        if distance < warning_threshold:
            self.log_constraint_event(
                timestep=timestep,
                constraint_type="obstacle_proximity",
                details={
                    "obstacle_id": obstacle_id,
                    "distance_m": round(distance, 3),
                    "warning_threshold_m": warning_threshold
                },
                is_violation=distance < warning_threshold * 0.5
            )
    
    def export_to_csv(self, filepath: Optional[str] = None) -> str:
        """
        Export state history to CSV for analysis.
        
        Args:
            filepath: Optional custom filepath. If None, uses default in log_dir.
            
        Returns:
            Path to the exported CSV file.
        """
        if filepath is None:
            filepath = os.path.join(self.log_dir, f"states_{self.session_timestamp}.csv")
        
        if not self.state_history:
            self.logger.warning("No state history to export")
            return filepath
        
        with open(filepath, 'w', newline='') as csvfile:
            fieldnames = list(self.state_history[0].keys())
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.state_history)
        
        self.logger.info(f"State history exported to {filepath}")
        return filepath
    
    def export_controls_to_csv(self, filepath: Optional[str] = None) -> str:
        """
        Export control history to CSV.
        
        Args:
            filepath: Optional custom filepath.
            
        Returns:
            Path to the exported CSV file.
        """
        if filepath is None:
            filepath = os.path.join(self.log_dir, f"controls_{self.session_timestamp}.csv")
        
        if not self.control_history:
            self.logger.warning("No control history to export")
            return filepath
        
        with open(filepath, 'w', newline='') as csvfile:
            fieldnames = list(self.control_history[0].keys())
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.control_history)
        
        self.logger.info(f"Control history exported to {filepath}")
        return filepath
    
    def export_to_json(self, filepath: Optional[str] = None) -> str:
        """
        Export all logs to JSON with full metadata.
        
        Args:
            filepath: Optional custom filepath.
            
        Returns:
            Path to the exported JSON file.
        """
        if filepath is None:
            filepath = os.path.join(self.log_dir, f"simulation_{self.session_timestamp}.json")
        
        export_data = {
            "session": {
                "timestamp": self.session_timestamp,
                "node_name": self.node_name,
                "total_entries": len(self.log_entries)
            },
            "entries": [entry.to_dict() for entry in self.log_entries]
        }
        
        with open(filepath, 'w') as jsonfile:
            json.dump(export_data, jsonfile, indent=2)
        
        self.logger.info(f"Logs exported to {filepath}")
        return filepath
    
    def get_summary(self) -> Dict[str, Any]:
        """
        Get summary statistics of logged data.
        
        Returns:
            Dictionary with summary statistics.
        """
        error_count = sum(1 for e in self.log_entries if e.level == "ERROR")
        warning_count = sum(1 for e in self.log_entries if e.level == "WARNING")
        
        summary = {
            "total_entries": len(self.log_entries),
            "state_updates": len(self.state_history),
            "control_actions": len(self.control_history),
            "errors": error_count,
            "warnings": warning_count,
            "session_timestamp": self.session_timestamp
        }
        
        if self.state_history:
            errors = [s["error_norm"] for s in self.state_history]
            summary["max_error_norm"] = max(errors)
            summary["mean_error_norm"] = sum(errors) / len(errors)
            summary["final_error_norm"] = errors[-1]
        
        return summary
    
    def finalize(self) -> None:
        """Finalize logging session and export all data."""
        summary = self.get_summary()
        self.log_simulation_event("Simulation completed", summary)
        
        self.export_to_csv()
        self.export_controls_to_csv()
        self.export_to_json()
        
        self.logger.info(f"Logging session finalized. Summary: {summary}")
