"""
Public package API for the General Unitree Interface Python bindings.

All real implementations live in the submodules; this file simply re-exports
them to provide a clean, flat public namespace.
"""

from .config import G1_HG_CONFIG, H1_2_HG_CONFIG, H1_GO2_CONFIG, RobotConfig
from .constants import (
    G1_NUM_MOTOR,
    H1_2_NUM_MOTOR,
    H1_NUM_MOTOR,
)
from .enums import ControlMode, MessageType, RobotType
from .interface import UnitreeInterface, create_robot, create_robot_with_config
from .state import ImuState, LowState, MotorCommand, MotorState, WirelessController

__all__ = [
    # Constants
    "G1_NUM_MOTOR",
    "H1_NUM_MOTOR",
    "H1_2_NUM_MOTOR",
    "G1_HG_CONFIG",
    "H1_GO2_CONFIG",
    "H1_2_HG_CONFIG",
    # Enums and config
    "RobotType",
    "MessageType",
    "ControlMode",
    "RobotConfig",
    # Core state / command types
    "ImuState",
    "MotorState",
    "MotorCommand",
    "WirelessController",
    "LowState",
    # Main interface
    "UnitreeInterface",
    # Factory helpers
    "create_robot",
    "create_robot_with_config",
]
