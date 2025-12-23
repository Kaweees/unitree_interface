"""
Public package API for the General Unitree Interface Python bindings.

All real implementations live in the submodules; this file simply re-exports
them to provide a clean, flat public namespace.
"""

from .config import G1_HG_CONFIG, H1_2_HG_CONFIG, H1_GO2_CONFIG, RobotConfig
from .constants import (
    DEX3_NUM_MOTORS,
    DEX3_NUM_PRESS_SENSORS,
    G1_NUM_MOTOR,
    H1_2_NUM_MOTOR,
    H1_NUM_MOTOR,
    LEFT_HAND_CMD_TOPIC,
    LEFT_HAND_STATE_TOPIC,
    RIGHT_HAND_CMD_TOPIC,
    RIGHT_HAND_STATE_TOPIC,
)
from .enums import ControlMode, HandType, MessageType, RobotType
from .hand import HandInterface, create_dual_hands, create_hand
from .hand_state import (
    HandImuState,
    HandMotorCommand,
    HandMotorState,
    HandPressSensorState,
    HandState,
)
from .interface import UnitreeInterface, create_robot, create_robot_with_config
from .state import ImuState, LowState, MotorCommand, MotorState, WirelessController

# fmt: off
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
# fmt: on
