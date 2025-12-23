"""
Robot configuration structures and predefined configurations.
"""

from __future__ import annotations

from dataclasses import dataclass

from .constants import G1_NUM_MOTOR, H1_2_NUM_MOTOR, H1_NUM_MOTOR
from .enums import MessageType, RobotType


@dataclass
class RobotConfig:
    """Robot configuration structure."""

    robot_type: RobotType
    message_type: MessageType
    num_motors: int
    name: str


G1_HG_CONFIG = RobotConfig(RobotType.G1, MessageType.HG, G1_NUM_MOTOR, "G1-HG")
H1_GO2_CONFIG = RobotConfig(RobotType.H1, MessageType.GO2, H1_NUM_MOTOR, "H1-GO2")
H1_2_HG_CONFIG = RobotConfig(RobotType.H1_2, MessageType.HG, H1_2_NUM_MOTOR, "H1-2-HG")


__all__ = [
    "G1_HG_CONFIG",
    "H1_2_HG_CONFIG",
    "H1_GO2_CONFIG",
    "RobotConfig",
]
