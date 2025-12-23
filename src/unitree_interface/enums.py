"""
Enum definitions for Unitree interface types.
"""

from __future__ import annotations

from enum import Enum


class RobotType(Enum):
    """Robot types supported by the interface."""

    G1 = 0  # G1 humanoid (29 motors)
    H1 = 1  # H1 humanoid (19 motors)
    H1_2 = 2  # H1-2 humanoid (29 motors)
    CUSTOM = 99  # Custom robot with specified motor count


class MessageType(Enum):
    """Message types for robot communication."""

    HG = 0  # Humanoid/Go1 message format
    GO2 = 1  # Go2 message format


class HandType(Enum):
    """Hand types supported by the interface."""

    LEFT_HAND = 0
    RIGHT_HAND = 1


class ControlMode(Enum):
    """Control mode for robots."""

    PR = 0  # Pitch/Roll mode
    AB = 1  # A/B mode


__all__ = ["ControlMode", "HandType", "MessageType", "RobotType"]
