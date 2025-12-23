"""
Numeric and topic constants for Unitree robots and hands.
"""

from __future__ import annotations

# Robot motor counts
G1_NUM_MOTOR: int = 29
H1_NUM_MOTOR: int = 19
H1_2_NUM_MOTOR: int = 29

# Dex3-1 hand constants (match the C++ bindings)
DEX3_NUM_MOTORS: int = 7
DEX3_NUM_PRESS_SENSORS: int = 9


# Topic definitions
HG_CMD_TOPIC: str = "rt/lowcmd"
HG_STATE_TOPIC: str = "rt/lowstate"
GO2_CMD_TOPIC: str = "rt/lowcmd"
GO2_STATE_TOPIC: str = "rt/lowstate"
TOPIC_JOYSTICK: str = "rt/wirelesscontroller"

# Hand topic definitions
LEFT_HAND_CMD_TOPIC: str = "rt/dex3/left/cmd"
LEFT_HAND_STATE_TOPIC: str = "rt/dex3/left/state"
RIGHT_HAND_CMD_TOPIC: str = "rt/dex3/right/cmd"
RIGHT_HAND_STATE_TOPIC: str = "rt/dex3/right/state"

__all__ = [
    "DEX3_NUM_MOTORS",
    "DEX3_NUM_PRESS_SENSORS",
    "G1_NUM_MOTOR",
    "GO2_CMD_TOPIC",
    "GO2_STATE_TOPIC",
    "H1_2_NUM_MOTOR",
    "H1_NUM_MOTOR",
    "HG_CMD_TOPIC",
    "HG_STATE_TOPIC",
    "LEFT_HAND_CMD_TOPIC",
    "LEFT_HAND_STATE_TOPIC",
    "RIGHT_HAND_CMD_TOPIC",
    "RIGHT_HAND_STATE_TOPIC",
    "TOPIC_JOYSTICK",
]
