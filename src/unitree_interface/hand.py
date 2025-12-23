from __future__ import annotations

"""
High-level Dex3 HandInterface facade.

This mirrors the C++ `HandInterface` pybind11 API:

- Constructor: HandInterface(network_interface, hand_type, re_init=True)
- Methods: read_hand_state, write_hand_command, create_zero_command,
  create_default_command, get_default_kp, get_default_kd, get_max_limits,
  get_min_limits, clamp_joint_angles, normalize_joint_angles.

The actual low-level communication is expected to be provided by a C++/DDS
backend; this module focuses on the public Python surface and helpers.
"""

from dataclasses import dataclass
from typing import Tuple, List

from .constants import (
    DEX3_NUM_MOTORS,
    DEX3_NUM_PRESS_SENSORS,
    LEFT_HAND_CMD_TOPIC,
    LEFT_HAND_STATE_TOPIC,
    RIGHT_HAND_CMD_TOPIC,
    RIGHT_HAND_STATE_TOPIC,
)
from .enums import HandType
from .hand_state import (
    HandMotorCommand,
    HandState,
    empty_hand_state,
)


# Joint limits for Dex3-1 hands (match the C++ implementation)
LEFT_HAND_MAX_LIMITS = (
    1.0472,
    1.0472,
    1.74533,
    0.0,
    0.0,
    0.0,
    0.0,
)
LEFT_HAND_MIN_LIMITS = (
    -1.0472,
    -0.724312,
    0.0,
    -1.5708,
    -1.74533,
    -1.5708,
    -1.74533,
)
RIGHT_HAND_MAX_LIMITS = (
    1.0472,
    0.724312,
    0.0,
    1.5708,
    1.74533,
    1.5708,
    1.74533,
)
RIGHT_HAND_MIN_LIMITS = (
    -1.0472,
    -1.0472,
    -1.74533,
    0.0,
    0.0,
    0.0,
    0.0,
)

# Default hand poses
DEFAULT_LEFT_HAND_POSE = (0.0,) * DEX3_NUM_MOTORS
DEFAULT_RIGHT_HAND_POSE = (0.0,) * DEX3_NUM_MOTORS


@dataclass
class HandInterface:
    """
    High-level Dex3 hand interface facade.

    The default implementation only provides typed helpers and does **not**
    communicate with hardware; backends are expected to subclass or wrap this.
    """

    network_interface: str
    hand_type: HandType
    re_init: bool = True

    def __post_init__(self) -> None:
        self._hand_name = "LeftHand" if self.hand_type is HandType.LEFT_HAND else "RightHand"
        # Default gains (match C++ defaults)
        self._default_kp = [0.5] * DEX3_NUM_MOTORS
        self._default_kd = [0.1] * DEX3_NUM_MOTORS

    # ------------------------------------------------------------------
    # Core API (to be implemented by a backend)
    # ------------------------------------------------------------------

    def read_hand_state(self) -> HandState:
        """
        Read current hand state.

        The base implementation returns an empty state; concrete backends
        should override this and provide real data.
        """
        return empty_hand_state()

    def write_hand_command(self, command: HandMotorCommand) -> None:
        """
        Send motor command to the hand.

        Base implementation is a no-op; concrete backends should override.
        """
        raise NotImplementedError("Low-level hand I/O must be provided by a backend implementation.")

    # ------------------------------------------------------------------
    # Utility helpers
    # ------------------------------------------------------------------

    def create_zero_command(self) -> HandMotorCommand:
        """Create a zero command with default gains."""
        zeros = [0.0] * DEX3_NUM_MOTORS
        return HandMotorCommand(
            q_target=zeros.copy(),
            dq_target=zeros.copy(),
            kp=self.get_default_kp(),
            kd=self.get_default_kd(),
            tau_ff=zeros.copy(),
        )

    def create_default_command(self) -> HandMotorCommand:
        """Create a command with default pose and default gains."""
        cmd = self.create_zero_command()
        if self.hand_type is HandType.LEFT_HAND:
            cmd.q_target = list(DEFAULT_LEFT_HAND_POSE)
        else:
            cmd.q_target = list(DEFAULT_RIGHT_HAND_POSE)
        return cmd

    def get_default_kp(self) -> List[float]:
        """Get default proportional gains for all joints."""
        return list(self._default_kp)

    def get_default_kd(self) -> List[float]:
        """Get default derivative gains for all joints."""
        return list(self._default_kd)

    # Limits ------------------------------------------------------------

    @property
    def hand_name(self) -> str:
        """Get the hand name string."""
        return self._hand_name

    def get_max_limits(self) -> List[float]:
        """Get maximum joint angle limits."""
        if self.hand_type is HandType.LEFT_HAND:
            return list(LEFT_HAND_MAX_LIMITS)
        return list(RIGHT_HAND_MAX_LIMITS)

    def get_min_limits(self) -> List[float]:
        """Get minimum joint angle limits."""
        if self.hand_type is HandType.LEFT_HAND:
            return list(LEFT_HAND_MIN_LIMITS)
        return list(RIGHT_HAND_MIN_LIMITS)

    def clamp_joint_angles(self, joint_angles: List[float]) -> None:
        """
        Clamp joint angles to valid range in-place.

        `joint_angles` is modified directly.
        """
        max_limits = self.get_max_limits()
        min_limits = self.get_min_limits()
        for i in range(min(len(joint_angles), DEX3_NUM_MOTORS)):
            lo = min_limits[i]
            hi = max_limits[i]
            x = joint_angles[i]
            joint_angles[i] = lo if x < lo else hi if x > hi else x

    def normalize_joint_angles(self, joint_angles: List[float]) -> List[float]:
        """
        Normalize joint angles to [0, 1] range based on joint limits.
        """
        max_limits = self.get_max_limits()
        min_limits = self.get_min_limits()
        out: List[float] = [0.0] * DEX3_NUM_MOTORS
        for i in range(min(len(joint_angles), DEX3_NUM_MOTORS)):
            lo = min_limits[i]
            hi = max_limits[i]
            span = hi - lo
            if span > 0.0:
                val = (joint_angles[i] - lo) / span
                if val < 0.0:
                    val = 0.0
                elif val > 1.0:
                    val = 1.0
                out[i] = val
            else:
                out[i] = 0.0
        return out

    # ------------------------------------------------------------------
    # Static / module-level helpers (mirror C++ API)
    # ------------------------------------------------------------------

    @staticmethod
    def create_left_hand(network_interface: str, re_init: bool = True) -> "HandInterface":
        """Create a left hand interface."""
        return HandInterface(network_interface=network_interface, hand_type=HandType.LEFT_HAND, re_init=re_init)

    @staticmethod
    def create_right_hand(network_interface: str, re_init: bool = True) -> "HandInterface":
        """Create a right hand interface."""
        return HandInterface(network_interface=network_interface, hand_type=HandType.RIGHT_HAND, re_init=re_init)


def create_hand(network_interface: str, hand_type: HandType, re_init: bool = True) -> HandInterface:
    """
    Convenience function to create a hand interface based on `HandType`.
    """
    if hand_type is HandType.LEFT_HAND:
        return HandInterface.create_left_hand(network_interface, re_init=re_init)
    return HandInterface.create_right_hand(network_interface, re_init=re_init)


def create_dual_hands(network_interface: str, re_init: bool = True) -> Tuple[HandInterface, HandInterface]:
    """
    Create both left and right hand interfaces.

    The left hand is created with the provided `re_init` flag; the right hand
    mirrors the C++ behaviour and does not re-initialise the underlying DDS
    connection when `re_init` is True.
    """
    left = HandInterface.create_left_hand(network_interface, re_init=re_init)
    # For the second hand we default to `re_init=False` to match the C++ behaviour
    right = HandInterface.create_right_hand(network_interface, re_init=False)
    return left, right


__all__ = [
    "HandInterface",
    "create_hand",
    "create_dual_hands",
    "LEFT_HAND_CMD_TOPIC",
    "LEFT_HAND_STATE_TOPIC",
    "RIGHT_HAND_CMD_TOPIC",
    "RIGHT_HAND_STATE_TOPIC",
    "DEX3_NUM_MOTORS",
    "DEX3_NUM_PRESS_SENSORS",
]



