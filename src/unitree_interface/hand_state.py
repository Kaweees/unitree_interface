from __future__ import annotations

"""
Hand‑specific state and command data structures.

These mirror the C++ `PyHand*` structs exposed via the pybind11 module.
"""

from dataclasses import dataclass
from typing import List

from .constants import DEX3_NUM_MOTORS, DEX3_NUM_PRESS_SENSORS


@dataclass
class HandMotorState:
    """Dex3 hand motor state."""

    # Joint positions [rad]
    q: List[float]
    # Joint velocities [rad/s]
    dq: List[float]
    # Estimated joint torques [N*m]
    tau_est: List[float]
    # Two temperature channels per motor (e.g. winding / driver)
    temperature: List[List[float]]
    # Motor voltages [V]
    voltage: List[float]


@dataclass
class HandMotorCommand:
    """Dex3 hand motor command."""

    q_target: List[float]
    dq_target: List[float]
    kp: List[float]
    kd: List[float]
    tau_ff: List[float]


@dataclass
class HandPressSensorState:
    """Dex3 finger pressure sensor array state."""

    # Shape: [DEX3_NUM_PRESS_SENSORS, 12]
    pressure: List[List[float]]
    temperature: List[List[float]]
    lost: List[int]
    # Reserve values, 4 uint32 per sensor
    reserve: List[List[int]]


@dataclass
class HandImuState:
    """Dex3 hand IMU state."""

    quaternion: List[float]
    gyroscope: List[float]
    accelerometer: List[float]
    rpy: List[float]
    temperature: float


@dataclass
class HandState:
    """Complete Dex3 hand state."""

    motor: HandMotorState
    press_sensor: HandPressSensorState
    imu: HandImuState
    power_v: float
    power_a: float
    system_v: float
    device_v: float
    error: List[int]
    reserve: List[int]


def empty_hand_motor_state() -> HandMotorState:
    """Create an empty (zero-initialised) `HandMotorState`."""
    return HandMotorState(
        q=[0.0] * DEX3_NUM_MOTORS,
        dq=[0.0] * DEX3_NUM_MOTORS,
        tau_est=[0.0] * DEX3_NUM_MOTORS,
        temperature=[[0.0, 0.0] for _ in range(DEX3_NUM_MOTORS)],
        voltage=[0.0] * DEX3_NUM_MOTORS,
    )


def empty_hand_press_sensor_state() -> HandPressSensorState:
    """Create an empty (zero-initialised) `HandPressSensorState`."""
    return HandPressSensorState(
        pressure=[[0.0] * 12 for _ in range(DEX3_NUM_PRESS_SENSORS)],
        temperature=[[0.0] * 12 for _ in range(DEX3_NUM_PRESS_SENSORS)],
        lost=[0] * DEX3_NUM_PRESS_SENSORS,
        reserve=[[0] * 4 for _ in range(DEX3_NUM_PRESS_SENSORS)],
    )


def empty_hand_imu_state() -> HandImuState:
    """Create an empty (zero-initialised) `HandImuState`."""
    return HandImuState(
        quaternion=[0.0] * 4,
        gyroscope=[0.0] * 3,
        accelerometer=[0.0] * 3,
        rpy=[0.0] * 3,
        temperature=0.0,
    )


def empty_hand_state() -> HandState:
    """Create a fully empty `HandState`."""
    return HandState(
        motor=empty_hand_motor_state(),
        press_sensor=empty_hand_press_sensor_state(),
        imu=empty_hand_imu_state(),
        power_v=0.0,
        power_a=0.0,
        system_v=0.0,
        device_v=0.0,
        error=[0, 0],
        reserve=[0, 0],
    )


__all__ = [
    "HandMotorState",
    "HandMotorCommand",
    "HandPressSensorState",
    "HandImuState",
    "HandState",
    "empty_hand_motor_state",
    "empty_hand_press_sensor_state",
    "empty_hand_imu_state",
    "empty_hand_state",
]
