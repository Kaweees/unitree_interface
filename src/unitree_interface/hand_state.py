"""
Hand-specific state and command data structures.

These mirror the C++ `PyHand*` structs exposed via the pybind11 module.
"""

from __future__ import annotations

from dataclasses import dataclass

from .constants import DEX3_NUM_MOTORS, DEX3_NUM_PRESS_SENSORS


@dataclass
class HandMotorState:
    """Dex3 hand motor state."""

    # Joint positions [rad]
    q: list[float]
    # Joint velocities [rad/s]
    dq: list[float]
    # Estimated joint torques [N*m]
    tau_est: list[float]
    # Two temperature channels per motor (e.g. winding / driver)
    temperature: list[list[float]]
    # Motor voltages [V]
    voltage: list[float]


@dataclass
class HandMotorCommand:
    """Dex3 hand motor command."""

    q_target: list[float]
    dq_target: list[float]
    kp: list[float]
    kd: list[float]
    tau_ff: list[float]


@dataclass
class HandPressSensorState:
    """Dex3 finger pressure sensor array state."""

    # Shape: [DEX3_NUM_PRESS_SENSORS, 12]
    pressure: list[list[float]]
    temperature: list[list[float]]
    lost: list[int]
    # Reserve values, 4 uint32 per sensor
    reserve: list[list[int]]


@dataclass
class HandImuState:
    """Dex3 hand IMU state."""

    quaternion: list[float]
    gyroscope: list[float]
    accelerometer: list[float]
    rpy: list[float]
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
    error: list[int]
    reserve: list[int]


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
    "HandImuState",
    "HandMotorCommand",
    "HandMotorState",
    "HandPressSensorState",
    "HandState",
    "empty_hand_imu_state",
    "empty_hand_motor_state",
    "empty_hand_press_sensor_state",
    "empty_hand_state",
]
