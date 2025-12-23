"""
Core state and command data structures for the Unitree interface.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ImuState:
    """IMU state data."""

    # Roll, Pitch, Yaw angles [rad] (3 elements)
    rpy: list[float]
    # Angular velocity [rad/s] (3 elements)
    omega: list[float]
    # Quaternion [w, x, y, z] (4 elements)
    quat: list[float]
    # Linear acceleration [m/s^2] (3 elements)
    accel: list[float]


@dataclass
class MotorState:
    """Motor state data for all joints."""

    # Joint positions [rad] (variable length)
    q: list[float]
    # Joint velocities [rad/s] (variable length)
    dq: list[float]
    # Estimated joint torques [N*m] (variable length)
    tau_est: list[float]
    # Motor temperatures [°C] (variable length)
    temperature: list[int]
    # Motor voltages [V] (variable length)
    voltage: list[float]


@dataclass
class MotorCommand:
    """Motor command data for all joints."""

    # Target joint positions [rad] (variable length)
    q_target: list[float]
    # Target joint velocities [rad/s] (variable length)
    dq_target: list[float]
    # Position gains (variable length)
    kp: list[float]
    # Velocity gains (variable length)
    kd: list[float]
    # Feedforward torques [N*m] (variable length)
    tau_ff: list[float]


@dataclass
class WirelessController:
    """Wireless controller state."""

    # Left stick
    lx: float
    ly: float
    # Right stick
    rx: float
    ry: float
    keys: int


@dataclass
class JoystickState:
    """Joystick state."""

    # Left stick [x, y] (2 elements)
    left_stick: list[float]
    # Right stick [x, y] (2 elements)
    right_stick: list[float]

    A: bool
    B: bool
    X: bool
    Y: bool
    L1: bool
    L2: bool
    R1: bool
    R2: bool


@dataclass
class LowState:
    """Complete robot low-level state."""

    # IMU state
    imu: ImuState
    # Motor state
    motor: MotorState
    # Robot mode machine state
    mode_machine: int


__all__ = [
    "ImuState",
    "LowState",
    "MotorCommand",
    "MotorState",
    "WirelessController",
]
