"""
High-level UnitreeInterface class and factory helpers.

This is a typed façade mirroring the pybind11 C++ bindings; the methods that
actually talk to hardware are left unimplemented for now.
"""

from __future__ import annotations

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_ as LowCmd_GO2
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as LowState_GO2
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as LowCmd_HG
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowState_HG

from .config import RobotConfig
from .constants import (
    G1_NUM_MOTOR,
    GO2_CMD_TOPIC,
    GO2_STATE_TOPIC,
    H1_2_NUM_MOTOR,
    H1_NUM_MOTOR,
    HG_CMD_TOPIC,
    HG_STATE_TOPIC,
)
from .enums import ControlMode, MessageType, RobotType
from .state import LowState, MotorCommand, WirelessController


class UnitreeInterface:
    """
    Main interface class for general Unitree robot control.

    This is a typed, high-level facade. Actual communication with the hardware
    must be implemented by a backend (e.g. a C++/pybind11 extension).
    """

    def __init__(
        self,
        network_interface: str,
        domain_id: int,
        config: RobotConfig,
    ) -> None:
        """
        Initialize UnitreeInterface.

        You can either pass a full `RobotConfig` via `config`, or specify
        `robot_type`/`message_type` (and optionally `num_motors` for custom robots).
        """
        self._network_interface = network_interface
        self._domain_id = domain_id
        self._config = config
        self._control_mode: ControlMode = ControlMode.PR
        self._default_kp: list[float] = self._get_default_kp()
        self._default_kd: list[float] = self._get_default_kd()
        self._initalize_dds()

    def __del__(self) -> None:
        """Destroy UnitreeInterface."""
        self._low_state_subscriber.Destroy()
        self._low_cmd_publisher.Destroy()

    @staticmethod
    def _derive_robot_name(robot_type: RobotType, message_type: MessageType) -> str:
        rt = {
            RobotType.G1: "G1",
            RobotType.H1: "H1",
            RobotType.H1_2: "H1-2",
            RobotType.CUSTOM: "CUSTOM",
        }.get(robot_type, "UNKNOWN")
        mt = "HG" if message_type is MessageType.HG else "GO2"
        return f"{rt}-{mt}"

    # ------------------------------------------------------------------
    # Core API (purely structural / to be implemented by a backend)
    # ------------------------------------------------------------------

    def read_low_state(self) -> LowState:
        """Read current robot low state."""
        raise NotImplementedError("Low-level I/O must be provided by a backend implementation.")

    def read_wireless_controller(self) -> WirelessController:
        """Read current wireless controller state."""
        raise NotImplementedError("Low-level I/O must be provided by a backend implementation.")

    def write_low_command(self, command: MotorCommand) -> None:
        """Write motor command to robot."""
        raise NotImplementedError("Low-level I/O must be provided by a backend implementation.")

    def set_control_mode(self, mode: ControlMode) -> None:
        """Set control mode (PR or AB)."""
        self._control_mode = mode

    def get_control_mode(self) -> ControlMode:
        """Get current control mode."""
        return self._control_mode

    # ------------------------------------------------------------------
    # Convenience helpers
    # ------------------------------------------------------------------

    def create_zero_command(self) -> MotorCommand:
        """
        Create a zero motor command with default gains.

        The default implementation only allocates correctly-sized vectors and
        does not talk to the robot.
        """
        zeros = [0.0] * self._config.num_motors
        return MotorCommand(
            q_target=zeros.copy(),
            dq_target=zeros.copy(),
            kp=self._default_kp.copy(),
            kd=self._default_kd.copy(),
            tau_ff=zeros.copy(),
        )

    def _get_default_kp(self) -> list[float]:
        """Get default position gains."""
        # Default pattern for humanoids
        kp: list[float] = [40.0] * self._config.num_motors

        match self._config.robot_type:
            case RobotType.CUSTOM:
                pass
            case RobotType.G1 | RobotType.H1 | RobotType.H1_2:
                # Legs
                for i in range(12):
                    kp[i] = 60.0
                # Waist
                for i in range(12, 15):
                    kp[i] = 60.0
                # Arms
                for i in range(15, self._config.num_motors):
                    kp[i] = 40.0
        return kp

    def _get_default_kd(self) -> list[float]:
        """Get default velocity gains."""
        return [1.0] * self._config.num_motors

    def _initalize_dds(self) -> None:
        """Initialize DDS publisher."""
        # Initialize the DDS factory
        ChannelFactoryInitialize(self._domain_id, self._network_interface)

        if self._config.message_type == MessageType.HG:
            # HG message type
            self._low_state_subscriber = ChannelSubscriber(HG_STATE_TOPIC, LowState_HG)
            self._low_state_subscriber.Init()
            self._low_cmd_publisher = ChannelPublisher(HG_CMD_TOPIC, LowCmd_HG)
            self._low_cmd_publisher.Init()
        else:
            # GO2 message type
            self._low_state_subscriber = ChannelSubscriber(GO2_STATE_TOPIC, LowState_GO2)
            self._low_state_subscriber.Init()
            self._low_cmd_publisher = ChannelPublisher(GO2_CMD_TOPIC, LowCmd_GO2)
            self._low_cmd_publisher.Init()

    # ------------------------------------------------------------------
    # Configuration accessors
    # ------------------------------------------------------------------

    def get_config(self) -> RobotConfig:
        """Get robot configuration."""
        return self._config

    def get_num_motors(self) -> int:
        """Get number of motors."""
        return self._config.num_motors

    def get_robot_name(self) -> str:
        """Get robot name."""
        return self._config.name

    # ------------------------------------------------------------------
    # Alternate constructors
    # ------------------------------------------------------------------

    @staticmethod
    def create_g1(network_interface: str, domain_id: int    ) -> UnitreeInterface:
        """Create G1 robot interface."""
        return UnitreeInterface(
            network_interface,
            domain_id,
            RobotConfig(
                robot_type=RobotType.G1,
                message_type=MessageType.HG,
                num_motors=G1_NUM_MOTOR,
                name=UnitreeInterface._derive_robot_name(RobotType.G1, MessageType.HG)
            ),
        )

    @staticmethod
    def create_h1(network_interface: str, domain_id: int) -> UnitreeInterface:
        """Create H1 robot interface."""
        return UnitreeInterface(
            network_interface,
            domain_id,
            RobotConfig(
                robot_type=RobotType.H1,
                message_type=MessageType.GO2,
                num_motors=H1_NUM_MOTOR,
                name=UnitreeInterface._derive_robot_name(RobotType.H1, MessageType.GO2)
            )
        ),

    @staticmethod
    def create_h1_2(network_interface: str, domain_id: int) -> UnitreeInterface:
        """Create H1-2 robot interface."""
        return UnitreeInterface(
            network_interface,
            domain_id,
            RobotConfig(
                robot_type=RobotType.H1_2,
                message_type=MessageType.HG,
                num_motors=H1_2_NUM_MOTOR,
                name=UnitreeInterface._derive_robot_name(RobotType.H1_2, MessageType.HG)
            ),
        )

    @staticmethod
    def create_custom(
        network_interface: str,
        num_motors: int,
        message_type: MessageType,
    ) -> UnitreeInterface:
        """Create custom robot interface."""
        return UnitreeInterface(
            network_interface,
            config=RobotConfig(
                robot_type=RobotType.CUSTOM,
                message_type=message_type,
                num_motors=num_motors,
                name=UnitreeInterface._derive_robot_name(RobotType.CUSTOM, message_type),
            ),
        )

    def __repr__(self) -> str:  # pragma: no cover - trivial
        return f"UnitreeInterface(network_interface={self._network_interface!r}, config={self._config!r})"


# =====================================================================
# Module-level factory functions
# =====================================================================


def create_robot(
    network_interface: str,
    domain_id: int,
    robot_type: RobotType,
) -> UnitreeInterface:
    """
    Create robot interface based on robot type.
    """
    match robot_type:
        case RobotType.CUSTOM:
            raise ValueError("Use `create_robot_with_config` or `UnitreeInterface.create_custom` for CUSTOM robots.")
        case RobotType.G1:
            return UnitreeInterface.create_g1(network_interface, domain_id)
        case RobotType.H1:
            return UnitreeInterface.create_h1(network_interface, domain_id)
        case RobotType.H1_2:
            return UnitreeInterface.create_h1_2(network_interface, domain_id)


def create_robot_with_config(network_interface: str, domain_id: int, config: RobotConfig) -> UnitreeInterface:
    """
    Create robot interface with configuration.
    """
    return UnitreeInterface(network_interface, domain_id, config)


__all__ = [
    "UnitreeInterface",
    "create_robot",
    "create_robot_with_config",
]
