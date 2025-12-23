"""
High-level UnitreeInterface class and factory helpers.

This is a typed façade mirroring the pybind11 C++ bindings; the methods that
actually talk to hardware are left unimplemented for now.
"""

from __future__ import annotations

from unitree_sdk2py.core.channel import (
    ChannelFactoryDestroy,
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_ as LowCmd_GO2
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as LowState_GO2
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as LowCmd_HG
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowState_HG
from unitree_sdk2py.utils.crc import CRC

from .config import RobotConfig
from .constants import (
    G1_NUM_MOTOR,
    GO2_CMD_TOPIC,
    GO2_STATE_TOPIC,
    H1_2_NUM_MOTOR,
    H1_NUM_MOTOR,
    HG_CMD_TOPIC,
    HG_STATE_TOPIC,
    TOPIC_JOYSTICK,
)
from .enums import ControlMode, MessageType, RobotType
from .state import ImuState, LowState, MotorCommand, MotorState, WirelessController


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
        # State storage
        self._motor_state: MotorState = MotorState(
            q=[0.0] * self._config.num_motors,
            dq=[0.0] * self._config.num_motors,
            tau_est=[0.0] * self._config.num_motors,
            temperature=[0] * self._config.num_motors,
            voltage=[0.0] * self._config.num_motors,
        )
        self._motor_command: MotorCommand = MotorCommand(
            q_target=[0.0] * self._config.num_motors,
            dq_target=[0.0] * self._config.num_motors,
            kp=self._default_kp.copy(),
            kd=self._default_kd.copy(),
            tau_ff=[0.0] * self._config.num_motors,
        )
        self._imu_state: ImuState = ImuState(
            rpy=[0.0, 0.0, 0.0],
            omega=[0.0, 0.0, 0.0],
            quat=[0.0, 0.0, 0.0, 0.0],
            accel=[0.0, 0.0, 0.0],
        )
        self._mode_machine: int = 0
        self._initialize_dds()
        self._wireless_controller: WirelessController = WirelessController(
            lx=0.0,
            ly=0.0,
            rx=0.0,
            ry=0.0,
            keys=0,
        )
        self._crc = CRC()

    def __del__(self) -> None:
        """Destroy UnitreeInterface."""
        self._low_state_subscriber.Destroy()
        self._low_cmd_publisher.Destroy()
        self._wireless_subscriber.Destroy()
        ChannelFactoryDestroy()

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
        return LowState(
            motor=self._motor_state,
            imu=self._imu_state,
            mode_machine=self._mode_machine,
        )

    def read_wireless_controller(self) -> WirelessController:
        """Read current wireless controller state."""
        return self._wireless_controller

    def write_motor_command(self) -> None:
        """Write motor command to robot."""
        # Create DDS message object on-the-fly
        match self._config.message_type:
            case MessageType.HG:
                low_cmd: LowCmd_HG = LowCmd_HG()
            case MessageType.GO2:
                low_cmd: LowCmd_GO2 = LowCmd_GO2()

        # Convert MotorCommand to DDS message format
        low_cmd.mode_pr = self._control_mode.value
        low_cmd.mode_machine = self._mode_machine

        for i in range(self._config.num_motors):
            low_cmd.motor_cmd[i].mode = 1  # 1:Enable, 0:Disable
            low_cmd.motor_cmd[i].q = self._motor_command.q_target[i]
            low_cmd.motor_cmd[i].dq = self._motor_command.dq_target[i]
            low_cmd.motor_cmd[i].kp = self._motor_command.kp[i]
            low_cmd.motor_cmd[i].kd = self._motor_command.kd[i]
            low_cmd.motor_cmd[i].tau = self._motor_command.tau_ff[i]

        # Calculate and set CRC
        low_cmd.crc = self._crc.Crc(low_cmd)

        # Publish the command
        self._low_cmd_publisher.Write(low_cmd)

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

    def _initialize_dds(self) -> None:
        """Initialize DDS publisher and subscriber."""
        # Initialize the DDS factory
        ChannelFactoryInitialize(self._domain_id, self._network_interface)

        if self._config.message_type == MessageType.HG:
            # HG message type
            self._low_state_subscriber = ChannelSubscriber(HG_STATE_TOPIC, LowState_HG)
            self._low_state_subscriber.Init(
                self.LowStateHandler,
                10,  # Queue size
            )
            self._low_cmd_publisher = ChannelPublisher(HG_CMD_TOPIC, LowCmd_HG)
            self._low_cmd_publisher.Init()
        else:
            # GO2 message type
            self._low_state_subscriber = ChannelSubscriber(GO2_STATE_TOPIC, LowState_GO2)
            self._low_state_subscriber.Init(
                self.LowStateHandler,
                10,  # Queue size
            )
            self._low_cmd_publisher = ChannelPublisher(GO2_CMD_TOPIC, LowCmd_GO2)
            self._low_cmd_publisher.Init()
        # Wireless controller subscriber (same for both message types)
        self._wireless_subscriber = ChannelSubscriber(TOPIC_JOYSTICK, WirelessController_)
        self._wireless_subscriber.Init(
            self.WirelessControllerHandler,
            10,  # Queue size
        )

    def LowStateHandler(self, state: LowState_HG | LowState_GO2) -> None:
        """Handle low state message."""
        match self._config.message_type:
            case MessageType.HG:
                low_state: LowState_HG = state  # type: ignore[assignment]
                self._process_low_state(low_state)
            case MessageType.GO2:
                low_state: LowState_GO2 = state  # type: ignore[assignment]
                self._process_low_state(low_state)

    def _process_low_state(self, low_state: LowState_HG | LowState_GO2) -> None:
        """Process low state message."""
        # Get motor state
        motor_states = low_state.motor_state()
        for i in range(self._config.num_motors):
            self._motor_state.q[i] = motor_states[i].q()
            self._motor_state.dq[i] = motor_states[i].dq()
            self._motor_state.tau_est[i] = motor_states[i].tau_est()
            self._motor_state.temperature[i] = motor_states[i].temperature()[0]
            self._motor_state.voltage[i] = motor_states[i].vol()

        # Get IMU state
        imu_state = low_state.imu_state()
        self._imu_state.omega = list(imu_state.gyroscope())
        self._imu_state.rpy = list(imu_state.rpy())
        self._imu_state.quat = list(imu_state.quaternion())
        self._imu_state.accel = list(imu_state.accelerometer())

        # Update mode machine
        if self._config.message_type == MessageType.HG and self._mode_machine != low_state.mode_machine():
            if self._mode_machine == 0:
                print(f"{self._config.name} type: {low_state.mode_machine()}")
            self._mode_machine = low_state.mode_machine()

    def WirelessControllerHandler(self, message: WirelessController_) -> None:
        """Handle wireless controller message."""
        self._wireless_controller.lx = message.lx()
        self._wireless_controller.ly = message.ly()
        self._wireless_controller.rx = message.rx()
        self._wireless_controller.ry = message.ry()
        self._wireless_controller.keys = message.keys()

    # ------------------------------------------------------------------
    # Alternate constructors
    # ------------------------------------------------------------------

    @staticmethod
    def create_custom(
        network_interface: str,
        domain_id: int,
        num_motors: int,
        message_type: MessageType,
    ) -> UnitreeInterface:
        """Create custom robot interface."""
        return UnitreeInterface(
            network_interface,
            domain_id,
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
            return UnitreeInterface(
                network_interface,
                domain_id,
                RobotConfig(
                    robot_type=RobotType.G1,
                    message_type=MessageType.HG,
                    num_motors=G1_NUM_MOTOR,
                    name=UnitreeInterface._derive_robot_name(RobotType.G1, MessageType.HG),
                ),
            )
        case RobotType.H1:
            return UnitreeInterface(
                network_interface,
                domain_id,
                RobotConfig(
                    robot_type=RobotType.H1,
                    message_type=MessageType.GO2,
                    num_motors=H1_NUM_MOTOR,
                    name=UnitreeInterface._derive_robot_name(RobotType.H1, MessageType.GO2),
                ),
            )
        case RobotType.H1_2:
            return UnitreeInterface(
                network_interface,
                domain_id,
                RobotConfig(
                    robot_type=RobotType.H1_2,
                    message_type=MessageType.HG,
                    num_motors=H1_2_NUM_MOTOR,
                    name=UnitreeInterface._derive_robot_name(RobotType.H1_2, MessageType.HG),
                ),
            )


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
