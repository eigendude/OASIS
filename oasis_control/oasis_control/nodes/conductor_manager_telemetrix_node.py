################################################################################
#
#  Copyright (C) 2022-2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manager for a model LEGO train station
#

from typing import Optional

import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from rclpy.logging import LoggingSeverity
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool as BoolMsg
from std_msgs.msg import Header as HeaderMsg

from oasis_control.input.checkerboard_slowdown import CheckerboardCruiseSlowdown
from oasis_control.input.park_mode import TrainParkMode
from oasis_control.input.station_input import StationInput
from oasis_control.lego_models.helipad_manager import HelipadManager
from oasis_control.lego_models.station_manager import StationManager
from oasis_control.managers.sampling_manager import SamplingManager
from oasis_control.managers.wol_manager import WolManager
from oasis_control.mcu.mcu_memory_manager import McuMemoryManager
from oasis_msgs.msg import ConductorState as ConductorStateMsg


################################################################################
# Hardware configuration
################################################################################


# Memory reporting interval, in seconds
REPORT_MCU_MEMORY_PERIOD_SECS: float = 1.0

# Sampling interval, in ms
SAMPLING_INTERVAL_MS = 100

# Station pin configuration
VSS_PIN: int = 0  # A0 (orange wire)
MOTOR_PWM_PIN: int = 5  # D5 (white wire)
MOTOR_DIR_PIN: int = 4  # D4 (white wire)
MOTOR_FF1_PIN: int = 8  # D8 (white wire)
MOTOR_FF2_PIN: int = 7  # D7 (white wire)
MOTOR_CURRENT_PIN: int = 1  # A1 (orange wire)
MOTOR_VOLTAGE_A_PIN: int = 3  # A3 (blue wire)
MOTOR_VOLTAGE_B_PIN: int = 4  # A4 (green wire)

# Helipad pin configuration
HELIPAD_IR_PIN: int = 2  # A2 (green wire)
HELIPAD_LED_PAIR_A_PIN: int = 10  # D10 (blue wire)
HELIPAD_LED_PAIR_B_PIN: int = 11  # D11 (blue wire)

################################################################################
# Automation parameters
################################################################################


# The hostname or IP of the computer that provides input
INPUT_HOSTNAME: str = "megapegasus.local"

# The hostname or IP of the computer that provides perception and rendering
VISION_HOSTNAME: str = "precision.local"

# Amount of time to wait for WoL services, in seconds
WOL_TIMEOUT_SECS: float = 5.0


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE = "oasis"

NODE_NAME = "conductor_manager"

PUBLISH_STATE_PERIOD_SECS = 0.1

# Publisher
PUBLISH_CONDUCTOR_STATE = "conductor_state"

# Subscribers
SUBSCRIBE_CHECKERBOARD_STATUS = "checkerboard_status"

# Parameters
PARAM_MOTOR_VOLTAGE_REVERSED: str = "motor_voltage_reversed"
DEFAULT_MOTOR_VOLTAGE_REVERSED: bool = False
PARAM_CHECKERBOARD_SLOWDOWN_ENABLED: str = "checkerboard_slowdown_enabled"
DEFAULT_CHECKERBOARD_SLOWDOWN_ENABLED: bool = True
PARAM_CHECKERBOARD_SLOWDOWN_DURATION_SEC: str = "checkerboard_slowdown_duration_sec"
DEFAULT_CHECKERBOARD_SLOWDOWN_DURATION_SEC: float = 3.0
PARAM_CHECKERBOARD_SLOWDOWN_CLEAR_CONFIRM_SEC: str = (
    "checkerboard_slowdown_clear_confirm_sec"
)
DEFAULT_CHECKERBOARD_SLOWDOWN_CLEAR_CONFIRM_SEC: float = 2.0
PARAM_CHECKERBOARD_SLOWDOWN_SCALE: str = "checkerboard_slowdown_scale"
DEFAULT_CHECKERBOARD_SLOWDOWN_SCALE: float = 0.65
PARAM_PARK_MODE_ENABLED: str = "park_mode_enabled"
DEFAULT_PARK_MODE_ENABLED: bool = True
PARAM_PARK_MODE_COMMAND: str = "park_mode_command"
DEFAULT_PARK_MODE_COMMAND: float = 0.75


################################################################################
# ROS node
################################################################################


class ConductorManagerNode(rclpy.node.Node):
    """
    A ROS node that manages a LEGO train power conductor's conductor.
    """

    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Enable debug logging
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        self.declare_parameter(
            PARAM_MOTOR_VOLTAGE_REVERSED,
            DEFAULT_MOTOR_VOLTAGE_REVERSED,
        )
        self.declare_parameter(
            PARAM_CHECKERBOARD_SLOWDOWN_ENABLED,
            DEFAULT_CHECKERBOARD_SLOWDOWN_ENABLED,
        )
        self.declare_parameter(
            PARAM_CHECKERBOARD_SLOWDOWN_DURATION_SEC,
            DEFAULT_CHECKERBOARD_SLOWDOWN_DURATION_SEC,
        )
        self.declare_parameter(
            PARAM_CHECKERBOARD_SLOWDOWN_CLEAR_CONFIRM_SEC,
            DEFAULT_CHECKERBOARD_SLOWDOWN_CLEAR_CONFIRM_SEC,
        )
        self.declare_parameter(
            PARAM_CHECKERBOARD_SLOWDOWN_SCALE,
            DEFAULT_CHECKERBOARD_SLOWDOWN_SCALE,
        )
        self.declare_parameter(
            PARAM_PARK_MODE_ENABLED,
            DEFAULT_PARK_MODE_ENABLED,
        )
        self.declare_parameter(
            PARAM_PARK_MODE_COMMAND,
            DEFAULT_PARK_MODE_COMMAND,
        )
        motor_voltage_reversed: bool = bool(
            self.get_parameter(PARAM_MOTOR_VOLTAGE_REVERSED).value
        )
        checkerboard_slowdown_enabled: bool = bool(
            self.get_parameter(PARAM_CHECKERBOARD_SLOWDOWN_ENABLED).value
        )
        checkerboard_slowdown_duration_sec: float = self._get_nonnegative_parameter(
            PARAM_CHECKERBOARD_SLOWDOWN_DURATION_SEC,
            DEFAULT_CHECKERBOARD_SLOWDOWN_DURATION_SEC,
        )
        checkerboard_slowdown_clear_confirm_sec: float = (
            self._get_nonnegative_parameter(
                PARAM_CHECKERBOARD_SLOWDOWN_CLEAR_CONFIRM_SEC,
                DEFAULT_CHECKERBOARD_SLOWDOWN_CLEAR_CONFIRM_SEC,
            )
        )
        checkerboard_slowdown_scale: float = self._get_unit_scale_parameter(
            PARAM_CHECKERBOARD_SLOWDOWN_SCALE,
            DEFAULT_CHECKERBOARD_SLOWDOWN_SCALE,
        )
        park_mode_enabled: bool = bool(
            self.get_parameter(PARAM_PARK_MODE_ENABLED).value
        )

        # Unitless normalized reverse command before max safe duty-cycle scaling
        park_mode_command: float = self._get_unit_scale_parameter(
            PARAM_PARK_MODE_COMMAND,
            DEFAULT_PARK_MODE_COMMAND,
        )
        self._checkerboard_slowdown_duration_sec: float = (
            checkerboard_slowdown_duration_sec
        )

        self._checkerboard_slowdown: CheckerboardCruiseSlowdown = (
            CheckerboardCruiseSlowdown(
                checkerboard_slowdown_enabled,
                checkerboard_slowdown_duration_sec,
                checkerboard_slowdown_clear_confirm_sec,
                checkerboard_slowdown_scale,
            )
        )
        self._park_mode: TrainParkMode = TrainParkMode(
            park_mode_enabled,
            park_mode_command,
        )

        # Subsystems
        self._mcu_memory_manager: McuMemoryManager = McuMemoryManager(self)
        self._sampling_manager: SamplingManager = SamplingManager(self)
        self._station_manager: StationManager = StationManager(
            self,
            VSS_PIN,
            MOTOR_PWM_PIN,
            MOTOR_DIR_PIN,
            MOTOR_FF1_PIN,
            MOTOR_FF2_PIN,
            MOTOR_CURRENT_PIN,
            MOTOR_VOLTAGE_A_PIN,
            MOTOR_VOLTAGE_B_PIN,
            motor_voltage_reversed,
        )
        self._helipad_manager: HelipadManager = HelipadManager(
            self,
            HELIPAD_IR_PIN,
            HELIPAD_LED_PAIR_A_PIN,
            HELIPAD_LED_PAIR_B_PIN,
        )
        self._station_input: StationInput = StationInput(
            self,
            self._station_manager,
            self._checkerboard_slowdown,
            self._park_mode,
        )
        self._wol_manager_input: Optional[WolManager] = WolManager(self, INPUT_HOSTNAME)
        self._wol_manager_vision: Optional[WolManager] = WolManager(
            self, VISION_HOSTNAME
        )

        # Reliable QOS profile for state topics
        state_qos_profile: rclpy.qos.QoSProfile = rclpy.qos.QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self._conductor_state_pub: rclpy.publisher.Publisher[ConductorStateMsg] = (
            self.create_publisher(
                msg_type=ConductorStateMsg,
                topic=PUBLISH_CONDUCTOR_STATE,
                qos_profile=state_qos_profile,
            )
        )

        # Subscriptions
        self._checkerboard_status_sub: rclpy.subscription.Subscription[BoolMsg] = (
            self.create_subscription(
                msg_type=BoolMsg,
                topic=SUBSCRIBE_CHECKERBOARD_STATUS,
                callback=self._on_checkerboard_status,
                qos_profile=rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value,
            )
        )

        # Timer parameters
        self._publish_timer: Optional[rclpy.node.Timer] = None

    def initialize(self) -> bool:
        # Initialize WoL managers
        wol_manager_input = self._wol_manager_input

        if wol_manager_input is not None and not wol_manager_input.initialize(
            WOL_TIMEOUT_SECS
        ):
            self._wol_manager_input = None

        wol_manager_vision = self._wol_manager_vision
        if wol_manager_vision is not None and not wol_manager_vision.initialize(
            WOL_TIMEOUT_SECS
        ):
            self._wol_manager_vision = None

        self.get_logger().debug("Starting conductor configuration")

        # Memory reporting
        if not self._mcu_memory_manager.initialize(REPORT_MCU_MEMORY_PERIOD_SECS):
            return False

        # Sampling interval
        if not self._sampling_manager.initialize(SAMPLING_INTERVAL_MS):
            return False

        # Initialize LEGO model control
        if not self._station_manager.initialize():
            return False

        if not self._helipad_manager.initialize():
            return False

        # Initialize input
        if not self._station_input.initialize():
            return False

        # Now that the station manager is initialized, start the publishing timer
        self._publish_timer = self.create_timer(
            timer_period_sec=PUBLISH_STATE_PERIOD_SECS, callback=self._publish_state
        )

        # Wake the input host
        if self._wol_manager_input is not None:
            self._wol_manager_input.send_wol()

        # Wake the vision host
        if self._wol_manager_vision is not None:
            self._wol_manager_vision.send_wol()

        self.get_logger().info("Conductor manager initialized successfully")

        return True

    def _publish_state(self) -> None:
        now_sec: float = self._now_sec()
        if self._station_input.update_checkerboard_slowdown(now_sec):
            self.get_logger().info(
                "Checkerboard slowdown expired; waiting for train trailing edge"
            )

        header = HeaderMsg()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = NODE_NAME  # TODO

        msg: ConductorStateMsg = ConductorStateMsg()
        msg.header = header
        msg.supply_voltage = self._station_manager.supply_voltage
        msg.supply_voltage_stddev = self._station_manager.supply_voltage_stddev
        msg.motor_voltage = self._station_manager.motor_voltage
        msg.motor_voltage_stddev = self._station_manager.motor_voltage_stddev
        msg.duty_cycle = self._station_manager.motor_duty_cycle
        msg.motor_current = self._station_manager.motor_current
        msg.motor_ff1_count = self._station_manager.motor_ff1_count
        msg.motor_ff2_count = self._station_manager.motor_ff2_count
        msg.cpu_fan_speed_rpm = 0.0  # Not implemented
        msg.total_ram = self._mcu_memory_manager.total_ram
        msg.ram_utilization = self._mcu_memory_manager.ram_utilization

        self._conductor_state_pub.publish(msg)

    def _on_checkerboard_status(self, checkerboard_status_msg: BoolMsg) -> None:
        now_sec: float = self._now_sec()
        was_initialized: bool = (
            self._checkerboard_slowdown.checkerboard_visible is not None
        )
        checkerboard_visible: bool = bool(checkerboard_status_msg.data)

        interrupted: bool = self._station_input.update_checkerboard_status(
            checkerboard_visible,
            now_sec,
        )

        if not was_initialized:
            self.get_logger().info(
                "Checkerboard status initialized: "
                f"{'visible' if checkerboard_visible else 'interrupted'}"
            )

        if interrupted:
            remaining_sec: float = (
                self._station_input.checkerboard_slowdown_remaining_sec(now_sec)
            )
            self.get_logger().info(
                "Checkerboard leading edge detected; "
                f"slowdown started for {remaining_sec:.1f}s"
            )

        if self._station_input.consume_checkerboard_slowdown_activation(now_sec):
            self.get_logger().info("Checkerboard slowdown active")

        if (
            self._station_input.consume_checkerboard_waiting_for_clear_activation(
                now_sec
            )
            and self._checkerboard_slowdown_duration_sec == 0.0
        ):
            self.get_logger().info(
                "Checkerboard slowdown expired; waiting for train trailing edge"
            )

        if self._station_input.consume_checkerboard_rearmed(now_sec):
            self.get_logger().info(
                "Checkerboard trailing edge detected; slowdown re-armed"
            )

    def _now_sec(self) -> float:
        now = self.get_clock().now()
        nanoseconds: Optional[int] = getattr(now, "nanoseconds", None)
        if nanoseconds is not None:
            return float(nanoseconds) * 1.0e-9

        stamp = now.to_msg()
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _get_nonnegative_parameter(
        self,
        parameter_name: str,
        default_value: float,
    ) -> float:
        value: float = float(self.get_parameter(parameter_name).value)
        if value < 0.0:
            self.get_logger().warning(
                f"{parameter_name} must be >= 0.0; using {default_value}"
            )
            return default_value

        return value

    def _get_unit_scale_parameter(
        self,
        parameter_name: str,
        default_value: float,
    ) -> float:
        value: float = float(self.get_parameter(parameter_name).value)
        if value <= 0.0 or value > 1.0:
            self.get_logger().warning(
                f"{parameter_name} must be in (0.0, 1.0]; using {default_value}"
            )
            return default_value

        return value
