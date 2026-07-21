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

import math
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

from oasis_control.input.park_mode import ParkModeLaunchProfile
from oasis_control.input.park_mode import TrainParkMode
from oasis_control.input.station_input import StationInput
from oasis_control.lego_models.helipad_manager import HelipadManager
from oasis_control.lego_models.station_manager import StationManager
from oasis_control.managers.sampling_manager import SamplingManager
from oasis_control.managers.wol_manager import WolManager
from oasis_control.mcu.mcu_memory_manager import McuMemoryManager
from oasis_msgs.msg import CameraScene as CameraSceneMsg
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
INPUT_STATE_PERIOD_SECS = 0.05

# Publisher
PUBLISH_CONDUCTOR_STATE = "conductor_state"

# Subscribers
SUBSCRIBE_CAMERA_SCENE = "camera_scene"
SUBSCRIBE_CHECKERBOARD_STATUS = "checkerboard_status"

# Parameters
PARAM_MOTOR_VOLTAGE_REVERSED: str = "motor_voltage_reversed"
DEFAULT_MOTOR_VOLTAGE_REVERSED: bool = False
PARAM_PARK_MODE_ENABLED: str = "park_mode_enabled"
DEFAULT_PARK_MODE_ENABLED: bool = True
PARAM_PARK_MODE_PRELOAD_COMMAND: str = "park_mode_preload_command"
DEFAULT_PARK_MODE_PRELOAD_COMMAND: float = 0.55
PARAM_PARK_MODE_PRELOAD_SEC: str = "park_mode_preload_sec"
DEFAULT_PARK_MODE_PRELOAD_SEC: float = 0.2
PARAM_PARK_MODE_TAKEUP_COMMAND: str = "park_mode_takeup_command"
DEFAULT_PARK_MODE_TAKEUP_COMMAND: float = 0.56
PARAM_PARK_MODE_TAKEUP_SEC: str = "park_mode_takeup_sec"
DEFAULT_PARK_MODE_TAKEUP_SEC: float = 0.2
PARAM_PARK_MODE_HOLD_SEC: str = "park_mode_hold_sec"
DEFAULT_PARK_MODE_HOLD_SEC: float = 0.2
PARAM_PARK_MODE_COMMAND: str = "park_mode_command"
DEFAULT_PARK_MODE_COMMAND: float = 0.9
PARAM_PARK_MODE_ACCEL_SEC: str = "park_mode_accel_sec"
DEFAULT_PARK_MODE_ACCEL_SEC: float = 2.5

DEFAULT_PARK_MODE_PROFILE: ParkModeLaunchProfile = ParkModeLaunchProfile(
    preload_command=DEFAULT_PARK_MODE_PRELOAD_COMMAND,
    preload_sec=DEFAULT_PARK_MODE_PRELOAD_SEC,
    takeup_command=DEFAULT_PARK_MODE_TAKEUP_COMMAND,
    takeup_sec=DEFAULT_PARK_MODE_TAKEUP_SEC,
    hold_sec=DEFAULT_PARK_MODE_HOLD_SEC,
    command=DEFAULT_PARK_MODE_COMMAND,
    accel_sec=DEFAULT_PARK_MODE_ACCEL_SEC,
)


def _is_valid_park_mode_profile(profile: ParkModeLaunchProfile) -> bool:
    commands: tuple[float, ...] = (
        profile.preload_command,
        profile.takeup_command,
        profile.command,
    )
    durations: tuple[float, ...] = (
        profile.preload_sec,
        profile.takeup_sec,
        profile.accel_sec,
    )

    commands_valid: bool = all(
        math.isfinite(command) and 0.0 < command <= 1.0 for command in commands
    )
    durations_valid: bool = all(
        math.isfinite(duration) and duration > 0.0 for duration in durations
    )
    hold_valid: bool = math.isfinite(profile.hold_sec) and profile.hold_sec >= 0.0
    commands_ordered: bool = (
        profile.preload_command < profile.takeup_command < profile.command
    )

    return commands_valid and durations_valid and hold_valid and commands_ordered


def _validated_park_mode_profile(
    profile: ParkModeLaunchProfile,
) -> ParkModeLaunchProfile:
    if _is_valid_park_mode_profile(profile):
        return profile

    return DEFAULT_PARK_MODE_PROFILE


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
            PARAM_PARK_MODE_ENABLED,
            DEFAULT_PARK_MODE_ENABLED,
        )
        self.declare_parameter(
            PARAM_PARK_MODE_PRELOAD_COMMAND,
            DEFAULT_PARK_MODE_PRELOAD_COMMAND,
        )
        self.declare_parameter(
            PARAM_PARK_MODE_PRELOAD_SEC,
            DEFAULT_PARK_MODE_PRELOAD_SEC,
        )
        self.declare_parameter(
            PARAM_PARK_MODE_TAKEUP_COMMAND,
            DEFAULT_PARK_MODE_TAKEUP_COMMAND,
        )
        self.declare_parameter(
            PARAM_PARK_MODE_TAKEUP_SEC,
            DEFAULT_PARK_MODE_TAKEUP_SEC,
        )
        self.declare_parameter(
            PARAM_PARK_MODE_HOLD_SEC,
            DEFAULT_PARK_MODE_HOLD_SEC,
        )
        self.declare_parameter(
            PARAM_PARK_MODE_COMMAND,
            DEFAULT_PARK_MODE_COMMAND,
        )
        self.declare_parameter(
            PARAM_PARK_MODE_ACCEL_SEC,
            DEFAULT_PARK_MODE_ACCEL_SEC,
        )
        motor_voltage_reversed: bool = bool(
            self.get_parameter(PARAM_MOTOR_VOLTAGE_REVERSED).value
        )
        park_mode_enabled: bool = bool(
            self.get_parameter(PARAM_PARK_MODE_ENABLED).value
        )

        park_mode_profile: ParkModeLaunchProfile = ParkModeLaunchProfile(
            preload_command=float(
                self.get_parameter(PARAM_PARK_MODE_PRELOAD_COMMAND).value
            ),
            preload_sec=float(self.get_parameter(PARAM_PARK_MODE_PRELOAD_SEC).value),
            takeup_command=float(
                self.get_parameter(PARAM_PARK_MODE_TAKEUP_COMMAND).value
            ),
            takeup_sec=float(self.get_parameter(PARAM_PARK_MODE_TAKEUP_SEC).value),
            hold_sec=float(self.get_parameter(PARAM_PARK_MODE_HOLD_SEC).value),
            command=float(self.get_parameter(PARAM_PARK_MODE_COMMAND).value),
            accel_sec=float(self.get_parameter(PARAM_PARK_MODE_ACCEL_SEC).value),
        )
        validated_profile: ParkModeLaunchProfile = _validated_park_mode_profile(
            park_mode_profile
        )
        if validated_profile is not park_mode_profile:
            self.get_logger().warning(
                "Invalid park-mode profile: commands must be finite and in "
                "(0.0, 1.0] with preload < take-up < final, and durations "
                "must be finite and positive except hold, which may be zero; "
                "using complete default profile"
            )
            park_mode_profile = validated_profile
        self._park_mode: TrainParkMode = TrainParkMode(
            park_mode_enabled,
            park_mode_profile,
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
        self._camera_scene_sub: rclpy.subscription.Subscription[CameraSceneMsg] = (
            self.create_subscription(
                msg_type=CameraSceneMsg,
                topic=SUBSCRIBE_CAMERA_SCENE,
                callback=self._on_camera_scene,
                qos_profile=rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value,
            )
        )

        # Timer parameters
        self._publish_timer: Optional[rclpy.node.Timer] = None
        self._input_state_timer: Optional[rclpy.node.Timer] = None

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
        self._input_state_timer = self.create_timer(
            timer_period_sec=INPUT_STATE_PERIOD_SECS, callback=self._update_input_state
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

    def _update_input_state(self) -> None:
        self._station_input.update_autonomous_train_control()

    def _on_checkerboard_status(self, checkerboard_status_msg: BoolMsg) -> None:
        checkerboard_visible: bool = bool(checkerboard_status_msg.data)
        self._station_input.update_checkerboard_status(checkerboard_visible)

    def _on_camera_scene(self, camera_scene_msg: CameraSceneMsg) -> None:
        self._station_input.update_camera_scene(camera_scene_msg, self._now_sec())

    def _now_sec(self) -> float:
        now = self.get_clock().now()
        nanoseconds: Optional[int] = getattr(now, "nanoseconds", None)
        if nanoseconds is not None:
            return float(nanoseconds) * 1.0e-9

        stamp = now.to_msg()
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

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
