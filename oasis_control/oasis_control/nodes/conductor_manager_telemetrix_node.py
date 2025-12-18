################################################################################
#
#  Copyright (C) 2022-2025 Garrett Brown
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
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Header as HeaderMsg

from oasis_control.input.station_input import StationInput
from oasis_control.lego_models.station_manager import StationManager
from oasis_control.managers.cpu_fan_manager import CPUFanManager
from oasis_control.managers.sampling_manager import SamplingManager
from oasis_control.managers.wol_manager import WolManager
from oasis_control.mcu.mcu_memory_manager import McuMemoryManager
from oasis_msgs.msg import ConductorState as ConductorStateMsg


################################################################################
# Hardware configuration
################################################################################


# CPU fan sampling interval, in ms
CPU_FAN_SAMPLING_INTERVAL_MS = 100

# Memory reporting interval, in seconds
REPORT_MCU_MEMORY_PERIOD_SECS: float = 1.0

# Sampling interval, in ms
SAMPLING_INTERVAL_MS = 100

# Pin configuration
VSS_PIN: int = 0  # A0
MOTOR_PWM_PIN: int = 5  # D5
MOTOR_DIR_PIN: int = 4  # D4
MOTOR_FF1_PIN: int = 8  # D8
MOTOR_FF2_PIN: int = 7  # D7
MOTOR_CURRENT_PIN: int = 1  # A1
CPU_FAN_PWM_PIN: int = 9  # D9
CPU_FAN_SPEED_PIN: int = 2  # D2

# Voltage dividers
# R1 is the input-side resistor, R2 is the ground-side resistor
VSS_R1: float = 26.62  # KΩ
VSS_R2: float = 9.83  # KΩ


################################################################################
# Automation parameters
################################################################################


# The hostname or IP of the computer that provides input
WOL_HOSTNAME: str = "megapegasus.local"

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

# Services
SERVICE_POWER_CONTROL = "power_control"


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

        # Subsystems
        self._cpu_fan_manager: CPUFanManager = CPUFanManager(
            self, CPU_FAN_PWM_PIN, CPU_FAN_SPEED_PIN
        )
        self._mcu_memory_manager: McuMemoryManager = McuMemoryManager(self)
        self._sampling_manager: SamplingManager = SamplingManager(self)
        self._station_manager: StationManager = StationManager(self)
        self._station_input: StationInput = StationInput(self, self._station_manager)
        self._wol_manager: Optional[WolManager] = WolManager(self, WOL_HOSTNAME)

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Publishers
        self._conductor_state_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=ConductorStateMsg,
            topic=PUBLISH_CONDUCTOR_STATE,
            qos_profile=qos_profile,
        )

        # Timer parameters
        self._publish_timer: Optional[rclpy.node.Timer] = None

    def initialize(self) -> bool:
        # Initialize WoL manager
        wol_manager = self._wol_manager
        if wol_manager is not None and not wol_manager.initialize(WOL_TIMEOUT_SECS):
            self._wol_manager = None

        self.get_logger().debug("Starting conductor configuration")

        # CPU fan
        if not self._cpu_fan_manager.initialize(CPU_FAN_SAMPLING_INTERVAL_MS):
            return False

        # Turn on fan
        self._cpu_fan_manager.write(CPU_FAN_PWM_PIN, 1.0)

        # Memory reporting
        if not self._mcu_memory_manager.initialize(REPORT_MCU_MEMORY_PERIOD_SECS):
            return False

        # Sampling interval
        if not self._sampling_manager.initialize(SAMPLING_INTERVAL_MS):
            return False

        # Initialize LEGO model control
        if not self._station_manager.initialize():
            return False

        # Initialize input
        if not self._station_input.initialize():
            return False

        # Now that the station manager is initialized, start the publishing timer
        self._publish_timer = self.create_timer(
            timer_period_sec=PUBLISH_STATE_PERIOD_SECS, callback=self._publish_state
        )

        # Wake the input host
        if self._wol_manager is not None:
            self._wol_manager.send_wol()

        self.get_logger().info("Conductor manager initialized successfully")

        return True

    def _publish_state(self) -> None:
        header = HeaderMsg()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = NODE_NAME  # TODO

        msg: ConductorStateMsg = ConductorStateMsg()
        msg.header = header
        msg.supply_voltage = self._station_manager.supply_voltage
        msg.motor_voltage = self._station_manager.motor_voltage
        msg.motor_current = self._station_manager.motor_current
        msg.motor_ff1_count = self._station_manager.motor_ff1_count
        msg.motor_ff2_count = self._station_manager.motor_ff2_count
        msg.cpu_fan_speed_rpm = self._cpu_fan_manager.cpu_fan_rpm
        msg.total_ram = self._mcu_memory_manager.total_ram
        msg.ram_utilization = self._mcu_memory_manager.ram_utilization

        self._conductor_state_pub.publish(msg)
