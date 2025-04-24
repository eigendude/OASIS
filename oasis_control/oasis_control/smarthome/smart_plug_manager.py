################################################################################
#
#  Copyright (C) 2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manager that controls and senses 4-wire CPU fans
#

from typing import Optional

import rclpy.node

from oasis_control.managers.cpu_fan_manager import CPUFanManager


################################################################################
# ROS parameters
################################################################################


# Subscribers
# SUBSCRIBE_CPU_FAN_SPEED = "cpu_fan_speed"  # TODO: CPU speed sensor is broke


################################################################################
# Hardware configuration
################################################################################


# CPU fan sampling interval, in ms
CPU_FAN_SAMPLING_INTERVAL_MS = 100

# Pins
CPU_FAN_PWM_PIN: int = 9  # D9
CPU_FAN_SPEED_PIN: int = 2  # D2


################################################################################
# ROS node
################################################################################


class CPUThermalManager:
    """
    A manager for themal sensors and 4-wire CPU fans.
    """

    def __init__(
        self, node: rclpy.node.Node, cpu_fan_host: Optional[str] = None
    ) -> None:
        """
        Initialize resources.
        """
        # Initialize ROS parameters
        self._node: rclpy.node.Node = node
        self._logger = node.get_logger()

        # Subsystems
        self._cpu_fan_manager: CPUFanManager = CPUFanManager(
            node=self._node,
            pwm_pin=CPU_FAN_PWM_PIN,
            speed_pin=CPU_FAN_SPEED_PIN,
            cpu_fan_host=cpu_fan_host,
        )

        """ TODO
        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSPresetProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Subscribers
        self._cpu_fan_speed_sub: rclpy.subscription.Subscription = (
            self._node.create_subscription(
                msg_type=CPUFanSpeedMsg,
                topic=SUBSCRIBE_CPU_FAN_SPEED,
                callback=self._on_cpu_fan_speed,
                qos_profile=qos_profile,
            )
        )
        """

    def initialize(self) -> bool:
        self._logger.debug("Starting CPU thermal manager configuration")

        #
        # CPU fan
        #

        # Initialize subsystem
        if not self._cpu_fan_manager.initialize(CPU_FAN_SAMPLING_INTERVAL_MS):
            return False

        # Set speed to max
        self._cpu_fan_manager.write(CPU_FAN_PWM_PIN, 1.0)

        self._logger.info("CPU thermal manager initialized successfully")

        return True

    @property
    def cpu_fan_rpm(self) -> float:
        return self._cpu_fan_manager.cpu_fan_rpm
