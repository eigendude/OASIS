################################################################################
#
#  Copyright (C) 2022-2023 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manager for a mcu that controls and senses 4-wire CPU fans
#

import asyncio

import rclpy.client
import rclpy.node
import rclpy.qos
import rclpy.subscription

from oasis_drivers.ros.ros_translator import RosTranslator
from oasis_drivers.telemetrix.telemetrix_types import DigitalMode
from oasis_msgs.msg import CPUFanSpeed as CPUFanSpeedMsg
from oasis_msgs.srv import PWMWrite as PWMWriteSvc
from oasis_msgs.srv import SetDigitalMode as SetDigitalModeSvc
from oasis_msgs.srv import SetSamplingInterval as SetSamplingIntervalSvc


################################################################################
# ROS parameters
################################################################################


# Subscribers
SUBSCRIBE_CPU_FAN_SPEED = "cpu_fan_speed"

# Service clients
CLIENT_CPU_FAN_WRITE = "cpu_fan_write"
CLIENT_SET_CPU_FAN_SAMPLING_INTERVAL = "set_cpu_fan_sampling_interval"
CLIENT_SET_DIGITAL_MODE = "set_digital_mode"


################################################################################
# ROS node
################################################################################


class CPUFanManager:
    """
    A manager for 4-wire CPU fans.
    """

    def __init__(self, node: rclpy.node.Node, pwm_pin: int, speed_pin: int) -> None:
        """
        Initialize resources.
        """
        # Initialize ROS parameters
        self._node: rclpy.node.Node = node
        self._logger = node.get_logger()

        # Initialize hardware config
        self._pwm_pin: int = pwm_pin
        self._speed_pin: int = speed_pin

        # Initialize hardware state
        self._cpu_fan_speed_rpm: float = 0

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

        # Service clients
        self._cpu_fan_write_client: rclpy.client.Client = self._node.create_client(
            srv_type=PWMWriteSvc, srv_name=CLIENT_CPU_FAN_WRITE
        )
        self._set_cpu_fan_sampling_interval_client: rclpy.client.Client = (
            self._node.create_client(
                srv_type=SetSamplingIntervalSvc,
                srv_name=CLIENT_SET_CPU_FAN_SAMPLING_INTERVAL,
            )
        )
        self._set_digital_mode_client: rclpy.client.Client = self._node.create_client(
            srv_type=SetDigitalModeSvc, srv_name=CLIENT_SET_DIGITAL_MODE
        )

    def initialize(self, sampling_interval_ms: int) -> bool:
        self._logger.debug("Waiting for CPU fan services...")
        self._cpu_fan_write_client.wait_for_service()
        self._set_cpu_fan_sampling_interval_client.wait_for_service()
        self._set_digital_mode_client.wait_for_service()

        self._logger.debug("Starting CPU fan configuration")

        # CPU fan PWM
        self._logger.debug(f"Enabling CPU fan PWM on D{self._pwm_pin}")
        if not self._set_digital_mode(self._pwm_pin, DigitalMode.CPU_FAN_PWM):
            return False

        # CPU fan tachometer
        self._logger.debug(f"Enabling CPU fan tachometer on D{self._speed_pin}")
        if not self._set_digital_mode(self._speed_pin, DigitalMode.CPU_FAN_TACHOMETER):
            return False

        # CPU fan sampling interval
        self._logger.debug(
            f"Setting CPU fan sampling interval to {sampling_interval_ms}ms"
        )
        if not self._set_cpu_fan_sampling_interval(sampling_interval_ms):
            return False

        self._logger.info("CPU fan manager initialized successfully")

        return True

    @property
    def cpu_fan_rpm(self) -> float:
        return self._cpu_fan_speed_rpm

    def write(self, digital_pin: int, magnitude: float) -> None:
        # Create message
        pwm_write_svc = PWMWriteSvc.Request()
        pwm_write_svc.digital_pin = digital_pin
        pwm_write_svc.duty_cycle = magnitude

        # Call service
        future: asyncio.Future = self._cpu_fan_write_client.call_async(pwm_write_svc)

        # Wait for result
        future.result()

    def _set_digital_mode(self, digital_pin: int, digital_mode: DigitalMode) -> bool:
        # Create message
        set_digital_mode_svc = SetDigitalModeSvc.Request()
        set_digital_mode_svc.digital_pin = digital_pin
        set_digital_mode_svc.digital_mode = RosTranslator.digital_mode_to_ros(
            digital_mode
        )

        # Call service
        future: asyncio.Future = self._set_digital_mode_client.call_async(
            set_digital_mode_svc
        )

        # Wait for result
        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is None:
            self._logger.error(f"Exception while calling service: {future.exception()}")
            return False

        return True

    def _set_cpu_fan_sampling_interval(self, sampling_interval_ms: int) -> bool:
        # Create message
        set_sampling_interval_svc = SetSamplingIntervalSvc.Request()
        set_sampling_interval_svc.sampling_interval_ms = sampling_interval_ms

        # Call service
        future: asyncio.Future = self._set_digital_mode_client.call_async(
            set_sampling_interval_svc
        )

        # Wait for result
        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is None:
            self._logger.error(f"Exception while calling service: {future.exception()}")
            return False

        return True

    def _on_cpu_fan_speed(self, cpu_fan_speed_msg: CPUFanSpeedMsg) -> None:
        # Translate parameters
        digital_pin: int = cpu_fan_speed_msg.digital_pin
        fan_speed_rpm: float = cpu_fan_speed_msg.fan_speed_rpm

        if digital_pin == self._pwm_pin:
            # Record state
            self._cpu_fan_speed_rpm = fan_speed_rpm
