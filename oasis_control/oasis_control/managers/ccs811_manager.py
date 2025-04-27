################################################################################
#
#  Copyright (C) 2022-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manager for a mcu that controls a CCS811 air quality sensor over I2C
#

from typing import List

import rclpy.node
import rclpy.qos
import rclpy.subscription

from oasis_control.managers.i2c_device_manager import I2CDeviceManager
from oasis_msgs.msg import AirQuality as AirQualityMsg
from oasis_msgs.msg import I2CDevice as I2CDeviceMsg
from oasis_msgs.msg import I2CDeviceType as I2CDeviceTypeMsg


################################################################################
# ROS parameters
################################################################################


# Subscribers
SUBSCRIBE_AIR_QUALITY = "air_quality"


################################################################################
# ROS node
################################################################################


class CCS811Manager(I2CDeviceManager):
    """
    A manager for a CCS811 air quality sensor connected to a microcontroller
    """

    def __init__(self, node: rclpy.node.Node, i2c_port: int, i2c_address: int) -> None:
        """
        Initialize resources.
        """
        super().__init__(node, i2c_port, i2c_address)

        # Initialize ROS parameters
        self._node: rclpy.node.Node = node
        self._logger = node.get_logger()

        # Initialize hardware state
        self._co2_ppb: float = 0.0
        self._tvoc_ppb: float = 0.0

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSPresetProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Subscribers
        self._air_quality_sub: rclpy.subscription.Subscription = (
            self._node.create_subscription(
                msg_type=AirQualityMsg,
                topic=SUBSCRIBE_AIR_QUALITY,
                callback=self._on_air_quality,
                qos_profile=qos_profile,
            )
        )

    def initialize(self) -> bool:
        if not super().initialize():
            return False

        self._logger.debug("Starting CCS811 configuration")

        # Set up I2C device
        self._logger.debug(
            f"Enabling CCS811 on I2C{self.i2c_port + 1} at address {hex(self.i2c_address)}"
        )
        if not self.i2c_begin_device(
            I2CDeviceTypeMsg.CCS811, self.i2c_port, self.i2c_address
        ):
            return False

        self._logger.info("CCS811 manager initialized successfully")

        return True

    @property
    def co2_ppb(self) -> float:
        return self._co2_ppb

    @property
    def tvoc_ppb(self) -> float:
        return self._tvoc_ppb

    def _on_air_quality(self, air_quality_msg: AirQualityMsg) -> None:
        # Translate parameters
        i2c_device: List[I2CDeviceMsg] = air_quality_msg.i2c_device
        co2_ppb: float = air_quality_msg.co2_ppb
        tvoc_ppb: float = air_quality_msg.tvoc_ppb

        if (
            len(i2c_device) > 0
            and i2c_device[0].i2c_port == self.i2c_port
            and i2c_device[0].i2c_address == self.i2c_address
        ):
            # Record state
            self._co2_ppb = co2_ppb
            self._tvoc_ppb = tvoc_ppb
