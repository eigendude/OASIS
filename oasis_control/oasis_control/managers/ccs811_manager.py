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
# Manager for a mcu that controls a CCS811 air quality sensor over I2C
#

import rclpy.node
import rclpy.qos
import rclpy.subscription

from oasis_control.managers.i2c_device_manager import I2CDeviceManager
from oasis_msgs.msg import GasConcentration as GasConcentrationMsg
from oasis_msgs.msg import I2CDeviceType as I2CDeviceTypeMsg


################################################################################
# ROS parameters
################################################################################


# Subscribers
SUBSCRIBE_EQUIVALENT_CO2 = "equivalent_co2"
SUBSCRIBE_TVOC = "tvoc"


# Parts per billion per part per million (10^-6 / 10^-9)
PPB_PER_PPM: float = 1000.0


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
        self._equivalent_co2_ppm: float = 0.0
        self._tvoc_ppm: float = 0.0

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Subscribers
        self._equivalent_co2_sub: rclpy.subscription.Subscription[
            GasConcentrationMsg
        ] = self._node.create_subscription(
            msg_type=GasConcentrationMsg,
            topic=SUBSCRIBE_EQUIVALENT_CO2,
            callback=self._on_equivalent_co2,
            qos_profile=qos_profile,
        )
        self._tvoc_sub: rclpy.subscription.Subscription[GasConcentrationMsg] = (
            self._node.create_subscription(
                msg_type=GasConcentrationMsg,
                topic=SUBSCRIBE_TVOC,
                callback=self._on_tvoc,
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
    def equivalent_co2_ppm(self) -> float:
        return self._equivalent_co2_ppm

    @property
    def tvoc_ppm(self) -> float:
        return self._tvoc_ppm

    @property
    def tvoc_ppb(self) -> float:
        return self._tvoc_ppm * PPB_PER_PPM

    def _on_equivalent_co2(self, equivalent_co2_msg: GasConcentrationMsg) -> None:
        self._equivalent_co2_ppm = equivalent_co2_msg.concentration_ppm

    def _on_tvoc(self, tvoc_msg: GasConcentrationMsg) -> None:
        self._tvoc_ppm = tvoc_msg.concentration_ppm
