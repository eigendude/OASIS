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

import asyncio

import rclpy.client
import rclpy.node

from oasis_msgs.msg import I2CDevice as I2CDeviceMsg
from oasis_msgs.srv import I2CBegin as I2CBeginSvc
from oasis_msgs.srv import I2CEnd as I2CEndSvc


################################################################################
# ROS parameters
################################################################################


# Service clients
CLIENT_I2C_BEGIN = "i2c_begin"
CLIENT_I2C_END = "i2c_end"


################################################################################
# ROS node
################################################################################


class I2CDeviceManager:
    """
    A manager for an I2C device connected to a microcontroller
    """

    def __init__(self, node: rclpy.node.Node, i2c_port: int, i2c_address: int) -> None:
        """
        Initialize resources.
        """
        # Initialize ROS parameters
        self._node: rclpy.node.Node = node
        self._logger = node.get_logger()

        # Initialize hardware config
        self._i2c_port: int = i2c_port
        self._i2c_address: int = i2c_address

        # Service clients
        self._i2c_begin_client: rclpy.client.Client = self._node.create_client(
            srv_type=I2CBeginSvc, srv_name=CLIENT_I2C_BEGIN
        )
        self._i2c_end_client: rclpy.client.Client = self._node.create_client(
            srv_type=I2CEndSvc, srv_name=CLIENT_I2C_END
        )

    @property
    def i2c_port(self) -> int:
        return self._i2c_port

    @property
    def i2c_address(self) -> int:
        return self._i2c_address

    def initialize(self) -> bool:
        self._logger.debug("Waiting for I2C services")
        self._logger.debug("  - Waiting for i2c_begin...")
        self._i2c_begin_client.wait_for_service()
        self._logger.debug("  - Waiting for i2c_end...")
        self._i2c_end_client.wait_for_service()
        return True

    def i2c_begin_device(
        self, i2c_device_type: int, i2c_port: int, i2c_address: int
    ) -> bool:
        # Create message
        i2c_begin_svc = I2CBeginSvc.Request()
        i2c_begin_svc.i2c_port = i2c_port

        i2c_device = I2CDeviceMsg()
        i2c_device.i2c_device_type = i2c_device_type
        i2c_device.i2c_port = i2c_port
        i2c_device.i2c_address = i2c_address

        i2c_begin_svc.i2c_devices.append(i2c_device)

        # Call service
        future: asyncio.Future = self._i2c_begin_client.call_async(i2c_begin_svc)

        # Wait for result
        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is None:
            self._logger.error(f"Exception while calling service: {future.exception()}")
            return False

        return True

    def i2c_end_device(
        self, i2c_device_type: int, i2c_port: int, i2c_address: int
    ) -> bool:
        # Create message
        i2c_end_svc = I2CEndSvc.Request()
        i2c_end_svc.i2c_port = i2c_port

        i2c_device = I2CDeviceMsg()
        i2c_device.i2c_device_type = i2c_device_type
        i2c_device.i2c_port = i2c_port
        i2c_device.i2c_address = i2c_address

        i2c_end_svc.i2c_devices.append(i2c_device)

        # Call service
        future: asyncio.Future = self._i2c_end_client.call_async(i2c_end_svc)

        # Wait for result
        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is None:
            self._logger.error(f"Exception while calling service: {future.exception()}")
            return False

        return True
