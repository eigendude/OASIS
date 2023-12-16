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
# Manager for a mcu that reports string messages
#

from typing import Optional

import rclpy.client
import rclpy.node
import rclpy.qos
import rclpy.subscription

from oasis_msgs.msg import MCUString as MCUStringMsg


# from std_msgs.msg import Header as HeaderMsg


################################################################################
# ROS parameters
################################################################################


# Subscribers
SUBSCRIBE_MCU_STRING = "mcu_string"


################################################################################
# Manager
################################################################################


class McuStringManager:
    """
    A manager for reporting MCU string messages
    """

    def __init__(self, node: rclpy.node.Node) -> None:
        """
        Initialize resources.
        """
        # Initialize manager state
        self._node: rclpy.node.Node = node

        # Initialize hardware state
        self._message: Optional[str] = None

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSPresetProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Subscribers
        self._mcu_string_sub: rclpy.subscription.Subscription = (
            self._node.create_subscription(
                msg_type=MCUStringMsg,
                topic=SUBSCRIBE_MCU_STRING,
                callback=self._on_mcu_string,
                qos_profile=qos_profile,
            )
        )

    @property
    def message(self) -> Optional[str]:
        return self._message

    def initialize(self) -> bool:
        self._node.get_logger().info("MCU string manager initialized successfully")
        return True

    def _on_mcu_string(self, mcu_string_msg: MCUStringMsg) -> None:
        # Translate parameters
        message: str = mcu_string_msg.message

        # Log message
        self._node.get_logger().debug(f"MCU: {message}")

        # Record state
        self._message = message
