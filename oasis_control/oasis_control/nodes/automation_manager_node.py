#!/usr/bin/env python3
################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Automation manager
#

import random  # TODO: Remove me
from typing import Optional

import rclpy.client
import rclpy.node
import rclpy.qos
import rclpy.subscription
import rclpy.task
from rclpy.logging import LoggingSeverity

from oasis_msgs.msg import PowerEvent as PowerEventMsg
from oasis_msgs.srv import PowerControl as PowerControlSvc


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE = "oasis"

NODE_NAME = (
    f"automation_manager_{random.randrange(1000, 9999)}"  # TODO: Better node naming
)

POWER_EVENT_TOPIC = "power_event"

POWER_CONTROL_SERVICE = "power_control"

################################################################################
# ROS node
################################################################################


class AutomationManagerNode(rclpy.node.Node):
    """
    A ROS node that monitors process and sensor information using psutil.
    """

    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Enable debug logging
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSPresetProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Subscribers
        self._power_event_subscription: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=PowerEventMsg,
                topic=POWER_EVENT_TOPIC,
                callback=self._on_power_event,
                qos_profile=qos_profile,
            )
        )

        # Service clients
        self._power_control_client: rclpy.client.Client = self.create_client(
            srv_type=PowerControlSvc, srv_name=POWER_CONTROL_SERVICE
        )
        self._in_flight_call: Optional[rclpy.task.Future] = None

        self.get_logger().info("Automation manager initialized")

    def _on_power_event(self, power_event_msg) -> None:
        power_mode: str = power_event_msg.power_mode

        request = PowerControlSvc.Request()
        request.device = ""  # TODO
        request.power_mode = power_mode

        self.get_logger().debug(
            f'Received power {power_mode} event for device "{power_event_msg.device}"'
        )

        service: str = self._power_control_client.srv_name

        # Cancel previous service call if one is in-flight
        if self._in_flight_call is not None:
            self.get_logger().debug(f"Cancelling previous power command for {service}")
            self._in_flight_call.cancel()

        self.get_logger().debug(f"Sending power {power_mode} for {service}")
        self._in_flight_call = self._power_control_client.call_async(request)
