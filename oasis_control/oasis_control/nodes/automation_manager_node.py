################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Automation manager
#

from typing import Optional

import rclpy.client
import rclpy.node
import rclpy.qos
import rclpy.subscription
import rclpy.task
from rclpy.logging import LoggingSeverity

from oasis_msgs.msg import PowerEvent as PowerEventMsg
from oasis_msgs.msg import PowerMode as PowerModeMsg
from oasis_msgs.srv import PowerControl as PowerControlSvc
from oasis_msgs.srv import SetDisplay as SetDisplaySvc


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE = "oasis"

NODE_NAME = "automation_manager"

POWER_EVENT_TOPIC = "power_event"

POWER_CONTROL_SERVICE = "power_control"
SET_DISPLAY_SERVICE = "set_display"


################################################################################
# ROS node
################################################################################


class AutomationManagerNode(rclpy.node.Node):
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
        self._power_control_in_flight: Optional[rclpy.task.Future] = None
        self._set_display_client: rclpy.client.Client = self.create_client(
            srv_type=SetDisplaySvc, srv_name=SET_DISPLAY_SERVICE
        )
        self._set_display_in_flight: Optional[rclpy.task.Future] = None

        self.get_logger().info("Automation manager initialized")

    def _on_power_event(self, power_event_msg) -> None:
        power_mode: str = power_event_msg.power_mode

        self.get_logger().debug(
            f'Received power {power_mode} event for device "{power_event_msg.header.frame_id}"'
        )

        # Set display power mode
        display_request: SetDisplaySvc = SetDisplaySvc.Request()
        display_request.dpms_mode = power_mode
        display_request.brightness = 100 if power_mode == PowerModeMsg.ON else 0

        display_service: str = self._set_display_client.srv_name

        # Cancel previous service call if one is in-flight
        if self._set_display_in_flight is not None:
            self.get_logger().debug(
                f"Cancelling previous display command for {display_service}"
            )
            self._set_display_in_flight.cancel()

        self.get_logger().debug(f"Sending display {power_mode} for {display_service}")
        self._set_display_in_flight = self._set_display_client.call_async(
            display_request
        )

        # Power control
        """ TODO: Disabled
        request: PowerControlSvc = PowerControlSvc.Request()
        request.device = power_event_msg.header.frame_id
        request.power_mode = power_mode

        service: str = self._power_control_client.srv_name

        # Cancel previous service call if one is in-flight
        if self._power_control_in_flight is not None:
            self.get_logger().debug(f"Cancelling previous power command for {service}")
            self._power_control_in_flight.cancel()

        self.get_logger().debug(f"Sending power {power_mode} for {service}")
        self._power_control_in_flight = self._power_control_client.call_async(request)
        """
