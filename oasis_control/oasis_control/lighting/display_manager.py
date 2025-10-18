################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manager that controls and senses 4-wire CPU fans
#

import rclpy.node
import rclpy.qos
import rclpy.task

from oasis_msgs.msg import Plug as PlugMsg
from oasis_msgs.msg import PowerMode as PowerModeMsg
from oasis_msgs.srv import SetDisplay as SetDisplaySvc


################################################################################
# ROS parameters
################################################################################


# ROS Topics
PLUG_TOPIC = "plug"

# ROS Services
SET_DISPLAY_SERVICE = "set_display"


################################################################################
# ROS node
################################################################################


class DisplayManager:
    """
    Manager that controls smart displays.
    """

    def __init__(
        self,
        node: rclpy.node.Node,
        smart_display_zones: list[str],
        smart_display_plug_id: str,
    ) -> None:
        """
        Initialize resources.
        """
        # Construction parameters
        self._smart_display_zones: list[str] = smart_display_zones
        self._smart_display_plug_id: str = smart_display_plug_id

        # Initialize ROS parameters
        self._node: rclpy.node.Node = node
        self._logger = node.get_logger()

        # Service clients
        self._service_clients: dict[str, rclpy.client.Client] = {}
        for display_host in self._smart_display_zones:
            set_display_service: str = f"{SET_DISPLAY_SERVICE}_{display_host}"
            set_display_client: rclpy.client.Client = self._node.create_client(
                srv_type=SetDisplaySvc,
                srv_name=set_display_service,
            )
            self._service_clients[display_host] = set_display_client
            self._logger.info(
                f'DisplayManager: Subscribing to service "{set_display_service}"'
            )
        self._set_display_in_flight: list[rclpy.task.Future] = []

        qos_profile: rclpy.qos.QoSPresetProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value  # Best-effort, keep-last
        )

        # Subscribers
        self._plug_sub: rclpy.subscription.Subscription = (
            self._node.create_subscription(
                msg_type=PlugMsg,
                topic=PLUG_TOPIC,
                callback=self._on_plug_status,
                qos_profile=qos_profile,
            )
        )

        self._logger.info("DisplayManager: Initialized")

    def _on_plug_status(self, plug_msg: PlugMsg) -> None:
        entity_id: str = plug_msg.header.frame_id
        power_mode: str = plug_msg.power_mode

        self._logger.debug(f'Received power {power_mode} event for plug "{entity_id}"')

        # Ignore messages from non-master plugs
        if entity_id != self._smart_display_plug_id:
            return

        # Cancel previous service calls if any are in-flight
        for set_display_cmd in self._set_display_in_flight:
            set_display_cmd.cancel()
        self._set_display_in_flight.clear()

        # Set display power mode
        for display_host in self._smart_display_zones:
            set_display_request: SetDisplaySvc.Request = SetDisplaySvc.Request()
            set_display_request.dpms_mode = power_mode
            set_display_request.brightness = 100 if power_mode == PowerModeMsg.ON else 0

            set_display_client: rclpy.client.Client = self._service_clients[
                display_host
            ]
            set_display_service: str = set_display_client.srv_name

            self._logger.debug(
                f"Sending display {power_mode} for {set_display_service}"
            )
            self._set_display_in_flight.append(
                set_display_client.call_async(set_display_request)
            )
