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
# Manager for RGB lighting
#

import rclpy.node
import rclpy.qos

from oasis_msgs.msg import RGB as RGBMsg
from oasis_msgs.msg import CameraScene as CameraSceneMsg
from oasis_msgs.srv import SetRGB as SetRGBSvc


################################################################################
# ROS parameters
################################################################################


# ROS Topics
CAMERA_SCENE_TOPIC: str = "camera_scene"
RGB_TOPIC: str = "rgb"

# ROS Services
SET_RGB_SERVICE: str = "set_rgb"


################################################################################
# Lighting parameters
################################################################################


LIGHT_FADE_TIME: float = 2.0


################################################################################
# Manager
################################################################################


class PresenceManager:
    """
    A manager for presence detection and lighting control.
    """

    def __init__(self, node: rclpy.node.Node) -> None:
        """
        Initialize resources.
        """
        # Initialize ROS parameters
        self._node: rclpy.node.Node = node
        self._logger = node.get_logger()

        # Lights we've observed
        self._light_ids: set[str] = set()

        # Occupancy state of the kitchen
        self._kitchen_occupied: bool = False

        # Service clients
        self._set_rgb_client: rclpy.client.Client = self._node.create_client(
            srv_type=SetRGBSvc,
            srv_name=SET_RGB_SERVICE,
        )

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Subscribers
        self._camera_scene_subs: list[rclpy.subscription.Subscription] = [
            self._node.create_subscription(
                msg_type=CameraSceneMsg,
                topic=f"{CAMERA_SCENE_TOPIC}_{camera_host}",
                callback=self._on_camera_scene,
                qos_profile=qos_profile,
            )
            for camera_host in [
                "kitchen"
            ]  # ["bar", "door", "hallway", "kitchen", "station"]
        ]
        self._rgb_sub: rclpy.subscription.Subscription = self._node.create_subscription(
            msg_type=RGBMsg,
            topic=RGB_TOPIC,
            callback=self._on_rgb_status,
            qos_profile=qos_profile,
        )

    def _on_rgb_status(self, msg: RGBMsg) -> None:
        """
        Callback for RGB status messages.
        """
        # Translate parameters
        entity_id: str = msg.header.frame_id

        # Remember the light ID
        self._light_ids.add(entity_id)

    def _on_camera_scene(self, msg: CameraSceneMsg) -> None:
        """
        Callback for camera scene messages.
        """
        # TODO: Get hostname for the message
        hostname: str = "kitchen"

        kitchen_occupied: bool = False
        if hostname == "kitchen":
            kitchen_occupied = len(msg.bounding_boxes) > 0

        # Check if the kitchen occupancy state has changed
        if kitchen_occupied != self._kitchen_occupied:
            # Update state
            self._kitchen_occupied = kitchen_occupied

            # Log the change in kitchen occupancy
            if kitchen_occupied:
                self._logger.debug("Kitchen is occupied")
            else:
                self._logger.debug("Kitchen is unoccupied")

            # Turn lights on or off based on kitchen occupancy
            for light_id in self._light_ids:
                light_req: SetRGBSvc.Request = SetRGBSvc.Request()
                light_req.light_id = light_id
                light_req.r, light_req.g, light_req.b = (
                    (1.0, 1.0, 1.0) if kitchen_occupied else (0.0, 0.0, 0.0)
                )
                light_req.transition = LIGHT_FADE_TIME
                self._set_rgb_client.call_async(light_req)
