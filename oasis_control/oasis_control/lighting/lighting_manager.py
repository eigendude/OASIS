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

import colorsys
from datetime import datetime
from datetime import timedelta

import rclpy.node
import rclpy.qos

from oasis_msgs.msg import RGB as RGBMsg
from oasis_msgs.srv import SetRGB as SetRGBSvc


################################################################################
# ROS parameters
################################################################################


# ROS Topics
RGB_TOPIC: str = "rgb"

# ROS Services
SET_RGB_SERVICE: str = "set_rgb"


################################################################################
# Lighting parameters
################################################################################


# Rate the lighting loop runs at
RGB_UPDATE_INTERVAL_SECS: float = 2.0

# How long to wait after a light is turned off before we start sending it
# updates again
RGB_CUTOFF_INTERVAL_SECS: float = 4.0

# How long to wait after a light is turned off before we stop sending it
# OFF comamnds
RGB_FORCE_OFF_INTERVAL_SECS: float = 3.0

# Time for a full rainbow cycle
RAINBOW_CYCLE_SECS: float = 25.0

# Amount of white mixed into the rainbow colors to increase lightness
RAINBOW_LIGHTNESS_BOOST: float = 0.3


################################################################################
# Manager
################################################################################


class LightingManager:
    """
    A manager for controlling RGB lighting.
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

        # Lights we think are on right now
        self._active_lights: set[str] = set()

        # When each light last went off
        self._last_turned_off: dict[str, datetime] = {}

        # Record when we started the rainbow cycle
        self._start_time: datetime = datetime.now()

        # Service clients
        self._set_rgb_client: rclpy.client.Client = self._node.create_client(
            srv_type=SetRGBSvc,
            srv_name=SET_RGB_SERVICE,
        )

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSPresetProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Subscribers
        self._rgb_sub: rclpy.subscription.Subscription = self._node.create_subscription(
            msg_type=RGBMsg,
            topic=RGB_TOPIC,
            callback=self._on_rgb_status,
            qos_profile=qos_profile,
        )

        # Timer to drive the rainbow update loop
        self._timer = self._node.create_timer(
            RGB_UPDATE_INTERVAL_SECS,
            self._update_active_lights,
        )

    def _on_rgb_status(self, msg: RGBMsg) -> None:
        """
        Callback for RGB status messages.
        """
        # Translate parameters
        entity_id: str = msg.header.frame_id
        color: tuple[float, float, float] = (
            msg.r,
            msg.g,
            msg.b,
        )
        is_now_on: bool = any((msg.r, msg.g, msg.b))

        self._logger.debug(
            f"Received RGB message: {entity_id} - {'ON' if is_now_on else 'OFF'} - Color: {color}"
        )

        self._light_ids.add(entity_id)

        if is_now_on:
            # Check if the light received an OFF event past the cutoff
            off_time: datetime | None = self._last_turned_off.get(entity_id)
            if off_time is None or (datetime.now() - off_time) >= timedelta(
                seconds=RGB_CUTOFF_INTERVAL_SECS
            ):
                self._active_lights.add(entity_id)
        else:
            # Light reported all-zero -> remove from active, record OFF time
            if entity_id in self._active_lights:
                self._active_lights.remove(entity_id)
            self._last_turned_off[entity_id] = datetime.now()

    def _update_active_lights(self) -> None:
        """
        Timer callback to set the color for all lights.
        """
        now: datetime = datetime.now()
        force_off: timedelta = timedelta(seconds=RGB_FORCE_OFF_INTERVAL_SECS)

        # Determine rainbow hue based on elapsed time
        elapsed_sec: float = (now - self._start_time).total_seconds()
        hue: float = (elapsed_sec % RAINBOW_CYCLE_SECS) / RAINBOW_CYCLE_SECS
        r_val: float
        g_val: float
        b_val: float
        r_val, g_val, b_val = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
        rainbow_color: tuple[float, float, float] = (
            min(1.0, r_val + (1.0 - r_val) * RAINBOW_LIGHTNESS_BOOST),
            min(1.0, g_val + (1.0 - g_val) * RAINBOW_LIGHTNESS_BOOST),
            min(1.0, b_val + (1.0 - b_val) * RAINBOW_LIGHTNESS_BOOST),
        )

        # Process all lights
        for entity_id in list(self._light_ids):
            off_time: datetime | None = self._last_turned_off.get(entity_id)

            # Force OFF if it was turned off recently
            if off_time is not None and (now - off_time) < force_off:
                self._logger.debug(
                    f"Sending OFF to {entity_id}: turned off {(now - off_time).total_seconds():.2f}s ago"
                )
                off_req: SetRGBSvc.Request = SetRGBSvc.Request()
                off_req.light_id = entity_id
                off_req.r = 0.0
                off_req.g = 0.0
                off_req.b = 0.0
                off_req.transition = RGB_UPDATE_INTERVAL_SECS
                self._set_rgb_client.call_async(off_req)
                continue

            # Skip inactive lights
            if entity_id not in self._active_lights:
                continue

            self._logger.debug(f"Sending ON to {entity_id} - Color: {rainbow_color}")

            on_req: SetRGBSvc.Request = SetRGBSvc.Request()
            on_req.light_id = entity_id
            on_req.r, on_req.g, on_req.b = rainbow_color
            on_req.transition = RGB_UPDATE_INTERVAL_SECS
            self._set_rgb_client.call_async(on_req)
