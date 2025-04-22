#!/usr/bin/env python3
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
# Home Assistant Bridge (MQTT Statestream -> ROS 2 and ROS 2 -> MQTT)
#
# Note: If you have mosquitto-clients installed, you can use the following
# command to echo all MQTT topics:
#
#   mosquitto_sub -h homeassistant.local -p 1883 -t '#' -v
#

import json
from datetime import datetime
from datetime import timezone
from typing import Any
from typing import Iterable
from typing import Optional

import paho.mqtt.client
import rclpy.logging
import rclpy.node
import rclpy.publisher
import rclpy.service
from builtin_interfaces.msg import Time as TimeMsg

from oasis_msgs.msg import RGB as RGBMsg
from oasis_msgs.msg import Plug as PlugMsg
from oasis_msgs.srv import SetPlug as SetPlugSrv
from oasis_msgs.srv import SetRGB as SetRGBSrv


################################################################################
# ROS parameters
################################################################################

NODE_NAME: str = "hass_bridge"

PLUG_TOPIC: str = "plug"
RGB_TOPIC: str = "rgb"

SET_PLUG_SERVICE: str = "set_plug"
SET_RGB_SERVICE: str = "set_rgb"

################################################################################
# MQTT parameters
################################################################################

MQTT_BROKER: str = "homeassistant.local"
MQTT_PORT: int = 1883

# State topic (read-only)
MQTT_PREFIX: str = "homeassistant/statestream"

# Command topic (ROS -> HA)
CMD_PREFIX: str = "homeassistant"

################################################################################
# ROS node
################################################################################


class HassBridgeNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Set logging to DEBUG level
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        #
        # Device state
        #

        # Keep track of the state of each device
        self._device_state: dict[str, bool] = {}  # entity_id -> state

        # Keep latest attribute payloads so we can colour‑decode lights
        self._attr_cache: dict[str, dict[str, Any]] = {}  # entity_id -> attributes

        # Last updated timestamps
        self._last_updated: dict[str, str] = {}  # entity_id -> last_updated

        # Activated lights
        self._activated_lights: dict[str, bool] = {}  # entity_id -> activated

        #
        # ROS publishers
        #

        self._plug_pub: rclpy.publisher.Publisher = self.create_publisher(
            PlugMsg, PLUG_TOPIC, 10
        )
        self._rgb_pub: rclpy.publisher.Publisher = self.create_publisher(
            RGBMsg, RGB_TOPIC, 10
        )

        #
        # ROS services
        #

        self._set_plug_service: rclpy.service.Service = self.create_service(
            srv_type=SetPlugSrv, srv_name=SET_PLUG_SERVICE, callback=self._on_set_plug
        )
        self._set_rgb_service: rclpy.service.Service = self.create_service(
            srv_type=SetRGBSrv, srv_name=SET_RGB_SERVICE, callback=self._on_set_rgb
        )

        #
        # MQTT parameters
        #

        # MQTT client
        self.mqtt = paho.mqtt.client.Client()
        self.mqtt.on_message = self._on_mqtt_message
        self.mqtt.connect(MQTT_BROKER, MQTT_PORT)

        # MQTT topics
        self.mqtt.subscribe(f"{MQTT_PREFIX}/light/+/+")

        # Start the network loop in a background thread
        self.mqtt.loop_start()

        self.get_logger().info("Home Assistant bridge initialized")

    def _on_mqtt_message(
        self,
        client: paho.mqtt.client.Client,
        userdata: Optional[object],
        msg: paho.mqtt.client.MQTTMessage,
    ) -> None:
        # Get the MQTT topic
        topic: str = msg.topic

        # Parse the MQTT topic
        parts: list[str] = topic.split("/")
        try:
            entity_id: str = parts[3]
            attr_name = parts[4]  # rgb_color, brightness, ...
        except IndexError as err:
            self.get_logger().error(f"Invalid MQTT topic: {topic} - {err}")
            return

        # Store it (JSON or plain text)
        try:
            value = json.loads(msg.payload)
        except json.JSONDecodeError:
            value = msg.payload.decode()

        # Store the value in the cache
        cache = self._attr_cache.setdefault(entity_id, {})
        cache[attr_name] = value

        # Publish values when a "friendly_name" message is received
        if attr_name == "friendly_name":
            # Check last updated time
            last_updated: str = self._last_updated.get(entity_id, "")

            # Skip if the last updated time hasn't changed
            if last_updated != cache.get("last_updated"):
                self._last_updated[entity_id] = cache["last_updated"]

                if cache.get("is_hue_group", False):
                    # Skip groups
                    pass
                elif cache.get("supported_color_modes", []) == ["onoff"]:
                    # Handle smart plugs
                    self._publish_plug(entity_id, cache)
                else:
                    # Handle lights
                    self._publish_rgb(entity_id, cache)

    def _on_set_plug(
        self, request: SetPlugSrv.Request, response: SetPlugSrv.Response
    ) -> SetPlugSrv.Response:
        """
        Handle a request to set the plug state.
        """
        # Construct the MQTT message
        topic: str = f"{CMD_PREFIX}/switch/{request.plug_id}/set"
        payload: str = json.dumps({"plug_id": request.plug_id, "state": request.state})

        self.get_logger().debug(
            f"{SET_PLUG_SERVICE}: {request.plug_id} -> {'ON' if request.state else 'OFF'}"
        )

        # Publish the message
        self.mqtt.publish(topic, payload)

        return response

    def _on_set_rgb(
        self, request: SetRGBSrv.Request, response: SetRGBSrv.Response
    ) -> SetRGBSrv.Response:
        """
        Handle a request to set the RGB state.

          * r, g, b in the SetRGB service are 0‑1 floats
          * We convert them to 0‑255 integers before publishing so that
            Home Assistant receives the same 8‑bit values it natively uses.
        """
        # Convert 0‑1 -> 0‑255 and clamp
        r8: int = max(0, min(255, int(round(request.r * 255))))
        g8: int = max(0, min(255, int(round(request.g * 255))))
        b8: int = max(0, min(255, int(round(request.b * 255))))

        # Construct the MQTT message
        topic: str = f"{CMD_PREFIX}/light/{request.light_id}/set"
        msg: dict[str, int] = {"r": r8, "g": g8, "b": b8}

        # Include transition, if provided
        if hasattr(request, "transition"):
            # Round to integer seconds
            msg["transition"] = int(round(request.transition))

        payload = json.dumps(msg)

        self.get_logger().debug(
            f"{SET_RGB_SERVICE}: {request.light_id} -> [{r8},{g8},{b8}]"
        )

        # Publish the message
        self.mqtt.publish(topic, payload)

        return response

    def _publish_plug(self, entity_id: str, attrs: dict[str, Any]) -> None:
        """
        Publish the plug state.
        """
        device_state: bool = attrs.get("state", "off").lower() == "on"

        msg: PlugMsg = PlugMsg()
        msg.header.stamp = self._get_timestamp(attrs)
        msg.header.frame_id = entity_id
        msg.state = device_state
        self._plug_pub.publish(msg)

        self.get_logger().debug(f"{entity_id}: {'ON' if device_state else 'OFF'}")

    def _publish_rgb(self, entity_id: str, attrs: dict[str, Any]) -> None:
        """
        Publish the light's RGB state.
        """
        rgb: Optional[Iterable[int]] = attrs.get("rgb_color")
        if not (isinstance(rgb, (list, tuple)) and len(rgb) == 3):
            rgb = (0, 0, 0)

        msg: RGBMsg = RGBMsg()
        msg.header.stamp = self._get_timestamp(attrs)
        msg.header.frame_id = entity_id
        msg.r, msg.g, msg.b = (c / 255.0 for c in rgb)
        self._rgb_pub.publish(msg)

        # Light is activated if any of the RGB values are > 0
        activated = any(c > 0 for c in rgb)

        if self._activated_lights.get(entity_id, False) != activated:
            # Update the activation state
            self._activated_lights[entity_id] = activated

            # Log the activation state
            if activated:
                self.get_logger().debug(
                    f"{entity_id}: {msg.r:.2f}, {msg.g:.2f}, {msg.b:.2f}"
                )
            else:
                self.get_logger().debug(f"{entity_id}: OFF")

    def _get_timestamp(self, attrs: dict[str, Any]) -> TimeMsg:
        """
        Get the timestamp from the attributes.
        """
        # attrs["last_updated"] looks like 2025-04-20T18:44:02.504096+00:00
        ts: Optional[str] = attrs.get("last_updated")
        if not isinstance(ts, str):
            return self.get_clock().now().to_msg()

        # Convert to a builtin_interfaces/Time message
        return self._iso_to_time_msg(ts)

    @staticmethod
    def _iso_to_time_msg(iso_ts: str) -> TimeMsg:
        """
        Convert an ISO‑8601 timestamp (with or without trailing ‘Z’) into a
        builtin_interfaces/Time message.
        """
        # Handle both “+00:00” and “Z”
        dt: datetime = datetime.fromisoformat(iso_ts.rstrip("Z")).astimezone(
            timezone.utc
        )

        sec: int = int(dt.timestamp())
        nsec: int = int((dt.timestamp() - sec) * 1_000_000_000)

        return TimeMsg(sec=sec, nanosec=nsec)
