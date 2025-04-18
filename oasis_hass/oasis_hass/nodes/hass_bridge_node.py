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
# Home Assistant Bridge
#

import json

import paho.mqtt.client
import rclpy.node
import rclpy.publisher
import rclpy.service
from rclpy.logging import LoggingSeverity

from oasis_msgs.msg import RGB as RGBMsg
from oasis_msgs.msg import Plug as PlugMsg
from oasis_msgs.srv import SetPlug as SetPlugSrv
from oasis_msgs.srv import SetRGB as SetRGBSrv


################################################################################
# ROS parameters
################################################################################

NODE_NAME = "hass_bridge"

PLUG_TOPIC = "plug"
RGB_TOPIC = "rgb"

SET_PLUG_SERVICE = "set_plug"
SET_RGB_SERVICE = "set_rgb"

################################################################################
# MQTT parameters
################################################################################

MQTT_BROKER = "homeassistant.local"
MQTT_PORT = 1883
MQTT_DISCOVERY_PREFIX = "homeassistant"

################################################################################
# ROS node
################################################################################


class HassBridgeNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Enable debug logging
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        # ROS publishers
        self._plug_pub: rclpy.publisher.Publisher = self.create_publisher(
            PlugMsg, PLUG_TOPIC, 10
        )
        self._rgb_pub: rclpy.publisher.Publisher = self.create_publisher(
            RGBMsg, RGB_TOPIC, 10
        )

        # Services
        self._set_plug_service: rclpy.service.Service = self.create_service(
            srv_type=SetPlugSrv, srv_name=SET_PLUG_SERVICE, callback=self._on_set_plug
        )
        self._set_rgb_service: rclpy.service.Service = self.create_service(
            srv_type=SetRGBSrv, srv_name=SET_RGB_SERVICE, callback=self._on_set_rgb
        )

        # MQTT parameters
        self.mqtt = paho.mqtt.client.Client()
        self.mqtt.on_message = self._on_mqtt_message
        self.mqtt.connect(MQTT_BROKER, MQTT_PORT)

        # Subscribe to MQTT topics
        # self.mqtt.subscribe(f"{MQTT_DISCOVERY_PREFIX}/light/oasis_rgb/state")
        # self.mqtt.subscribe(f"{MQTT_DISCOVERY_PREFIX}/switch/oasis_plug/state")

        # Start the network loop in a background thread
        self.mqtt.loop_start()

        self.get_logger().info("Home Assistant bridge initialized")

    def _on_set_plug(
        self, request: SetPlugSrv.Request, response: SetPlugSrv.Response
    ) -> SetRGBSrv.Response:
        """
        Handle a request to set the plug state.
        """
        # Construct the MQTT message
        topic = f"{MQTT_DISCOVERY_PREFIX}/switch/oasis_plug/set"
        payload = json.dumps({"plug_id": request.plug_id, "state": request.state})

        self.get_logger().info(
            f"Service '/{SET_PLUG_SERVICE}' → MQTT {topic}: {payload}"
        )

        # Publish the message
        self.mqtt.publish(topic, payload)

        return response

    def _on_set_rgb(
        self, request: SetRGBSrv.Request, response: SetRGBSrv.Response
    ) -> SetRGBSrv.Response:
        """
        Handle a request to set the RGB state.
        """
        # Construct the MQTT message
        topic = f"{MQTT_DISCOVERY_PREFIX}/light/oasis_rgb/set"
        payload = json.dumps({"r": request.r, "g": request.g, "b": request.b})

        self.get_logger().info(
            f"Service '/{SET_RGB_SERVICE}' → MQTT {topic}: {payload}"
        )

        # Publish the message
        self.mqtt.publish(topic, payload)

        return response

    def _on_mqtt_message(self, client, userdata, msg):
        topic = msg.topic

        try:
            data = json.loads(msg.payload)
        except json.JSONDecodeError as err:
            self.get_logger().error(f"Failed to decode JSON payload: {err}")
            return

        self.get_logger().debug(f"MQTT ← {topic}: {data}")

        if topic.endswith("/light/oasis_rgb/set"):
            # Forward directly as a ROS RGBMsg
            msg = RGBMsg()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.r = float(data.get("r", 0.0))
            msg.g = float(data.get("g", 0.0))
            msg.b = float(data.get("b", 0.0))
            self.get_logger().debug(f"Publishing ROS RGBMsg → '{RGB_TOPIC}': {msg}")
            self._rgb_pub.publish(msg)
        elif topic.endswith("/switch/oasis_plug/set"):
            # Forward directly as a ROS PlugMsg
            msg = PlugMsg()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.plug_id = data.get("plug_id", "")
            msg.state = bool(data.get("state", False))
            self.get_logger().debug(f"Publishing ROS PlugMsg → '{PLUG_TOPIC}': {msg}")
            self._plug_pub.publish(msg)
        else:
            self.get_logger().warn(f"Ignoring unexpected MQTT topic: {topic}")
