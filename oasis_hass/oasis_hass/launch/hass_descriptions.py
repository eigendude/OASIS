################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from launch import LaunchDescription
from launch_ros.actions import Node

from oasis_hass.utils.smarthome_config import SmarthomeConfig


################################################################################
# Smarthome parameters
################################################################################


CONFIG: SmarthomeConfig = SmarthomeConfig()

# The host and zone IDs used for Home Assistant
HOME_ASSISTANT_ID: str = CONFIG.HOME_ASSISTANT_ID

# Path to the MQTT parameters file for Home Assistant
MQTT_PARAMS_FILE: str = CONFIG.MQTT_PARAMS_FILE


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE: str = "oasis"

HASS_PACKAGE_NAME: str = CONFIG.HASS_PACKAGE_NAME

DRIVERS_PACKAGE_NAME: str = "oasis_drivers_py"

################################################################################
# Home Assistant nodes
################################################################################


class HomeAssistantDescriptions:
    @staticmethod
    def add_home_assistant(ld: LaunchDescription) -> None:
        hass_mqtt_bridge_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=HASS_PACKAGE_NAME,
            executable="hass_mqtt_bridge",
            name="hass_mqtt_bridge",
            output="screen",
            remappings=[
                # Topics
                ("plug", f"{HOME_ASSISTANT_ID}/plug"),
                ("rgb", f"{HOME_ASSISTANT_ID}/rgb"),
                # Services
                ("set_plug", f"{HOME_ASSISTANT_ID}/set_plug"),
                ("set_rgb", f"{HOME_ASSISTANT_ID}/set_rgb"),
            ],
        )
        ld.add_action(hass_mqtt_bridge_node)

        # Start the generic MQTT -> ROS bridge
        # TODO: Disabled to save resources until mqtt_client is implemented
        """
        mqtt_client_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package="mqtt_client",
            executable="mqtt_client",
            name="mqtt_client",
            output="screen",
            parameters=[MQTT_PARAMS_FILE],
            remappings=[
                # Topics
                ("statestream", f"{HOME_ASSISTANT_ID}/statestream"),
            ],
        )
        ld.add_action(mqtt_client_node)
        """
