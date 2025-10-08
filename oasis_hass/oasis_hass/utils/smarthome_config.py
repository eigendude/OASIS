################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import os
import socket
from typing import Any

import yaml
from ament_index_python import get_package_share_directory


class SmarthomeConfig:
    ############################################################################
    # System parameters
    ############################################################################

    HOSTNAME: str = socket.gethostname().replace("-", "_")

    ############################################################################
    # ROS parameters
    ############################################################################

    HASS_PACKAGE_NAME: str = "oasis_hass"

    ############################################################################
    # Smarthome parameters
    ############################################################################

    HOME_ASSISTANT_ID: str = "homeassistant"

    # The zone ID used for the Kinect V2 camera
    KINECT_V2_ZONE_ID: str = "hallway"

    MQTT_PARAMS_FILE: str = os.path.join(
        get_package_share_directory(HASS_PACKAGE_NAME),
        "mqtt_client",
        "mqtt_client_params.yaml",
    )

    ############################################################################
    # Smarthome configuration
    ############################################################################

    def __init__(self) -> None:
        # Load the smarthome configuration
        config_path: str = os.path.join(
            get_package_share_directory(self.HASS_PACKAGE_NAME),
            "config",
            "smarthome.yaml",
        )

        print(f"Loading smarthome config from {config_path}")

        with open(config_path, "r") as file:
            smarthome_config: dict[str, Any] = yaml.safe_load(file)

        # Host aliases
        self._host_id: str = smarthome_config["host_id_map"].get(
            self.HOSTNAME, self.HOSTNAME
        )

        # Zone configuration
        self._zone_id: str = smarthome_config["zone_id_map"].get(
            self._host_id, self._host_id
        )
        self._smart_display_zones: list[str] = smarthome_config.get(
            "smart_display_zones", []
        )
        self._smart_display_plug_id: str = smarthome_config.get(
            "smart_display_plug_id", ""
        )
        self._camera_zones: list[str] = smarthome_config.get("camera_zones", [])
        self._camera_driver_map: dict[str, str] = smarthome_config.get(
            "camera_driver_map", {}
        )

    @property
    def HOST_ID(self) -> str:
        return self._host_id

    @property
    def ZONE_ID(self) -> str:
        return self._zone_id

    @property
    def SMART_DISPLAY_ZONES(self) -> list[str]:
        return self._smart_display_zones

    @property
    def SMART_DISPLAY_PLUG_ID(self) -> str:
        return self._smart_display_plug_id

    @property
    def CAMERA_ZONES(self) -> list[str]:
        return self._camera_zones

    @property
    def CAMERA_DRIVER_MAP(self) -> dict[str, str]:
        return self._camera_driver_map

    def get_camera_driver(self, host_id: str, zone_id: str) -> str:
        """Return the configured camera driver for a host or zone."""

        # Prefer explicit host configuration and fall back to zone configuration
        return self._camera_driver_map.get(
            host_id, self._camera_driver_map.get(zone_id, "")
        )
