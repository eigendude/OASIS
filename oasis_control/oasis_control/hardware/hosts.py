################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from __future__ import annotations

from oasis_control.hardware.config import HostHardwareConfig
from oasis_control.hardware.config import MCUManagerConfig
from oasis_control.hardware.config import MCUManagerImplementation


# Camera scene source used by the station conductor manager
STATION_CAMERA_SCENE_ZONE: str = "hallway"
STATION_CAMERA_SCENE_RESOLUTION: str = "sd"


def get_host_hardware_config(
    host_id: str,
    zone_id: str,
) -> HostHardwareConfig:
    """Return control-facing hardware for a smarthome host and deployment zone"""

    if not zone_id:
        raise ValueError("zone_id must not be empty")

    if host_id == "airlab":
        return HostHardwareConfig(enable_oled_visualizer=True)
    if host_id == "falcon":
        return HostHardwareConfig(
            enable_ahrs_speedometer=True,
            enable_cockpit_visualizer=True,
            mcu_manager=MCUManagerConfig(
                node_name="engineer",
                implementation=MCUManagerImplementation.ENGINEER,
                conductor_host="station",
            ),
        )
    if host_id == "station":
        return HostHardwareConfig(
            enable_wol_server=True,
            mcu_manager=MCUManagerConfig(
                node_name="conductor",
                implementation=MCUManagerImplementation.CONDUCTOR,
                input_provider="megapegasus",
                camera_scene_zone=STATION_CAMERA_SCENE_ZONE,
                camera_scene_resolution=STATION_CAMERA_SCENE_RESOLUTION,
                wol_server_id=host_id,
                motor_voltage_reversed=True,
            ),
        )

    return HostHardwareConfig()
