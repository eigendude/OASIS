################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from launch.launch_description import LaunchDescription

from oasis_control.launch.control_descriptions import ControlDescriptions
from oasis_drivers.launch.driver_descriptions import DriverDescriptions as Drivers
from oasis_hass.utils.smarthome_config import SmarthomeConfig


################################################################################
# Smarthome parameters
################################################################################


CONFIG: SmarthomeConfig = SmarthomeConfig()

# Get the hostname
HOSTNAME: str = CONFIG.HOSTNAME

# Host aliases
HOST_ID: str = CONFIG.HOST_ID

# Zone configuration
ZONE_ID: str = CONFIG.ZONE_ID

# Zones with a smart display that can be controlled
SMART_DISPLAY_ZONES: list[str] = CONFIG.SMART_DISPLAY_ZONES

# Plug ID that controls smart displays
SMART_DISPLAY_PLUG_ID: str = CONFIG.SMART_DISPLAY_PLUG_ID

# Zones with a camera feed
CAMERA_ZONES: list[str] = CONFIG.CAMERA_ZONES

# The host and zone IDs used for Home Assistant
HOME_ASSISTANT_ID: str = CONFIG.HOME_ASSISTANT_ID

# Machine that broadcasts peripheral input
INPUT_PROVIDER: str = "megapegasus"  # TODO

print(f"Launching on {HOSTNAME} in zone {ZONE_ID}")


################################################################################
# Launch description
################################################################################


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    if HOST_ID == HOME_ASSISTANT_ID:
        ControlDescriptions.add_home_manager(
            ld,
            SMART_DISPLAY_ZONES,
            SMART_DISPLAY_PLUG_ID,
            CAMERA_ZONES,
            HOME_ASSISTANT_ID,
        )

    # Navigation
    if HOST_ID == "falcon":
        # ControlDescriptions.add_imu_fuser(ld, HOST_ID)
        # ControlDescriptions.add_speedometer(ld, HOST_ID)
        ControlDescriptions.add_tilt_sensor(ld, HOST_ID)

    # Microcontroller nodes
    if HOST_ID == "station":
        # TODO
        CALIBRATION_RESOLUTION: str = "sd"

        ControlDescriptions.add_mcu_manager_telemetrix(
            ld,
            HOST_ID,
            "conductor",
            HOST_ID,
            INPUT_PROVIDER,
            CALIBRATION_RESOLUTION,
        )

        Drivers.add_wol_server(ld, HOST_ID)
    elif HOST_ID == "jetson":
        ControlDescriptions.add_mcu_manager(ld, HOST_ID, "engine")
    elif HOST_ID == "falcon":
        ControlDescriptions.add_mcu_manager_with_pwm(ld, HOST_ID, "engineer")

    return ld
