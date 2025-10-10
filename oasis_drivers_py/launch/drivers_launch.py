################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from typing import Optional

from launch import LaunchDescription

from oasis_drivers.launch.driver_descriptions import DriverDescriptions as Drivers
from oasis_drivers.launch.mcu_descriptions import MCUDescriptions as MCU
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

# The zone ID used for the Kinect V2 camera
KINECT_V2_ZONE_ID: str = CONFIG.KINECT_V2_ZONE_ID

# Zones with a smart display that can be controlled
SMART_DISPLAY_ZONES: list[str] = CONFIG.SMART_DISPLAY_ZONES

print(f"Launching on {HOSTNAME} in zone {ZONE_ID}")


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE: str = "oasis"

PYTHON_PACKAGE_NAME: str = "oasis_drivers_py"


################################################################################
# Hardware/video parameters
################################################################################

#
# Video configuration
#

VIDEO_DEVICE: str = "/dev/video0"

# TODO: Hardware configuration
if HOST_ID == "falcon":
    # IMAGE_SIZE: list[int] = [1280, 720]
    IMAGE_SIZE: list[int] = [640, 480]
else:
    IMAGE_SIZE: list[int] = [640, 480]


#
# Microcontroller configuration
#

MCU_NODE: Optional[str] = None
MCU_TYPE: Optional[str] = None


class MCUType:
    FIRMATA: str = "firmata"
    TELEMETRIX: str = "telemetrix"


if HOST_ID == "falcon":
    MCU_NODE = "engineer"
    MCU_TYPE = MCUType.FIRMATA
elif HOST_ID == "jetson":
    MCU_NODE = "engine"
    MCU_TYPE = MCUType.TELEMETRIX
elif HOST_ID == "station":
    MCU_NODE = "conductor"
    MCU_TYPE = MCUType.TELEMETRIX

AVR_COM_PORT: str = "/dev/ttyACM0"


################################################################################
# Launch description
################################################################################


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    #
    # General drivers
    #

    Drivers.add_system_monitor(ld, HOST_ID)

    # Disabled to save resources
    # Drivers.add_serial_port_scanner(ld, HOST_ID)

    #
    # Smarthome drivers
    #

    if HOST_ID in SMART_DISPLAY_ZONES:
        Drivers.add_display_server(ld, ZONE_ID)

    #
    # Camera drivers
    #

    # Laptop cameras
    if HOST_ID == "bar":
        Drivers.add_v4l2_camera(ld, ZONE_ID, VIDEO_DEVICE, IMAGE_SIZE)
    if HOST_ID == "kitchen":
        Drivers.add_v4l2_camera(ld, ZONE_ID, VIDEO_DEVICE, IMAGE_SIZE)

    # Kinect V2 camera
    if HOST_ID == "nas":
        Drivers.add_kinect_v2(ld, KINECT_V2_ZONE_ID)

    # LEGO models
    if HOST_ID == "falcon":
        Drivers.add_ros2_camera(ld, ZONE_ID, IMAGE_SIZE)
    if HOST_ID == "station":
        Drivers.add_ros2_camera(ld, ZONE_ID, IMAGE_SIZE)

    # Smarthome cameras
    if HOST_ID == "door":
        # Drivers.add_ros2_camera(ld, "doorbell", IMAGE_SIZE)
        # Drivers.add_ros2_camera(ld, "entryway", IMAGE_SIZE)
        pass

    #
    # MCU bridges
    #

    if MCU_NODE is not None:
        if MCU_TYPE == MCUType.FIRMATA:
            MCU.add_firmata_bridge(ld, HOST_ID, MCU_NODE, AVR_COM_PORT)
        elif MCU_TYPE == MCUType.TELEMETRIX:
            MCU.add_telemetrix_bridge(ld, HOST_ID, MCU_NODE, AVR_COM_PORT)

    return ld
