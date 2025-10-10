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

from oasis_drivers.launch.driver_descriptions import DriverDescriptions as Drivers
from oasis_hass.launch.hass_descriptions import HomeAssistantDescriptions as HA
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

# The host and zone IDs used for Home Assistant
HOME_ASSISTANT_ID: str = CONFIG.HOME_ASSISTANT_ID

print(f"Launching on {HOSTNAME} in zone {ZONE_ID}")


################################################################################
# Launch description
################################################################################


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    HA.add_home_assistant(ld)

    # Also run the WoL server
    Drivers.add_wol_server(ld, HOME_ASSISTANT_ID)

    return ld
