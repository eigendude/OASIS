################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from launch.launch_description import LaunchDescription
from oasis_perception.launch.perception_descriptions import PerceptionDescriptions

from oasis_hass.utils.smarthome_config import SmarthomeConfig


################################################################################
# System parameters
################################################################################


CONFIG: SmarthomeConfig = SmarthomeConfig()

HOST_ID: str = CONFIG.HOST_ID

# TODO: Select "pinhole" or "fisheye" from configuration
CAMERA_MODEL: str = "fisheye"


################################################################################
# Launch description
################################################################################


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    PerceptionDescriptions.add_calibration(ld, HOST_ID, CAMERA_MODEL)

    return ld
