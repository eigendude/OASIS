################################################################################
#
#  Copyright (C) 2025 Garrett Brown
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

ZONE_ID: str = CONFIG.ZONE_ID

ROS_NAMESPACE: str = "oasis"
PYTHON_PACKAGE_NAME: str = "oasis_drivers_py"


################################################################################
# Launch description
################################################################################


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ups_server_node = Node(
        namespace=ROS_NAMESPACE,
        package=PYTHON_PACKAGE_NAME,
        executable="ups_server",
        name=f"ups_server_{ZONE_ID}",
        output="screen",
        remappings=[
            ("ups_status", f"{ZONE_ID}/ups_status"),
            ("ups_command", f"{ZONE_ID}/ups_command"),
        ],
    )

    ld.add_action(ups_server_node)

    return ld
