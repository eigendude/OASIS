################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

import socket

from launch import LaunchDescription
from launch_ros.actions import Node


################################################################################
# System parameters
################################################################################

# Get the hostname
HOSTNAME = socket.gethostname()

################################################################################
# ROS parameters
################################################################################

ROS_NAMESPACE = "oasis"

PACKAGE_NAME = "oasis_hass"

################################################################################
# Launch description
################################################################################


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    hass_bridge_node = Node(
        namespace=ROS_NAMESPACE,
        package=PACKAGE_NAME,
        executable="hass_bridge",
        name=f"hass_bridge_{HOSTNAME}",
        output="screen",
        remappings=[
            ("plug", f"{HOSTNAME}/plug"),
            ("rgb", f"{HOSTNAME}/rgb"),
            ("set_plug", f"{HOSTNAME}/set_plug"),
            ("set_rgb", f"{HOSTNAME}/set_rgb"),
        ],
    )
    ld.add_action(hass_bridge_node)

    return ld
