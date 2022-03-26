################################################################################
#
#  Copyright (C) 2021 Garrett Brown
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

PACKAGE_NAME = "oasis_control"


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    if HOSTNAME in [
        "asus",
        "inspiron",
        "lenovo",
        "netbook",
    ]:
        automation_manager_node = Node(
            namespace=ROS_NAMESPACE,
            package=PACKAGE_NAME,
            executable="automation_manager",
            name=f"automation_manager_{HOSTNAME}",
            output="screen",
            remappings=[
                # TODO: Hardware configuration
                ("power_event", "nuc/power_event"),
                ("power_control", f"{HOSTNAME}/power_control"),
            ],
        )
        ld.add_action(automation_manager_node)

    return ld
