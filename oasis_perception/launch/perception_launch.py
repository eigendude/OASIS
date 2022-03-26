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

PACKAGE_NAME = "oasis_perception"


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    manager_node = Node(
        namespace=ROS_NAMESPACE,
        package=PACKAGE_NAME,
        executable="background_subtractor",
        name=f"background_subtractor_{HOSTNAME}",
        output="screen",
    )
    ld.add_action(manager_node)

    return ld
