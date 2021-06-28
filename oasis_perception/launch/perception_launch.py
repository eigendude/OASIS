################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from launch import LaunchDescription
from launch_ros.actions import Node


ROS_NAMESPACE = "oasis"

PACKAGE_NAME = "oasis_perception"


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    manager_node = Node(
        namespace=ROS_NAMESPACE,
        package=PACKAGE_NAME,
        executable="background_subtractor",
        output="screen",
    )
    ld.add_action(manager_node)

    return ld
