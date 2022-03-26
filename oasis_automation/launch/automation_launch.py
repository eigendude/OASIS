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

PACKAGE_NAME = "oasis_automation"


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    asus_manager_node = Node(
        namespace=ROS_NAMESPACE,
        package=PACKAGE_NAME,
        executable="automation_manager",
        output="screen",
        remappings=[
            # TODO: Hardware configuration
            ("power_event", "nuc/power_event"),
            ("power_control", "asus/power_control"),
        ],
    )
    ld.add_action(asus_manager_node)

    inspiron_manager_node = Node(
        namespace=ROS_NAMESPACE,
        package=PACKAGE_NAME,
        executable="automation_manager",
        output="screen",
        remappings=[
            # TODO: Hardware configuration
            ("power_event", "nuc/power_event"),
            ("power_control", "inspiron/power_control"),
        ],
    )
    ld.add_action(inspiron_manager_node)

    lenovo_manager_node = Node(
        namespace=ROS_NAMESPACE,
        package=PACKAGE_NAME,
        executable="automation_manager",
        output="screen",
        remappings=[
            # TODO: Hardware configuration
            ("power_event", "nuc/power_event"),
            ("power_control", "lenovo/power_control"),
        ],
    )
    ld.add_action(lenovo_manager_node)

    netbook_manager_node = Node(
        namespace=ROS_NAMESPACE,
        package=PACKAGE_NAME,
        executable="automation_manager",
        output="screen",
        remappings=[
            # TODO: Hardware configuration
            ("power_event", "nuc/power_event"),
            ("power_control", "netbook/power_control"),
        ],
    )
    ld.add_action(netbook_manager_node)

    return ld
