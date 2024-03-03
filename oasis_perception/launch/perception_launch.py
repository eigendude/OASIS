################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
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

    if HOSTNAME == "cinder":
        bgs_abl_node = Node(
            namespace=ROS_NAMESPACE,
            package=PACKAGE_NAME,
            executable="background_subtractor_abl",
            name=f"background_subtractor_abl_{HOSTNAME}",
            output="screen",
        )
        ld.add_action(bgs_abl_node)

        bgs_asbl_node = Node(
            namespace=ROS_NAMESPACE,
            package=PACKAGE_NAME,
            executable="background_subtractor_asbl",
            name=f"background_subtractor_asbl_{HOSTNAME}",
            output="screen",
        )
        ld.add_action(bgs_asbl_node)

    elif HOSTNAME == "starship":
        multi_modeler_node = Node(
            namespace=ROS_NAMESPACE,
            package=PACKAGE_NAME,
            executable="multi_modeler",
            name=f"multi_modeler_{HOSTNAME}",
            output="screen",
        )
        ld.add_action(multi_modeler_node)

        """
        monocular_slam_node = Node(
            namespace=ROS_NAMESPACE,
            package=PACKAGE_NAME,
            executable="monocular_slam",
            name=f"monocular_slam_{HOSTNAME}",
            output="screen",
        )
        ld.add_action(monocular_slam_node)
        """

    elif HOSTNAME == "jetson":
        MCU_NODE = "engine"
        monocular_inertial_slam_node = Node(
            namespace=ROS_NAMESPACE,
            package=PACKAGE_NAME,
            executable="monocular_inertial_slam",
            name=f"monocular_inertial_slam_{HOSTNAME}",
            output="screen",
            remappings=[
                ("i2c_imu", f"{MCU_NODE}/i2c_imu"),
            ],
        )
        ld.add_action(monocular_inertial_slam_node)

    return ld
