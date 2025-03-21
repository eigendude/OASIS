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
HOSTNAME = socket.gethostname().replace("-", "_")


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE = "oasis"

CPP_PACKAGE_NAME = "oasis_perception_cpp"
PYTHON_PACKAGE_NAME = "oasis_perception_py"


print(f"Launching on {HOSTNAME}")


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    if HOSTNAME == "cinder":
        """
        bgs_abl_node = Node(
            namespace=ROS_NAMESPACE,
            package=CPP_PACKAGE_NAME,
            executable="background_subtractor_abl",
            name=f"background_subtractor_abl_{HOSTNAME}",
            output="screen",
        )
        ld.add_action(bgs_abl_node)

        bgs_asbl_node = Node(
            namespace=ROS_NAMESPACE,
            package=CPP_PACKAGE_NAME,
            executable="background_subtractor_asbl",
            name=f"background_subtractor_asbl_{HOSTNAME}",
            output="screen",
        )
        ld.add_action(bgs_asbl_node)
        """

        CAMERA_NODE = "kinect2"
        background_modeler_node = Node(
            namespace=ROS_NAMESPACE,
            package=CPP_PACKAGE_NAME,
            executable="background_modeler",
            name=f"background_modeler_{CAMERA_NODE}",
            output="screen",
            remappings=[
                ("image_raw", f"{CAMERA_NODE}/hd/image_color"),
                ("image_raw/compressed", f"{CAMERA_NODE}/hd/image_color/compressed"),
                ("background", f"{CAMERA_NODE}/background"),
            ],
        )
        ld.add_action(background_modeler_node)

    elif HOSTNAME == "starship":
        multi_modeler_node = Node(
            namespace=ROS_NAMESPACE,
            package=CPP_PACKAGE_NAME,
            executable="multi_modeler",
            name=f"multi_modeler_{HOSTNAME}",
            output="screen",
        )
        ld.add_action(multi_modeler_node)

        """
        monocular_slam_node = Node(
            namespace=ROS_NAMESPACE,
            package=CPP_PACKAGE_NAME,
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
            package=CPP_PACKAGE_NAME,
            executable="monocular_inertial_slam",
            name=f"monocular_inertial_slam_{HOSTNAME}",
            output="screen",
            remappings=[
                ("i2c_imu", f"{MCU_NODE}/i2c_imu"),
            ],
        )
        ld.add_action(monocular_inertial_slam_node)

    elif HOSTNAME == "station":
        background_modeler_node = Node(
            namespace=ROS_NAMESPACE,
            package=CPP_PACKAGE_NAME,
            executable="background_modeler",
            name=f"background_modeler_{HOSTNAME}",
            output="screen",
            remappings=[
                ("image_raw", f"{HOSTNAME}/image_raw"),
                ("image_raw/compressed", f"{HOSTNAME}/image_raw/compressed"),
                ("background", f"{HOSTNAME}/background"),
            ],
        )
        ld.add_action(background_modeler_node)


    return ld
