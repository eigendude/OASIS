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
HOSTNAME = socket.gethostname().replace("-", "_")


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE = "oasis"

PYTHON_PACKAGE_NAME = "oasis_perception_py"


################################################################################
# Launch description
################################################################################


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    # Camera calibrator node
    calibrator_node: Node = Node(
        package=PYTHON_PACKAGE_NAME,
        executable="camera_calibrator",
        name=f"camera_calibrator_{HOSTNAME}",
        namespace=ROS_NAMESPACE,
        output="screen",
        remappings=[
            ("calibration", f"{HOSTNAME}/calibration"),
            ("camera/set_camera_info", f"v4l2_camera_{HOSTNAME}/set_camera_info"),
            ("image", f"{HOSTNAME}/image_raw"),
        ],
    )
    ld.add_action(calibrator_node)

    return ld
