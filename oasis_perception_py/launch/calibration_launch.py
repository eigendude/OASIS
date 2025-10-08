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
# System parameters
################################################################################

CONFIG: SmarthomeConfig = SmarthomeConfig()

# Get the hostname and host alias
HOSTNAME: str = CONFIG.HOSTNAME
HOST_ID: str = CONFIG.HOST_ID

# Zone configuration
ZONE_ID: str = CONFIG.ZONE_ID

# Camera driver configuration
CAMERA_DRIVER: str = CONFIG.get_camera_driver(HOST_ID, ZONE_ID)


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

    camera_driver: str = CAMERA_DRIVER or "v4l2_camera"
    if camera_driver == "v4l2_camera":
        camera_node: str = f"v4l2_camera_{HOSTNAME}"
    elif camera_driver == "camera_ros":
        camera_node = f"camera_ros_{HOSTNAME}"
    else:
        raise ValueError(f"Unsupported camera driver: {camera_driver}")

    # Camera calibrator node
    calibrator_node: Node = Node(
        package=PYTHON_PACKAGE_NAME,
        executable="camera_calibrator",
        name=f"camera_calibrator_{HOSTNAME}",
        namespace=ROS_NAMESPACE,
        output="screen",
        remappings=[
            ("calibration", f"{HOSTNAME}/calibration"),
            ("camera/set_camera_info", f"{camera_node}/set_camera_info"),
            ("image", f"{HOSTNAME}/image_raw"),
        ],
    )
    ld.add_action(calibrator_node)

    return ld
