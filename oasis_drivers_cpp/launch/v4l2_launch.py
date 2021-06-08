################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of Oasis - https://github.com/eigendude/oasis
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from launch import LaunchDescription
from launch_ros.actions import Node


# TODO
ROS_NAMESPACE = "oasis"

# TODO: Hardware/video parameters
MACHINE = "lenovo"
VIDEO_DEVICE = "/dev/video0"
IMAGE_SIZE = [640, 480]


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    v4l2_node = Node(
        namespace=ROS_NAMESPACE,
        package="v4l2_camera",
        executable="v4l2_camera_node",
        output="screen",
        parameters=[
            {
                "image_size": IMAGE_SIZE,
                "video_device": VIDEO_DEVICE,
            },
        ],
        remappings=[
            ("camera_info", f"{MACHINE}/camera_info"),
            ("image_raw", f"{MACHINE}/image_raw"),
            ("image_raw/compressed", f"{MACHINE}/image_raw/compressed"),
            ("image_raw/compressedDepth", f"{MACHINE}/image_raw/compressedDepth"),
            ("image_raw/theora", f"{MACHINE}/image_raw/theora"),
        ],
    )

    ld.add_action(v4l2_node)

    return ld
