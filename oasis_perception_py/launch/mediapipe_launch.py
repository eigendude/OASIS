################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from launch.launch_description import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from oasis_perception.launch.perception_descriptions import (
    COMPONENT_CONSOLE_OUTPUT_FORMAT,
)
from oasis_perception.launch.perception_descriptions import CPP_PACKAGE_NAME
from oasis_perception.launch.perception_descriptions import ROS_NAMESPACE


def generate_launch_description() -> LaunchDescription:
    container: ComposableNodeContainer = ComposableNodeContainer(
        namespace=ROS_NAMESPACE,
        name="mediapipe_container",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        arguments=["--executor-type", "multi-threaded"],
        additional_env={
            "RCUTILS_CONSOLE_OUTPUT_FORMAT": COMPONENT_CONSOLE_OUTPUT_FORMAT,
        },
        composable_node_descriptions=[
            ComposableNode(
                namespace=ROS_NAMESPACE,
                package=CPP_PACKAGE_NAME,
                plugin="oasis_perception::HelloWorldComponent",
                name="hello_world",
            ),
            ComposableNode(
                namespace=ROS_NAMESPACE,
                package=CPP_PACKAGE_NAME,
                plugin="oasis_perception::PoseLandmarkerComponent",
                name="pose_landmarker",
            ),
        ],
    )

    return LaunchDescription([container])
