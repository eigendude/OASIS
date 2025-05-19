################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE: str = "oasis"

PERCEPTION_CPP_PACKAGE_NAME: str = "oasis_perception_cpp"


################################################################################
# Node definitions
################################################################################


#
# Hello World
#


def add_hello_world(ld: LaunchDescription) -> None:
    #
    # NOTE:
    #
    # When using Abseil within a ROS 2 `ComposableNode` deployed inside a
    # `component_container` or `component_container_mt`, the runtime crashes
    # due to duplicate Abseil symbols.
    #
    # This issue arises from Abseil's design: it provides many inline
    # functions and templates in headers, which means every Translation Unit (TU)
    # that includes Abseil headers instantiates its own copy of internal symbols.
    # When multiple `.so` plugins (shared libraries) are built independently,
    # each can embed its own versions of Abseil internals.
    #
    # At runtime, the ROS 2 component manager (`component_container_mt`)
    # loads all these plugins with `dlopen()`. The loader does not deduplicate
    # symbols across `.so`s, so this violates the One-Definition Rule (ODR),
    # leading to undefined behavior, typically manifesting as a segmentation fault.
    #
    # In our case, the crash occurred when linking the `mediapipe_monolithic`
    # library (built with Bazel) into a ROS-built component. The node ran fine
    # standalone via `ros2 run`, but failed once loaded into the
    # `component_container_mt` due to Abseil duplication at runtime.
    #
    # POTENTIAL SOLUTION:
    #
    # Patch MediaPipe to provide a stable, isolated interface that avoids
    # leaking Abseil symbols into shared libraries. For example:
    #
    #   - Create a façade API or DTO-style interface in a single .cc file
    #     (e.g. `mediapipe_safe_api.cc`) that implements a clean C++ API boundary.
    #   - Internally link against Abseil, but return plain-old-data (POD)
    #     types such as `bool`, enums, or structs — not `absl::Status`.
    #   - Avoid any Abseil headers in `.h` files that might be included in
    #     downstream nodes or components.
    #
    # This technique consolidates Abseil usage to a single TU and reduces
    # symbol duplication across `.so` boundaries, allowing MediaPipe to be
    # safely used in multi-component ROS systems.
    #
    hello_world_container: ComposableNodeContainer = ComposableNodeContainer(
        namespace=ROS_NAMESPACE,
        name="hello_world_container",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                namespace=ROS_NAMESPACE,
                package=PERCEPTION_CPP_PACKAGE_NAME,
                plugin="oasis_perception::HelloWorldComponent",
                name="hello_world",
            ),
        ],
    )
    ld.add_action(hello_world_container)


################################################################################
# Launch description
################################################################################


def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()

    add_hello_world(ld)

    return ld
