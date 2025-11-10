################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from typing import List

from launch.launch_description import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from oasis_perception.utils.perception_paths import PerceptionPaths

from oasis_hass.utils.smarthome_config import SmarthomeConfig


################################################################################
# Smarthome parameters
################################################################################


CONFIG: SmarthomeConfig = SmarthomeConfig()

# The zone ID used for the Kinect V2 camera
KINECT_V2_ZONE_ID: str = CONFIG.KINECT_V2_ZONE_ID


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE: str = "oasis"

CPP_PACKAGE_NAME: str = "oasis_perception_cpp"
PYTHON_PACKAGE_NAME: str = "oasis_perception_py"


################################################################################
# Node descriptions
################################################################################


class PerceptionDescriptions:
    #
    # Helper function
    #

    @staticmethod
    def add_perception_components(
        ld: LaunchDescription, host_id: str, composable_nodes: list[ComposableNode]
    ) -> None:
        if not composable_nodes:
            return

        perception_container: ComposableNodeContainer = ComposableNodeContainer(
            namespace=ROS_NAMESPACE,
            name=f"perception_container_{host_id}",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=composable_nodes,
        )
        ld.add_action(perception_container)

    #
    # Background modeler
    #

    @staticmethod
    def add_background_modeler(
        composable_nodes: list[ComposableNode], system_ids: List[str]
    ) -> None:
        composable_nodes.extend(
            [
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package=CPP_PACKAGE_NAME,
                    plugin="oasis_perception_cpp::BackgroundModelerComponent",
                    name=f"background_modeler_{system_id}",
                    parameters=[{"system_id": system_id}],
                    remappings=[
                        (
                            f"{system_id}_image",
                            (
                                # Use different remappings for Kinect V2
                                f"{system_id}/sd/image_color"
                                if system_id == KINECT_V2_ZONE_ID
                                else f"{system_id}/image_rect"
                            ),
                        ),
                        (f"{system_id}_background", f"{system_id}/background"),
                    ],
                )
                for system_id in system_ids
            ]
        )

    #
    # Background subtractor
    #

    @staticmethod
    def add_background_subtractor(
        composable_nodes: list[ComposableNode], system_ids: List[str]
    ) -> None:
        composable_nodes.extend(
            [
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package=CPP_PACKAGE_NAME,
                    plugin="oasis_perception_cpp::BackgroundSubtractorComponent",
                    name=f"background_subtractor_{system_id}",
                    parameters=[{"system_id": system_id}],
                    remappings=[
                        (
                            f"{system_id}_image",
                            (
                                # Use different remappings for Kinect V2
                                f"{system_id}/sd/image_color"
                                if system_id == KINECT_V2_ZONE_ID
                                else f"{system_id}/image_rect"
                            ),
                        ),
                        (f"{system_id}_foreground", f"{system_id}/foreground"),
                        (f"{system_id}_subtracted", f"{system_id}/subtracted"),
                    ],
                )
                for system_id in system_ids
            ]
        )

    #
    # Calibration node
    #

    @staticmethod
    def add_calibration(
        ld: LaunchDescription, system_id: str, camera_model: str
    ) -> None:
        # Get camera driver from smarthome configuration
        camera_driver: str = CONFIG.get_camera_driver(system_id)

        if camera_driver:
            camera_node: str = {
                "libcamera": f"camera_ros_{system_id}",
                "v4l2": f"v4l2_camera_{system_id}",
            }[camera_driver]

            calibrator_node: Node = Node(
                namespace=ROS_NAMESPACE,
                package=PYTHON_PACKAGE_NAME,
                executable="camera_calibrator",
                name=f"camera_calibrator_{system_id}",
                output="screen",
                parameters=[{"camera_model": camera_model}],
                remappings=[
                    # Topics
                    ("calibration_image", f"{system_id}/calibration_image"),
                    ("calibration_status", f"{system_id}/calibration_status"),
                    ("image", f"{system_id}/image_raw"),
                    # Services
                    ("camera/set_camera_info", f"{camera_node}/set_camera_info"),
                ],
            )
            ld.add_action(calibrator_node)

    #
    # Hello World
    #

    @staticmethod
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
                    package=CPP_PACKAGE_NAME,
                    plugin="oasis_perception::HelloWorldComponent",
                    name="hello_world",
                ),
            ],
        )
        ld.add_action(hello_world_container)

    #
    # Image downscaler
    #

    @staticmethod
    def add_image_downscaler(
        composable_nodes: list[ComposableNode],
        system_ids: list[str],
        input_topic: str,
        output_suffix: str,
        image_transport: str,
        max_width: int,
        max_height: int,
    ) -> None:
        parameter_suffix: str = f"{input_topic}/{output_suffix}"

        composable_nodes.extend(
            [
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package=CPP_PACKAGE_NAME,
                    plugin="oasis_perception::ImageDownscalerComponent",
                    name=f"image_downscaler_{system_id}",
                    parameters=[
                        {
                            "system_id": system_id,
                            "image_transport": image_transport,
                            "output_suffix": parameter_suffix,
                            "max_width": max_width,
                            "max_height": max_height,
                        }
                    ],
                    remappings=[
                        (f"{system_id}_image", f"{system_id}/{input_topic}"),
                        (
                            f"{system_id}_{parameter_suffix}",
                            f"{system_id}/{input_topic}/{output_suffix}",
                        ),
                    ],
                )
                for system_id in system_ids
            ]
        )

    #
    # Image rectifier
    #

    @staticmethod
    def add_image_rectifier(
        composable_nodes: list[ComposableNode],
        system_ids: list[str],
        image_transport: str,
    ) -> None:
        composable_nodes.extend(
            [
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name=f"rectify_node_{system_id}",
                    parameters=[
                        {
                            "interpolation": 1,  # Linear
                            "image_transport": image_transport,
                        },
                    ],
                    remappings=[
                        # Topics
                        ("camera_info", f"{system_id}/camera_info"),
                        ("image_rect", f"{system_id}/image_rect"),
                        ("image", f"{system_id}/image_raw"),
                    ],
                )
                for system_id in system_ids
            ]
        )

    #
    # Monocular SLAM
    #

    @staticmethod
    def add_monocular_slam(
        composable_nodes: list[ComposableNode],
        system_ids: List[str],
        image_transport: str,
        camera_name: str,
    ) -> None:
        vocabulary_file: str | None = PerceptionPaths.find_orb_slam3_vocabulary()
        if vocabulary_file is None:
            raise FileNotFoundError("ORB_SLAM3 vocabulary file not found.")

        settings_file: str | None = PerceptionPaths.find_orb_slam3_settings(camera_name)
        if settings_file is None:
            raise FileNotFoundError("ORB_SLAM3 settings file not found.")

        composable_nodes.extend(
            [
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package=CPP_PACKAGE_NAME,
                    plugin="oasis_perception::MonocularSlamComponent",
                    name=f"monocular_slam_{system_id}",
                    parameters=[
                        {
                            "system_id": system_id,
                            "image_transport": image_transport,
                            "vocabulary_file": vocabulary_file,
                            "settings_file": settings_file,
                        }
                    ],
                    remappings=[
                        (
                            f"{system_id}_image",
                            (
                                # Use different remappings for Kinect V2
                                f"{system_id}/sd/image_color"
                                if system_id == KINECT_V2_ZONE_ID
                                else f"{system_id}/image_raw"
                            ),
                        ),
                        (f"{system_id}_slam_map_image", f"{system_id}/slam_map_image"),
                    ],
                )
                for system_id in system_ids
            ]
        )

    #
    # Monocular inertial SLAM
    #

    @staticmethod
    def add_monocular_inertial_slam(
        composable_nodes: list[ComposableNode],
        system_ids: List[str],
        image_transport: str,
        camera_name: str,
    ) -> None:
        vocabulary_file: str | None = PerceptionPaths.find_orb_slam3_vocabulary()
        if vocabulary_file is None:
            raise FileNotFoundError("ORB_SLAM3 vocabulary file not found.")

        settings_file: str | None = PerceptionPaths.find_orb_slam3_settings(camera_name)
        if settings_file is None:
            raise FileNotFoundError("ORB_SLAM3 settings file not found.")

        composable_nodes.extend(
            [
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package=CPP_PACKAGE_NAME,
                    plugin="oasis_perception::MonocularInertialSlamComponent",
                    name=f"monocular_inertial_slam_{system_id}",
                    parameters=[
                        {
                            "system_id": system_id,
                            "image_transport": image_transport,
                            "vocabulary_file": vocabulary_file,
                            "settings_file": settings_file,
                        }
                    ],
                    remappings=[
                        (
                            f"{system_id}_image",
                            (
                                # Use different remappings for Kinect V2
                                f"{system_id}/sd/image_color"
                                if system_id == KINECT_V2_ZONE_ID
                                else f"{system_id}/image_rect"
                            ),
                        ),
                        (f"{system_id}_imu", f"{system_id}/i2c_imu"),
                        (f"{system_id}_slam_map_image", f"{system_id}/slam_map_image"),
                    ],
                )
                for system_id in system_ids
            ]
        )

    #
    # Optical flow
    #

    @staticmethod
    def add_optical_flow(
        composable_nodes: list[ComposableNode],
        system_ids: List[str],
        image_transport: str,
    ) -> None:
        composable_nodes.extend(
            [
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package=CPP_PACKAGE_NAME,
                    plugin="oasis_perception::OpticalFlowComponent",
                    name=f"optical_flow_{system_id}",
                    parameters=[
                        {
                            "system_id": system_id,
                            "image_transport": image_transport,
                        }
                    ],
                    remappings=[
                        (
                            f"{system_id}_image",
                            (
                                # Use different remappings for Kinect V2
                                f"{system_id}/sd/image_color"
                                if system_id == KINECT_V2_ZONE_ID
                                else f"{system_id}/image_raw"
                            ),
                        ),
                        (f"{system_id}_flow_image", f"{system_id}/flow_image"),
                        (f"{system_id}_scene_score", f"{system_id}/scene_score"),
                    ],
                )
                for system_id in system_ids
            ]
        )

    #
    # Pose landmarker
    #

    @staticmethod
    def add_pose_landmarker(ld: LaunchDescription, zone_id: str) -> None:
        node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="pose_landmarker",
            name=f"pose_landmarker_{zone_id}",
            output="screen",
            remappings=[
                # Topics
                ("camera_scene", f"{zone_id}/camera_scene"),
                (
                    "image",
                    (
                        # Use different remappings for Kinect V2
                        f"{zone_id}/sd/image_color"
                        if zone_id == KINECT_V2_ZONE_ID
                        else f"{zone_id}/image_rect"
                    ),
                ),
                ("pose", f"{zone_id}/pose"),
                ("pose_image", f"{zone_id}/pose_image"),
            ],
        )
        ld.add_action(node)

    #
    # Pose renderer
    #

    @staticmethod
    def add_pose_renderer(
        ld: LaunchDescription, zone_ids: list[str], host_id: str
    ) -> None:
        """
        Launch a pose_renderer that subscribes to multiple
        /oasis/<zone>/pose topics and publishes transparent
        overlays on /oasis/<zone>/pose_image_<host_id>.
        """
        node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="pose_renderer",
            name=f"pose_renderer_{host_id}",
            output="screen",
            parameters=[
                {"host_id": host_id},
                {"zone_ids": zone_ids},
            ],
        )
        ld.add_action(node)
