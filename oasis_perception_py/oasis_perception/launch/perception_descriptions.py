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
            # Reduce noisy INFO logs when loading components while retaining node logs
            arguments=[
                "--ros-args",
                "--log-level",
                f"{ROS_NAMESPACE}.perception_container_{host_id}:=warn",
            ],
            composable_node_descriptions=composable_nodes,
        )
        ld.add_action(perception_container)

    #
    # AprilTag detector
    #

    @staticmethod
    def add_apriltag_detector(
        composable_nodes: list[ComposableNode],
        system_ids: List[str],
        input_topic: str,
        input_resolution: str,
        image_transport: str,
    ) -> None:
        RESOLUTION_PREFIX = f"{input_resolution}/" if input_resolution else ""

        composable_nodes.extend(
            [
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package="apriltag_ros",
                    plugin="AprilTagNode",
                    name=f"apriltag_{system_id}",
                    parameters=[
                        {
                            "image_transport": image_transport,
                            "family": "36h11",
                            "size": 0.2,  # 200 mm tag edge size in meters (250mm total size with black border)
                            "max_hamming": 2,  # Good robustness/performance tradeoff
                            "pose_estimation_method": "pnp",
                            "qos_profile": "sensor_data",
                            #
                            # Tuning parameters
                            #
                            "detector.threads": 4,  # TODO: Detect this?
                            "detector.decimate": 1.0,  # Process at full resolution
                            "detector.blur": 0.0,  # No blur before quad detection
                            "detector.refine": True,  # Refine edges (slightly slower, more accurate corners)
                            # How much sharpening to apply to the image used for decoding the tag bits.
                            # A small positive value (like 0.25) can help with low-contrast tags.
                            # Too high can amplify noise.
                            "detector.sharpening": 0.25,
                            "detector.debug": False,  # Disable writing debug images and info to disk
                            "profile": False,  # Noisy and useless in production
                        }
                    ],
                    remappings=[
                        ("camera_info", f"{system_id}/{RESOLUTION_PREFIX}camera_info"),
                        ("image_rect", f"{system_id}/{RESOLUTION_PREFIX}{input_topic}"),
                        ("detections", f"{system_id}/{RESOLUTION_PREFIX}apriltags"),
                    ],
                )
                for system_id in system_ids
            ]
        )

    #
    # AprilTag visualizer
    #

    @staticmethod
    def add_apriltag_visualizer(
        composable_nodes: list[ComposableNode],
        system_ids: List[str],
        input_image: str,
        input_resolution: str,
        output_image: str,
        image_transport: str,
    ) -> None:
        RESOLUTION_PREFIX = f"{input_resolution}/" if input_resolution else ""

        composable_nodes.extend(
            [
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package=CPP_PACKAGE_NAME,
                    plugin="oasis_perception::AprilTagVizComponent",
                    name=f"apriltag_viz_{system_id}_{input_image}_{output_image}",
                    parameters=[
                        {
                            "system_id": system_id,
                            "image_transport": image_transport,
                        }
                    ],
                    remappings=[
                        (
                            f"{system_id}_image",
                            f"{system_id}/{RESOLUTION_PREFIX}{input_image}",
                        ),
                        (
                            f"{system_id}_apriltags",
                            f"{system_id}/{RESOLUTION_PREFIX}apriltags",
                        ),
                        (
                            f"{system_id}_apriltags_image",
                            f"{system_id}/{RESOLUTION_PREFIX}{output_image}",
                        ),
                    ],
                )
                for system_id in system_ids
            ]
        )

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
        input_resolution: str,
        output_resolution: str,
        image_transport: str,
        output_width: int = -1,
        output_height: int = -1,
        max_width: int = -1,
        max_height: int = -1,
    ) -> None:
        RESOLUTION_PREFIX = f"{input_resolution}/" if input_resolution else ""
        RESOLUTION_SUFFIX = f"_{output_resolution}" if output_resolution else ""

        size_params = {}
        if output_width > 0:
            size_params["output_width"] = output_width
        if output_height > 0:
            size_params["output_height"] = output_height
        if max_width > 0:
            size_params["max_width"] = max_width
        if max_height > 0:
            size_params["max_height"] = max_height

        composable_nodes.extend(
            [
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package=CPP_PACKAGE_NAME,
                    plugin="oasis_perception::ImageDownscalerComponent",
                    name=f"image_downscaler_{system_id}{RESOLUTION_SUFFIX}",
                    parameters=[
                        {
                            "system_id": system_id,
                            "image_transport": image_transport,
                            "output_resolution": output_resolution,
                            **size_params,
                        }
                    ],
                    remappings=[
                        (
                            f"{system_id}_camera_info",
                            f"{system_id}/{RESOLUTION_PREFIX}camera_info",
                        ),
                        (
                            f"{system_id}_camera_info_{output_resolution}",
                            f"{system_id}/{output_resolution}/camera_info",
                        ),
                        (
                            f"{system_id}_image",
                            f"{system_id}/{RESOLUTION_PREFIX}{input_topic}",
                        ),
                        (
                            f"{system_id}_image_{output_resolution}",
                            f"{system_id}/{output_resolution}/{input_topic}",
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
        input_resolution: str,
        image_transport: str,
    ) -> None:
        RESOLUTION_PREFIX = f"{input_resolution}/" if input_resolution else ""
        RESOLUTION_SUFFIX = f"_{input_resolution}" if input_resolution else ""

        composable_nodes.extend(
            [
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name=f"rectify_node_{system_id}{RESOLUTION_SUFFIX}",
                    parameters=[
                        {
                            "interpolation": 1,  # Linear
                            "image_transport": image_transport,
                        },
                    ],
                    remappings=[
                        # Topics
                        ("camera_info", f"{system_id}/{RESOLUTION_PREFIX}camera_info"),
                        ("image", f"{system_id}/{RESOLUTION_PREFIX}image_raw"),
                        ("image_rect", f"{system_id}/{RESOLUTION_PREFIX}image_rect"),
                    ],
                )
                for system_id in system_ids
            ]
        )

    #
    # Map visualization
    #

    @staticmethod
    def add_map_viz(
        composable_nodes: list[ComposableNode],
        system_ids: List[str],
        camera_name: str,
        camera_resolution: str,
        image_transport: str,
    ) -> None:
        RESOLUTION_PREFIX = f"{camera_resolution}/" if camera_resolution else ""
        RESOLUTION_SUFFIX = f"_{camera_resolution}" if camera_resolution else ""

        settings_file: str | None = PerceptionPaths.find_orb_slam_oasis_settings(
            camera_name + RESOLUTION_SUFFIX
        )
        if settings_file is None:
            raise FileNotFoundError("ORB_SLAM_OASIS settings file not found.")

        composable_nodes.extend(
            [
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package=CPP_PACKAGE_NAME,
                    plugin="oasis_perception::MapVizComponent",
                    name=f"map_viz_{system_id}",
                    parameters=[
                        {"system_id": system_id},
                        {"settings_file": settings_file},
                        {"image_transport": image_transport},
                    ],
                    remappings=[
                        (
                            f"{system_id}_apriltags",
                            f"{system_id}/{RESOLUTION_PREFIX}apriltags",
                        ),
                        (
                            f"{system_id}_camera_info",
                            f"{system_id}/{RESOLUTION_PREFIX}camera_info",
                        ),
                        (
                            f"{system_id}_slam_pose",
                            f"{system_id}/{RESOLUTION_PREFIX}slam_pose",
                        ),
                        (
                            f"{system_id}_slam_point_cloud",
                            f"{system_id}/{RESOLUTION_PREFIX}slam_point_cloud",
                        ),
                        (
                            f"{system_id}_slam_map_image",
                            f"{system_id}/{RESOLUTION_PREFIX}slam_map_image",
                        ),
                        (
                            f"{system_id}_image_raw",
                            f"{system_id}/{RESOLUTION_PREFIX}image_raw",
                        ),
                    ],
                )
                for system_id in system_ids
            ]
        )

    #
    # SLAM resolution relay
    #

    @staticmethod
    def add_slam_resolution_relay(
        ld: LaunchDescription,
        system_ids: list[str],
        input_resolution: str,
        output_resolution: str,
    ) -> None:
        node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="slam_resolution_relay",
            name=f"slam_resolution_relay_{input_resolution}_to_{output_resolution}",
            output="screen",
            parameters=[
                {"system_ids": system_ids},
                {"input_resolution": input_resolution},
                {"output_resolution": output_resolution},
            ],
        )
        ld.add_action(node)

    #
    # Mesh viewer
    #

    @staticmethod
    def add_mesh_viewer(
        composable_nodes: list[ComposableNode],
        system_ids: list[str],
    ) -> None:
        composable_nodes.extend(
            [
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package=CPP_PACKAGE_NAME,
                    plugin="oasis_perception::MeshViewerComponent",
                    name=f"mesh_viewer_{system_id}",
                    parameters=[{"system_id": system_id}],
                    remappings=[
                        (
                            f"{system_id}_slam_mesh_image",
                            f"{system_id}/slam_mesh_image",
                        ),
                        (
                            f"{system_id}_slam_point_cloud",
                            f"{system_id}/slam_point_cloud",
                        ),
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
        input_resolution: str,
    ) -> None:
        RESOLUTION_PREFIX = f"{input_resolution}/" if input_resolution else ""
        RESOLUTION_SUFFIX = f"_{input_resolution}" if input_resolution else ""

        vocabulary_file: str | None = PerceptionPaths.find_orb_slam_oasis_vocabulary()
        if vocabulary_file is None:
            raise FileNotFoundError("ORB_SLAM_OASIS vocabulary file not found.")

        settings_file: str | None = PerceptionPaths.find_orb_slam_oasis_settings(
            camera_name + RESOLUTION_SUFFIX
        )
        if settings_file is None:
            raise FileNotFoundError("ORB_SLAM_OASIS settings file not found.")

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
                                else f"{system_id}/{RESOLUTION_PREFIX}image_raw"
                            ),
                        ),
                        (
                            f"{system_id}_slam_point_cloud",
                            f"{system_id}/{RESOLUTION_PREFIX}slam_point_cloud",
                        ),
                        (
                            f"{system_id}_slam_pose",
                            f"{system_id}/{RESOLUTION_PREFIX}slam_pose",
                        ),
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
        input_resolution: str,
    ) -> None:
        RESOLUTION_PREFIX = f"{input_resolution}/" if input_resolution else ""
        RESOLUTION_SUFFIX = f"_{input_resolution}" if input_resolution else ""

        vocabulary_file: str | None = PerceptionPaths.find_orb_slam_oasis_vocabulary()
        if vocabulary_file is None:
            raise FileNotFoundError("ORB_SLAM_OASIS vocabulary file not found.")

        settings_file: str | None = PerceptionPaths.find_orb_slam_oasis_settings(
            camera_name + RESOLUTION_SUFFIX
        )
        if settings_file is None:
            raise FileNotFoundError("ORB_SLAM_OASIS settings file not found.")

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
                                else f"{system_id}/{RESOLUTION_PREFIX}image_raw"
                            ),
                        ),
                        (f"{system_id}_imu", f"{system_id}/i2c_imu"),
                        (
                            f"{system_id}_slam_point_cloud",
                            f"{system_id}/{RESOLUTION_PREFIX}slam_point_cloud",
                        ),
                        (
                            f"{system_id}_slam_pose",
                            f"{system_id}/{RESOLUTION_PREFIX}slam_pose",
                        ),
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
                                else f"{system_id}/sd/image_raw"
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
    def add_pose_landmarker(
        ld: LaunchDescription,
        zone_id: str,
        input_topic: str,
        input_resolution: str,
        image_transport: str,
    ) -> None:
        RESOLUTION_PREFIX = f"{input_resolution}/" if input_resolution else ""

        node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="pose_landmarker",
            name=f"pose_landmarker_{zone_id}",
            output="screen",
            parameters=[
                {
                    "image_transport": image_transport,
                }
            ],
            remappings=[
                # Topics
                ("camera_scene", f"{zone_id}/{RESOLUTION_PREFIX}camera_scene"),
                ("image", f"{zone_id}/{RESOLUTION_PREFIX}{input_topic}"),
                ("pose_landmarks", f"{zone_id}/{RESOLUTION_PREFIX}pose_landmarks"),
                ("pose_image", f"{zone_id}/{RESOLUTION_PREFIX}pose_image"),
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
