################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from typing import Any
from typing import Optional

from launch.launch_description import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

from oasis_hass.utils.smarthome_config import SmarthomeConfig


################################################################################
# Smarthome parameters
################################################################################


CONFIG: SmarthomeConfig = SmarthomeConfig()

# Get the hostname
HOSTNAME: str = CONFIG.HOSTNAME

# Host aliases
HOST_ID: str = CONFIG.HOST_ID

# Zone configuration
ZONE_ID: str = CONFIG.ZONE_ID

# The host and zone IDs used for Home Assistant
HOME_ASSISTANT_ID: str = CONFIG.HOME_ASSISTANT_ID

# The zone ID used for the Kinect V2 camera
KINECT_V2_ZONE_ID: str = CONFIG.KINECT_V2_ZONE_ID

# Path to the MQTT parameters file for Home Assistant
MQTT_PARAMS_FILE: str = CONFIG.MQTT_PARAMS_FILE


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE: str = "oasis"

PYTHON_PACKAGE_NAME: str = "oasis_drivers_py"
HASS_PACKAGE_NAME: str = CONFIG.HASS_PACKAGE_NAME


################################################################################
# Driver nodes
################################################################################


class DriverDescriptions:
    #
    # Helper function
    #

    @staticmethod
    def add_driver_components(
        ld: LaunchDescription,
        host_id: str,
        composable_nodes: list[ComposableNode],
        log_level: Optional[str] = None,
    ) -> None:
        if not composable_nodes:
            return

        container_arguments: list[str] | None = None
        if log_level is not None:
            container_arguments = ["--ros-args", "--log-level", log_level]

        driver_container: ComposableNodeContainer = ComposableNodeContainer(
            namespace=ROS_NAMESPACE,
            name=f"driver_container_{host_id}",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            arguments=container_arguments,
            composable_node_descriptions=composable_nodes,
        )
        ld.add_action(driver_container)

    #
    # Display server
    #

    @staticmethod
    def add_display_server(ld: LaunchDescription, zone_id: str) -> None:
        display_server_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="display_server",
            name=f"display_server_{zone_id}",
            output="screen",
            remappings=[
                # Topics
                ("set_display", f"{zone_id}/set_display"),
            ],
        )
        ld.add_action(display_server_node)

    #
    # Kinect V2 camera
    #

    @staticmethod
    def add_kinect_v2(ld: LaunchDescription, zone_id: str) -> None:
        # TODO: For now use a separate node for the main Kinect V2 bridge.
        # Running as a component breaks the legacy image transport code.
        kinect2_bridge_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package="kinect2_bridge",
            executable="kinect2_bridge",
            name=f"kinect2_bridge_{zone_id}",
            output="screen",
            parameters=[
                {"base_name": zone_id, "reg_method": "opencl"},
            ],
        )
        ld.add_action(kinect2_bridge_node)

        kinect2_container: ComposableNodeContainer = ComposableNodeContainer(
            namespace=ROS_NAMESPACE,
            name=f"kinect2_container_{zone_id}",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[
                # TODO: Composable nodes seem to break when not using the modern image_transport API
                # ComposableNode(
                #     package="kinect2_bridge",
                #     plugin="kinect2_bridge::Kinect2BridgeComponent",
                #     name=f"kinect2_bridge_{zone_id}",
                # ),
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package="kinect2_bridge",
                    plugin="kinect2_bridge::Kinect2DownscalerComponent",
                    name=f"kinect2_downscaler_{zone_id}",
                    parameters=[
                        {
                            "base_name": zone_id,
                        },
                    ],
                ),
            ],
        )
        ld.add_action(kinect2_container)

    #
    # ROS2 (libcamera) camera
    #

    @staticmethod
    def add_ros2_camera(
        composable_nodes: list[ComposableNode],
        zone_id: str,
        image_format: str,
        image_size: list[int],
        sensor_mode: str,
        libcamera_params: Optional[dict[str, object]] = None,
        rectify: bool = False,
    ) -> None:
        camera_parameters: dict[str, Any] = {
            "role": "video",
            "format": image_format,
            "width": image_size[0],
            "height": image_size[1],
            "sensor_mode": sensor_mode,
        }

        # Merge explicit libcamera params from top-level; explicit wins
        if libcamera_params:
            camera_parameters.update(libcamera_params)

        composable_nodes.append(
            ComposableNode(
                namespace=ROS_NAMESPACE,
                package="camera_ros",
                plugin="camera::CameraNode",
                name=f"camera_ros_{zone_id}",
                parameters=[
                    camera_parameters,
                ],
                remappings=[
                    # Topics
                    (
                        f"camera_ros_{zone_id}/camera_info",
                        f"{zone_id}/camera_info",
                    ),
                    (
                        f"camera_ros_{zone_id}/image_raw",
                        f"{zone_id}/image_raw",
                    ),
                    # TODO: Need to remap compressed image topic?
                    (
                        f"camera_ros_{zone_id}/image_raw/compressed",
                        f"{zone_id}/image_raw/compressed",
                    ),
                ],
            )
        )

        if rectify:
            composable_nodes.append(
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name=f"rectify_node_{zone_id}",
                    parameters=[
                        {"interpolation": 1},  # Linear
                    ],
                    remappings=[
                        # Topics
                        ("camera_info", f"{zone_id}/camera_info"),
                        ("image_rect", f"{zone_id}/image_rect"),
                        ("image", f"{zone_id}/image_raw"),
                    ],
                ),
            )

    #
    # Serial port scanner
    #

    @staticmethod
    def add_serial_port_scanner(ld: LaunchDescription, host_id: str) -> None:
        serial_port_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="serial_scanner",
            name=f"serial_scanner_{host_id}",
            output="screen",
            remappings=[
                # Topics
                ("serial_ports", f"{host_id}/serial_ports"),
            ],
        )
        ld.add_action(serial_port_node)

    #
    # System monitor
    #

    @staticmethod
    def add_system_monitor(ld: LaunchDescription, host_id: str) -> None:
        system_monitor_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="system_monitor",
            name=f"system_monitor_{host_id}",
            output="screen",
            remappings=[
                # Topics
                ("system_telemetry", f"{host_id}/system_telemetry"),
            ],
        )
        ld.add_action(system_monitor_node)

    #
    # UPS server
    #

    @staticmethod
    def add_ups_server(ld: LaunchDescription, system_id: str) -> None:
        ups_server_node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="ups_server",
            name=f"ups_server_{system_id}",
            output="screen",
            remappings=[
                ("ups_status", f"{system_id}/ups_status"),
                ("ups_command", f"{system_id}/ups_command"),
            ],
        )
        ld.add_action(ups_server_node)

    #
    # V4L2 camera
    #

    @staticmethod
    def add_v4l2_camera(
        ld: LaunchDescription, zone_id: str, video_device: str, image_size: list[int]
    ) -> None:
        video_container: ComposableNodeContainer = ComposableNodeContainer(
            namespace=ROS_NAMESPACE,
            name=f"video_container_{zone_id}",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package="v4l2_camera",
                    plugin="v4l2_camera::V4L2Camera",
                    name=f"v4l2_camera_{zone_id}",
                    parameters=[
                        {
                            "image_size": image_size,
                            "video_device": video_device,
                        },
                    ],
                    remappings=[
                        # Topics
                        ("camera_info", f"{zone_id}/camera_info"),
                        ("image_raw", f"{zone_id}/image_raw"),
                        ("image_rect", f"{zone_id}/image_rect"),
                        # Services
                        (
                            f"v4l2_camera_{zone_id}/set_camera_info",
                            f"{zone_id}/set_camera_info",
                        ),
                    ],
                ),
                ComposableNode(
                    namespace=ROS_NAMESPACE,
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name=f"rectify_node_{zone_id}",
                    parameters=[
                        {"image_transport": "compressed", "interpolation": 1},  # Linear
                    ],
                    remappings=[
                        # Topics
                        ("camera_info", f"{zone_id}/camera_info"),
                        ("image_rect", f"{zone_id}/image_rect"),
                        ("image", f"{zone_id}/image_raw"),
                    ],
                ),
            ],
        )
        ld.add_action(video_container)

    #
    # WoL server
    #

    @staticmethod
    def add_wol_server(ld: LaunchDescription, system_id: str) -> None:
        wol_server_node: Node = Node(
            namespace=ROS_NAMESPACE,
            package=PYTHON_PACKAGE_NAME,
            executable="wol_server",
            name="wol_server",
            output="screen",
            remappings=[
                # Services
                ("get_mac_address", f"{system_id}/get_mac_address"),
                ("wol", f"{system_id}/wol"),
            ],
        )
        ld.add_action(wol_server_node)  #
