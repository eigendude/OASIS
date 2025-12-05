################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from launch.launch_description import LaunchDescription
from launch_ros.descriptions import ComposableNode
from oasis_perception.launch.perception_descriptions import PerceptionDescriptions

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

# TODO: Select camera name from configuration
SLAM_CAMERA_NAME: str = (
    "imx708_wide__base_axi_pcie_120000_rp1_i2c_80000_imx708_1a_1920x1080_2304x1296_BGGR_PISP_COMP1_RAW"
)

print(f"Launching perception on {HOSTNAME} in zone {ZONE_ID}")


################################################################################
# Launch description
################################################################################


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    composable_nodes: list[ComposableNode] = []

    # Vision pipeline
    if HOST_ID == "vision":
        # Ingestion: Single source, fanout to nodes
        PerceptionDescriptions.add_image_downscaler(
            composable_nodes,
            ["falcon"],
            input_topic="image_raw",
            input_resolution="",
            output_resolution="hd",
            image_transport="compressed",
            max_width=1920,
            max_height=1080,
        )
        PerceptionDescriptions.add_image_downscaler(
            composable_nodes,
            ["falcon"],
            input_topic="image_raw",
            input_resolution="",
            output_resolution="hd720",
            image_transport="compressed",
            max_width=1280,
            max_height=720,
        )
        PerceptionDescriptions.add_image_downscaler(
            composable_nodes,
            ["falcon", "station"],
            input_topic="image_raw",
            input_resolution="",
            output_resolution="sd",
            image_transport="compressed",
            max_width=640,
            max_height=480,
        )
        PerceptionDescriptions.add_image_downscaler(
            composable_nodes,
            ["hallway"],
            input_topic="image_color",
            input_resolution="qhd",
            output_resolution="sd",
            image_transport="compressed",
            max_width=640,
            max_height=480,
        )

        # Image rectification
        PerceptionDescriptions.add_image_rectifier(
            composable_nodes,
            ["falcon"],
            input_resolution="hd",
            image_transport="raw",
        )
        PerceptionDescriptions.add_image_rectifier(
            composable_nodes,
            ["falcon"],
            input_resolution="sd",
            image_transport="raw",
        )

        # AprilTag detection
        PerceptionDescriptions.add_apriltag_detector(
            composable_nodes,
            ["falcon"],
            input_topic="image_rect",
            input_resolution="hd",
            image_transport="raw",
        )

        # Pose landmarking
        PerceptionDescriptions.add_pose_landmarker(
            ld,
            "falcon",
            input_topic="image_rect",
            input_resolution="sd",
            image_transport="raw",
        )
        PerceptionDescriptions.add_pose_landmarker(
            ld,
            "hallway",
            input_topic="image_color",
            input_resolution="sd",
            image_transport="raw",
        )

        # Monocular SLAM
        PerceptionDescriptions.add_monocular_slam(
            composable_nodes,
            ["falcon"],
            image_transport="raw",
            camera_name=SLAM_CAMERA_NAME,
            input_resolution="hd720",
        )

        # Republish SLAM outputs under the hd namespace for 1080p pipelines
        PerceptionDescriptions.add_slam_resolution_relay(
            ld,
            ["falcon"],
            input_resolution="hd720",
            output_resolution="hd",
        )

        # Calibration demo (TODO: separate checkerboard detection)
        PerceptionDescriptions.add_calibration(
            ld,
            system_id="station",
            camera_model="pinhole",
            input_resolution="sd",
            # TODO: image_transport="compressed",
        )

    # Rendering pipeline
    if HOST_ID == "vision":
        # AprilTag visualization
        PerceptionDescriptions.add_apriltag_visualizer(
            composable_nodes,
            ["falcon"],
            input_image="image_raw",
            input_resolution="hd",
            output_image="apriltags_image",
            image_transport="compressed",
        )

        # Map visualization
        PerceptionDescriptions.add_map_viz(
            composable_nodes,
            ["falcon"],
            camera_name=SLAM_CAMERA_NAME,
            camera_resolution="hd",
            image_transport="compressed",
        )

    PerceptionDescriptions.add_perception_components(ld, HOST_ID, composable_nodes)

    return ld
