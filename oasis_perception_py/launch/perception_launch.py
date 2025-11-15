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

# Zones with a smart display that can be controlled
SMART_DISPLAY_ZONES: list[str] = CONFIG.SMART_DISPLAY_ZONES

# Zones with a camera feed
CAMERA_ZONES: list[str] = CONFIG.CAMERA_ZONES

# TODO: Select "pinhole" or "fisheye" from configuration
CAMERA_MODEL: str = "fisheye"

# TODO: Select camera name from configuration
CAMERA_NAME: str = (
    "imx708_wide__base_axi_pcie_120000_rp1_i2c_80000_imx708_1a_1920x1080_2304x1296_BGGR_PISP_COMP1_RAW"
)

print(f"Launching perception on {HOSTNAME} in zone {ZONE_ID}")


################################################################################
# Hardware configuration
################################################################################


PERCEPTION_SERVER_APRILTAGS: list[str] = []
PERCEPTION_SERVER_BACKGROUND: list[str] = []
PERCEPTION_SERVER_CALIBRATION: list[str] = []
PERCEPTION_SERVER_FLOW: list[str] = []
PERCEPTION_SERVER_IMAGE_DOWNSCALER: list[str] = []
PERCEPTION_SERVER_IMAGE_RECT: list[str] = []
PERCEPTION_SERVER_MESH_VIEWER: list[str] = []
PERCEPTION_SERVER_MONOCULAR_SLAM: list[str] = []
PERCEPTION_SERVER_MONOCULAR_INERTIAL_SLAM: list[str] = []
PERCEPTION_SERVER_POSE_LANDMARKS: list[str] = []


if HOST_ID == "falcon":
    # PERCEPTION_SERVER_FLOW.extend(["falcon"])
    pass
elif HOST_ID == "nas":
    # PERCEPTION_SERVER_CALIBRATION.extend(
    #     ["bar", "doorbell", "entryway", "hallway", "kitchen", "livingroom"]
    # )
    PERCEPTION_SERVER_POSE_LANDMARKS.extend(["hallway"])
elif HOST_ID == "oceanplatform":
    PERCEPTION_SERVER_APRILTAGS.extend(["falcon"])
    # PERCEPTION_SERVER_BACKGROUND.extend(["station"])
    # PERCEPTION_SERVER_FLOW.extend(["falcon", "station"])
    PERCEPTION_SERVER_IMAGE_DOWNSCALER.extend(["falcon"])
    PERCEPTION_SERVER_IMAGE_RECT.extend(["falcon"])
    # PERCEPTION_SERVER_MESH_VIEWER.extend(["falcon"])
    PERCEPTION_SERVER_MONOCULAR_SLAM.extend(["falcon"])
    PERCEPTION_SERVER_POSE_LANDMARKS.extend(["falcon"])


################################################################################
# Launch description
################################################################################


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    composable_nodes: list[ComposableNode] = []

    if PERCEPTION_SERVER_APRILTAGS:
        PerceptionDescriptions.add_apriltag_detector(
            composable_nodes, PERCEPTION_SERVER_APRILTAGS, image_transport="raw"
        )

    if PERCEPTION_SERVER_BACKGROUND:
        PerceptionDescriptions.add_background_modeler(
            composable_nodes, PERCEPTION_SERVER_BACKGROUND
        )
        PerceptionDescriptions.add_background_subtractor(
            composable_nodes, PERCEPTION_SERVER_BACKGROUND
        )

    if PERCEPTION_SERVER_CALIBRATION:
        for host_id in PERCEPTION_SERVER_CALIBRATION:
            PerceptionDescriptions.add_calibration(ld, host_id, CAMERA_MODEL)

    if PERCEPTION_SERVER_FLOW:
        PerceptionDescriptions.add_optical_flow(
            composable_nodes, PERCEPTION_SERVER_FLOW, image_transport="compressed"
        )

    if PERCEPTION_SERVER_IMAGE_DOWNSCALER:
        PerceptionDescriptions.add_image_downscaler(
            composable_nodes,
            PERCEPTION_SERVER_IMAGE_DOWNSCALER,
            input_topic="image_raw",
            output_resolution="sd",
            image_transport="compressed",
            max_width=640,
            max_height=480,
        )

    if PERCEPTION_SERVER_IMAGE_RECT:
        PerceptionDescriptions.add_image_rectifier(
            composable_nodes,
            PERCEPTION_SERVER_IMAGE_RECT,
            input_resolution="sd",
            image_transport="raw",
            reliable_transport=True,
        )

    if PERCEPTION_SERVER_MESH_VIEWER:
        PerceptionDescriptions.add_mesh_viewer(
            composable_nodes,
            PERCEPTION_SERVER_MESH_VIEWER,
        )

    if PERCEPTION_SERVER_MONOCULAR_SLAM:
        PerceptionDescriptions.add_monocular_slam(
            composable_nodes,
            PERCEPTION_SERVER_MONOCULAR_SLAM,
            image_transport="compressed",
            camera_name=CAMERA_NAME,
        )

    if PERCEPTION_SERVER_MONOCULAR_INERTIAL_SLAM:
        PerceptionDescriptions.add_monocular_inertial_slam(
            composable_nodes,
            PERCEPTION_SERVER_MONOCULAR_INERTIAL_SLAM,
            image_transport="compressed",
            camera_name=CAMERA_NAME,
        )

    if PERCEPTION_SERVER_POSE_LANDMARKS:
        for host_id in PERCEPTION_SERVER_POSE_LANDMARKS:
            PerceptionDescriptions.add_pose_landmarker(
                ld, host_id, image_transport="raw"
            )

    # TODO
    # if ZONE_ID in SMART_DISPLAY_ZONES:
    #     PerceptionDescriptions..add_pose_renderer(ld, CAMERA_ZONES, HOST_ID)

    PerceptionDescriptions.add_perception_components(ld, HOST_ID, composable_nodes)

    return ld
