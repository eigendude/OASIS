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

print(f"Launching on {HOSTNAME} in zone {ZONE_ID}")


################################################################################
# Hardware configuration
################################################################################


PERCEPTION_SERVER_BACKGROUND: list[str] = []
PERCEPTION_SERVER_CALIBRATION: list[str] = []
PERCEPTION_SERVER_FLOW: list[str] = []
PERCEPTION_SERVER_POSE_LANDMARKS: list[str] = []

if HOST_ID == "falcon":
    PERCEPTION_SERVER_FLOW.extend(["falcon"])
elif HOST_ID == "nas":
    # PERCEPTION_SERVER_CALIBRATION.extend(
    #     ["bar", "doorbell", "entryway", "hallway", "kitchen", "livingroom"]
    # )
    PERCEPTION_SERVER_POSE_LANDMARKS.extend(["hallway"])
elif HOST_ID == "oceanplatform":
    # PERCEPTION_SERVER_BACKGROUND.extend(["station"])
    PERCEPTION_SERVER_FLOW.extend(["station"])
    # PERCEPTION_SERVER_POSE_LANDMARKS.extend(["falcon"])


################################################################################
# Launch description
################################################################################


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    composable_nodes: list[ComposableNode] = []

    if PERCEPTION_SERVER_BACKGROUND:
        PerceptionDescriptions.add_background_modeler(
            composable_nodes, PERCEPTION_SERVER_BACKGROUND
        )
        PerceptionDescriptions.add_background_subtractor(
            composable_nodes, PERCEPTION_SERVER_BACKGROUND
        )

    if PERCEPTION_SERVER_CALIBRATION:
        for host_id in PERCEPTION_SERVER_CALIBRATION:
            PerceptionDescriptions.add_calibration(ld, host_id)

    if PERCEPTION_SERVER_FLOW:
        PerceptionDescriptions.add_optical_flow(
            composable_nodes, PERCEPTION_SERVER_FLOW
        )

    if PERCEPTION_SERVER_POSE_LANDMARKS:
        for host_id in PERCEPTION_SERVER_POSE_LANDMARKS:
            PerceptionDescriptions.add_pose_landmarker(ld, host_id)

    # TODO
    # if ZONE_ID in SMART_DISPLAY_ZONES:
    #     PerceptionDescriptions..add_pose_renderer(ld, CAMERA_ZONES, HOST_ID)

    PerceptionDescriptions.add_perception_components(ld, HOST_ID, composable_nodes)

    return ld
