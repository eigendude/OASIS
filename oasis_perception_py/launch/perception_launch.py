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
from launch_ros.actions import Node

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

# The host and zone IDs used for Home Assistant
HOME_ASSISTANT_ID: str = CONFIG.HOME_ASSISTANT_ID

# The zone ID used for the Kinect V2 camera
KINECT_V2_ZONE_ID: str = CONFIG.KINECT_V2_ZONE_ID

print(f"Launching on {HOSTNAME} in zone {ZONE_ID}")


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE: str = "oasis"

CPP_PACKAGE_NAME: str = "oasis_perception_cpp"
PYTHON_PACKAGE_NAME: str = "oasis_perception_py"


# The host ID used for perception
# TODO: Move to smarthome config
PERCEPTION_HOST_ID: str = "oceanplatform"

PERCEPTION_SERVER_BACKGROUND: list[str] = []
PERCEPTION_SERVER_POSE_LANDMARKS: list[str] = []
PERCEPTION_SERVER_CALIBRATION: list[str] = []

if HOST_ID == PERCEPTION_HOST_ID:
    PERCEPTION_SERVER_BACKGROUND.extend(
        ["bar", "doorbell", "entryway", "hallway", "kitchen", "livingroom"]
    )
    # PERCEPTION_SERVER_POSE_LANDMARKS.extend(
    #     ["bar", "doorbell", "entryway", "hallway", "kitchen", "livingroom"]
    # )
    # PERCEPTION_SERVER_CALIBRATION.extend(
    #     ["bar", "doorbell", "entryway", "hallway", "kitchen", "livingroom"]
    # )

print(f"Launching on {HOSTNAME} in zone {ZONE_ID}")


################################################################################
# Node definitions
################################################################################


#
# Background modeler
#


def add_background_modeler(ld: LaunchDescription, zone_id: str) -> None:
    node: Node = Node(
        namespace=ROS_NAMESPACE,
        package=CPP_PACKAGE_NAME,
        executable="background_modeler",
        name=f"background_modeler_{zone_id}",
        output="screen",
        remappings=[
            (
                "image",
                (
                    # Use different remappings for Kinect V2
                    f"{zone_id}/sd/image_color"
                    if zone_id == KINECT_V2_ZONE_ID
                    else f"{zone_id}/image_rect"
                ),
            ),
            ("background", f"{zone_id}/background"),
        ],
    )
    ld.add_action(node)


#
# Calibration node
#


def add_calibration(ld: LaunchDescription, zone_id: str) -> None:
    # TODO: Smarthome configuration
    camera_node: str
    if zone_id in ["bar", "kitchen"]:
        camera_node = f"v4l2_camera_{zone_id}"
    elif zone_id in ["door", "station"]:
        camera_node = f"camera_ros_{zone_id}"
    elif zone_id in [KINECT_V2_ZONE_ID]:
        # TODO
        return
    else:
        return

    node: Node = Node(
        namespace=ROS_NAMESPACE,
        package=PYTHON_PACKAGE_NAME,
        executable="camera_calibrator",
        name=f"camera_calibrator_{zone_id}",
        output="screen",
        remappings=[
            # Topics
            ("calibration", f"{zone_id}/calibration"),
            ("image", f"{zone_id}/image_raw"),
            # Services
            ("camera/set_camera_info", f"{camera_node}/set_camera_info"),
        ],
    )
    ld.add_action(node)


#
# Pose landmarker
#


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


def add_pose_renderer(ld: LaunchDescription, zone_ids: list[str], host_id: str) -> None:
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


################################################################################
# Launch description
################################################################################


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    if PERCEPTION_SERVER_BACKGROUND:
        for host_id in PERCEPTION_SERVER_BACKGROUND:
            add_background_modeler(ld, host_id)

    if PERCEPTION_SERVER_POSE_LANDMARKS:
        for host_id in PERCEPTION_SERVER_POSE_LANDMARKS:
            add_pose_landmarker(ld, host_id)

    if PERCEPTION_SERVER_CALIBRATION:
        for host_id in PERCEPTION_SERVER_CALIBRATION:
            add_calibration(ld, host_id)

    # TODO
    # if ZONE_ID in SMART_DISPLAY_ZONES:
    #     add_pose_renderer(ld, CAMERA_ZONES, HOST_ID)

    """
    if HOST_ID == "cinder":
        bgs_abl_node = Node(
            namespace=ROS_NAMESPACE,
            package=CPP_PACKAGE_NAME,
            executable="background_subtractor_abl",
            name=f"background_subtractor_abl_{HOST_ID}",
            output="screen",
        )
        ld.add_action(bgs_abl_node)

        bgs_asbl_node = Node(
            namespace=ROS_NAMESPACE,
            package=CPP_PACKAGE_NAME,
            executable="background_subtractor_asbl",
            name=f"background_subtractor_asbl_{HOST_ID}",
            output="screen",
        )
        ld.add_action(bgs_asbl_node)

    elif HOST_ID == "starship":
        multi_modeler_node = Node(
            namespace=ROS_NAMESPACE,
            package=CPP_PACKAGE_NAME,
            executable="multi_modeler",
            name=f"multi_modeler_{HOST_ID}",
            output="screen",
        )
        ld.add_action(multi_modeler_node)

        monocular_slam_node = Node(
            namespace=ROS_NAMESPACE,
            package=CPP_PACKAGE_NAME,
            executable="monocular_slam",
            name=f"monocular_slam_{HOST_ID}",
            output="screen",
        )
        ld.add_action(monocular_slam_node)

    elif HOST_ID == "jetson":
        MCU_NODE = "engine"
        monocular_inertial_slam_node = Node(
            namespace=ROS_NAMESPACE,
            package=CPP_PACKAGE_NAME,
            executable="monocular_inertial_slam",
            name=f"monocular_inertial_slam_{HOST_ID}",
            output="screen",
            remappings=[
                ("i2c_imu", f"{MCU_NODE}/i2c_imu"),
            ],
        )
        ld.add_action(monocular_inertial_slam_node)
    """

    return ld
