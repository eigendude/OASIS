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

from launch import LaunchDescription
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

PERCEPTION_SERVER_BACKGROUND: list[str] = []
PERCEPTION_SERVER_CALIBRATION: list[str] = []
PERCEPTION_SERVER_FLOW: list[str] = []
PERCEPTION_SERVER_MONOCULAR_SLAM: list[str] = []
PERCEPTION_SERVER_POSE_LANDMARKS: list[str] = []

if HOST_ID == "falcon":
    PERCEPTION_SERVER_FLOW.extend(["falcon"])
    PERCEPTION_SERVER_MONOCULAR_SLAM.extend(["falcon"])
elif HOST_ID == "nas":
    # PERCEPTION_SERVER_CALIBRATION.extend(
    #     ["bar", "doorbell", "entryway", "hallway", "kitchen", "livingroom"]
    # )
    PERCEPTION_SERVER_POSE_LANDMARKS.extend(["hallway"])
elif HOST_ID == "oceanplatform":
    # PERCEPTION_SERVER_BACKGROUND.extend(["station"])
    PERCEPTION_SERVER_FLOW.extend(["station"])
    PERCEPTION_SERVER_POSE_LANDMARKS.extend(["falcon"])
elif HOST_ID == "macbook_vm":
    PERCEPTION_SERVER_MONOCULAR_SLAM.extend(["falcon"])


################################################################################
# Node Helpers
################################################################################


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


################################################################################
# Node definitions
################################################################################


#
# Background modeler
#


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
# Monocular SLAM
#


def add_monocular_slam(
    composable_nodes: list[ComposableNode], system_ids: List[str]
) -> None:
    composable_nodes.extend(
        [
            ComposableNode(
                namespace=ROS_NAMESPACE,
                package=CPP_PACKAGE_NAME,
                plugin="oasis_perception::MonocularSlamComponent",
                name=f"monocular_slam_{system_id}",
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
                ],
            )
            for system_id in system_ids
        ]
    )


#
# Optical flow
#


def add_optical_flow(
    composable_nodes: list[ComposableNode], system_ids: List[str]
) -> None:
    composable_nodes.extend(
        [
            ComposableNode(
                namespace=ROS_NAMESPACE,
                package=CPP_PACKAGE_NAME,
                plugin="oasis_perception::OpticalFlowComponent",
                name=f"optical_flow_{system_id}",
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
                    (f"{system_id}_flow", f"{system_id}/flow"),
                ],
            )
            for system_id in system_ids
        ]
    )


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

    composable_nodes: list[ComposableNode] = []

    if PERCEPTION_SERVER_BACKGROUND:
        add_background_modeler(composable_nodes, PERCEPTION_SERVER_BACKGROUND)
        add_background_subtractor(composable_nodes, PERCEPTION_SERVER_BACKGROUND)

    if PERCEPTION_SERVER_CALIBRATION:
        for host_id in PERCEPTION_SERVER_CALIBRATION:
            add_calibration(ld, host_id)

    if PERCEPTION_SERVER_FLOW:
        add_optical_flow(composable_nodes, PERCEPTION_SERVER_FLOW)

    if PERCEPTION_SERVER_MONOCULAR_SLAM:
        add_monocular_slam(composable_nodes, PERCEPTION_SERVER_MONOCULAR_SLAM)

    if PERCEPTION_SERVER_POSE_LANDMARKS:
        for host_id in PERCEPTION_SERVER_POSE_LANDMARKS:
            add_pose_landmarker(ld, host_id)

    add_perception_components(ld, HOST_ID, composable_nodes)

    # TODO
    # if ZONE_ID in SMART_DISPLAY_ZONES:
    #     add_pose_renderer(ld, CAMERA_ZONES, HOST_ID)

    """
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
