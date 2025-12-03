################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
Relay SLAM topics between resolution namespaces.

ORB-SLAM currently produces point clouds and poses in the ``hd720`` namespace
while other perception nodes (e.g., AprilTag detection) operate at ``hd``. This
node republishes SLAM outputs from one resolution namespace into another so
consumers that expect ``hd``-scoped topics can use SLAM data without changing
ORB-SLAM inputs.
"""

from __future__ import annotations

from typing import Dict
from typing import Iterable

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import PointCloud2


################################################################################
# ROS parameters
################################################################################


DEFAULT_NODE_NAME = "slam_resolution_relay"

SYSTEM_IDS_PARAMETER = "system_ids"
INPUT_RESOLUTION_PARAMETER = "input_resolution"
OUTPUT_RESOLUTION_PARAMETER = "output_resolution"

DEFAULT_SYSTEM_IDS: list[str] = [""]
DEFAULT_INPUT_RESOLUTION = "hd720"
DEFAULT_OUTPUT_RESOLUTION = "hd"


################################################################################
# ROS node
################################################################################


class SlamResolutionRelayNode(Node):
    """Republish SLAM outputs under a different resolution namespace."""

    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)

        # Declare parameters
        self.declare_parameter(SYSTEM_IDS_PARAMETER, DEFAULT_SYSTEM_IDS)
        self.declare_parameter(INPUT_RESOLUTION_PARAMETER, DEFAULT_INPUT_RESOLUTION)
        self.declare_parameter(OUTPUT_RESOLUTION_PARAMETER, DEFAULT_OUTPUT_RESOLUTION)

        # Read parameters
        system_ids = self._sanitize_system_ids(
            self.get_parameter(SYSTEM_IDS_PARAMETER)
            .get_parameter_value()
            .string_array_value
        )
        input_resolution: str = (
            self.get_parameter(INPUT_RESOLUTION_PARAMETER)
            .get_parameter_value()
            .string_value
        )
        output_resolution: str = (
            self.get_parameter(OUTPUT_RESOLUTION_PARAMETER)
            .get_parameter_value()
            .string_value
        )
        if not system_ids:
            self.get_logger().warning("No system IDs provided; nothing to relay")
            system_ids = []

        self._pose_publishers: Dict[str, Publisher] = {}
        self._point_cloud_publishers: Dict[str, Publisher] = {}
        for system_id in system_ids:
            input_prefix = self._resolve_prefix(system_id, input_resolution)
            output_prefix = self._resolve_prefix(system_id, output_resolution)

            pose_topic_in = f"{input_prefix}slam_pose"
            pose_topic_out = f"{output_prefix}slam_pose"
            cloud_topic_in = f"{input_prefix}slam_point_cloud"
            cloud_topic_out = f"{output_prefix}slam_point_cloud"

            input_prefix_clean = input_prefix.rstrip("/")
            output_prefix_clean = output_prefix.rstrip("/")

            self.get_logger().info(
                f"Relaying SLAM topics for {system_id}: "
                f"{input_prefix_clean} -> {output_prefix_clean}"
            )

            pose_pub = self.create_publisher(PoseStamped, pose_topic_out, 1)
            cloud_pub = self.create_publisher(PointCloud2, cloud_topic_out, 1)

            self.create_subscription(
                PoseStamped,
                pose_topic_in,
                lambda msg, pub=pose_pub: pub.publish(msg),
                1,
            )
            self.create_subscription(
                PointCloud2,
                cloud_topic_in,
                lambda msg, pub=cloud_pub: pub.publish(msg),
                1,
            )

            self._pose_publishers[system_id] = pose_pub
            self._point_cloud_publishers[system_id] = cloud_pub

    def stop(self) -> None:
        self.get_logger().info("Shutting down slam_resolution_relay")
        self.destroy_node()

    @staticmethod
    def _resolve_prefix(system_id: str, resolution: str) -> str:
        if not system_id:
            return ""
        resolution_prefix = f"{resolution}/" if resolution else ""
        return f"{system_id}/{resolution_prefix}"

    @staticmethod
    def _sanitize_system_ids(system_ids: Iterable[str]) -> list[str]:
        return [system_id for system_id in system_ids if system_id]


__all__ = ["SlamResolutionRelayNode"]
