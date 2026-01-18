################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
ROS publishers for AHRS localization
"""

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry as OdometryMsg
from oasis_msgs.msg import AhrsDiagnostics as AhrsDiagnosticsMsg
from oasis_msgs.msg import AhrsState as AhrsStateMsg
from oasis_msgs.msg import EkfUpdateReport as EkfUpdateReportMsg
import rclpy.node
import rclpy.publisher
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster

from oasis_control.localization.ahrs.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.ahrs_ros_msgs import to_diag_msg
from oasis_control.localization.ahrs.ahrs_ros_msgs import to_odom_msg
from oasis_control.localization.ahrs.ahrs_ros_msgs import to_pose_cov_msg
from oasis_control.localization.ahrs.ahrs_ros_msgs import to_state_msg
from oasis_control.localization.ahrs.ahrs_ros_msgs import to_tf_msgs
from oasis_control.localization.ahrs.ahrs_ros_msgs import to_update_report_msg
from oasis_control.localization.ahrs.ahrs_types import AhrsOutputs
from oasis_control.localization.ahrs.ahrs_types import AhrsUpdateData


ACCEL_UPDATE_TOPIC: str = "ahrs/updates/accel"
GYRO_UPDATE_TOPIC: str = "ahrs/updates/gyro"
MAG_UPDATE_TOPIC: str = "ahrs/updates/mag"

EXTRINSICS_T_BI_TOPIC: str = "ahrs/extrinsics/t_bi"
EXTRINSICS_T_BM_TOPIC: str = "ahrs/extrinsics/t_bm"

ODOM_TOPIC: str = "ahrs/odom"
WORLD_ODOM_TOPIC: str = "ahrs/world_odom"

STATE_TOPIC: str = "ahrs/state"
DIAG_TOPIC: str = "ahrs/diag"


class AhrsPublishers:
    """
    ROS publisher wrapper for AHRS outputs
    """

    def __init__(
        self, node: rclpy.node.Node, config: AhrsConfig, qos: QoSProfile
    ) -> None:
        self._node: rclpy.node.Node = node
        self._config: AhrsConfig = config

        self._tf_broadcaster: TransformBroadcaster = TransformBroadcaster(node)

        self._odom_pub: rclpy.publisher.Publisher = node.create_publisher(
            OdometryMsg, ODOM_TOPIC, qos
        )
        self._world_odom_pub: rclpy.publisher.Publisher = node.create_publisher(
            OdometryMsg, WORLD_ODOM_TOPIC, qos
        )
        self._t_bi_pub: rclpy.publisher.Publisher = node.create_publisher(
            PoseWithCovarianceStamped, EXTRINSICS_T_BI_TOPIC, qos
        )
        self._t_bm_pub: rclpy.publisher.Publisher = node.create_publisher(
            PoseWithCovarianceStamped, EXTRINSICS_T_BM_TOPIC, qos
        )
        self._state_pub: rclpy.publisher.Publisher = node.create_publisher(
            AhrsStateMsg, STATE_TOPIC, qos
        )
        self._diag_pub: rclpy.publisher.Publisher = node.create_publisher(
            AhrsDiagnosticsMsg, DIAG_TOPIC, qos
        )
        self._gyro_update_pub: rclpy.publisher.Publisher = node.create_publisher(
            EkfUpdateReportMsg, GYRO_UPDATE_TOPIC, qos
        )
        self._accel_update_pub: rclpy.publisher.Publisher = node.create_publisher(
            EkfUpdateReportMsg, ACCEL_UPDATE_TOPIC, qos
        )
        self._mag_update_pub: rclpy.publisher.Publisher = node.create_publisher(
            EkfUpdateReportMsg, MAG_UPDATE_TOPIC, qos
        )

    def publish_outputs(self, outputs: AhrsOutputs) -> None:
        self._publish_update(outputs.gyro_update, self._gyro_update_pub)
        self._publish_update(outputs.accel_update, self._accel_update_pub)
        self._publish_update(outputs.mag_update, self._mag_update_pub)

        if not outputs.frontier_advanced:
            return

        if outputs.t_filter is None:
            return

        if outputs.frame_transforms is not None:
            tf_msgs: list[TransformStamped] = to_tf_msgs(
                outputs.t_filter, outputs.frame_transforms, self._config
            )
            self._tf_broadcaster.sendTransform(tf_msgs)

            odom_msg: OdometryMsg = to_odom_msg(
                outputs.t_filter,
                outputs.frame_transforms.t_odom_base,
                frame_id=self._config.odom_frame_id,
                child_frame_id=self._config.body_frame_id,
            )
            world_odom_msg: OdometryMsg = to_odom_msg(
                outputs.t_filter,
                outputs.frame_transforms.t_world_odom,
                frame_id=self._config.world_frame_id,
                child_frame_id=self._config.odom_frame_id,
            )
            self._odom_pub.publish(odom_msg)
            self._world_odom_pub.publish(world_odom_msg)

        if outputs.state is not None:
            state_msg: AhrsStateMsg = to_state_msg(outputs.state)
            self._state_pub.publish(state_msg)

            t_bi_msg: PoseWithCovarianceStamped = to_pose_cov_msg(
                outputs.t_filter, outputs.state.t_bi
            )
            t_bm_msg: PoseWithCovarianceStamped = to_pose_cov_msg(
                outputs.t_filter, outputs.state.t_bm
            )
            self._t_bi_pub.publish(t_bi_msg)
            self._t_bm_pub.publish(t_bm_msg)

        if outputs.diagnostics is not None:
            diag_msg: AhrsDiagnosticsMsg = to_diag_msg(outputs.diagnostics)
            diag_msg.header.frame_id = self._config.world_frame_id
            self._diag_pub.publish(diag_msg)

    def _publish_update(
        self,
        update: Optional[AhrsUpdateData],
        publisher: rclpy.publisher.Publisher,
    ) -> None:
        if update is None:
            return

        msg: EkfUpdateReportMsg = to_update_report_msg(update)
        publisher.publish(msg)
