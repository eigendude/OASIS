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
ROS message builders for AHRS localization
"""

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry as OdometryMsg
from oasis_msgs.msg import AhrsDiagnostics as AhrsDiagnosticsMsg
from oasis_msgs.msg import AhrsState as AhrsStateMsg
from oasis_msgs.msg import EkfUpdateReport as EkfUpdateReportMsg
from oasis_msgs.msg import Matrix as MatrixMsg

from oasis_control.localization.ahrs.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.ahrs_conversions import ros_time_from_ahrs
from oasis_control.localization.ahrs.ahrs_types import AhrsDiagnosticsData
from oasis_control.localization.ahrs.ahrs_types import AhrsFrameOutputs
from oasis_control.localization.ahrs.ahrs_types import AhrsFrameTransform
from oasis_control.localization.ahrs.ahrs_types import AhrsMatrix
from oasis_control.localization.ahrs.ahrs_types import AhrsSe3Transform
from oasis_control.localization.ahrs.ahrs_types import AhrsStateData
from oasis_control.localization.ahrs.ahrs_types import AhrsTime
from oasis_control.localization.ahrs.ahrs_types import AhrsUpdateData


def to_update_report_msg(update: AhrsUpdateData) -> EkfUpdateReportMsg:
    msg: EkfUpdateReportMsg = EkfUpdateReportMsg()
    msg.header.stamp = ros_time_from_ahrs(update.t_meas)
    msg.sensor = update.sensor
    msg.frame_id = update.frame_id
    msg.update_seq = 0
    msg.update_index_in_stamp = 0
    msg.accepted = update.accepted
    msg.reject_reason = update.reject_reason or ""
    msg.z_dim = len(update.z)
    msg.z = list(update.z)
    msg.z_hat = list(update.z_hat)
    msg.nu = list(update.nu)
    msg.r = _to_matrix_msg(update.r)
    msg.s_hat = _to_matrix_msg(update.s_hat)
    msg.s = _to_matrix_msg(update.s)
    msg.maha_d2 = update.maha_d2
    msg.gate_d2_threshold = update.gate_threshold
    msg.reproj_rms_px = 0.0

    return msg


def to_state_msg(state: AhrsStateData) -> AhrsStateMsg:
    msg: AhrsStateMsg = AhrsStateMsg()
    msg.header.stamp = ros_time_from_ahrs(state.t_filter)
    msg.header.frame_id = state.world_frame_id
    msg.world_frame_id = state.world_frame_id
    msg.odom_frame_id = state.odom_frame_id
    msg.body_frame_id = state.body_frame_id
    msg.state_seq = state.state_seq
    msg.initialized = state.initialized
    msg.position_wb_m = _to_point(state.p_wb_m)
    msg.velocity_wb_mps = _to_vector3(state.v_wb_mps)
    msg.orientation_wb = _to_quaternion(state.q_wb_wxyz)
    msg.gyro_bias = _to_vector3(state.b_g_rps)
    msg.accel_bias = _to_vector3(state.b_a_mps2)
    msg.accel_a = list(state.a_a)
    msg.t_bi = _to_pose(state.t_bi)
    msg.t_bm = _to_pose(state.t_bm)
    msg.gravity_w_mps2 = _to_vector3(state.g_w_mps2)
    msg.magnetic_field_w_t = _to_vector3(state.m_w_t)
    msg.error_dim = state.p_cov.rows
    msg.error_state_names = list(state.error_state_names)
    msg.p = _to_matrix_msg(state.p_cov)

    return msg


def to_diag_msg(diag: AhrsDiagnosticsData) -> AhrsDiagnosticsMsg:
    msg: AhrsDiagnosticsMsg = AhrsDiagnosticsMsg()
    if diag.t_filter is not None:
        msg.header.stamp = ros_time_from_ahrs(diag.t_filter)
        msg.t_filter_sec = _seconds_from_ahrs(diag.t_filter)
    else:
        msg.t_filter_sec = 0.0
    msg.diag_seq = diag.diag_seq
    msg.buffer_node_count = diag.buffer_node_count
    msg.buffer_span_sec = diag.buffer_span_sec
    msg.replay_happened = diag.replay_happened
    msg.dropped_missing_stamp = diag.dropped_missing_stamp
    msg.dropped_future_stamp = diag.dropped_future_stamp
    msg.dropped_too_old = diag.dropped_too_old
    msg.dropped_nan_cov = diag.dropped_nan_cov
    msg.dropped_imu_coverage_gap = diag.dropped_imu_gap
    msg.dropped_clock_jump_reset = diag.dropped_clock_jump_reset
    msg.last_reset_reason = diag.last_reset_reason

    return msg


def to_pose_cov_msg(
    t: AhrsTime,
    t_se3: AhrsSe3Transform,
    covariance_6x6: Optional[list[float]] = None,
) -> PoseWithCovarianceStamped:
    msg: PoseWithCovarianceStamped = PoseWithCovarianceStamped()
    msg.header.stamp = ros_time_from_ahrs(t)
    msg.header.frame_id = t_se3.parent_frame
    msg.pose = _to_pose_covariance(t_se3, covariance_6x6)

    return msg


def to_odom_msg(
    t: AhrsTime,
    transform: AhrsFrameTransform,
    frame_id: str,
    child_frame_id: str,
) -> OdometryMsg:
    msg: OdometryMsg = OdometryMsg()
    msg.header.stamp = ros_time_from_ahrs(t)
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.pose = _to_pose_with_covariance(transform)
    msg.twist = _zero_twist_with_covariance()

    return msg


def to_tf_msgs(
    t: AhrsTime, frame_transforms: AhrsFrameOutputs, config: AhrsConfig
) -> list[TransformStamped]:
    t_world_odom: TransformStamped = _to_transform_stamped(
        t,
        frame_transforms.t_world_odom,
        config,
    )
    t_odom_base: TransformStamped = _to_transform_stamped(
        t,
        frame_transforms.t_odom_base,
        config,
    )

    return [t_world_odom, t_odom_base]


def _to_transform_stamped(
    t: AhrsTime,
    transform: AhrsFrameTransform,
    config: AhrsConfig,
) -> TransformStamped:
    msg: TransformStamped = TransformStamped()
    msg.header.stamp = ros_time_from_ahrs(t)
    msg.header.frame_id = _map_frame_id(transform.parent_frame, config)
    msg.child_frame_id = _map_frame_id(transform.child_frame, config)
    msg.transform = _to_transform(transform)

    return msg


def _map_frame_id(frame_id: str, config: AhrsConfig) -> str:
    if frame_id == "world":
        return config.world_frame_id
    if frame_id == "odom":
        return config.odom_frame_id
    if frame_id == "base_link":
        return config.body_frame_id

    return frame_id


def _to_matrix_msg(matrix: AhrsMatrix) -> MatrixMsg:
    msg: MatrixMsg = MatrixMsg()
    msg.rows = matrix.rows
    msg.cols = matrix.cols
    msg.data = list(matrix.data)

    return msg


def _to_pose(transform: AhrsSe3Transform) -> Pose:
    pose: Pose = Pose()
    pose.position = _to_point(transform.translation_m)
    pose.orientation = _to_quaternion(transform.rotation_wxyz)

    return pose


def _to_pose_covariance(
    transform: AhrsSe3Transform, covariance: Optional[list[float]]
) -> PoseWithCovariance:
    pose_cov: PoseWithCovariance = PoseWithCovariance()
    pose_cov.pose = _to_pose(transform)
    pose_cov.covariance = _to_covariance_6x6(covariance)

    return pose_cov


def _to_pose_with_covariance(transform: AhrsFrameTransform) -> PoseWithCovariance:
    pose_cov: PoseWithCovariance = PoseWithCovariance()
    pose_cov.pose.position = _to_point(transform.translation_m)
    pose_cov.pose.orientation = _to_quaternion(transform.rotation_wxyz)
    pose_cov.covariance = _to_covariance_6x6(None)

    return pose_cov


def _zero_twist_with_covariance() -> TwistWithCovariance:
    twist_cov: TwistWithCovariance = TwistWithCovariance()
    twist_cov.twist = Twist()
    twist_cov.twist.linear = Vector3()
    twist_cov.twist.angular = Vector3()
    twist_cov.covariance = _to_covariance_6x6(None)

    return twist_cov


def _to_transform(transform: AhrsFrameTransform) -> Transform:
    msg: Transform = Transform()
    msg.translation = _to_vector3(transform.translation_m)
    msg.rotation = _to_quaternion(transform.rotation_wxyz)

    return msg


def _to_point(values: list[float]) -> Point:
    point: Point = Point()
    point.x = values[0]
    point.y = values[1]
    point.z = values[2]

    return point


def _to_vector3(values: list[float]) -> Vector3:
    vec: Vector3 = Vector3()
    vec.x = values[0]
    vec.y = values[1]
    vec.z = values[2]

    return vec


def _to_quaternion(values_wxyz: list[float]) -> Quaternion:
    quat: Quaternion = Quaternion()
    quat.w = values_wxyz[0]
    quat.x = values_wxyz[1]
    quat.y = values_wxyz[2]
    quat.z = values_wxyz[3]

    return quat


def _to_covariance_6x6(covariance: Optional[list[float]]) -> list[float]:
    if covariance is None:
        return [0.0] * 36
    if len(covariance) != 36:
        raise ValueError("Expected 36 covariance entries")

    return list(covariance)


def _seconds_from_ahrs(t: AhrsTime) -> float:
    return float(t.sec) + float(t.nanosec) * 1e-9
