################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

from dataclasses import dataclass

from oasis_control.localization.common.algebra.covariance import rotate_covariance
from oasis_control.localization.common.algebra.quat import Matrix3
from oasis_control.localization.common.algebra.quat import Quaternion
from oasis_control.localization.common.algebra.quat import Vector3
from oasis_control.localization.common.algebra.quat import quaternion_multiply_xyzw
from oasis_control.localization.common.algebra.quat import quaternion_to_rotation_matrix
from oasis_control.localization.common.algebra.quat import rotate_vector
from oasis_control.localization.common.data.accel_sample import AccelSample
from oasis_control.localization.common.data.gravity_sample import GravitySample
from oasis_control.localization.common.data.imu_sample import ImuSample


@dataclass(frozen=True)
class MountingTransform:
    """
    Fixed IMU-to-base rotation used by AHRS runtime mounting.

    Fields:
        parent_frame_id: target base frame that receives rotated data
        child_frame_id: source IMU frame that provides data to rotate
        quaternion_xyzw: fixed quaternion `q_BI` that rotates vectors from
            `imu_link` into `base_link`
        rotation_matrix: 3x3 equivalent of quaternion_xyzw
    """

    parent_frame_id: str
    child_frame_id: str
    quaternion_xyzw: Quaternion
    rotation_matrix: Matrix3


@dataclass(frozen=True)
class MountedImuSample:
    """
    IMU sample expressed in the mounted base frame.

    Fields:
        timestamp_ns: measurement timestamp in integer nanoseconds
        frame_id: mounted output frame, expected to be base_link
        orientation_xyzw: mounted world-to-base quaternion in ROS xyzw order
        orientation_covariance_rad2: mounted orientation covariance
        orientation_covariance_unknown: true when orientation covariance remains
            unknown by ROS convention
        angular_velocity_rads: mounted angular velocity in the base frame
        angular_velocity_covariance_rads2: mounted angular covariance
        linear_acceleration_mps2: mounted linear acceleration in the base frame
        linear_acceleration_covariance_mps2_2: mounted linear covariance
    """

    timestamp_ns: int
    frame_id: str
    orientation_xyzw: Quaternion
    orientation_covariance_rad2: Matrix3 | None
    orientation_covariance_unknown: bool
    angular_velocity_rads: Vector3
    angular_velocity_covariance_rads2: Matrix3 | None
    linear_acceleration_mps2: Vector3
    linear_acceleration_covariance_mps2_2: Matrix3 | None


@dataclass(frozen=True)
class MountedGravitySample:
    """
    Gravity-direction sample expressed in the mounted base frame.

    Fields:
        timestamp_ns: measurement timestamp in integer nanoseconds
        frame_id: mounted output frame, expected to be base_link
        gravity_mps2: mounted gravity-direction vector in the base frame
        gravity_covariance_mps2_2: mounted gravity covariance when available
    """

    timestamp_ns: int
    frame_id: str
    gravity_mps2: Vector3
    gravity_covariance_mps2_2: Matrix3 | None


@dataclass(frozen=True)
class MountedAccelSample:
    """
    Gravity-included accel sample expressed in the mounted base frame.

    Fields:
        timestamp_ns: measurement timestamp in integer nanoseconds
        frame_id: mounted output frame, expected to be base_link
        accel_mps2: mounted gravity-included acceleration in the base frame
        accel_covariance_mps2_2: mounted accel covariance when available
    """

    timestamp_ns: int
    frame_id: str
    accel_mps2: Vector3
    accel_covariance_mps2_2: Matrix3 | None


def make_mounting_transform(
    parent_frame_id: str,
    child_frame_id: str,
    quaternion_xyzw: Quaternion,
) -> MountingTransform:
    """
    Build a fixed `T_BI` mounting transform from a normalized quaternion.

    The stored quaternion is `q_BI`, so `parent_frame_id` is the target frame
    (`base_link`) and `child_frame_id` is the source frame (`imu_link`). The
    matching ROS TF is therefore published as `base_link -> imu_link`, and
    `lookup_transform(base_link, imu_link, ...)` returns the same `q_BI`.
    """

    return MountingTransform(
        parent_frame_id=parent_frame_id,
        child_frame_id=child_frame_id,
        quaternion_xyzw=quaternion_xyzw,
        rotation_matrix=quaternion_to_rotation_matrix(quaternion_xyzw),
    )


def apply_mounting_to_imu(
    imu_sample: ImuSample, mounting_transform: MountingTransform
) -> MountedImuSample:
    """
    Express an IMU sample in the configured base frame.

    `imu_sample.orientation_xyzw` is expected to be `q_WI`. Applying the fixed
    mounting `q_BI` yields the mounted base attitude `q_WB = q_BI ⊗ q_WI`.
    """

    orientation_covariance_rad2: Matrix3 | None = None
    if (
        imu_sample.orientation_covariance_rad2 is not None
        and not imu_sample.orientation_covariance_unknown
    ):
        # Preserve the full upstream orientation covariance and only rotate it
        # through the fixed IMU-to-base mounting transform.
        orientation_covariance_rad2 = rotate_covariance(
            mounting_transform.rotation_matrix,
            imu_sample.orientation_covariance_rad2,
        )

    angular_velocity_covariance_rads2: Matrix3 | None = None
    if imu_sample.angular_velocity_covariance_rads2 is not None:
        angular_velocity_covariance_rads2 = rotate_covariance(
            mounting_transform.rotation_matrix,
            imu_sample.angular_velocity_covariance_rads2,
        )

    linear_acceleration_covariance_mps2_2: Matrix3 | None = None
    if imu_sample.linear_acceleration_covariance_mps2_2 is not None:
        linear_acceleration_covariance_mps2_2 = rotate_covariance(
            mounting_transform.rotation_matrix,
            imu_sample.linear_acceleration_covariance_mps2_2,
        )

    mounted_orientation_xyzw: Quaternion = quaternion_multiply_xyzw(
        mounting_transform.quaternion_xyzw,
        imu_sample.orientation_xyzw,
    )

    return MountedImuSample(
        timestamp_ns=imu_sample.timestamp_ns,
        frame_id=mounting_transform.parent_frame_id,
        orientation_xyzw=mounted_orientation_xyzw,
        orientation_covariance_rad2=orientation_covariance_rad2,
        orientation_covariance_unknown=imu_sample.orientation_covariance_unknown,
        angular_velocity_rads=rotate_vector(
            mounting_transform.rotation_matrix, imu_sample.angular_velocity_rads
        ),
        angular_velocity_covariance_rads2=angular_velocity_covariance_rads2,
        linear_acceleration_mps2=rotate_vector(
            mounting_transform.rotation_matrix, imu_sample.linear_acceleration_mps2
        ),
        linear_acceleration_covariance_mps2_2=(linear_acceleration_covariance_mps2_2),
    )


def apply_mounting_to_gravity(
    gravity_sample: GravitySample,
    mounting_transform: MountingTransform,
) -> MountedGravitySample:
    """
    Express a gravity-direction sample in the configured base frame.

    The stored rotation matrix is `R_BI`, so the mounted gravity vector is
    `g_B = R_BI g_I`.
    """

    gravity_covariance_mps2_2: Matrix3 | None = None
    if gravity_sample.gravity_covariance_mps2_2 is not None:
        gravity_covariance_mps2_2 = rotate_covariance(
            mounting_transform.rotation_matrix,
            gravity_sample.gravity_covariance_mps2_2,
        )

    return MountedGravitySample(
        timestamp_ns=gravity_sample.timestamp_ns,
        frame_id=mounting_transform.parent_frame_id,
        gravity_mps2=rotate_vector(
            mounting_transform.rotation_matrix, gravity_sample.gravity_mps2
        ),
        gravity_covariance_mps2_2=gravity_covariance_mps2_2,
    )


def apply_mounting_to_accel(
    accel_sample: AccelSample,
    mounting_transform: MountingTransform,
) -> MountedAccelSample:
    """
    Express a gravity-included accel sample in the configured base frame.

    The stored rotation matrix is `R_BI`, so the mounted accel vector is
    `a_B = R_BI a_I`.
    """

    accel_covariance_mps2_2: Matrix3 | None = None
    if accel_sample.accel_covariance_mps2_2 is not None:
        accel_covariance_mps2_2 = rotate_covariance(
            mounting_transform.rotation_matrix,
            accel_sample.accel_covariance_mps2_2,
        )

    return MountedAccelSample(
        timestamp_ns=accel_sample.timestamp_ns,
        frame_id=mounting_transform.parent_frame_id,
        accel_mps2=rotate_vector(
            mounting_transform.rotation_matrix, accel_sample.accel_mps2
        ),
        accel_covariance_mps2_2=accel_covariance_mps2_2,
    )
