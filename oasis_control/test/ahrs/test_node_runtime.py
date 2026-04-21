################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the ROS-facing AHRS node runtime contract."""

from __future__ import annotations

import math
from typing import Any

import pytest  # type: ignore[import-not-found]
import rclpy
from geometry_msgs.msg import (
    AccelWithCovarianceStamped as AccelWithCovarianceStampedMsg,
)
from sensor_msgs.msg import Imu as ImuMsg

from oasis_control.localization.ahrs.processing.boot_mounting_calibrator import (
    BootMountingCalibrator,
)
from oasis_control.localization.common.algebra.quat import quaternion_conjugate_xyzw
from oasis_control.localization.common.measurements.gravity_observable_attitude import (
    gravity_covariance_to_roll_pitch_variance_rad2,
)
from oasis_msgs.msg import AhrsStatus as AhrsStatusMsg

from ._node_test_helpers import last_diag
from ._node_test_helpers import make_cached_mounting_transform
from ._node_test_helpers import make_gravity_message
from ._node_test_helpers import make_imu_message
from ._node_test_helpers import make_node


FULL_ORIENTATION_COVARIANCE_ROW_MAJOR: list[float] = [
    4.0,
    1.0,
    0.5,
    1.0,
    3.0,
    -0.25,
    0.5,
    -0.25,
    2.0,
]


ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR: list[float] = [
    3.0,
    -1.0,
    0.25,
    -1.0,
    4.0,
    0.5,
    0.25,
    0.5,
    2.0,
]


@pytest.fixture(scope="module", autouse=True)
def rclpy_context() -> Any:
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def test_valid_imu_publishes_mounted_outputs_and_tf() -> None:
    quarter_turn_about_z_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    )
    node, diag_pub, gravity_pub, imu_pub, odom_pub, tf_broadcaster = make_node(
        make_cached_mounting_transform(quarter_turn_about_z_xyzw)
    )

    try:
        node._handle_gravity(make_gravity_message())
        node._handle_imu(make_imu_message())

        gravity_message: AccelWithCovarianceStampedMsg = gravity_pub.messages[-1]
        imu_message: ImuMsg = imu_pub.messages[-1]
        odom_message = odom_pub.messages[-1]
        tf_messages = tf_broadcaster.transforms[-1]

        assert len(gravity_pub.messages) == 1
        assert len(imu_pub.messages) == 1
        assert len(odom_pub.messages) == 1
        assert last_diag(diag_pub).status == AhrsStatusMsg.STATUS_OK
        assert gravity_message.header.frame_id == "base_link"
        assert imu_message.header.frame_id == "base_link"
        assert odom_message.header.frame_id == "odom"
        assert odom_message.child_frame_id == "base_link"
        assert any(
            transform.header.frame_id == "world" and transform.child_frame_id == "odom"
            for transform in tf_messages
        )
        mounting_transform = next(
            transform
            for transform in tf_messages
            if transform.header.frame_id == "base_link"
            and transform.child_frame_id == "imu_link"
        )
        odom_to_base_transform = next(
            transform
            for transform in tf_messages
            if transform.header.frame_id == "odom"
            and transform.child_frame_id == "base_link"
        )
        assert math.isclose(
            mounting_transform.transform.rotation.z,
            quarter_turn_about_z_xyzw[2],
        )
        assert math.isclose(
            mounting_transform.transform.rotation.w,
            quarter_turn_about_z_xyzw[3],
        )
        assert math.isclose(
            odom_to_base_transform.transform.rotation.x,
            0.0,
            abs_tol=1.0e-6,
        )
        assert math.isclose(
            odom_to_base_transform.transform.rotation.y,
            0.0,
            abs_tol=1.0e-6,
        )
        assert math.isclose(
            odom_to_base_transform.transform.rotation.z,
            0.0,
            abs_tol=1.0e-6,
        )
        assert math.isclose(
            odom_to_base_transform.transform.rotation.w,
            1.0,
            abs_tol=1.0e-6,
        )
    finally:
        node.stop()


def test_session_yaw_zero_tracks_subsequent_yaw_relative_to_startup() -> None:
    node, _, _, imu_pub, odom_pub, tf_broadcaster = make_node(
        make_cached_mounting_transform((0.0, 0.0, 0.0, 1.0))
    )

    try:
        node._handle_gravity(make_gravity_message())
        node._handle_imu(
            make_imu_message(
                quaternion_xyzw=_driver_quaternion_from_mounted_yaw_rad(
                    math.radians(30.0)
                ),
                timestamp_ns=1_000_000_000,
            )
        )
        initial_imu_message: ImuMsg = imu_pub.messages[-1]
        initial_odom_message = odom_pub.messages[-1]
        initial_tf_messages = tf_broadcaster.transforms[-1]
        initial_odom_to_base_transform = next(
            transform
            for transform in initial_tf_messages
            if transform.header.frame_id == "odom"
            and transform.child_frame_id == "base_link"
        )

        assert math.isclose(
            _yaw_from_xyzw(
                (
                    float(initial_imu_message.orientation.x),
                    float(initial_imu_message.orientation.y),
                    float(initial_imu_message.orientation.z),
                    float(initial_imu_message.orientation.w),
                )
            ),
            0.0,
            abs_tol=1.0e-6,
        )
        assert math.isclose(
            _yaw_from_xyzw(
                (
                    float(initial_odom_message.pose.pose.orientation.x),
                    float(initial_odom_message.pose.pose.orientation.y),
                    float(initial_odom_message.pose.pose.orientation.z),
                    float(initial_odom_message.pose.pose.orientation.w),
                )
            ),
            0.0,
            abs_tol=1.0e-6,
        )
        assert math.isclose(
            _yaw_from_xyzw(
                (
                    float(initial_odom_to_base_transform.transform.rotation.x),
                    float(initial_odom_to_base_transform.transform.rotation.y),
                    float(initial_odom_to_base_transform.transform.rotation.z),
                    float(initial_odom_to_base_transform.transform.rotation.w),
                )
            ),
            0.0,
            abs_tol=1.0e-6,
        )

        node._handle_imu(
            make_imu_message(
                quaternion_xyzw=_driver_quaternion_from_mounted_yaw_rad(
                    math.radians(75.0)
                ),
                timestamp_ns=2_000_000_000,
            )
        )
        updated_imu_message: ImuMsg = imu_pub.messages[-1]
        updated_odom_message = odom_pub.messages[-1]
        updated_tf_messages = tf_broadcaster.transforms[-1]
        mounting_transform = next(
            transform
            for transform in updated_tf_messages
            if transform.header.frame_id == "base_link"
            and transform.child_frame_id == "imu_link"
        )
        updated_odom_to_base_transform = next(
            transform
            for transform in updated_tf_messages
            if transform.header.frame_id == "odom"
            and transform.child_frame_id == "base_link"
        )

        expected_relative_yaw_rad: float = math.radians(45.0)
        assert math.isclose(
            _yaw_from_xyzw(
                (
                    float(updated_imu_message.orientation.x),
                    float(updated_imu_message.orientation.y),
                    float(updated_imu_message.orientation.z),
                    float(updated_imu_message.orientation.w),
                )
            ),
            expected_relative_yaw_rad,
            abs_tol=1.0e-6,
        )
        assert math.isclose(
            _yaw_from_xyzw(
                (
                    float(updated_odom_message.pose.pose.orientation.x),
                    float(updated_odom_message.pose.pose.orientation.y),
                    float(updated_odom_message.pose.pose.orientation.z),
                    float(updated_odom_message.pose.pose.orientation.w),
                )
            ),
            expected_relative_yaw_rad,
            abs_tol=1.0e-6,
        )
        assert math.isclose(
            _yaw_from_xyzw(
                (
                    float(updated_odom_to_base_transform.transform.rotation.x),
                    float(updated_odom_to_base_transform.transform.rotation.y),
                    float(updated_odom_to_base_transform.transform.rotation.z),
                    float(updated_odom_to_base_transform.transform.rotation.w),
                )
            ),
            expected_relative_yaw_rad,
            abs_tol=1.0e-6,
        )
        assert math.isclose(
            _yaw_from_xyzw(
                (
                    float(mounting_transform.transform.rotation.x),
                    float(mounting_transform.transform.rotation.y),
                    float(mounting_transform.transform.rotation.z),
                    float(mounting_transform.transform.rotation.w),
                )
            ),
            0.0,
            abs_tol=1.0e-6,
        )
    finally:
        node.stop()


def test_gravity_output_is_rotated_into_base_link_with_source_timestamp() -> None:
    quarter_turn_about_z_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    )
    node, _, gravity_pub, _, _, _ = make_node(
        make_cached_mounting_transform(quarter_turn_about_z_xyzw)
    )

    try:
        node._handle_gravity(
            make_gravity_message(
                gravity_vector=(2.0, 0.5, -9.81),
                timestamp_ns=1_234_567_890,
            )
        )
        node._handle_imu(make_imu_message(timestamp_ns=1_300_000_000))

        gravity_message: AccelWithCovarianceStampedMsg = gravity_pub.messages[-1]

        assert gravity_message.header.frame_id == "base_link"
        assert gravity_message.header.stamp.sec == 1
        assert gravity_message.header.stamp.nanosec == 234_567_890
        assert gravity_message.accel.accel.linear.x == pytest.approx(-0.5)
        assert gravity_message.accel.accel.linear.y == pytest.approx(2.0)
        assert gravity_message.accel.accel.linear.z == pytest.approx(-9.81)
    finally:
        node.stop()


def test_gravity_output_rotates_linear_covariance_block_into_base_link() -> None:
    quarter_turn_about_z_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    )
    node, _, gravity_pub, _, _, _ = make_node(
        make_cached_mounting_transform(quarter_turn_about_z_xyzw)
    )

    try:
        node._handle_gravity(
            make_gravity_message(
                covariance=[
                    4.0,
                    1.0,
                    0.5,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    3.0,
                    -0.25,
                    0.0,
                    0.0,
                    0.0,
                    0.5,
                    -0.25,
                    2.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            )
        )
        node._handle_imu(make_imu_message(timestamp_ns=1_000_000_000))

        published_covariance: list[float] = gravity_pub.messages[-1].accel.covariance
        linear_covariance_block: list[float] = [
            published_covariance[0],
            published_covariance[1],
            published_covariance[2],
            published_covariance[6],
            published_covariance[7],
            published_covariance[8],
            published_covariance[12],
            published_covariance[13],
            published_covariance[14],
        ]

        assert linear_covariance_block == pytest.approx(
            ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR,
            abs=1.0e-12,
        )
    finally:
        node.stop()


def test_stale_or_missing_gravity_does_not_publish_mounted_gravity() -> None:
    node, _, gravity_pub, _, _, _ = make_node(
        make_cached_mounting_transform((0.0, 0.0, 0.0, 1.0))
    )

    try:
        node._handle_imu(make_imu_message(timestamp_ns=1_000_000_000))
        assert len(gravity_pub.messages) == 0

        node._handle_gravity(make_gravity_message(timestamp_ns=100_000_000))
        node._handle_imu(make_imu_message(timestamp_ns=1_000_000_000))

        assert len(gravity_pub.messages) == 0
    finally:
        node.stop()


def test_boot_mounting_calibration_publishes_measured_fixed_tf() -> None:
    node, diag_pub, _, imu_pub, odom_pub, tf_broadcaster = make_node()
    node._mounting_calibrator = BootMountingCalibrator(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        calibration_duration_sec=2.0,
        min_sample_count=3,
    )

    try:
        gravity_vector = (
            9.81 * math.sin(math.radians(20.0)),
            -9.81 * math.sin(math.radians(30.0)) * math.cos(math.radians(20.0)),
            -9.81 * math.cos(math.radians(30.0)) * math.cos(math.radians(20.0)),
        )

        node._handle_gravity(
            make_gravity_message(
                gravity_vector=gravity_vector,
                timestamp_ns=0,
            )
        )
        node._handle_gravity(
            make_gravity_message(
                gravity_vector=gravity_vector,
                timestamp_ns=1_000_000_000,
            )
        )
        node._handle_gravity(
            make_gravity_message(
                gravity_vector=gravity_vector,
                timestamp_ns=2_000_000_000,
            )
        )
        published_mounting_messages = tf_broadcaster.transforms[-1]
        published_mounting_transform = next(
            transform
            for transform in published_mounting_messages
            if transform.header.frame_id == "base_link"
            and transform.child_frame_id == "imu_link"
        )
        solved_mounting_quaternion = (
            float(published_mounting_transform.transform.rotation.x),
            float(published_mounting_transform.transform.rotation.y),
            float(published_mounting_transform.transform.rotation.z),
            float(published_mounting_transform.transform.rotation.w),
        )

        # At boot `base_link` is level by policy, so the raw BNO086 driver
        # sample is physically `q_IW = q_BI`. AHRS canonicalizes that to
        # `q_WI = q_IB` before applying mounting.
        node._handle_imu(
            make_imu_message(
                quaternion_xyzw=solved_mounting_quaternion,
                timestamp_ns=2_100_000_000,
            )
        )

        assert len(imu_pub.messages) == 1
        assert len(odom_pub.messages) == 1
        assert last_diag(diag_pub).has_mounting is True
        assert last_diag(diag_pub).status == AhrsStatusMsg.STATUS_OK

        tf_messages = tf_broadcaster.transforms[-1]
        mounting_transform = next(
            transform
            for transform in tf_messages
            if transform.header.frame_id == "base_link"
            and transform.child_frame_id == "imu_link"
        )
        assert not math.isclose(
            mounting_transform.transform.rotation.x,
            0.0,
            abs_tol=1.0e-6,
        )
        assert not math.isclose(
            mounting_transform.transform.rotation.y,
            0.0,
            abs_tol=1.0e-6,
        )

        solved_mounting_yaw_rad = math.atan2(
            2.0
            * (
                mounting_transform.transform.rotation.w
                * mounting_transform.transform.rotation.z
                + mounting_transform.transform.rotation.x
                * mounting_transform.transform.rotation.y
            ),
            1.0
            - 2.0
            * (
                mounting_transform.transform.rotation.y
                * mounting_transform.transform.rotation.y
                + mounting_transform.transform.rotation.z
                * mounting_transform.transform.rotation.z
            ),
        )
        assert math.isclose(
            solved_mounting_yaw_rad,
            0.0,
            abs_tol=1.0e-6,
        )

        odom_to_base_transform = next(
            transform
            for transform in tf_messages
            if transform.header.frame_id == "odom"
            and transform.child_frame_id == "base_link"
        )
        assert math.isclose(
            odom_to_base_transform.transform.rotation.x,
            0.0,
            abs_tol=1.0e-6,
        )
        assert math.isclose(
            odom_to_base_transform.transform.rotation.y,
            0.0,
            abs_tol=1.0e-6,
        )
        assert math.isclose(
            odom_to_base_transform.transform.rotation.z,
            0.0,
            abs_tol=1.0e-6,
        )
        assert math.isclose(
            odom_to_base_transform.transform.rotation.w,
            1.0,
            abs_tol=1.0e-6,
        )

        imu_message: ImuMsg = imu_pub.messages[-1]
        assert imu_message.header.frame_id == "base_link"
        assert math.isclose(imu_message.orientation.x, 0.0, abs_tol=1.0e-6)
        assert math.isclose(imu_message.orientation.y, 0.0, abs_tol=1.0e-6)
    finally:
        node.stop()


def test_runtime_mounting_reduces_raw_driver_tilt_for_level_base() -> None:
    node, _, _, imu_pub, _, _ = make_node()
    node._mounting_calibrator = BootMountingCalibrator(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        calibration_duration_sec=2.0,
        min_sample_count=3,
    )

    try:
        gravity_vector = (
            -0.21263461167016393,
            0.2315803810024176,
            -9.804960838731313,
        )
        raw_driver_orientation_xyzw = (
            -0.011806184567030423,
            -0.010837526983655135,
            -0.00012796627922305811,
            0.99987156457191,
        )

        node._handle_gravity(
            make_gravity_message(
                gravity_vector=gravity_vector,
                timestamp_ns=0,
            )
        )
        node._handle_gravity(
            make_gravity_message(
                gravity_vector=gravity_vector,
                timestamp_ns=1_000_000_000,
            )
        )
        node._handle_gravity(
            make_gravity_message(
                gravity_vector=gravity_vector,
                timestamp_ns=2_000_000_000,
            )
        )
        node._handle_imu(
            make_imu_message(
                quaternion_xyzw=raw_driver_orientation_xyzw,
                timestamp_ns=2_100_000_000,
            )
        )

        mounted_message: ImuMsg = imu_pub.messages[-1]
        assert math.isclose(mounted_message.orientation.x, 0.0, abs_tol=2.0e-2)
        assert math.isclose(mounted_message.orientation.y, 0.0, abs_tol=2.0e-2)
        assert math.isclose(mounted_message.orientation.z, 0.0, abs_tol=2.0e-2)
        assert math.isclose(mounted_message.orientation.w, 1.0, abs_tol=2.0e-2)
    finally:
        node.stop()


def test_diag_status_changes_as_inputs_arrive() -> None:
    node, diag_pub, _, _, _, _ = make_node(
        make_cached_mounting_transform((0.0, 0.0, 0.0, 1.0))
    )

    try:
        node._publish_runtime_outputs()
        assert last_diag(diag_pub).status == AhrsStatusMsg.STATUS_WAITING_FOR_IMU
        assert last_diag(diag_pub).status_text == "Waiting for IMU samples"

        node._handle_imu(make_imu_message())
        assert last_diag(diag_pub).status == AhrsStatusMsg.STATUS_WAITING_FOR_GRAVITY
        assert last_diag(diag_pub).status_text == "Waiting for gravity samples"

        node._handle_gravity(make_gravity_message())
        node._handle_imu(make_imu_message(timestamp_ns=1_100_000_000))
        assert last_diag(diag_pub).status == AhrsStatusMsg.STATUS_OK
        assert last_diag(diag_pub).has_mounting is True
        assert last_diag(diag_pub).has_gravity is True
        assert last_diag(diag_pub).gravity_gated_in is True
        assert last_diag(diag_pub).gravity_rejected is False
        assert last_diag(diag_pub).status_text == "Mounted attitude output available"
    finally:
        node.stop()


def test_bad_frame_updates_diagnostics_message() -> None:
    node, diag_pub, _, _, _, _ = make_node(
        make_cached_mounting_transform((0.0, 0.0, 0.0, 1.0))
    )

    try:
        node._handle_imu(make_imu_message(frame_id="camera_link"))
        assert last_diag(diag_pub).status == AhrsStatusMsg.STATUS_BAD_IMU_FRAME
        assert last_diag(diag_pub).rejected_imu_count == 1
        assert last_diag(diag_pub).status_text == "Bad IMU frame"

        node._handle_gravity(make_gravity_message(frame_id="camera_link"))
        assert last_diag(diag_pub).status == AhrsStatusMsg.STATUS_BAD_IMU_FRAME
        assert last_diag(diag_pub).rejected_gravity_count == 1
    finally:
        node.stop()


def test_latest_gravity_sample_is_used_deterministically() -> None:
    node, diag_pub, _, _, _, _ = make_node(
        make_cached_mounting_transform((0.0, 0.0, 0.0, 1.0))
    )

    try:
        node._handle_gravity(make_gravity_message(gravity_vector=(0.0, 0.0, -9.81)))
        node._handle_gravity(make_gravity_message(gravity_vector=(0.0, 9.81, 0.0)))
        node._handle_imu(make_imu_message())

        assert last_diag(diag_pub).gravity_residual_norm > 1.0
        assert last_diag(diag_pub).gravity_rejected is True
    finally:
        node.stop()


def test_mounting_unavailable_is_reported_without_outputs() -> None:
    node, diag_pub, gravity_pub, imu_pub, odom_pub, _ = make_node()

    try:
        node._handle_gravity(make_gravity_message())
        node._handle_imu(make_imu_message())

        assert len(gravity_pub.messages) == 0
        assert len(imu_pub.messages) == 0
        assert len(odom_pub.messages) == 0
        assert last_diag(diag_pub).status == AhrsStatusMsg.STATUS_MOUNTING_UNAVAILABLE
        assert last_diag(diag_pub).has_mounting is False
        assert (
            last_diag(diag_pub).last_mounting_lookup_error
            == "boot mounting calibration in progress"
        )
        assert last_diag(diag_pub).status_text == "Mounting calibration not solved"
    finally:
        node.stop()


def test_stale_imu_does_not_publish_new_output() -> None:
    node, diag_pub, gravity_pub, imu_pub, odom_pub, _ = make_node(
        make_cached_mounting_transform((0.0, 0.0, 0.0, 1.0))
    )

    try:
        node._handle_gravity(make_gravity_message())
        node._handle_imu(make_imu_message(timestamp_ns=2_000_000_000))
        initial_gravity_publish_count: int = len(gravity_pub.messages)
        initial_imu_publish_count: int = len(imu_pub.messages)
        initial_odom_publish_count: int = len(odom_pub.messages)

        node._handle_imu(make_imu_message(timestamp_ns=1_000_000_000))

        assert len(gravity_pub.messages) == initial_gravity_publish_count
        assert len(imu_pub.messages) == initial_imu_publish_count
        assert len(odom_pub.messages) == initial_odom_publish_count
        assert last_diag(diag_pub).dropped_stale_imu_count == 1
        assert last_diag(diag_pub).status_text == "Mounted attitude output available"
    finally:
        node.stop()


def test_stale_gravity_does_not_overwrite_latest_sample() -> None:
    node, diag_pub, _, _, _, _ = make_node(
        make_cached_mounting_transform((0.0, 0.0, 0.0, 1.0))
    )

    try:
        node._handle_gravity(
            make_gravity_message(
                gravity_vector=(0.0, 9.81, 0.0),
                timestamp_ns=2_000_000_000,
            )
        )
        node._handle_gravity(
            make_gravity_message(
                gravity_vector=(0.0, 0.0, -9.81),
                timestamp_ns=1_000_000_000,
            )
        )
        node._handle_imu(make_imu_message(timestamp_ns=2_100_000_000))

        assert last_diag(diag_pub).dropped_stale_gravity_count == 1
        assert last_diag(diag_pub).gravity_residual_norm > 1.0
    finally:
        node.stop()


def test_gravity_rejection_updates_diagnostics_message() -> None:
    node, diag_pub, _, _, _, _ = make_node(
        make_cached_mounting_transform((0.0, 0.0, 0.0, 1.0))
    )

    try:
        node._handle_gravity(make_gravity_message(gravity_vector=(0.0, 9.81, 0.0)))
        node._handle_imu(make_imu_message())

        assert last_diag(diag_pub).gravity_rejected is True
        assert last_diag(diag_pub).gravity_gated_in is False
        assert last_diag(diag_pub).gravity_rejection_count == 1
        assert last_diag(diag_pub).last_gravity_rejection_reason == "residual_norm"
        assert last_diag(diag_pub).status_text == "Gravity consistency rejected"
    finally:
        node.stop()


def test_resting_gravity_with_overconfident_covariance_gates_in() -> None:
    node, diag_pub, _, _, _, _ = make_node(
        make_cached_mounting_transform((0.0, 0.0, 0.0, 1.0))
    )

    try:
        node._handle_gravity(
            make_gravity_message(
                gravity_vector=(0.05, 0.0, -9.81),
                covariance=[
                    1.0e-6,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0e-6,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0e-6,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            )
        )
        node._handle_imu(make_imu_message())

        assert last_diag(diag_pub).gravity_gated_in is True
        assert last_diag(diag_pub).gravity_rejected is False
        assert last_diag(diag_pub).last_gravity_rejection_reason == ""
        assert last_diag(diag_pub).gravity_residual_norm < 0.35
        assert math.isfinite(last_diag(diag_pub).gravity_mahalanobis_distance)
        assert last_diag(diag_pub).gravity_mahalanobis_distance > 1.0
    finally:
        node.stop()


def test_unsolved_mounting_reports_calibration_progress() -> None:
    node, diag_pub, _, _, _, _ = make_node()

    try:
        node._handle_gravity(make_gravity_message(timestamp_ns=1_000_000_000))
        node._handle_imu(make_imu_message(timestamp_ns=1_100_000_000))

        assert (
            last_diag(diag_pub).last_mounting_lookup_error
            == "boot mounting calibration in progress"
        )
    finally:
        node.stop()


def test_imu_output_replaces_unknown_upstream_covariance_with_honest_split() -> None:
    node, _, _, imu_pub, _, _ = make_node(
        make_cached_mounting_transform((0.0, 0.0, 0.0, 1.0))
    )

    try:
        node._handle_gravity(make_gravity_message())
        node._handle_imu(
            make_imu_message(
                orientation_covariance=[-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
        )

        published_covariance: list[float] = imu_pub.messages[-1].orientation_covariance
        expected_roll_pitch_variance_rad2: float = (
            gravity_covariance_to_roll_pitch_variance_rad2(
                gravity_mps2=(0.0, 0.0, -9.81),
                gravity_covariance_mps2_2=(
                    (0.04, 0.0, 0.0),
                    (0.0, 0.04, 0.0),
                    (0.0, 0.0, 0.04),
                ),
            ).roll_variance_rad2
        )

        assert math.isclose(
            published_covariance[0],
            expected_roll_pitch_variance_rad2,
            abs_tol=1.0e-12,
        )
        assert math.isclose(
            published_covariance[4],
            expected_roll_pitch_variance_rad2,
            abs_tol=1.0e-12,
        )
        assert math.isclose(published_covariance[8], 0.0, abs_tol=1.0e-12)
    finally:
        node.stop()


def test_imu_output_uses_gravity_roll_pitch_scale_not_upstream_orientation_buckets() -> (
    None
):
    quarter_turn_about_z_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    )
    node, _, _, imu_pub, _, _ = make_node(
        make_cached_mounting_transform(quarter_turn_about_z_xyzw)
    )

    try:
        node._handle_gravity(make_gravity_message())
        node._handle_imu(
            make_imu_message(
                orientation_covariance=FULL_ORIENTATION_COVARIANCE_ROW_MAJOR
            )
        )

        expected_roll_pitch_variance_rad2: float = (
            gravity_covariance_to_roll_pitch_variance_rad2(
                gravity_mps2=(0.0, 0.0, -9.81),
                gravity_covariance_mps2_2=(
                    (0.04, 0.0, 0.0),
                    (0.0, 0.04, 0.0),
                    (0.0, 0.0, 0.04),
                ),
            ).roll_variance_rad2
        )

        assert imu_pub.messages[-1].orientation_covariance == pytest.approx(
            [
                expected_roll_pitch_variance_rad2,
                0.0,
                0.0,
                0.0,
                expected_roll_pitch_variance_rad2,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            abs=1.0e-12,
        )
    finally:
        node.stop()


def test_odom_output_reuses_honest_published_orientation_covariance_block() -> None:
    quarter_turn_about_z_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    )
    node, _, _, _, odom_pub, _ = make_node(
        make_cached_mounting_transform(quarter_turn_about_z_xyzw)
    )

    try:
        node._handle_gravity(make_gravity_message())
        node._handle_imu(
            make_imu_message(
                orientation_covariance=FULL_ORIENTATION_COVARIANCE_ROW_MAJOR
            )
        )

        pose_covariance = odom_pub.messages[-1].pose.covariance
        expected_roll_pitch_variance_rad2: float = (
            gravity_covariance_to_roll_pitch_variance_rad2(
                gravity_mps2=(0.0, 0.0, -9.81),
                gravity_covariance_mps2_2=(
                    (0.04, 0.0, 0.0),
                    (0.0, 0.04, 0.0),
                    (0.0, 0.0, 0.04),
                ),
            ).roll_variance_rad2
        )
        assert pose_covariance[21] == pytest.approx(
            expected_roll_pitch_variance_rad2,
            abs=1.0e-12,
        )
        assert pose_covariance[22] == pytest.approx(
            0.0,
            abs=1.0e-12,
        )
        assert pose_covariance[23] == pytest.approx(
            0.0,
            abs=1.0e-12,
        )
        assert pose_covariance[27] == pytest.approx(
            0.0,
            abs=1.0e-12,
        )
        assert pose_covariance[28] == pytest.approx(
            expected_roll_pitch_variance_rad2,
            abs=1.0e-12,
        )
        assert pose_covariance[29] == pytest.approx(
            0.0,
            abs=1.0e-12,
        )
        assert pose_covariance[33] == pytest.approx(
            0.0,
            abs=1.0e-12,
        )
        assert pose_covariance[34] == pytest.approx(
            0.0,
            abs=1.0e-12,
        )
        assert pose_covariance[35] == pytest.approx(
            0.0,
            abs=1.0e-12,
        )
    finally:
        node.stop()


def test_imu_output_yaw_variance_tracks_recent_yaw_jitter_with_wrap() -> None:
    node, _, _, imu_pub, _, _ = make_node(
        make_cached_mounting_transform((0.0, 0.0, 0.0, 1.0))
    )

    try:
        node._handle_gravity(make_gravity_message(timestamp_ns=900_000_000))
        node._handle_imu(
            make_imu_message(
                quaternion_xyzw=_driver_quaternion_from_mounted_yaw_rad(
                    math.radians(179.8)
                ),
                timestamp_ns=1_000_000_000,
            )
        )
        node._handle_gravity(make_gravity_message(timestamp_ns=1_400_000_000))
        node._handle_imu(
            make_imu_message(
                quaternion_xyzw=_driver_quaternion_from_mounted_yaw_rad(
                    math.radians(-179.9)
                ),
                timestamp_ns=1_500_000_000,
            )
        )
        node._handle_gravity(make_gravity_message(timestamp_ns=1_900_000_000))
        node._handle_imu(
            make_imu_message(
                quaternion_xyzw=_driver_quaternion_from_mounted_yaw_rad(
                    math.radians(179.9)
                ),
                timestamp_ns=2_000_000_000,
            )
        )
        node._handle_gravity(make_gravity_message(timestamp_ns=2_400_000_000))
        node._handle_imu(
            make_imu_message(
                quaternion_xyzw=_driver_quaternion_from_mounted_yaw_rad(
                    math.radians(-179.8)
                ),
                timestamp_ns=2_500_000_000,
            )
        )

        assert imu_pub.messages[-1].orientation_covariance[8] < math.radians(0.25) ** 2
    finally:
        node.stop()


def _yaw_from_xyzw(quaternion_xyzw: tuple[float, float, float, float]) -> float:
    x_value: float
    y_value: float
    z_value: float
    w_value: float
    x_value, y_value, z_value, w_value = quaternion_conjugate_xyzw(quaternion_xyzw)
    return math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value),
    )


def _quaternion_from_yaw_rad(yaw_rad: float) -> tuple[float, float, float, float]:
    half_yaw_rad: float = 0.5 * yaw_rad
    return (
        0.0,
        0.0,
        math.sin(half_yaw_rad),
        math.cos(half_yaw_rad),
    )


def _driver_quaternion_from_mounted_yaw_rad(
    mounted_yaw_rad: float,
) -> tuple[float, float, float, float]:
    return _quaternion_from_yaw_rad(mounted_yaw_rad)


def _driver_quaternion_from_mounted_roll_pitch_yaw_rad(
    *,
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> tuple[float, float, float, float]:
    half_roll_rad: float = roll_rad * 0.5
    half_pitch_rad: float = pitch_rad * 0.5
    half_yaw_rad: float = yaw_rad * 0.5

    sin_half_roll: float = math.sin(half_roll_rad)
    cos_half_roll: float = math.cos(half_roll_rad)
    sin_half_pitch: float = math.sin(half_pitch_rad)
    cos_half_pitch: float = math.cos(half_pitch_rad)
    sin_half_yaw: float = math.sin(half_yaw_rad)
    cos_half_yaw: float = math.cos(half_yaw_rad)

    return (
        sin_half_roll * cos_half_pitch * cos_half_yaw
        - cos_half_roll * sin_half_pitch * sin_half_yaw,
        cos_half_roll * sin_half_pitch * cos_half_yaw
        + sin_half_roll * cos_half_pitch * sin_half_yaw,
        cos_half_roll * cos_half_pitch * sin_half_yaw
        - sin_half_roll * sin_half_pitch * cos_half_yaw,
        cos_half_roll * cos_half_pitch * cos_half_yaw
        + sin_half_roll * sin_half_pitch * sin_half_yaw,
    )
