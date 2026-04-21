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
from sensor_msgs.msg import Imu as ImuMsg

from oasis_control.localization.ahrs.processing.boot_mounting_calibrator import (
    BootMountingCalibrator,
)
from oasis_control.localization.ahrs_tilt_estimator import AhrsTiltEstimator
from oasis_msgs.msg import AhrsStatus as AhrsStatusMsg

from ._node_test_helpers import FakeTfBuffer
from ._node_test_helpers import last_diag
from ._node_test_helpers import make_gravity_message
from ._node_test_helpers import make_imu_message
from ._node_test_helpers import make_mounting_transform
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
    node, diag_pub, imu_pub, odom_pub, tf_broadcaster = make_node(
        FakeTfBuffer(make_mounting_transform(quarter_turn_about_z_xyzw))
    )

    try:
        node._handle_gravity(make_gravity_message())
        node._handle_imu(make_imu_message())

        imu_message: ImuMsg = imu_pub.messages[-1]
        odom_message = odom_pub.messages[-1]
        tf_messages = tf_broadcaster.transforms[-1]

        assert len(imu_pub.messages) == 1
        assert len(odom_pub.messages) == 1
        assert last_diag(diag_pub).status == AhrsStatusMsg.STATUS_OK
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
    node, _, imu_pub, odom_pub, tf_broadcaster = make_node(
        FakeTfBuffer(make_mounting_transform((0.0, 0.0, 0.0, 1.0)))
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


def test_boot_mounting_calibration_publishes_measured_fixed_tf() -> None:
    node, diag_pub, imu_pub, odom_pub, tf_broadcaster = make_node(FakeTfBuffer(None))
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
    node, _, imu_pub, _, _ = make_node(FakeTfBuffer(None))
    node._mounting_calibrator = BootMountingCalibrator(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        calibration_duration_sec=2.0,
        min_sample_count=3,
    )
    tilt_estimator = AhrsTiltEstimator()

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

        raw_tilt = tilt_estimator.update(
            orientation_xyzw=raw_driver_orientation_xyzw,
        )
        mounted_message: ImuMsg = imu_pub.messages[-1]
        mounted_tilt = tilt_estimator.update(
            orientation_xyzw=(
                float(mounted_message.orientation.x),
                float(mounted_message.orientation.y),
                float(mounted_message.orientation.z),
                float(mounted_message.orientation.w),
            ),
        )

        assert raw_tilt is not None
        assert mounted_tilt is not None
        assert abs(raw_tilt.roll_rad) > math.radians(1.0)
        assert abs(raw_tilt.pitch_rad) > math.radians(1.0)
        assert math.isclose(mounted_tilt.roll_rad, 0.0, abs_tol=1.0e-6)
        assert math.isclose(mounted_tilt.pitch_rad, 0.0, abs_tol=1.0e-6)
    finally:
        node.stop()


def test_diag_status_changes_as_inputs_arrive() -> None:
    node, diag_pub, _, _, _ = make_node(
        FakeTfBuffer(make_mounting_transform((0.0, 0.0, 0.0, 1.0)))
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
    node, diag_pub, _, _, _ = make_node(
        FakeTfBuffer(make_mounting_transform((0.0, 0.0, 0.0, 1.0)))
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
    node, diag_pub, _, _, _ = make_node(
        FakeTfBuffer(make_mounting_transform((0.0, 0.0, 0.0, 1.0)))
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
    node, diag_pub, imu_pub, odom_pub, _ = make_node(FakeTfBuffer(None))

    try:
        node._handle_gravity(make_gravity_message())
        node._handle_imu(make_imu_message())

        assert len(imu_pub.messages) == 0
        assert len(odom_pub.messages) == 0
        assert last_diag(diag_pub).status == AhrsStatusMsg.STATUS_MOUNTING_UNAVAILABLE
        assert last_diag(diag_pub).has_mounting is False
        assert last_diag(diag_pub).transform_lookup_failure_count == 1
        assert last_diag(diag_pub).status_text == "Mounting transform unavailable"
    finally:
        node.stop()


def test_stale_imu_does_not_publish_new_output() -> None:
    node, diag_pub, imu_pub, odom_pub, _ = make_node(
        FakeTfBuffer(make_mounting_transform((0.0, 0.0, 0.0, 1.0)))
    )

    try:
        node._handle_gravity(make_gravity_message())
        node._handle_imu(make_imu_message(timestamp_ns=2_000_000_000))
        initial_imu_publish_count: int = len(imu_pub.messages)
        initial_odom_publish_count: int = len(odom_pub.messages)

        node._handle_imu(make_imu_message(timestamp_ns=1_000_000_000))

        assert len(imu_pub.messages) == initial_imu_publish_count
        assert len(odom_pub.messages) == initial_odom_publish_count
        assert last_diag(diag_pub).dropped_stale_imu_count == 1
        assert last_diag(diag_pub).status_text == "Mounted attitude output available"
    finally:
        node.stop()


def test_stale_gravity_does_not_overwrite_latest_sample() -> None:
    node, diag_pub, _, _, _ = make_node(
        FakeTfBuffer(make_mounting_transform((0.0, 0.0, 0.0, 1.0)))
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
    node, diag_pub, _, _, _ = make_node(
        FakeTfBuffer(make_mounting_transform((0.0, 0.0, 0.0, 1.0)))
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
    node, diag_pub, _, _, _ = make_node(
        FakeTfBuffer(make_mounting_transform((0.0, 0.0, 0.0, 1.0)))
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


def test_repeated_mounting_lookup_failures_increment_diagnostics_message() -> None:
    fake_tf_buffer: FakeTfBuffer = FakeTfBuffer(None)
    node, diag_pub, _, _, _ = make_node(fake_tf_buffer)

    try:
        node._handle_gravity(make_gravity_message())
        node._handle_imu(make_imu_message(timestamp_ns=1_000_000_000))
        node._handle_imu(make_imu_message(timestamp_ns=2_000_000_000))

        assert fake_tf_buffer.lookup_count == 2
        assert last_diag(diag_pub).transform_lookup_failure_count == 2
        assert "missing transform" in last_diag(diag_pub).last_mounting_lookup_error
    finally:
        node.stop()


def test_invalid_mounting_quaternion_is_accounted_for() -> None:
    node, diag_pub, imu_pub, odom_pub, _ = make_node(
        FakeTfBuffer(make_mounting_transform((0.0, 0.0, 0.0, 0.0)))
    )

    try:
        node._handle_gravity(make_gravity_message())
        node._handle_imu(make_imu_message())

        assert len(imu_pub.messages) == 0
        assert len(odom_pub.messages) == 0
        assert last_diag(diag_pub).invalid_mounting_transform_count == 1
        assert (
            last_diag(diag_pub).last_mounting_lookup_error
            == "mounting quaternion is non-finite or zero-norm"
        )
        assert last_diag(diag_pub).status_text == "Mounting transform unavailable"
    finally:
        node.stop()


def test_successful_mounting_lookup_is_cached() -> None:
    fake_tf_buffer: FakeTfBuffer = FakeTfBuffer(
        make_mounting_transform((0.0, 0.0, 0.0, 1.0))
    )
    node, diag_pub, _, _, _ = make_node(fake_tf_buffer)

    try:
        node._handle_gravity(make_gravity_message())
        node._handle_imu(make_imu_message(timestamp_ns=1_000_000_000))
        node._handle_imu(make_imu_message(timestamp_ns=2_000_000_000))

        assert fake_tf_buffer.lookup_count == 1
        assert last_diag(diag_pub).has_mounting is True
        assert last_diag(diag_pub).transform_lookup_failure_count == 0
    finally:
        node.stop()


def test_imu_output_preserves_unknown_orientation_covariance_semantics() -> None:
    node, _, imu_pub, _, _ = make_node(
        FakeTfBuffer(make_mounting_transform((0.0, 0.0, 0.0, 1.0)))
    )

    try:
        node._handle_gravity(make_gravity_message())
        node._handle_imu(
            make_imu_message(
                orientation_covariance=[-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            )
        )

        assert imu_pub.messages[-1].orientation_covariance[0] == -1.0
    finally:
        node.stop()


def test_imu_output_publishes_mapped_orientation_covariance_unchanged() -> None:
    quarter_turn_about_z_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    )
    node, _, imu_pub, _, _ = make_node(
        FakeTfBuffer(make_mounting_transform(quarter_turn_about_z_xyzw))
    )

    try:
        node._handle_gravity(make_gravity_message())
        node._handle_imu(
            make_imu_message(
                orientation_covariance=FULL_ORIENTATION_COVARIANCE_ROW_MAJOR
            )
        )

        assert imu_pub.messages[-1].orientation_covariance == pytest.approx(
            ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR,
            abs=1.0e-12,
        )
    finally:
        node.stop()


def test_odom_output_reuses_mapped_orientation_covariance_block() -> None:
    quarter_turn_about_z_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    )
    node, _, _, odom_pub, _ = make_node(
        FakeTfBuffer(make_mounting_transform(quarter_turn_about_z_xyzw))
    )

    try:
        node._handle_gravity(make_gravity_message())
        node._handle_imu(
            make_imu_message(
                orientation_covariance=FULL_ORIENTATION_COVARIANCE_ROW_MAJOR
            )
        )

        pose_covariance = odom_pub.messages[-1].pose.covariance
        assert pose_covariance[21] == pytest.approx(
            ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[0],
            abs=1.0e-12,
        )
        assert pose_covariance[22] == pytest.approx(
            ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[1],
            abs=1.0e-12,
        )
        assert pose_covariance[23] == pytest.approx(
            ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[2],
            abs=1.0e-12,
        )
        assert pose_covariance[27] == pytest.approx(
            ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[3],
            abs=1.0e-12,
        )
        assert pose_covariance[28] == pytest.approx(
            ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[4],
            abs=1.0e-12,
        )
        assert pose_covariance[29] == pytest.approx(
            ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[5],
            abs=1.0e-12,
        )
        assert pose_covariance[33] == pytest.approx(
            ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[6],
            abs=1.0e-12,
        )
        assert pose_covariance[34] == pytest.approx(
            ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[7],
            abs=1.0e-12,
        )
        assert pose_covariance[35] == pytest.approx(
            ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[8],
            abs=1.0e-12,
        )
    finally:
        node.stop()


def _yaw_from_xyzw(quaternion_xyzw: tuple[float, float, float, float]) -> float:
    x_value, y_value, z_value, w_value = quaternion_xyzw
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
    return _quaternion_from_yaw_rad(-mounted_yaw_rad)
