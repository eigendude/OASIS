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


rclpy = pytest.importorskip("rclpy")

from sensor_msgs.msg import Imu as ImuMsg

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
        odom_to_base_transform = next(
            transform
            for transform in tf_messages
            if transform.header.frame_id == "odom"
            and transform.child_frame_id == "base_link"
        )
        assert math.isclose(
            odom_to_base_transform.transform.rotation.z,
            quarter_turn_about_z_xyzw[2],
        )
        assert math.isclose(
            odom_to_base_transform.transform.rotation.w,
            quarter_turn_about_z_xyzw[3],
        )
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

        assert (
            imu_pub.messages[-1].orientation_covariance
            == ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR
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
        assert pose_covariance[21] == ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[0]
        assert pose_covariance[22] == ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[1]
        assert pose_covariance[23] == ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[2]
        assert pose_covariance[27] == ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[3]
        assert pose_covariance[28] == ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[4]
        assert pose_covariance[29] == ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[5]
        assert pose_covariance[33] == ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[6]
        assert pose_covariance[34] == ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[7]
        assert pose_covariance[35] == ROTATED_ORIENTATION_COVARIANCE_ROW_MAJOR[8]
    finally:
        node.stop()
