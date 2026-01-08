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

import math
from types import ModuleType

import pytest


np: ModuleType = pytest.importorskip(
    "numpy", reason="TODO: requires numpy for EKF tests"
)

from oasis_control.localization.ekf.apriltag_pose_selector import (
    AprilTagPoseDetectionCandidate,
)
from oasis_control.localization.ekf.apriltag_pose_selector import (
    select_best_apriltag_candidate,
)
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_types import EventAprilTagPose
from oasis_control.localization.ekf.pose_math import quaternion_from_rpy
from oasis_control.localization.ekf.pose_math import quaternion_inverse
from oasis_control.localization.ekf.pose_math import quaternion_log
from oasis_control.localization.ekf.pose_math import quaternion_multiply


def _build_config() -> EkfConfig:
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_sec=1.0,
        epsilon_wall_future=0.1,
        dt_clock_jump_max=1.0,
        dt_imu_max=1.0,
        pos_var=1.0,
        vel_var=1.0,
        ang_var=1.0,
        accel_noise_var=0.1,
        gyro_noise_var=0.1,
        gravity_mps2=9.81,
        max_dt_sec=0.1,
        checkpoint_interval_sec=0.5,
        apriltag_pos_var=1.0,
        apriltag_yaw_var=1.0,
        apriltag_gate_d2=0.0,
        tag_size_m=0.16,
        tag_anchor_family="tag36h11",
        tag_anchor_id=0,
        tag_landmark_prior_sigma_t_m=0.1,
        tag_landmark_prior_sigma_rot_rad=0.1,
    )


def _build_pose_covariance(*, pos_var: float, rot_var: float) -> list[float]:
    covariance: list[float] = [0.0] * 36
    for index in range(3):
        covariance[index * 6 + index] = pos_var
    for index in range(3):
        covariance[(index + 3) * 6 + (index + 3)] = rot_var
    return covariance


def test_world_correction_preserves_odom_continuity() -> None:
    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)
    covariance_diag: list[float] = [1.0] * 9
    core.set_state(
        position_m=[1.0, 0.0, 0.0],
        velocity_mps=[0.0, 0.0, 0.0],
        rpy_rad=[0.0, 0.0, 0.0],
        covariance_diag=covariance_diag,
        t_meas=0.0,
    )

    quat_meas: tuple[float, float, float, float] = quaternion_from_rpy(0.0, 0.0, 0.0)
    covariance: list[float] = _build_pose_covariance(pos_var=1.0e-4, rot_var=1.0e-4)
    event: EventAprilTagPose = EventAprilTagPose(
        timestamp_s=1.0,
        p_meas_world_base_m=[1.2, 0.0, 0.0],
        q_meas_world_base_xyzw=list(quat_meas),
        covariance=covariance,
        tag_id=0,
        frame_id="camera",
        source_topic="apriltags",
        family="tag36h11",
        det_index_in_msg=0,
    )

    core.update_with_apriltag_pose(event, 1.0)

    world_base_pose = core.world_base_pose()
    odom_base_pose = core.odom_base_pose()
    world_odom_pose = core.world_odom_pose()

    assert math.isclose(world_base_pose.position_m[0], 1.2, abs_tol=5.0e-2)
    assert math.isclose(odom_base_pose.position_m[0], 1.0, abs_tol=1.0e-3)
    assert math.isclose(world_odom_pose.position_m[0], 0.2, abs_tol=5.0e-2)


def test_orientation_residual_log_map_sign() -> None:
    q_pred: tuple[float, float, float, float] = quaternion_from_rpy(0.0, 0.0, 0.0)
    q_meas: tuple[float, float, float, float] = quaternion_from_rpy(0.0, 0.0, 0.1)
    q_error: tuple[float, float, float, float] = quaternion_multiply(
        q_meas, quaternion_inverse(q_pred)
    )
    r_theta: tuple[float, float, float] = quaternion_log(q_error)

    assert math.isclose(r_theta[0], 0.0, abs_tol=1.0e-6)
    assert math.isclose(r_theta[1], 0.0, abs_tol=1.0e-6)
    assert math.isclose(r_theta[2], 0.1, abs_tol=1.0e-3)


def _candidate(*, tag_id: int, det_index: int) -> AprilTagPoseDetectionCandidate:
    covariance: list[float] = _build_pose_covariance(pos_var=1.0, rot_var=1.0)
    return AprilTagPoseDetectionCandidate(
        family="tag36h11",
        tag_id=tag_id,
        det_index_in_msg=det_index,
        frame_id="camera",
        source_topic="apriltags",
        p_world_base_m=[0.0, 0.0, 0.0],
        q_world_base_xyzw=[0.0, 0.0, 0.0, 1.0],
        covariance=covariance,
        decision_margin=1.0,
        pose_error=None,
    )


def test_deterministic_apriltag_selection_lowest_id() -> None:
    candidates: list[AprilTagPoseDetectionCandidate] = [
        _candidate(tag_id=7, det_index=0),
        _candidate(tag_id=2, det_index=1),
        _candidate(tag_id=5, det_index=2),
    ]
    selected: AprilTagPoseDetectionCandidate = select_best_apriltag_candidate(
        candidates
    )
    assert selected.tag_id == 2
