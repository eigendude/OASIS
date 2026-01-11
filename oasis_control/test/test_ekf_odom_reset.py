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

from typing import Any

from oasis_control.localization.ekf.core.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_state import Pose3
from oasis_control.localization.ekf.se3 import quat_from_rotvec


_NS_PER_S: int = 1_000_000_000
_NS_PER_MS: int = 1_000_000


def _ns_from_s(seconds: int) -> int:
    return seconds * _NS_PER_S


def _ns_from_ms(milliseconds: int) -> int:
    return milliseconds * _NS_PER_MS


def _build_config() -> EkfConfig:
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_ns=_ns_from_s(1),
        epsilon_wall_future_ns=_ns_from_ms(100),
        dt_clock_jump_max_ns=_ns_from_s(5),
        dt_imu_max_ns=_ns_from_s(1),
        pos_var=1.0,
        vel_var=1.0,
        ang_var=1.0,
        accel_noise_var=0.1,
        gyro_noise_var=0.1,
        gravity_mps2=9.81,
        max_dt_ns=_ns_from_ms(10),
        checkpoint_interval_ns=_ns_from_ms(100),
        apriltag_pos_var=0.05,
        apriltag_yaw_var=0.01,
        apriltag_gate_d2=0.0,
        apriltag_reproj_rms_gate_px=0.0,
        tag_size_m=0.16,
        tag_anchor_family="tag36h11",
        tag_anchor_id=0,
        tag_landmark_prior_sigma_t_m=0.1,
        tag_landmark_prior_sigma_rot_rad=0.1,
        extrinsic_prior_sigma_t_m=1.0,
        extrinsic_prior_sigma_rot_rad=math.pi,
    )


def _assert_pose_equal(
    *, expected: Pose3, actual: Pose3, tol: float = 1.0e-9
) -> None:
    expected_translation: list[float] = expected.translation_m.tolist()
    actual_translation: list[float] = actual.translation_m.tolist()
    for exp, act in zip(expected_translation, actual_translation):
        assert abs(exp - act) <= tol
    expected_rotation: list[float] = expected.rotation_wxyz.tolist()
    actual_rotation: list[float] = actual.rotation_wxyz.tolist()
    for exp, act in zip(expected_rotation, actual_rotation):
        assert abs(exp - act) <= tol


def _assert_covariance_symmetric(covariance: list[list[float]], tol: float) -> None:
    size: int = len(covariance)
    row: int
    for row in range(size):
        col: int
        for col in range(size):
            assert abs(covariance[row][col] - covariance[col][row]) <= tol


def _assert_covariance_psdish(covariance: list[list[float]], tol: float) -> None:
    size: int = len(covariance)
    index: int
    for index in range(size):
        assert covariance[index][index] >= -tol


def test_odom_reset_preserves_covariance_invariants() -> None:
    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)

    odom_before: Pose3 = core._state.pose_ob.copy()
    vel_before: Any = core._state.vel_o_mps.copy()

    core._state.pose_ob.translation_m[:] = [0.2, -0.1, 0.4]
    core._state.pose_ob.rotation_wxyz = quat_from_rotvec([0.0, 0.0, 0.2])
    core._state.world_odom.translation_m[:] = [0.5, 0.0, -0.2]
    core._state.world_odom.rotation_wxyz = quat_from_rotvec([0.0, 0.1, 0.0])

    dim: int = core._state.index.total_dim
    covariance: Any = core._state.covariance
    covariance[:] = 0.0
    diag: int
    for diag in range(dim):
        covariance[diag, diag] = 1.0

    core._apply_odom_continuity_reset(odom_before, vel_before)

    _assert_pose_equal(expected=odom_before, actual=core._state.pose_ob)
    cov_list: list[list[float]] = core._state.covariance.tolist()
    _assert_covariance_symmetric(cov_list, tol=1.0e-9)
    _assert_covariance_psdish(cov_list, tol=1.0e-9)
