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
        max_dt_ns=_ns_from_ms(100),
        checkpoint_interval_ns=_ns_from_ms(100),
        apriltag_pos_var=0.1,
        apriltag_yaw_var=0.1,
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


def test_odom_continuity_reset_preserves_covariance() -> None:
    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)
    core._initialized = True

    core._state.pose_ob.translation_m[:] = [0.3, -0.2, 1.1]
    core._state.pose_ob.rotation_wxyz = quat_from_rotvec([0.0, 0.0, 0.2])
    core._state.world_odom.translation_m[:] = [1.0, 0.5, -0.3]
    core._state.world_odom.rotation_wxyz = quat_from_rotvec([0.0, 0.0, -0.1])

    odom_before: Pose3 = core._state.pose_ob.copy()
    vel_before: Any = core._state.vel_o_mps.copy()

    core._state.pose_ob.translation_m[:] = [0.8, 0.4, 0.9]
    core._state.pose_ob.rotation_wxyz = quat_from_rotvec([0.0, 0.0, -0.3])
    core._state.world_odom.translation_m[:] = [1.2, -0.4, 0.2]
    core._state.world_odom.rotation_wxyz = quat_from_rotvec([0.0, 0.0, 0.3])

    core._apply_odom_continuity_reset(odom_before, vel_before)

    reset_pose: Pose3 = core._state.pose_ob
    expected_translation: list[float] = odom_before.translation_m.tolist()
    expected_rotation: list[float] = odom_before.rotation_wxyz.tolist()

    index: int
    for index in range(3):
        assert math.isclose(
            float(reset_pose.translation_m[index]),
            expected_translation[index],
            abs_tol=1.0e-12,
        )
    for index in range(4):
        assert math.isclose(
            float(reset_pose.rotation_wxyz[index]),
            expected_rotation[index],
            abs_tol=1.0e-12,
        )

    covariance: list[list[float]] = core._state.covariance.tolist()
    dim: int = len(covariance)
    row: int
    col: int
    for row in range(dim):
        for col in range(dim):
            assert math.isclose(
                covariance[row][col],
                covariance[col][row],
                abs_tol=1.0e-9,
            )
        assert covariance[row][row] >= -1.0e-9
