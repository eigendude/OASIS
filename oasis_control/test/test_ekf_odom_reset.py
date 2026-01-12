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

from typing import Any

import pytest

from oasis_control.localization.ekf.core.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_state import EkfState
from oasis_control.localization.ekf.ekf_state import Pose3


np: Any = pytest.importorskip("numpy")

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
        extrinsic_prior_sigma_rot_rad=3.141592653589793,
        mag_alpha=0.01,
        mag_r_min=[
            1.0e-12,
            0.0,
            0.0,
            0.0,
            1.0e-12,
            0.0,
            0.0,
            0.0,
            1.0e-12,
        ],
        mag_r_max=[
            2.5e-9,
            0.0,
            0.0,
            0.0,
            2.5e-9,
            0.0,
            0.0,
            0.0,
            2.5e-9,
        ],
        mag_r0_default=[
            4.0e-10,
            0.0,
            0.0,
            0.0,
            4.0e-10,
            0.0,
            0.0,
            0.0,
            4.0e-10,
        ],
        mag_world_t=[1.0, 0.0, 0.0],
    )


def test_odom_reset_covariance_invariants() -> None:
    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)
    state: EkfState = core._state

    odom_before: Pose3 = Pose3(
        translation_m=np.array([1.0, 0.0, 0.0], dtype=float),
        rotation_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
    )
    vel_before: np.ndarray = np.array([0.1, -0.2, 0.3], dtype=float)

    state.pose_ob = Pose3(
        translation_m=np.array([1.2, 0.1, -0.1], dtype=float),
        rotation_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
    )
    state.world_odom = Pose3(
        translation_m=np.array([0.5, -0.2, 0.1], dtype=float),
        rotation_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
    )
    state.covariance = np.eye(state.index.total_dim, dtype=float)

    core._apply_odom_continuity_reset(odom_before, vel_before)

    assert np.allclose(state.pose_ob.translation_m, odom_before.translation_m)
    assert np.allclose(state.pose_ob.rotation_wxyz, odom_before.rotation_wxyz)
    assert np.allclose(state.vel_o_mps, vel_before)
    assert np.allclose(state.covariance, state.covariance.T)
    assert np.all(np.diag(state.covariance) >= 0.0)
