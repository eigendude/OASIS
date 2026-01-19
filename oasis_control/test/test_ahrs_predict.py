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

from oasis_control.localization.ahrs.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.ahrs_error_state import AhrsErrorStateLayout
from oasis_control.localization.ahrs.ahrs_predict import predict_covariance
from oasis_control.localization.ahrs.ahrs_predict import predict_nominal
from oasis_control.localization.ahrs.ahrs_quat import quat_from_rotvec_wxyz
from oasis_control.localization.ahrs.ahrs_state import AhrsNominalState
from oasis_control.localization.ahrs.ahrs_state import default_nominal_state


def _config(**overrides: float) -> AhrsConfig:
    values: dict[str, float] = {
        "q_v": 0.0,
        "q_w": 0.0,
        "q_bg": 0.0,
        "q_ba": 0.0,
        "q_a": 0.0,
        "q_bi": 0.0,
        "q_bm": 0.0,
        "q_g": 0.0,
        "q_m": 0.0,
    }
    for key, value in overrides.items():
        values[key] = value

    return AhrsConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_sec=0.0,
        epsilon_wall_future_sec=0.0,
        dt_clock_jump_max_sec=0.0,
        dt_imu_max_sec=0.0,
        gyro_gate_d2_threshold=9.0,
        mag_alpha=1.0,
        mag_r_min=[0.0] * 9,
        mag_r_max=[0.0] * 9,
        mag_r0=[0.0] * 9,
        q_v=values["q_v"],
        q_w=values["q_w"],
        q_bg=values["q_bg"],
        q_ba=values["q_ba"],
        q_a=values["q_a"],
        q_bi=values["q_bi"],
        q_bm=values["q_bm"],
        q_g=values["q_g"],
        q_m=values["q_m"],
    )


def test_ahrs_predict_nominal_integrates_p_v() -> None:
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )
    state.v_wb_mps = [1.0, 0.0, 0.0]
    state.g_w_mps2 = [0.0, 0.0, -9.81]

    predicted: AhrsNominalState = predict_nominal(state, dt_sec=1.0)

    assert predicted.p_wb_m[0] == 1.0
    assert predicted.v_wb_mps[2] == -9.81


def test_ahrs_predict_quaternion_small_angle_normalized() -> None:
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )
    state.omega_wb_rps = [0.001, 0.0, 0.0]

    predicted: AhrsNominalState = predict_nominal(state, dt_sec=0.01)

    norm: float = math.sqrt(sum(value * value for value in predicted.q_wb_wxyz))
    assert math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-12)


def test_ahrs_predict_small_angle_exp_map_is_normalized() -> None:
    dq_wb: list[float] = quat_from_rotvec_wxyz([1.0e-15, 0.0, 0.0])

    norm: float = math.sqrt(sum(value * value for value in dq_wb))
    assert math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-15)


def test_ahrs_predict_covariance_zero_dt_returns_copy() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    dim: int = layout.dim
    p: list[float] = [0.0] * (dim * dim)
    p[0] = 0.25
    p[1] = -0.1
    p[dim] = -0.1

    predicted: list[float] = predict_covariance(layout, p, dt_sec=0.0, config=_config())

    assert predicted == p
    assert predicted is not p


def test_ahrs_predict_covariance_symmetry() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    dim: int = layout.dim
    p: list[float] = [0.0] * (dim * dim)
    p[0] = 1.0
    p[1] = 0.2
    p[dim] = 0.2

    predicted: list[float] = predict_covariance(
        layout, p, dt_sec=0.5, config=_config(q_v=0.1)
    )

    for r in range(dim):
        for c in range(dim):
            assert predicted[r * dim + c] == predicted[c * dim + r]


def test_ahrs_predict_covariance_has_process_noise_on_expected_blocks() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    dim: int = layout.dim
    p: list[float] = [0.0] * (dim * dim)

    predicted: list[float] = predict_covariance(
        layout, p, dt_sec=0.5, config=_config(q_v=0.3)
    )

    sl_v: slice = layout.sl_v()
    for i in range(sl_v.start, sl_v.stop):
        assert predicted[i * dim + i] > 0.0
