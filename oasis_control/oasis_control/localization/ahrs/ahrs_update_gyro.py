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
Gyro measurement update for the AHRS core

Measurement model:
    z: Gyro angular velocity in rad/s, IMU frame assumed aligned with body
       and body frame for this update
    z_hat: Predicted angular velocity from omega_wb_rps in the nominal state
    nu: Innovation nu = z - z_hat
    R: Gyro measurement covariance in (rad/s)^2, row-major 3x3
    S_hat: Predicted innovation covariance H P H^T, row-major 3x3
    S: Innovation covariance S = S_hat + R, row-major 3x3

Gating:
    Mahalanobis distance d^2 = nu^T S^{-1} nu is compared against the
    configured gyro_gate_d2_threshold

Error-state mapping:
    The gyro update uses only the omega error-state block at layout.sl_omega
    This v1 update corrects omega_wb_rps and the omega covariance block only
    Cross-covariances are left unchanged and should be updated in a future
    full-state correction pass
"""

from __future__ import annotations

from oasis_control.localization.ahrs.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.ahrs_error_state import AhrsErrorStateLayout
from oasis_control.localization.ahrs.ahrs_linalg import innovation_covariance
from oasis_control.localization.ahrs.ahrs_linalg import is_finite_seq
from oasis_control.localization.ahrs.ahrs_linalg import mahalanobis_d2_3
from oasis_control.localization.ahrs.ahrs_linalg import mat3_inv
from oasis_control.localization.ahrs.ahrs_linalg import mat_mul
from oasis_control.localization.ahrs.ahrs_linalg import mat_transpose
from oasis_control.localization.ahrs.ahrs_linalg import mat_vec_mul
from oasis_control.localization.ahrs.ahrs_linalg import symmetrize
from oasis_control.localization.ahrs.ahrs_state import AhrsNominalState
from oasis_control.localization.ahrs.ahrs_types import AhrsMatrix
from oasis_control.localization.ahrs.ahrs_types import AhrsTime
from oasis_control.localization.ahrs.ahrs_types import AhrsUpdateData
from oasis_control.localization.ahrs.ahrs_types import ImuSample


def _copy_state(state: AhrsNominalState) -> AhrsNominalState:
    return AhrsNominalState(
        p_wb_m=list(state.p_wb_m),
        v_wb_mps=list(state.v_wb_mps),
        q_wb_wxyz=list(state.q_wb_wxyz),
        omega_wb_rps=list(state.omega_wb_rps),
        b_g_rps=list(state.b_g_rps),
        b_a_mps2=list(state.b_a_mps2),
        a_a=list(state.a_a),
        t_bi=state.t_bi,
        t_bm=state.t_bm,
        g_w_mps2=list(state.g_w_mps2),
        m_w_t=list(state.m_w_t),
    )


def _empty_matrix() -> AhrsMatrix:
    return AhrsMatrix(rows=0, cols=0, data=[])


def _matrix3(data: list[float]) -> AhrsMatrix:
    return AhrsMatrix(rows=3, cols=3, data=list(data))


def _extract_block(
    p: list[float], dim: int, sl_rows: slice, sl_cols: slice
) -> list[float]:
    rows: int = sl_rows.stop - sl_rows.start
    cols: int = sl_cols.stop - sl_cols.start
    if rows != 3 or cols != 3:
        raise ValueError("gyro update expects a 3x3 block")
    block: list[float] = [0.0] * (rows * cols)
    for r in range(rows):
        row_idx: int = sl_rows.start + r
        base: int = row_idx * dim + sl_cols.start
        for c in range(cols):
            block[r * cols + c] = p[base + c]
    return block


def _set_block(
    p: list[float],
    dim: int,
    sl_rows: slice,
    sl_cols: slice,
    block: list[float],
) -> None:
    rows: int = sl_rows.stop - sl_rows.start
    cols: int = sl_cols.stop - sl_cols.start
    if len(block) != rows * cols:
        raise ValueError("block size does not match slice dimensions")
    for r in range(rows):
        row_idx: int = sl_rows.start + r
        base: int = row_idx * dim + sl_cols.start
        for c in range(cols):
            p[base + c] = block[r * cols + c]


def _build_reject_report(
    *,
    frame_id: str,
    t_meas: AhrsTime,
    reason: str,
    z: list[float],
    z_hat: list[float],
    nu: list[float],
    r: AhrsMatrix,
    s_hat: AhrsMatrix,
    s: AhrsMatrix,
    maha_d2: float,
    gate_threshold: float,
) -> AhrsUpdateData:
    return AhrsUpdateData(
        sensor="gyro",
        frame_id=frame_id,
        t_meas=t_meas,
        accepted=False,
        reject_reason=reason,
        z=z,
        z_hat=z_hat,
        nu=nu,
        r=r,
        s_hat=s_hat,
        s=s,
        maha_d2=maha_d2,
        gate_threshold=gate_threshold,
    )


def update_gyro(
    layout: AhrsErrorStateLayout,
    state: AhrsNominalState,
    p: list[float],
    imu: ImuSample,
    config: AhrsConfig,
    *,
    frame_id: str,
    t_meas: AhrsTime,
) -> tuple[AhrsNominalState, list[float], AhrsUpdateData]:
    """
    Apply a gyro update to the AHRS nominal state and covariance
    """

    dim: int = layout.dim
    if len(p) != dim * dim:
        raise ValueError("p must have length dim * dim")

    z: list[float] = list(imu.angular_velocity_rps)
    r_raw: list[float] = list(imu.angular_velocity_cov)

    if len(z) != 3 or len(r_raw) != 9:
        return (
            state,
            list(p),
            _build_reject_report(
                frame_id=frame_id,
                t_meas=t_meas,
                reason="invalid_measurement",
                z=[],
                z_hat=[],
                nu=[],
                r=_empty_matrix(),
                s_hat=_empty_matrix(),
                s=_empty_matrix(),
                maha_d2=0.0,
                gate_threshold=config.gyro_gate_d2_threshold,
            ),
        )

    if not is_finite_seq(z) or not is_finite_seq(r_raw):
        return (
            state,
            list(p),
            _build_reject_report(
                frame_id=frame_id,
                t_meas=t_meas,
                reason="invalid_measurement",
                z=list(z),
                z_hat=[],
                nu=[],
                r=_matrix3(r_raw),
                s_hat=_empty_matrix(),
                s=_empty_matrix(),
                maha_d2=0.0,
                gate_threshold=config.gyro_gate_d2_threshold,
            ),
        )

    z_hat: list[float] = list(state.omega_wb_rps)
    nu: list[float] = [
        z[0] - z_hat[0],
        z[1] - z_hat[1],
        z[2] - z_hat[2],
    ]

    if not is_finite_seq(z_hat) or not is_finite_seq(nu):
        return (
            state,
            list(p),
            _build_reject_report(
                frame_id=frame_id,
                t_meas=t_meas,
                reason="invalid_prediction",
                z=list(z),
                z_hat=list(z_hat),
                nu=list(nu),
                r=_matrix3(r_raw),
                s_hat=_empty_matrix(),
                s=_empty_matrix(),
                maha_d2=0.0,
                gate_threshold=config.gyro_gate_d2_threshold,
            ),
        )

    sl_omega: slice = layout.sl_omega()
    s_hat_raw: list[float] = _extract_block(p, dim, sl_omega, sl_omega)
    s_hat: list[float] = symmetrize(s_hat_raw, 3)
    r: list[float] = symmetrize(r_raw, 3)
    s: list[float] = innovation_covariance(s_hat, r)
    s = symmetrize(s, 3)

    maha_d2: float
    try:
        maha_d2 = mahalanobis_d2_3(nu, s)
    except ValueError:
        return (
            state,
            list(p),
            _build_reject_report(
                frame_id=frame_id,
                t_meas=t_meas,
                reason="singular_innovation_covariance",
                z=list(z),
                z_hat=list(z_hat),
                nu=list(nu),
                r=_matrix3(r),
                s_hat=_matrix3(s_hat),
                s=_matrix3(s),
                maha_d2=0.0,
                gate_threshold=config.gyro_gate_d2_threshold,
            ),
        )

    gate_threshold: float = config.gyro_gate_d2_threshold
    if gate_threshold > 0.0 and maha_d2 > gate_threshold:
        return (
            state,
            list(p),
            _build_reject_report(
                frame_id=frame_id,
                t_meas=t_meas,
                reason="mahalanobis_gate",
                z=list(z),
                z_hat=list(z_hat),
                nu=list(nu),
                r=_matrix3(r),
                s_hat=_matrix3(s_hat),
                s=_matrix3(s),
                maha_d2=maha_d2,
                gate_threshold=gate_threshold,
            ),
        )

    s_inv: list[float]
    try:
        s_inv = mat3_inv(s)
    except ValueError:
        return (
            state,
            list(p),
            _build_reject_report(
                frame_id=frame_id,
                t_meas=t_meas,
                reason="singular_innovation_covariance",
                z=list(z),
                z_hat=list(z_hat),
                nu=list(nu),
                r=_matrix3(r),
                s_hat=_matrix3(s_hat),
                s=_matrix3(s),
                maha_d2=maha_d2,
                gate_threshold=gate_threshold,
            ),
        )

    k: list[float] = mat_mul(s_hat, 3, 3, s_inv, 3, 3)
    delta_omega: list[float] = mat_vec_mul(k, 3, 3, nu)

    updated_state: AhrsNominalState = _copy_state(state)
    updated_state.omega_wb_rps = [
        state.omega_wb_rps[0] + delta_omega[0],
        state.omega_wb_rps[1] + delta_omega[1],
        state.omega_wb_rps[2] + delta_omega[2],
    ]

    identity: list[float] = [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    i_minus_k: list[float] = [
        identity[0] - k[0],
        identity[1] - k[1],
        identity[2] - k[2],
        identity[3] - k[3],
        identity[4] - k[4],
        identity[5] - k[5],
        identity[6] - k[6],
        identity[7] - k[7],
        identity[8] - k[8],
    ]

    temp: list[float] = mat_mul(i_minus_k, 3, 3, s_hat, 3, 3)
    temp = mat_mul(temp, 3, 3, mat_transpose(i_minus_k, 3, 3), 3, 3)
    k_r: list[float] = mat_mul(k, 3, 3, r, 3, 3)
    k_r_kt: list[float] = mat_mul(k_r, 3, 3, mat_transpose(k, 3, 3), 3, 3)
    p_omega_updated: list[float] = [
        temp[0] + k_r_kt[0],
        temp[1] + k_r_kt[1],
        temp[2] + k_r_kt[2],
        temp[3] + k_r_kt[3],
        temp[4] + k_r_kt[4],
        temp[5] + k_r_kt[5],
        temp[6] + k_r_kt[6],
        temp[7] + k_r_kt[7],
        temp[8] + k_r_kt[8],
    ]
    p_omega_updated = symmetrize(p_omega_updated, 3)

    updated_p: list[float] = list(p)
    _set_block(updated_p, dim, sl_omega, sl_omega, p_omega_updated)
    updated_p = symmetrize(updated_p, dim)

    report: AhrsUpdateData = AhrsUpdateData(
        sensor="gyro",
        frame_id=frame_id,
        t_meas=t_meas,
        accepted=True,
        reject_reason=None,
        z=list(z),
        z_hat=list(z_hat),
        nu=list(nu),
        r=_matrix3(r),
        s_hat=_matrix3(s_hat),
        s=_matrix3(s),
        maha_d2=maha_d2,
        gate_threshold=gate_threshold,
    )

    return updated_state, updated_p, report
