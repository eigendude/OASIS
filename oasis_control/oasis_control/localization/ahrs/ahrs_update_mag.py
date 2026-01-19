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
Magnetometer measurement update for the AHRS core

Measurement model:
    z: Magnetic field in tesla, magnetometer aligned with body for now
    z_hat: Predicted body-frame field, R(q_wb) * m_w
    nu: Innovation nu = z - z_hat
    R: Mag measurement covariance in tesla^2, row-major 3x3
    S_hat: Predicted innovation covariance H P H^T, row-major 3x3
    S: Innovation covariance S = S_hat + R, row-major 3x3

Linearization:
    q_wb represents the world-to-body rotation. With right-multiplied
    error-state perturbations q_new = q_nominal ⊗ Exp(delta_theta), the
    first-order perturbation of the predicted body vector is

        d z_hat / d delta_theta ≈ -[z_hat]×

    The update supports the magnetic field error-state block with

        d z_hat / d delta_m = R(q_wb)

Direction-only mode:
    When mag_use_direction_only is enabled, the update uses unit vectors for
    z and z_hat. The covariance is scaled by 1 / |z|^2 and the field Jacobian
    is scaled by 1 / |z_hat| to approximate the unit-vector normalization

Gating:
    Mahalanobis distance d^2 = nu^T S^{-1} nu is compared against the
    configured mag_gate_d2_threshold
"""

from __future__ import annotations

import math
from typing import Optional

from oasis_control.localization.ahrs.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.ahrs_error_state import AhrsErrorStateLayout
from oasis_control.localization.ahrs.ahrs_inject import inject_error_state
from oasis_control.localization.ahrs.ahrs_linalg import is_finite_seq
from oasis_control.localization.ahrs.ahrs_linalg import mat3_det
from oasis_control.localization.ahrs.ahrs_linalg import symmetrize
from oasis_control.localization.ahrs.ahrs_quat import quat_rotate_wxyz
from oasis_control.localization.ahrs.ahrs_state import AhrsNominalState
from oasis_control.localization.ahrs.ahrs_types import AhrsMatrix
from oasis_control.localization.ahrs.ahrs_types import AhrsTime
from oasis_control.localization.ahrs.ahrs_types import AhrsUpdateData
from oasis_control.localization.ahrs.ahrs_types import MagSample
from oasis_control.localization.ahrs.ahrs_update_core import kalman_update


Vector3 = list[float]


# Dimensionless threshold for treating vectors as degenerate
_EPS_NORM: float = 1.0e-6


def _empty_matrix() -> AhrsMatrix:
    return AhrsMatrix(rows=0, cols=0, data=[])


def _matrix3(data: list[float]) -> AhrsMatrix:
    return AhrsMatrix(rows=3, cols=3, data=list(data))


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
        sensor="mag",
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


def _skew(v: Vector3) -> list[float]:
    vx: float = v[0]
    vy: float = v[1]
    vz: float = v[2]
    return [
        0.0,
        -vz,
        vy,
        vz,
        0.0,
        -vx,
        -vy,
        vx,
        0.0,
    ]


def _norm3(v: list[float]) -> float:
    vx: float = v[0]
    vy: float = v[1]
    vz: float = v[2]
    return math.sqrt(vx * vx + vy * vy + vz * vz)


def _scale3(v: list[float], s: float) -> list[float]:
    return [value * s for value in v]


def _unit3(v: list[float], eps: float) -> tuple[list[float], bool]:
    norm: float = _norm3(v)
    if norm < eps:
        return [0.0, 0.0, 0.0], False
    return _scale3(v, 1.0 / norm), True


def _rotation_matrix_wb(q_wb_wxyz: list[float]) -> list[float]:
    col0: Vector3 = quat_rotate_wxyz(q_wb_wxyz, [1.0, 0.0, 0.0])
    col1: Vector3 = quat_rotate_wxyz(q_wb_wxyz, [0.0, 1.0, 0.0])
    col2: Vector3 = quat_rotate_wxyz(q_wb_wxyz, [0.0, 0.0, 1.0])
    return [
        col0[0],
        col1[0],
        col2[0],
        col0[1],
        col1[1],
        col2[1],
        col0[2],
        col1[2],
        col2[2],
    ]


def _outer_product_3(v: list[float]) -> list[float]:
    vx: float = v[0]
    vy: float = v[1]
    vz: float = v[2]
    return [
        vx * vx,
        vx * vy,
        vx * vz,
        vy * vx,
        vy * vy,
        vy * vz,
        vz * vx,
        vz * vy,
        vz * vz,
    ]


def _is_valid_covariance(matrix: list[float]) -> bool:
    if len(matrix) != 9:
        return False
    if not is_finite_seq(matrix):
        return False
    symmetric: list[float] = symmetrize(matrix, 3)
    for i in range(9):
        if abs(symmetric[i] - matrix[i]) > 1.0e-9:
            return False
    a00: float = symmetric[0]
    a01: float = symmetric[1]
    a10: float = symmetric[3]
    a11: float = symmetric[4]

    if a00 <= 0.0:
        return False
    det2: float = a00 * a11 - a01 * a10
    if det2 <= 0.0:
        return False
    det: float = mat3_det(symmetric)
    if det <= 0.0:
        return False
    return True


def _select_mag_covariance(
    r_state: Optional[list[float]],
    r_raw: list[float],
    r_default: list[float],
) -> list[float]:
    if r_state is not None and _is_valid_covariance(r_state):
        return symmetrize(r_state, 3)
    if _is_valid_covariance(r_raw):
        return symmetrize(r_raw, 3)
    if _is_valid_covariance(r_default):
        return symmetrize(r_default, 3)
    if len(r_default) == 9 and is_finite_seq(r_default):
        return symmetrize(r_default, 3)
    if len(r_raw) == 9 and is_finite_seq(r_raw):
        return symmetrize(r_raw, 3)
    return [0.0] * 9


def _clamp_matrix(
    matrix: list[float],
    min_matrix: list[float],
    max_matrix: list[float],
) -> list[float]:
    out: list[float] = []
    for i in range(9):
        low: float = min_matrix[i]
        high: float = max_matrix[i]
        if low > high:
            low, high = high, low
        out.append(min(max(matrix[i], low), high))
    return out


def _update_mag_covariance(
    r_prev: list[float],
    *,
    nu: list[float],
    s_hat: list[float],
    config: AhrsConfig,
) -> list[float]:
    alpha: float = config.mag_alpha
    if alpha <= 0.0:
        return list(r_prev)

    if len(config.mag_r_min) != 9 or len(config.mag_r_max) != 9:
        return list(r_prev)

    innovation_cov: list[float] = _outer_product_3(nu)
    blended: list[float] = []
    for i in range(9):
        blended.append(
            (1.0 - alpha) * r_prev[i] + alpha * (innovation_cov[i] - s_hat[i])
        )

    blended_sym: list[float] = symmetrize(blended, 3)
    r_min: list[float] = symmetrize(config.mag_r_min, 3)
    r_max: list[float] = symmetrize(config.mag_r_max, 3)
    clamped: list[float] = _clamp_matrix(blended_sym, r_min, r_max)
    return symmetrize(clamped, 3)


def update_mag(
    layout: AhrsErrorStateLayout,
    state: AhrsNominalState,
    p: list[float],
    mag: MagSample,
    config: AhrsConfig,
    *,
    frame_id: str,
    t_meas: AhrsTime,
    r_state: Optional[list[float]] = None,
) -> tuple[AhrsNominalState, list[float], AhrsUpdateData, list[float]]:
    """
    Apply a magnetometer update to the AHRS nominal state and covariance
    """

    dim: int = layout.dim
    if len(p) != dim * dim:
        raise ValueError("p must have length dim * dim")

    z: list[float] = list(mag.magnetic_field_t)
    r_raw: list[float] = list(mag.magnetic_field_cov)
    r_next: list[float]

    if len(z) != 3 or len(r_raw) != 9:
        r_next = _select_mag_covariance(r_state, r_raw, config.mag_r0)
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
                r=_matrix3(r_next),
                s_hat=_empty_matrix(),
                s=_empty_matrix(),
                maha_d2=0.0,
                gate_threshold=config.mag_gate_d2_threshold,
            ),
            r_next,
        )

    if not is_finite_seq(z) or not is_finite_seq(r_raw):
        r_next = _select_mag_covariance(r_state, r_raw, config.mag_r0)
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
                r=_matrix3(r_next),
                s_hat=_empty_matrix(),
                s=_empty_matrix(),
                maha_d2=0.0,
                gate_threshold=config.mag_gate_d2_threshold,
            ),
            r_next,
        )

    m_norm: float = _norm3(state.m_w_t)
    r_cov: list[float] = _select_mag_covariance(r_state, r_raw, config.mag_r0)
    if m_norm < _EPS_NORM:
        r_cov = symmetrize(r_cov, 3)
        return (
            state,
            list(p),
            _build_reject_report(
                frame_id=frame_id,
                t_meas=t_meas,
                reason="uninitialized_magnetic_field",
                z=list(z),
                z_hat=[],
                nu=[],
                r=_matrix3(r_cov),
                s_hat=_empty_matrix(),
                s=_empty_matrix(),
                maha_d2=0.0,
                gate_threshold=config.mag_gate_d2_threshold,
            ),
            r_cov,
        )

    z_hat: list[float] = quat_rotate_wxyz(state.q_wb_wxyz, state.m_w_t)
    if not is_finite_seq(z_hat):
        return (
            state,
            list(p),
            _build_reject_report(
                frame_id=frame_id,
                t_meas=t_meas,
                reason="invalid_prediction",
                z=list(z),
                z_hat=list(z_hat),
                nu=[],
                r=_matrix3(r_cov),
                s_hat=_empty_matrix(),
                s=_empty_matrix(),
                maha_d2=0.0,
                gate_threshold=config.mag_gate_d2_threshold,
            ),
            r_cov,
        )

    z_hat_norm: float = _norm3(z_hat)
    if z_hat_norm < _EPS_NORM:
        return (
            state,
            list(p),
            _build_reject_report(
                frame_id=frame_id,
                t_meas=t_meas,
                reason="degenerate_prediction",
                z=list(z),
                z_hat=list(z_hat),
                nu=[],
                r=_matrix3(r_cov),
                s_hat=_empty_matrix(),
                s=_empty_matrix(),
                maha_d2=0.0,
                gate_threshold=config.mag_gate_d2_threshold,
            ),
            r_cov,
        )

    z_used: list[float] = list(z)
    z_hat_used: list[float] = list(z_hat)
    m_scale: float = 1.0

    if config.mag_use_direction_only:
        z_norm: float = _norm3(z)
        if z_norm < _EPS_NORM:
            return (
                state,
                list(p),
                _build_reject_report(
                    frame_id=frame_id,
                    t_meas=t_meas,
                    reason="invalid_measurement",
                    z=list(z),
                    z_hat=list(z_hat),
                    nu=[],
                    r=_matrix3(r_cov),
                    s_hat=_empty_matrix(),
                    s=_empty_matrix(),
                    maha_d2=0.0,
                    gate_threshold=config.mag_gate_d2_threshold,
                ),
                r_cov,
            )

        z_unit: list[float]
        z_hat_unit: list[float]
        z_unit, _ = _unit3(z, _EPS_NORM)
        z_hat_unit, _ = _unit3(z_hat, _EPS_NORM)
        z_used = z_unit
        z_hat_used = z_hat_unit

        # 1 / tesla^2 scaling to match unit-vector covariance
        r_scale: float = 1.0 / (z_norm * z_norm)
        r_scaled: list[float] = [value * r_scale for value in r_cov]
        r_cov = symmetrize(r_scaled, 3)

        # 1 / tesla scaling to match unit-vector normalization
        m_scale = 1.0 / z_hat_norm
    else:
        r_cov = symmetrize(r_cov, 3)

    nu: list[float] = [
        z_used[0] - z_hat_used[0],
        z_used[1] - z_hat_used[1],
        z_used[2] - z_hat_used[2],
    ]

    if not is_finite_seq(z_hat_used) or not is_finite_seq(nu):
        return (
            state,
            list(p),
            _build_reject_report(
                frame_id=frame_id,
                t_meas=t_meas,
                reason="invalid_prediction",
                z=list(z_used),
                z_hat=list(z_hat_used),
                nu=list(nu),
                r=_matrix3(r_cov),
                s_hat=_empty_matrix(),
                s=_empty_matrix(),
                maha_d2=0.0,
                gate_threshold=config.mag_gate_d2_threshold,
            ),
            r_cov,
        )

    sl_theta: slice = layout.sl_theta()
    sl_m: slice = layout.sl_m()
    h: list[float] = [0.0] * (3 * dim)

    skew_z_hat: list[float] = _skew(z_hat_used)
    for row in range(3):
        for col in range(3):
            h[row * dim + sl_theta.start + col] = -skew_z_hat[row * 3 + col]

    r_wb: list[float] = _rotation_matrix_wb(state.q_wb_wxyz)
    for row in range(3):
        for col in range(3):
            h[row * dim + sl_m.start + col] = r_wb[row * 3 + col] * m_scale

    updated_p: list[float]
    report: AhrsUpdateData
    accepted: bool
    delta_x: list[float]
    updated_p, report, accepted, delta_x = kalman_update(
        layout,
        p,
        z=z_used,
        z_hat=z_hat_used,
        nu=nu,
        h=h,
        r=r_cov,
        gate_threshold=config.mag_gate_d2_threshold,
        sensor="mag",
        frame_id=frame_id,
        t_meas=t_meas,
    )

    if not accepted:
        return state, list(p), report, r_cov

    updated_state: AhrsNominalState = inject_error_state(layout, state, delta_x)

    r_next = r_cov
    if report.s_hat.rows == 3 and report.s_hat.cols == 3:
        s_hat: list[float] = list(report.s_hat.data)
        r_next = _update_mag_covariance(r_cov, nu=nu, s_hat=s_hat, config=config)

    return updated_state, updated_p, report, r_next
