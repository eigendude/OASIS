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
Core measurement update utilities for the AHRS error-state EKF

This module implements the common EKF measurement update for the error-state
filter using row-major list matrices. The innovation is defined as
``nu = z - z_hat``. The measurement Jacobian ``H`` maps the stacked
error-state to the measurement space, so the predicted innovation covariance
is ``S_hat = H P H^T``. The total innovation covariance is
``S = S_hat + R`` where ``R`` is the measurement noise covariance. The
Kalman gain is ``K = P H^T S^{-1}`` and the correction is
``delta_x = K nu``. Covariance is updated with the Joseph form
``P_new = (I - K H) P (I - K H)^T + K R K^T`` to preserve symmetry.
"""

from __future__ import annotations

from oasis_control.localization.ahrs.ahrs_error_state import AhrsErrorStateLayout
from oasis_control.localization.ahrs.ahrs_linalg import is_finite_seq
from oasis_control.localization.ahrs.ahrs_linalg import mat3_inv
from oasis_control.localization.ahrs.ahrs_linalg import mat_mul
from oasis_control.localization.ahrs.ahrs_linalg import mat_transpose
from oasis_control.localization.ahrs.ahrs_linalg import mat_vec_mul
from oasis_control.localization.ahrs.ahrs_linalg import symmetrize
from oasis_control.localization.ahrs.ahrs_types import AhrsMatrix
from oasis_control.localization.ahrs.ahrs_types import AhrsTime
from oasis_control.localization.ahrs.ahrs_types import AhrsUpdateData


Matrix = list[float]
Vector = list[float]


def _empty_matrix() -> AhrsMatrix:
    return AhrsMatrix(rows=0, cols=0, data=[])


def _matrix(rows: int, cols: int, data: list[float]) -> AhrsMatrix:
    return AhrsMatrix(rows=rows, cols=cols, data=list(data))


def _build_reject_report(
    *,
    sensor: str,
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
        sensor=sensor,
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


def _identity_matrix(dim: int) -> Matrix:
    if dim <= 0:
        raise ValueError("dim must be positive")
    out: Matrix = [0.0] * (dim * dim)
    for i in range(dim):
        out[i * dim + i] = 1.0
    return out


def _mahalanobis_d2_3(nu: Vector, s_inv: Matrix) -> float:
    nu_x: float = nu[0]
    nu_y: float = nu[1]
    nu_z: float = nu[2]
    s00: float = s_inv[0]
    s01: float = s_inv[1]
    s02: float = s_inv[2]
    s10: float = s_inv[3]
    s11: float = s_inv[4]
    s12: float = s_inv[5]
    s20: float = s_inv[6]
    s21: float = s_inv[7]
    s22: float = s_inv[8]
    sol0: float = s00 * nu_x + s01 * nu_y + s02 * nu_z
    sol1: float = s10 * nu_x + s11 * nu_y + s12 * nu_z
    sol2: float = s20 * nu_x + s21 * nu_y + s22 * nu_z
    return nu_x * sol0 + nu_y * sol1 + nu_z * sol2


def kalman_update(
    layout: AhrsErrorStateLayout,
    p: list[float],
    *,
    z: list[float],
    z_hat: list[float],
    nu: list[float],
    h: list[float],
    r: list[float],
    gate_threshold: float,
    sensor: str,
    frame_id: str,
    t_meas: AhrsTime,
) -> tuple[list[float], AhrsUpdateData, bool, list[float]]:
    """
    Apply a measurement update to the covariance using the Joseph form

    The layout argument is only used for layout.dim
    """

    dim: int = layout.dim
    if len(p) != dim * dim:
        raise ValueError("p must have length dim * dim")

    m: int = len(nu)
    if m <= 0:
        raise ValueError("nu must be non-empty")

    if len(z) != m or len(z_hat) != m:
        return (
            list(p),
            _build_reject_report(
                sensor=sensor,
                frame_id=frame_id,
                t_meas=t_meas,
                reason="invalid_measurement",
                z=list(z),
                z_hat=list(z_hat),
                nu=list(nu),
                r=_matrix(m, m, r) if len(r) == m * m else _empty_matrix(),
                s_hat=_empty_matrix(),
                s=_empty_matrix(),
                maha_d2=0.0,
                gate_threshold=gate_threshold,
            ),
            False,
            [0.0] * dim,
        )

    if len(h) != m * dim or len(r) != m * m:
        raise ValueError("h or r size does not match measurement dimensions")

    if not is_finite_seq(h) or not is_finite_seq(r) or not is_finite_seq(z):
        return (
            list(p),
            _build_reject_report(
                sensor=sensor,
                frame_id=frame_id,
                t_meas=t_meas,
                reason="invalid_measurement",
                z=list(z),
                z_hat=list(z_hat),
                nu=list(nu),
                r=_matrix(m, m, r) if len(r) == m * m else _empty_matrix(),
                s_hat=_empty_matrix(),
                s=_empty_matrix(),
                maha_d2=0.0,
                gate_threshold=gate_threshold,
            ),
            False,
            [0.0] * dim,
        )

    if not is_finite_seq(z_hat) or not is_finite_seq(nu):
        return (
            list(p),
            _build_reject_report(
                sensor=sensor,
                frame_id=frame_id,
                t_meas=t_meas,
                reason="invalid_prediction",
                z=list(z),
                z_hat=list(z_hat),
                nu=list(nu),
                r=_matrix(m, m, r),
                s_hat=_empty_matrix(),
                s=_empty_matrix(),
                maha_d2=0.0,
                gate_threshold=gate_threshold,
            ),
            False,
            [0.0] * dim,
        )

    if not is_finite_seq(p):
        return (
            list(p),
            _build_reject_report(
                sensor=sensor,
                frame_id=frame_id,
                t_meas=t_meas,
                reason="invalid_prediction",
                z=list(z),
                z_hat=list(z_hat),
                nu=list(nu),
                r=_matrix(m, m, r),
                s_hat=_empty_matrix(),
                s=_empty_matrix(),
                maha_d2=0.0,
                gate_threshold=gate_threshold,
            ),
            False,
            [0.0] * dim,
        )

    if m != 3:
        raise NotImplementedError("only 3D measurements are supported")

    h_t: Matrix = mat_transpose(h, m, dim)
    hp: Matrix = mat_mul(h, m, dim, p, dim, dim)
    s_hat_raw: Matrix = mat_mul(hp, m, dim, h_t, dim, m)
    s_hat: Matrix = symmetrize(s_hat_raw, m)
    r_sym: Matrix = symmetrize(r, m)
    s: Matrix = symmetrize(
        [s_hat[i] + r_sym[i] for i in range(m * m)],
        m,
    )

    s_inv: Matrix
    try:
        s_inv = mat3_inv(s)
    except ValueError:
        return (
            list(p),
            _build_reject_report(
                sensor=sensor,
                frame_id=frame_id,
                t_meas=t_meas,
                reason="singular_innovation_covariance",
                z=list(z),
                z_hat=list(z_hat),
                nu=list(nu),
                r=_matrix(m, m, r_sym),
                s_hat=_matrix(m, m, s_hat),
                s=_matrix(m, m, s),
                maha_d2=0.0,
                gate_threshold=gate_threshold,
            ),
            False,
            [0.0] * dim,
        )

    maha_d2: float = _mahalanobis_d2_3(nu, s_inv)
    if gate_threshold > 0.0 and maha_d2 > gate_threshold:
        return (
            list(p),
            _build_reject_report(
                sensor=sensor,
                frame_id=frame_id,
                t_meas=t_meas,
                reason="mahalanobis_gate",
                z=list(z),
                z_hat=list(z_hat),
                nu=list(nu),
                r=_matrix(m, m, r_sym),
                s_hat=_matrix(m, m, s_hat),
                s=_matrix(m, m, s),
                maha_d2=maha_d2,
                gate_threshold=gate_threshold,
            ),
            False,
            [0.0] * dim,
        )

    p_h_t: Matrix = mat_mul(p, dim, dim, h_t, dim, m)
    k: Matrix = mat_mul(p_h_t, dim, m, s_inv, m, m)
    delta_x: Vector = mat_vec_mul(k, dim, m, nu)

    kh: Matrix = mat_mul(k, dim, m, h, m, dim)
    identity: Matrix = _identity_matrix(dim)
    i_minus_kh: Matrix = [identity[i] - kh[i] for i in range(dim * dim)]

    temp: Matrix = mat_mul(i_minus_kh, dim, dim, p, dim, dim)
    temp = mat_mul(temp, dim, dim, mat_transpose(i_minus_kh, dim, dim), dim, dim)

    k_r: Matrix = mat_mul(k, dim, m, r_sym, m, m)
    k_r_kt: Matrix = mat_mul(k_r, dim, m, mat_transpose(k, dim, m), m, dim)

    p_new: Matrix = [temp[i] + k_r_kt[i] for i in range(dim * dim)]
    p_new = symmetrize(p_new, dim)

    report: AhrsUpdateData = AhrsUpdateData(
        sensor=sensor,
        frame_id=frame_id,
        t_meas=t_meas,
        accepted=True,
        reject_reason=None,
        z=list(z),
        z_hat=list(z_hat),
        nu=list(nu),
        r=_matrix(m, m, r_sym),
        s_hat=_matrix(m, m, s_hat),
        s=_matrix(m, m, s),
        maha_d2=maha_d2,
        gate_threshold=gate_threshold,
    )

    return p_new, report, True, delta_x
