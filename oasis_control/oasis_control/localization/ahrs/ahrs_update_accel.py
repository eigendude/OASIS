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
Accelerometer measurement update for the AHRS core

Measurement model:
    z: Acceleration in m/s^2, IMU frame assumed aligned with body
    z_hat: Predicted body-frame gravity, R(q_wb) * g_w
    nu: Innovation nu = z - z_hat
    R: Accel measurement covariance in (m/s^2)^2, row-major 3x3
    S_hat: Predicted innovation covariance H P H^T, row-major 3x3
    S: Innovation covariance S = S_hat + R, row-major 3x3

Linearization:
    q_wb represents the world-to-body rotation. With right-multiplied
    error-state perturbations q_new = q_nominal ⊗ Exp(delta_theta), the
    first-order perturbation of the predicted body vector is

        d z_hat / d delta_theta ≈ -[z_hat]×

    where [v]× is the skew-symmetric matrix for v. The update also supports
    the gravity error-state block with d z_hat / d delta_g = R(q_wb).

Gating:
    Mahalanobis distance d^2 = nu^T S^{-1} nu is compared against the
    configured accel_gate_d2_threshold
"""

from __future__ import annotations

import math

from oasis_control.localization.ahrs.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.ahrs_error_state import AhrsErrorStateLayout
from oasis_control.localization.ahrs.ahrs_inject import inject_error_state
from oasis_control.localization.ahrs.ahrs_linalg import is_finite_seq
from oasis_control.localization.ahrs.ahrs_linalg import symmetrize
from oasis_control.localization.ahrs.ahrs_quat import quat_rotate_wxyz
from oasis_control.localization.ahrs.ahrs_state import AhrsNominalState
from oasis_control.localization.ahrs.ahrs_types import AhrsMatrix
from oasis_control.localization.ahrs.ahrs_types import AhrsTime
from oasis_control.localization.ahrs.ahrs_types import AhrsUpdateData
from oasis_control.localization.ahrs.ahrs_types import ImuSample
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
        sensor="accel",
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


def update_accel(
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
    Apply an accelerometer update to the AHRS nominal state and covariance
    """

    dim: int = layout.dim
    if len(p) != dim * dim:
        raise ValueError("p must have length dim * dim")

    z: list[float] = list(imu.linear_acceleration_mps2)
    r_raw: list[float] = list(imu.linear_acceleration_cov)

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
                gate_threshold=config.accel_gate_d2_threshold,
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
                gate_threshold=config.accel_gate_d2_threshold,
            ),
        )

    g_norm: float = _norm3(state.g_w_mps2)
    r_cov: list[float] = []
    if g_norm < _EPS_NORM:
        r_cov = symmetrize(r_raw, 3)
        return (
            state,
            list(p),
            _build_reject_report(
                frame_id=frame_id,
                t_meas=t_meas,
                reason="uninitialized_gravity",
                z=list(z),
                z_hat=[],
                nu=[],
                r=_matrix3(r_cov),
                s_hat=_empty_matrix(),
                s=_empty_matrix(),
                maha_d2=0.0,
                gate_threshold=config.accel_gate_d2_threshold,
            ),
        )

    z_hat: list[float] = quat_rotate_wxyz(state.q_wb_wxyz, state.g_w_mps2)
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
                r=_matrix3(r_raw),
                s_hat=_empty_matrix(),
                s=_empty_matrix(),
                maha_d2=0.0,
                gate_threshold=config.accel_gate_d2_threshold,
            ),
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
                r=_matrix3(r_raw),
                s_hat=_empty_matrix(),
                s=_empty_matrix(),
                maha_d2=0.0,
                gate_threshold=config.accel_gate_d2_threshold,
            ),
        )

    z_used: list[float] = list(z)
    z_hat_used: list[float] = list(z_hat)
    g_scale: float = 1.0

    if config.accel_use_direction_only:
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
                    r=_matrix3(r_raw),
                    s_hat=_empty_matrix(),
                    s=_empty_matrix(),
                    maha_d2=0.0,
                    gate_threshold=config.accel_gate_d2_threshold,
                ),
            )

        z_unit: list[float]
        z_hat_unit: list[float]
        z_unit, _ = _unit3(z, _EPS_NORM)
        z_hat_unit, _ = _unit3(z_hat, _EPS_NORM)
        z_used = z_unit
        z_hat_used = z_hat_unit

        # 1 / (m/s^2)^2 scaling to match unit-vector covariance
        r_scale: float = 1.0 / (z_norm * z_norm)

        r_scaled: list[float] = [value * r_scale for value in r_raw]
        r_cov = symmetrize(r_scaled, 3)

        # 1 / (m/s^2) scaling to match unit-vector normalization
        g_scale = 1.0 / g_norm
    else:
        r_cov = symmetrize(r_raw, 3)

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
                gate_threshold=config.accel_gate_d2_threshold,
            ),
        )

    sl_theta: slice = layout.sl_theta()
    sl_g: slice = layout.sl_g()
    h: list[float] = [0.0] * (3 * dim)

    skew_z_hat: list[float] = _skew(z_hat_used)
    for row in range(3):
        for col in range(3):
            h[row * dim + sl_theta.start + col] = -skew_z_hat[row * 3 + col]

    r_wb: list[float] = _rotation_matrix_wb(state.q_wb_wxyz)
    for row in range(3):
        for col in range(3):
            h[row * dim + sl_g.start + col] = r_wb[row * 3 + col] * g_scale

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
        gate_threshold=config.accel_gate_d2_threshold,
        sensor="accel",
        frame_id=frame_id,
        t_meas=t_meas,
    )

    if not accepted:
        return state, list(p), report

    updated_state: AhrsNominalState = inject_error_state(layout, state, delta_x)
    return updated_state, updated_p, report
