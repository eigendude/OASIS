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
    z_hat: Predicted angular velocity from omega_wb_rps plus gyro bias
    nu: Innovation nu = z - z_hat
    R: Gyro measurement covariance in (rad/s)^2, row-major 3x3
    S_hat: Predicted innovation covariance H P H^T, row-major 3x3
    S: Innovation covariance S = S_hat + R, row-major 3x3

Gating:
    Mahalanobis distance d^2 = nu^T S^{-1} nu is compared against the
    configured gyro_gate_d2_threshold

Error-state mapping:
    The gyro update uses omega and gyro-bias error-state blocks. The update
    applies a full-state Joseph covariance update and injects the resulting
    correction into the nominal state.
"""

from __future__ import annotations

from oasis_control.localization.ahrs.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.ahrs_error_state import AhrsErrorStateLayout
from oasis_control.localization.ahrs.ahrs_inject import inject_error_state
from oasis_control.localization.ahrs.ahrs_linalg import is_finite_seq
from oasis_control.localization.ahrs.ahrs_linalg import symmetrize
from oasis_control.localization.ahrs.ahrs_state import AhrsNominalState
from oasis_control.localization.ahrs.ahrs_types import AhrsMatrix
from oasis_control.localization.ahrs.ahrs_types import AhrsTime
from oasis_control.localization.ahrs.ahrs_types import AhrsUpdateData
from oasis_control.localization.ahrs.ahrs_types import ImuSample
from oasis_control.localization.ahrs.ahrs_update_core import kalman_update


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

    z_hat: list[float] = [
        state.omega_wb_rps[0] + state.b_g_rps[0],
        state.omega_wb_rps[1] + state.b_g_rps[1],
        state.omega_wb_rps[2] + state.b_g_rps[2],
    ]
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
    sl_bg: slice = layout.sl_bg()
    h: list[float] = [0.0] * (3 * dim)
    for i in range(3):
        h[i * dim + sl_omega.start + i] = 1.0
        h[i * dim + sl_bg.start + i] = 1.0

    r: list[float] = symmetrize(r_raw, 3)

    updated_p: list[float]
    report: AhrsUpdateData
    accepted: bool
    delta_x: list[float]
    updated_p, report, accepted, delta_x = kalman_update(
        layout,
        p,
        z=z,
        z_hat=z_hat,
        nu=nu,
        h=h,
        r=r,
        gate_threshold=config.gyro_gate_d2_threshold,
        sensor="gyro",
        frame_id=frame_id,
        t_meas=t_meas,
    )

    if not accepted:
        return state, list(p), report

    updated_state: AhrsNominalState = inject_error_state(layout, state, delta_x)
    return updated_state, updated_p, report
