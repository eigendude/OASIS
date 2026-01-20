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
from typing import List
from typing import Sequence

from oasis_control.localization.ahrs.math_utils.quat import Quaternion
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.state_mapping import StateMapping


class ImuModel:
    """Gyro and accelerometer measurement models for the AHRS EKF.

    Purpose:
        Provide predicted IMU measurements and Jacobians for gyro and accel
        updates without treating IMU data as process inputs.

    Responsibility:
        Define the gyro and accelerometer measurement predictions and
        residuals used during EKF updates.

    Inputs/outputs:
        - Inputs: AhrsState, ImuPacket (z_ω, z_a), and extrinsics T_BI.
        - Outputs: predicted measurements, residuals, and Jacobians.

    Dependencies:
        - Uses Quaternion utilities and SE(3) extrinsics.
        - Consumed by UpdateStep for gyro/accel updates.

    Public API (to be implemented):
        - predict_gyro(state)
        - predict_accel(state)
        - residual_gyro(z_omega, z_hat)
        - residual_accel(z_accel, z_hat)
        - jacobian_gyro(state)
        - jacobian_accel(state)

    Data contract:
        - z_omega is a 3x1 gyro measurement in {I}.
        - z_accel is a 3x1 accel measurement in {I}.
        - R_omega and R_accel are 3x3 covariances.

    Frames and units:
        - Gyro residual ν is in {I} with units rad/s.
        - Accel residual ν is in {I} with units m/s^2.
        - Specific force convention: accel measures a - g, so at rest
          z_accel ≈ -g expressed in the sensor frame.

    Determinism and edge cases:
        - Residual sign convention is ν = z - z_hat.
        - Gyro update primarily touches omega_WB and b_g.
        - Accel update primarily constrains gravity g_W, attitude q_WB, and
          systematic accel parameters (b_a, A_a), with other states affected
          only through cross-covariances.
        - If A_a is near-singular, reject or condition the update.

    Equations:
        Gyro prediction in {I}:
            ω_hat_I = R_IB * ω_WB + b_g
            ν = z_ω - ω_hat_I

        Accel prediction in {I}:
            a_WB := 0 (deterministic mean of the process prior)
            f_B = R_WB * (a_WB - g_W)
            f_I = R_IB * f_B
            a_hat_I = A_a^{-1} * f_I + b_a
            ν = z_a - a_hat_I

        Notes:
            a_WB := 0 reflects the process model mean (smoothness prior with
            zero mean), supports gravity initialization, and avoids
            introducing a separate acceleration state. A future extension may
            introduce an explicit acceleration state or a deterministic
            finite-difference policy, but the current spec uses a_WB := 0.

    Numerical stability notes:
        - Use a stable inversion for A_a.
        - Keep quaternion normalized when computing R_WB.

    Suggested unit tests:
        - At rest, z_accel matches -g in sensor frame.
        - Residual sign convention matches ν = z - z_hat.
    """

    _MIN_DET: float = 1e-12
    _MAX_COND: float = 1e12

    @staticmethod
    def predict_gyro(state: AhrsState) -> List[float]:
        """Return the predicted gyro measurement in {I}."""
        R_BI: List[List[float]]
        _p_BI: List[float]
        R_BI, _p_BI = state.T_BI
        R_IB: List[List[float]] = _transpose3(R_BI)
        omega_I: List[float] = _matvec3(R_IB, state.omega_WB)
        return [
            omega_I[0] + state.b_g[0],
            omega_I[1] + state.b_g[1],
            omega_I[2] + state.b_g[2],
        ]

    @staticmethod
    def predict_accel(state: AhrsState) -> List[float]:
        """Return the predicted accel measurement in {I}."""
        A_inv: List[List[float]] = _invert_3x3(state.A_a)
        R_WB: List[List[float]] = Quaternion.to_matrix(state.q_WB)
        f_B: List[float] = _matvec3(R_WB, [-state.g_W[0], -state.g_W[1], -state.g_W[2]])
        R_BI: List[List[float]]
        _p_BI: List[float]
        R_BI, _p_BI = state.T_BI
        R_IB: List[List[float]] = _transpose3(R_BI)
        f_I: List[float] = _matvec3(R_IB, f_B)
        a_hat: List[float] = _matvec3(A_inv, f_I)
        return [
            a_hat[0] + state.b_a[0],
            a_hat[1] + state.b_a[1],
            a_hat[2] + state.b_a[2],
        ]

    @staticmethod
    def residual_gyro(z_omega: Sequence[float], z_hat: Sequence[float]) -> List[float]:
        """Return the gyro residual ν = z - z_hat."""
        _assert_vector_length("z_omega", z_omega, 3)
        _assert_vector_length("z_hat", z_hat, 3)
        return [z_omega[0] - z_hat[0], z_omega[1] - z_hat[1], z_omega[2] - z_hat[2]]

    @staticmethod
    def residual_accel(z_accel: Sequence[float], z_hat: Sequence[float]) -> List[float]:
        """Return the accel residual ν = z - z_hat."""
        _assert_vector_length("z_accel", z_accel, 3)
        _assert_vector_length("z_hat", z_hat, 3)
        return [z_accel[0] - z_hat[0], z_accel[1] - z_hat[1], z_accel[2] - z_hat[2]]

    @staticmethod
    def jacobian_gyro(state: AhrsState) -> List[List[float]]:
        """Return the gyro measurement Jacobian."""
        R_BI: List[List[float]]
        _p_BI: List[float]
        R_BI, _p_BI = state.T_BI
        R_IB: List[List[float]] = _transpose3(R_BI)
        H: List[List[float]] = _zeros_matrix(3, StateMapping.dimension())
        delta_omega: slice = StateMapping.slice_delta_omega()
        delta_b_g: slice = StateMapping.slice_delta_b_g()
        for i in range(3):
            for j in range(3):
                H[i][delta_omega.start + j] = R_IB[i][j]
        for i in range(3):
            H[i][delta_b_g.start + i] = 1.0
        return H

    @staticmethod
    def jacobian_accel(state: AhrsState) -> List[List[float]]:
        """Return the accelerometer measurement Jacobian."""
        A_inv: List[List[float]] = _invert_3x3(state.A_a)
        R_WB: List[List[float]] = Quaternion.to_matrix(state.q_WB)
        R_BI: List[List[float]]
        _p_BI: List[float]
        R_BI, _p_BI = state.T_BI
        R_IB: List[List[float]] = _transpose3(R_BI)
        g_skew: List[List[float]] = _skew(state.g_W)
        R_IB_R_WB: List[List[float]] = _matmul3(R_IB, R_WB)
        A_inv_R_IB_R_WB: List[List[float]] = _matmul3(A_inv, R_IB_R_WB)
        J_theta: List[List[float]] = _matmul3(A_inv_R_IB_R_WB, g_skew)

        f_B: List[float] = _matvec3(R_WB, [-state.g_W[0], -state.g_W[1], -state.g_W[2]])
        f_I: List[float] = _matvec3(R_IB, f_B)
        y_vec: List[float] = _matvec3(A_inv, f_I)

        H: List[List[float]] = _zeros_matrix(3, StateMapping.dimension())
        delta_theta: slice = StateMapping.slice_delta_theta()
        for i in range(3):
            for j in range(3):
                H[i][delta_theta.start + j] = J_theta[i][j]

        delta_A_a: slice = StateMapping.slice_delta_A_a()
        for i in range(3):
            for j in range(3):
                col: int = delta_A_a.start + i * 3 + j
                scale: float = -y_vec[j]
                for row in range(3):
                    H[row][col] = scale * A_inv[row][i]

        delta_b_a: slice = StateMapping.slice_delta_b_a()
        for i in range(3):
            H[i][delta_b_a.start + i] = 1.0

        delta_g_W: slice = StateMapping.slice_delta_g_W()
        J_g: List[List[float]] = _scale_mat3(A_inv_R_IB_R_WB, -1.0)
        for i in range(3):
            for j in range(3):
                H[i][delta_g_W.start + j] = J_g[i][j]
        return H


def _assert_vector_length(name: str, vec: Sequence[float], length: int) -> None:
    if len(vec) != length:
        raise ValueError(f"{name} must have length {length}")


def _transpose3(A: Sequence[Sequence[float]]) -> List[List[float]]:
    return [
        [A[0][0], A[1][0], A[2][0]],
        [A[0][1], A[1][1], A[2][1]],
        [A[0][2], A[1][2], A[2][2]],
    ]


def _matvec3(A: Sequence[Sequence[float]], v: Sequence[float]) -> List[float]:
    return [
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    ]


def _matmul3(
    A: Sequence[Sequence[float]],
    B: Sequence[Sequence[float]],
) -> List[List[float]]:
    result: List[List[float]] = [[0.0 for _ in range(3)] for _ in range(3)]
    for i in range(3):
        for j in range(3):
            acc: float = 0.0
            for k in range(3):
                acc += A[i][k] * B[k][j]
            result[i][j] = acc
    return result


def _skew(v: Sequence[float]) -> List[List[float]]:
    if len(v) != 3:
        raise ValueError("Vector must have length 3")
    return [
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ]


def _identity3() -> List[List[float]]:
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _scale_mat3(A: Sequence[Sequence[float]], scale: float) -> List[List[float]]:
    return [
        [A[0][0] * scale, A[0][1] * scale, A[0][2] * scale],
        [A[1][0] * scale, A[1][1] * scale, A[1][2] * scale],
        [A[2][0] * scale, A[2][1] * scale, A[2][2] * scale],
    ]


def _invert_3x3(A: Sequence[Sequence[float]]) -> List[List[float]]:
    if len(A) != 3 or any(len(row) != 3 for row in A):
        raise ValueError("A_a must be 3x3")
    a00: float = A[0][0]
    a01: float = A[0][1]
    a02: float = A[0][2]
    a10: float = A[1][0]
    a11: float = A[1][1]
    a12: float = A[1][2]
    a20: float = A[2][0]
    a21: float = A[2][1]
    a22: float = A[2][2]

    c00: float = a11 * a22 - a12 * a21
    c01: float = -(a10 * a22 - a12 * a20)
    c02: float = a10 * a21 - a11 * a20
    c10: float = -(a01 * a22 - a02 * a21)
    c11: float = a00 * a22 - a02 * a20
    c12: float = -(a00 * a21 - a01 * a20)
    c20: float = a01 * a12 - a02 * a11
    c21: float = -(a00 * a12 - a02 * a10)
    c22: float = a00 * a11 - a01 * a10

    det: float = a00 * c00 + a01 * c01 + a02 * c02
    if abs(det) <= ImuModel._MIN_DET:
        raise ValueError("A_a is singular or ill-conditioned")

    inv_det: float = 1.0 / det
    inv: List[List[float]] = [
        [c00 * inv_det, c10 * inv_det, c20 * inv_det],
        [c01 * inv_det, c11 * inv_det, c21 * inv_det],
        [c02 * inv_det, c12 * inv_det, c22 * inv_det],
    ]
    cond: float = _matrix_norm(A) * _matrix_norm(inv)
    if not math.isfinite(cond) or cond > ImuModel._MAX_COND:
        raise ValueError("A_a is singular or ill-conditioned")
    return inv


def _matrix_norm(A: Sequence[Sequence[float]]) -> float:
    max_sum: float = 0.0
    for row in A:
        row_sum: float = 0.0
        for value in row:
            row_sum += abs(value)
        if row_sum > max_sum:
            max_sum = row_sum
    return max_sum


def _zeros_matrix(rows: int, cols: int) -> List[List[float]]:
    return [[0.0 for _ in range(cols)] for _ in range(rows)]
