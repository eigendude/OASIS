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

from typing import List
from typing import Sequence

from oasis_control.localization.ahrs.math_utils.quat import Quaternion
from oasis_control.localization.ahrs.math_utils.small_linalg3 import matmul3
from oasis_control.localization.ahrs.math_utils.small_linalg3 import matvec3
from oasis_control.localization.ahrs.math_utils.small_linalg3 import scale_mat
from oasis_control.localization.ahrs.math_utils.small_linalg3 import skew
from oasis_control.localization.ahrs.math_utils.small_linalg3 import transpose3
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

    @staticmethod
    def predict_gyro(state: AhrsState) -> List[float]:
        """Return predicted gyro measurement in {I}."""
        R_BI: List[List[float]] = state.T_BI[0]
        R_IB: List[List[float]] = transpose3(R_BI)
        omega_I: List[float] = matvec3(R_IB, state.omega_WB)
        return _add_vec(omega_I, state.b_g)

    @staticmethod
    def predict_accel(state: AhrsState) -> List[float]:
        """Return predicted accel measurement in {I}."""
        R_WB: List[List[float]] = Quaternion.to_matrix(state.q_WB)
        f_B: List[float] = matvec3(R_WB, _scale_vec(state.g_W, -1.0))
        R_BI: List[List[float]] = state.T_BI[0]
        R_IB: List[List[float]] = transpose3(R_BI)
        f_I: List[float] = matvec3(R_IB, f_B)
        A_inv: List[List[float]] = _invert_3x3(state.A_a)
        a_pred: List[float] = matvec3(A_inv, f_I)
        return _add_vec(a_pred, state.b_a)

    @staticmethod
    def residual_gyro(z_omega: Sequence[float], z_hat: Sequence[float]) -> List[float]:
        """Return ν = z - z_hat for gyro update."""
        _assert_vector_length("z_omega", z_omega, 3)
        _assert_vector_length("z_hat", z_hat, 3)
        return [
            z_omega[0] - z_hat[0],
            z_omega[1] - z_hat[1],
            z_omega[2] - z_hat[2],
        ]

    @staticmethod
    def residual_accel(z_accel: Sequence[float], z_hat: Sequence[float]) -> List[float]:
        """Return ν = z - z_hat for accel update."""
        _assert_vector_length("z_accel", z_accel, 3)
        _assert_vector_length("z_hat", z_hat, 3)
        return [
            z_accel[0] - z_hat[0],
            z_accel[1] - z_hat[1],
            z_accel[2] - z_hat[2],
        ]

    @staticmethod
    def jacobian_gyro(state: AhrsState) -> List[List[float]]:
        """Return gyro Jacobian H for δω and δb_g blocks."""
        R_BI: List[List[float]] = state.T_BI[0]
        R_IB: List[List[float]] = transpose3(R_BI)
        H: List[List[float]] = [
            [0.0 for _ in range(StateMapping.dimension())] for _ in range(3)
        ]
        omega_slice: slice = StateMapping.slice_delta_omega()
        i: int
        j: int
        for i in range(3):
            for j in range(3):
                H[i][omega_slice.start + j] = R_IB[i][j]
        b_g_slice: slice = StateMapping.slice_delta_b_g()
        for i in range(3):
            H[i][b_g_slice.start + i] = 1.0
        return H

    @staticmethod
    def jacobian_accel(state: AhrsState) -> List[List[float]]:
        """Return accel Jacobian H for δθ, δA_a, δb_a, and δg_W blocks."""
        R_WB: List[List[float]] = Quaternion.to_matrix(state.q_WB)
        R_BI: List[List[float]] = state.T_BI[0]
        R_IB: List[List[float]] = transpose3(R_BI)
        A_inv: List[List[float]] = _invert_3x3(state.A_a)
        f_B: List[float] = matvec3(R_WB, _scale_vec(state.g_W, -1.0))
        f_I: List[float] = matvec3(R_IB, f_B)
        A_inv_f: List[float] = matvec3(A_inv, f_I)
        H: List[List[float]] = [
            [0.0 for _ in range(StateMapping.dimension())] for _ in range(3)
        ]
        theta_slice: slice = StateMapping.slice_delta_theta()
        skew_g: List[List[float]] = skew(state.g_W)
        R_WB_skew: List[List[float]] = matmul3(R_WB, skew_g)
        theta_block: List[List[float]] = matmul3(R_IB, R_WB_skew)
        H_theta: List[List[float]] = matmul3(A_inv, theta_block)
        i: int
        j: int
        for i in range(3):
            for j in range(3):
                H[i][theta_slice.start + j] = H_theta[i][j]
        A_slice: slice = StateMapping.slice_delta_A_a()
        row: int
        for i in range(3):
            for j in range(3):
                col: int = A_slice.start + i * 3 + j
                scale: float = -A_inv_f[j]
                for row in range(3):
                    H[row][col] = scale * A_inv[row][i]
        b_a_slice: slice = StateMapping.slice_delta_b_a()
        for i in range(3):
            H[i][b_a_slice.start + i] = 1.0
        g_slice: slice = StateMapping.slice_delta_g_W()
        g_block: List[List[float]] = matmul3(R_IB, R_WB)
        H_g: List[List[float]] = matmul3(A_inv, scale_mat(g_block, -1.0))
        for i in range(3):
            for j in range(3):
                H[i][g_slice.start + j] = H_g[i][j]
        return H


def _assert_vector_length(name: str, vec: Sequence[float], length: int) -> None:
    """Validate vector length."""
    if len(vec) != length:
        raise ValueError(f"{name} must have length {length}")


def _add_vec(a: Sequence[float], b: Sequence[float]) -> List[float]:
    """Return a + b for length-3 vectors."""
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def _scale_vec(v: Sequence[float], scale: float) -> List[float]:
    """Return scale * v for length-3 vectors."""
    return [v[0] * scale, v[1] * scale, v[2] * scale]


def _invert_3x3(A: Sequence[Sequence[float]]) -> List[List[float]]:
    """Return inverse of a 3x3 matrix or raise for singularity."""
    if len(A) != 3 or any(len(row) != 3 for row in A):
        raise ValueError("A_a must be 3x3")
    a00, a01, a02 = A[0]
    a10, a11, a12 = A[1]
    a20, a21, a22 = A[2]
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
    max_abs: float = max(
        abs(a00),
        abs(a01),
        abs(a02),
        abs(a10),
        abs(a11),
        abs(a12),
        abs(a20),
        abs(a21),
        abs(a22),
    )
    scale: float = max(1.0, max_abs)
    if abs(det) <= 1e-12 * scale * scale * scale:
        raise ValueError("A_a is singular or ill-conditioned")
    inv_det: float = 1.0 / det
    return [
        [c00 * inv_det, c10 * inv_det, c20 * inv_det],
        [c01 * inv_det, c11 * inv_det, c21 * inv_det],
        [c02 * inv_det, c12 * inv_det, c22 * inv_det],
    ]
