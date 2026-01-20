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


class MagModel:
    """Magnetometer measurement model for the AHRS core and EKF.

    Responsibility:
        Define the magnetometer prediction, residual, and Jacobian used
        during EKF updates.

    Purpose:
        Provide predicted magnetometer measurements and Jacobians in the
        magnetometer frame for EKF updates.

    Inputs/outputs:
        - Inputs: AhrsState, MagPacket (z_m), extrinsics T_BM.
        - Outputs: predicted mag measurement and residual in {M}.

    Dependencies:
        - Uses Quaternion utilities for R_WB and SE(3) for R_MB.
        - Works with NoiseAdaptation for adaptive R_m.

    Public API (to be implemented):
        - predict(state)
        - residual(z_m, z_hat)
        - jacobian(state)

    Data contract:
        - z_m is a 3x1 magnetometer measurement in {M}.
        - R_m is a 3x3 covariance in {M}.

    Frames and units:
        - Residual ν is in {M}, units tesla.
        - Raw measurements are not rotated into {B}; prediction is in {M}.

    Determinism and edge cases:
        - Deterministic mapping from state to predicted magnetometer
          measurement.
        - Residual sign convention is ν = z - z_hat.
        - If the magnetic field vector is near zero, reject the update.

    Equations:
        Prediction:
            m_hat_M = R_MB * R_WB * m_W
            ν = z_m - m_hat_M

    Numerical stability notes:
        - Normalize quaternions before forming R_WB.
        - Ensure R_m remains SPD if adapted.

    Suggested unit tests:
        - Residual uses ν = z - z_hat.
        - Prediction matches expected frame chain R_MB * R_WB.
    """

    _MIN_FIELD_NORM: float = 1e-9

    @staticmethod
    def predict(state: AhrsState) -> List[float]:
        """Return the predicted magnetometer measurement in {M}."""
        _assert_vector_length("m_W", state.m_W, 3)
        _assert_nonzero_field(state.m_W)
        R_WB: List[List[float]] = Quaternion.to_matrix(state.q_WB)
        R_BM: List[List[float]]
        _p_BM: List[float]
        R_BM, _p_BM = state.T_BM
        _assert_rotation(R_BM)
        R_MB: List[List[float]] = _transpose3(R_BM)
        temp: List[float] = _matvec3(R_WB, state.m_W)
        return _matvec3(R_MB, temp)

    @staticmethod
    def residual(z_m: Sequence[float], z_hat: Sequence[float]) -> List[float]:
        """Return the residual ν = z - z_hat in {M}."""
        _assert_vector_length("z_m", z_m, 3)
        _assert_vector_length("z_hat", z_hat, 3)
        return [z_m[0] - z_hat[0], z_m[1] - z_hat[1], z_m[2] - z_hat[2]]

    @staticmethod
    def jacobian(state: AhrsState) -> List[List[float]]:
        """Return the measurement Jacobian for the magnetometer update."""
        _assert_vector_length("m_W", state.m_W, 3)
        _assert_nonzero_field(state.m_W)
        R_WB: List[List[float]] = Quaternion.to_matrix(state.q_WB)
        R_BM: List[List[float]]
        _p_BM: List[float]
        R_BM, _p_BM = state.T_BM
        _assert_rotation(R_BM)
        R_MB: List[List[float]] = _transpose3(R_BM)
        R_MB_R_WB: List[List[float]] = _matmul3(R_MB, R_WB)
        H: List[List[float]] = _zeros_matrix(3, StateMapping.dimension())

        skew_m: List[List[float]] = _skew(state.m_W)
        J_theta: List[List[float]] = _matmul3(R_MB_R_WB, skew_m)
        for i in range(3):
            for j in range(3):
                H[i][StateMapping.slice_delta_theta().start + j] = -J_theta[i][j]

        J_m: List[List[float]] = R_MB_R_WB
        for i in range(3):
            for j in range(3):
                H[i][StateMapping.slice_delta_m_W().start + j] = J_m[i][j]
        return H


def _assert_nonzero_field(m_W: Sequence[float]) -> None:
    norm: float = math.sqrt(m_W[0] * m_W[0] + m_W[1] * m_W[1] + m_W[2] * m_W[2])
    if norm < MagModel._MIN_FIELD_NORM:
        raise ValueError("m_W norm is too small for magnetometer update")


def _assert_vector_length(name: str, vec: Sequence[float], length: int) -> None:
    if len(vec) != length:
        raise ValueError(f"{name} must have length {length}")


def _assert_rotation(R: Sequence[Sequence[float]]) -> None:
    if len(R) != 3 or any(len(row) != 3 for row in R):
        raise ValueError("Rotation matrix must be 3x3")


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


def _zeros_matrix(rows: int, cols: int) -> List[List[float]]:
    return [[0.0 for _ in range(cols)] for _ in range(rows)]
