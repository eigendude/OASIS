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
from oasis_control.localization.ahrs.math_utils.small_linalg3 import dot
from oasis_control.localization.ahrs.math_utils.small_linalg3 import matmul3
from oasis_control.localization.ahrs.math_utils.small_linalg3 import matvec3
from oasis_control.localization.ahrs.math_utils.small_linalg3 import scale_mat
from oasis_control.localization.ahrs.math_utils.small_linalg3 import skew
from oasis_control.localization.ahrs.math_utils.small_linalg3 import transpose3
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

    @staticmethod
    def predict(state: AhrsState) -> List[float]:
        """Return predicted magnetometer measurement in {M}."""
        _assert_vector_length("m_W", state.m_W, 3)
        _assert_finite_vector("m_W", state.m_W)
        norm_sq: float = dot(state.m_W, state.m_W)
        if norm_sq < 1e-18:
            raise ValueError("m_W magnitude is near zero")
        R_WB: List[List[float]] = Quaternion.to_matrix(state.q_WB)
        R_BM: List[List[float]] = state.T_BM[0]
        R_MB: List[List[float]] = transpose3(R_BM)
        m_body: List[float] = matvec3(R_WB, state.m_W)
        return matvec3(R_MB, m_body)

    @staticmethod
    def residual(z_m: Sequence[float], z_hat: Sequence[float]) -> List[float]:
        """Return ν = z - z_hat for magnetometer update."""
        _assert_vector_length("z_m", z_m, 3)
        _assert_vector_length("z_hat", z_hat, 3)
        return [z_m[0] - z_hat[0], z_m[1] - z_hat[1], z_m[2] - z_hat[2]]

    @staticmethod
    def jacobian(state: AhrsState) -> List[List[float]]:
        """Return the measurement Jacobian H for magnetometer update."""
        _assert_vector_length("m_W", state.m_W, 3)
        _assert_finite_vector("m_W", state.m_W)
        norm_sq: float = dot(state.m_W, state.m_W)
        if norm_sq < 1e-18:
            raise ValueError("m_W magnitude is near zero")
        R_WB: List[List[float]] = Quaternion.to_matrix(state.q_WB)
        R_BM: List[List[float]] = state.T_BM[0]
        R_MB: List[List[float]] = transpose3(R_BM)
        H: List[List[float]] = [
            [0.0 for _ in range(StateMapping.dimension())] for _ in range(3)
        ]
        skew_m: List[List[float]] = skew(state.m_W)
        R_WB_skew: List[List[float]] = matmul3(R_WB, skew_m)
        H_theta: List[List[float]] = matmul3(R_MB, scale_mat(R_WB_skew, -1.0))
        theta_slice: slice = StateMapping.slice_delta_theta()
        i: int
        j: int
        for i in range(3):
            for j in range(3):
                H[i][theta_slice.start + j] = H_theta[i][j]
        H_m: List[List[float]] = matmul3(R_MB, R_WB)
        m_slice: slice = StateMapping.slice_delta_m_W()
        for i in range(3):
            for j in range(3):
                H[i][m_slice.start + j] = H_m[i][j]
        return H


def _assert_vector_length(name: str, vec: Sequence[float], length: int) -> None:
    """Validate vector length."""
    if len(vec) != length:
        raise ValueError(f"{name} must have length {length}")


def _assert_finite_vector(name: str, vec: Sequence[float]) -> None:
    """Validate vector entries are finite."""
    value: float
    for value in vec:
        if not math.isfinite(value):
            raise ValueError(f"{name} contains non-finite value")
