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

from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.state_mapping import StateMapping


class StationaryModel:
    """Pseudo-measurement models for stationary updates in the AHRS EKF.

    Responsibility:
        Define the ZUPT and optional no-turn pseudo-measurements, residuals,
        and Jacobians for stationary optimization.

    Purpose:
        Provide measurement model hooks that allow the EKF to inject
        deterministic velocity and angular-rate pseudo-measurements.

    Inputs/outputs:
        - Inputs: AhrsState and optional pseudo-measurement vectors.
        - Outputs: predicted measurements, residuals, and Jacobians.

    Dependencies:
        - Used by UpdateStep and AhrsEkf.
        - Requires StateMapping to select delta_v and delta_omega blocks.

    Public API (to be implemented):
        - predict_zupt(state)
        - residual_zupt(z_v, z_hat)
        - jacobian_zupt(state)
        - predict_no_turn(state)
        - residual_no_turn(z_omega, z_hat)
        - jacobian_no_turn(state)

    Data contract:
        - z_v is a 3x1 zero-velocity vector in {W}.
        - z_omega is a 3x1 zero angular-rate vector in {B}.
        - R_v and R_omega are full 3x3 SPD covariances.

    Frames and units:
        - ZUPT residual ν_v in {W}, units m/s.
        - No-turn residual ν_ω in {B}, units rad/s.
        - Covariances use squared units and remain full.

    Determinism and edge cases:
        - Residual sign convention is ν = z - z_hat.
        - z_v and z_omega are deterministic zero vectors.
        - Reject updates if supplied R is not SPD.
        - Never diagonalize covariances.

    Equations:
        ZUPT:
            z_v = 0
            h(x) = v_WB
            ν = -v_WB
            H selects delta_v block (3 x N).

        No-turn:
            z_omega = 0
            h(x) = ω_WB
            ν = -ω_WB
            H selects delta_omega block (3 x N).

    Numerical stability notes:
        - Ensure Jacobians use the canonical StateMapping indices.

    Suggested unit tests:
        - ZUPT residual equals -v_WB.
        - No-turn residual equals -omega_WB.
    """

    @staticmethod
    def predict_zupt(state: AhrsState) -> List[float]:
        """Return the ZUPT predicted measurement z_hat = v_WB."""
        return list(state.v_WB)

    @staticmethod
    def residual_zupt(z_v: Sequence[float], z_hat: Sequence[float]) -> List[float]:
        """Return the ZUPT residual ν = z - z_hat."""
        _assert_vector_length("z_v", z_v, 3)
        _assert_vector_length("z_hat", z_hat, 3)
        return [z_v[0] - z_hat[0], z_v[1] - z_hat[1], z_v[2] - z_hat[2]]

    @staticmethod
    def jacobian_zupt(state: AhrsState) -> List[List[float]]:
        """Return the ZUPT Jacobian selecting the δv block."""
        _ = state
        H: List[List[float]] = _zeros_matrix(3, StateMapping.dimension())
        delta_v: slice = StateMapping.slice_delta_v()
        for i in range(3):
            H[i][delta_v.start + i] = 1.0
        return H

    @staticmethod
    def predict_no_turn(state: AhrsState) -> List[float]:
        """Return the no-turn predicted measurement z_hat = ω_WB."""
        return list(state.omega_WB)

    @staticmethod
    def residual_no_turn(
        z_omega: Sequence[float],
        z_hat: Sequence[float],
    ) -> List[float]:
        """Return the no-turn residual ν = z - z_hat."""
        _assert_vector_length("z_omega", z_omega, 3)
        _assert_vector_length("z_hat", z_hat, 3)
        return [
            z_omega[0] - z_hat[0],
            z_omega[1] - z_hat[1],
            z_omega[2] - z_hat[2],
        ]

    @staticmethod
    def jacobian_no_turn(state: AhrsState) -> List[List[float]]:
        """Return the no-turn Jacobian selecting the δω block."""
        _ = state
        H: List[List[float]] = _zeros_matrix(3, StateMapping.dimension())
        delta_omega: slice = StateMapping.slice_delta_omega()
        for i in range(3):
            H[i][delta_omega.start + i] = 1.0
        return H


def _assert_vector_length(name: str, vec: Sequence[float], length: int) -> None:
    if len(vec) != length:
        raise ValueError(f"{name} must have length {length}")


def _zeros_matrix(rows: int, cols: int) -> List[List[float]]:
    return [[0.0 for _ in range(cols)] for _ in range(rows)]
