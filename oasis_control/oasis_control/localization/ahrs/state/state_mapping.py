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


class StateMapping:
    """Canonical ordering of the AHRS error-state vector.

    Responsibility:
        Define the deterministic layout of the error-state vector used for
        covariance storage, serialization, and Jacobian construction.

    Purpose:
        Provide deterministic index ranges for each error-state block so that
        all covariance and Jacobian math uses the same ordering.

    Inputs/outputs:
        - Inputs are structured error-state fields.
        - Outputs are vector indices and slices into the error-state vector.

    Dependencies:
        - Used by AhrsErrorState, AhrsCovariance, and filter steps.

    Public API:
        - dimension()
        - slice_delta_p()
        - slice_delta_v()
        - slice_delta_theta()
        - slice_delta_omega()
        - slice_delta_b_g()
        - slice_delta_b_a()
        - slice_delta_A_a()
        - slice_delta_xi_BI()
        - slice_delta_xi_BM()
        - slice_delta_g_W()
        - slice_delta_m_W()

    Data contract:
        Total dimension N = 45 with canonical ordering:
        - delta_p: [0:3]
        - delta_v: [3:6]
        - delta_theta: [6:9]
        - delta_omega: [9:12]
        - delta_b_g: [12:15]
        - delta_b_a: [15:18]
        - delta_A_a: [18:27]
        - delta_xi_BI: [27:33]
        - delta_xi_BM: [33:39]
        - delta_g_W: [39:42]
        - delta_m_W: [42:45]

    Frames and units:
        - See AhrsErrorState and Units for units of each block.

    Determinism and edge cases:
        - Ordering is fixed and must be identical across all modules.
        - This mapping is the canonical ordering for covariance serialization.
        - Index ranges must not change without a migration path.

    Equations:
        - Vectorization is linear; no scaling or weighting is applied.

    Numerical stability notes:
        - Ordering is deterministic, not numerically sensitive.

    Suggested unit tests:
        - dimension() returns 45.
        - Slices are contiguous and cover [0, 45).
    """

    _DELTA_P: slice = slice(0, 3)
    _DELTA_V: slice = slice(3, 6)
    _DELTA_THETA: slice = slice(6, 9)
    _DELTA_OMEGA: slice = slice(9, 12)
    _DELTA_B_G: slice = slice(12, 15)
    _DELTA_B_A: slice = slice(15, 18)
    _DELTA_A_A: slice = slice(18, 27)
    _DELTA_XI_BI: slice = slice(27, 33)
    _DELTA_XI_BM: slice = slice(33, 39)
    _DELTA_G_W: slice = slice(39, 42)
    _DELTA_M_W: slice = slice(42, 45)

    @staticmethod
    def dimension() -> int:
        """Return the dimension of the error-state vector."""
        StateMapping._assert_layout()
        return 45

    @staticmethod
    def slice_delta_p() -> slice:
        """Return the slice for δp."""
        return StateMapping._DELTA_P

    @staticmethod
    def slice_delta_v() -> slice:
        """Return the slice for δv."""
        return StateMapping._DELTA_V

    @staticmethod
    def slice_delta_theta() -> slice:
        """Return the slice for δθ."""
        return StateMapping._DELTA_THETA

    @staticmethod
    def slice_delta_omega() -> slice:
        """Return the slice for δω."""
        return StateMapping._DELTA_OMEGA

    @staticmethod
    def slice_delta_b_g() -> slice:
        """Return the slice for δb_g."""
        return StateMapping._DELTA_B_G

    @staticmethod
    def slice_delta_b_a() -> slice:
        """Return the slice for δb_a."""
        return StateMapping._DELTA_B_A

    @staticmethod
    def slice_delta_A_a() -> slice:
        """Return the slice for δA_a."""
        return StateMapping._DELTA_A_A

    @staticmethod
    def slice_delta_xi_BI() -> slice:
        """Return the slice for δξ_BI."""
        return StateMapping._DELTA_XI_BI

    @staticmethod
    def slice_delta_xi_BM() -> slice:
        """Return the slice for δξ_BM."""
        return StateMapping._DELTA_XI_BM

    @staticmethod
    def slice_delta_g_W() -> slice:
        """Return the slice for δg_W."""
        return StateMapping._DELTA_G_W

    @staticmethod
    def slice_delta_m_W() -> slice:
        """Return the slice for δm_W."""
        return StateMapping._DELTA_M_W

    @staticmethod
    def _assert_layout() -> None:
        """Validate the state layout is contiguous and complete."""
        slices: List[slice] = [
            StateMapping._DELTA_P,
            StateMapping._DELTA_V,
            StateMapping._DELTA_THETA,
            StateMapping._DELTA_OMEGA,
            StateMapping._DELTA_B_G,
            StateMapping._DELTA_B_A,
            StateMapping._DELTA_A_A,
            StateMapping._DELTA_XI_BI,
            StateMapping._DELTA_XI_BM,
            StateMapping._DELTA_G_W,
            StateMapping._DELTA_M_W,
        ]
        expected_start: int = 0
        for slc in slices:
            if slc.step not in (None, 1):
                raise ValueError("StateMapping layout invalid")
            if slc.start != expected_start:
                raise ValueError("StateMapping layout invalid")
            if slc.stop is None:
                raise ValueError("StateMapping layout invalid")
            if slc.stop < slc.start:
                raise ValueError("StateMapping layout invalid")
            expected_start = slc.stop
        if expected_start != 45:
            raise ValueError("StateMapping layout invalid")
        _ = _slice_lengths(slices)


def _slice_lengths(slices: Sequence[slice]) -> List[int]:
    """Return lengths of slices for validation."""
    lengths: List[int] = []
    slc: slice
    for slc in slices:
        if slc.start is None or slc.stop is None:
            raise ValueError("StateMapping layout invalid")
        lengths.append(slc.stop - slc.start)
    return lengths
