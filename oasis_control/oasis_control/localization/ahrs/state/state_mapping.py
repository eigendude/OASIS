################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Canonical ordering of the AHRS error-state vector.

Responsibility:
    Define the deterministic layout of the error-state vector used for
    covariance storage, serialization, and Jacobian construction.

Inputs/outputs:
    - Inputs are structured error-state fields.
    - Outputs are vector indices and slices into the error-state vector.

Dependencies:
    - Used by AhrsErrorState, AhrsCovariance, and filter steps.

Determinism:
    Ordering is fixed and must be identical across all modules.
"""


class StateMapping:
    """Canonical mapping between structured fields and the error-state vector.

    Purpose:
        Provide deterministic index ranges for each error-state block so that
        all covariance and Jacobian math uses the same ordering.

    Public API (to be implemented):
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

    pass
