################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS error state definitions."""

class AhrsErrorState:
    """
    Error-state definition for the AHRS covariance.

    The stacked error vector is ordered as:

        [δp, δv, δθ, δω, δb_g, δb_a, δA_a,
         δξ_BI, δξ_BM, δg_W, δm_W]

    Each block is expressed in the same frame as its corresponding mean state
    component. The δA_a block stores a row-major perturbation of the 3×3 matrix.
    """
    pass
