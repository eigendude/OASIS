################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS linalg definitions."""

class LinearAlgebra:
    """
    Linear-algebra helpers for full covariance handling.

    All covariance matrices are maintained as full, symmetric matrices. When a
    numerical operation produces minor asymmetry, the result is symmetrized:

        P_sym = 0.5 * (P + Pᵀ)

    This class documents the contract for matrix operations used by the AHRS
    without binding to a specific numerical backend.
    """
    pass
