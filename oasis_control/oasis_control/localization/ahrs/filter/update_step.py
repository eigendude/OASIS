################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS update step definitions."""


class UpdateStep:
    """
    Reusable EKF update step for a single measurement.

    Inputs include the innovation ν, Jacobian H, and measurement covariance R.
    Outputs include the state perturbation δx and updated covariance following:

        S = H P Hᵀ + R
        K = P Hᵀ S⁻¹
        δx = K ν
    """

    pass
