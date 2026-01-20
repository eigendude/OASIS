################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS noise adaptation definitions."""

class NoiseAdaptation:
    """
    Adaptive magnetometer noise model using covariance matching.

    The update uses the innovation ν and predicted covariance without noise Ŝ:

        R_m ← clamp_SPD((1-α) R_m + α (ν νᵀ - Ŝ), R_min, R_max)

    The covariance is kept symmetric positive definite by clamping eigenvalues
    between R_min and R_max.
    """
    pass
