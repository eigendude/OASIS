################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS stats definitions."""


class Statistics:
    """
    Statistical utilities for EKF innovation bookkeeping.

    Innovation statistics follow:

        S = H P Hᵀ + R
        d² = νᵀ S⁻¹ ν

    Where ν is the innovation residual in the measurement frame. The interface
    captures the formulas and expected outputs used for update reporting.
    """

    pass
