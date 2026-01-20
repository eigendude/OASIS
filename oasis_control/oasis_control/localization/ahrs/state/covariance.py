################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS covariance definitions."""


class AhrsCovariance:
    """
    Covariance container for the AHRS error state.

    The covariance P is a full matrix over the error-state ordering. After each
    update, P is symmetrized as:

        P ← 0.5 * (P + Pᵀ)

    Block access uses the error-state layout defined by the mapping utilities.
    """

    pass
