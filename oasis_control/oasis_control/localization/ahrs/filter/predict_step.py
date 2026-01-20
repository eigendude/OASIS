################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS predict step definitions."""


class PredictStep:
    """
    Reusable EKF predict step for continuous-time dynamics.

    Given Jacobians A and G and noise intensity Q_c, the discrete propagation is:

        F ≈ I + AΔt
        Q ≈ G Q_c Gᵀ Δt

    The mean state is integrated using the process model equations.
    """

    pass
