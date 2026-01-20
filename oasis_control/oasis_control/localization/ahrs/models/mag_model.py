################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS mag model definitions."""


class MagModel:
    """
    Magnetometer measurement model in the magnetometer frame {M}.

    Prediction and residual are formed directly in {M}:

        m̂_M = R_MB * R_WB * m_W
        ν_m = z_m - m̂_M

    The model uses an adaptive measurement covariance R_m(t) and does not rotate
    raw measurements into the body frame.
    """

    pass
