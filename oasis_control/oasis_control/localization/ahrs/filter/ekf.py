################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS ekf definitions."""


class AhrsEkf:
    """
    Extended Kalman filter orchestrator for the AHRS.

    Core API:

    - predict(Δt): propagate the mean and covariance via the process model
    - update_gyro(...): apply gyro measurement update
    - update_accel(...): apply accelerometer update
    - update_mag(...): apply magnetometer update

    The update step uses the standard EKF equations:

        S = H P Hᵀ + R
        K = P Hᵀ S⁻¹
        δx = K ν
        x ← x ⊕ δx
        P ← (I - K H) P (I - K H)ᵀ + K R Kᵀ
    """

    pass
