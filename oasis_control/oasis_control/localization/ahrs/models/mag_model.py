################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

class MagModel:
    """Magnetometer measurement model for the AHRS core and EKF.

    Responsibility:
        Define the magnetometer prediction, residual, and Jacobian used
        during EKF updates.

    Purpose:
        Provide predicted magnetometer measurements and Jacobians in the
        magnetometer frame for EKF updates.

    Inputs/outputs:
        - Inputs: AhrsState, MagPacket (z_m), extrinsics T_BM.
        - Outputs: predicted mag measurement and residual in {M}.

    Dependencies:
        - Uses Quaternion utilities for R_WB and SE(3) for R_MB.
        - Works with NoiseAdaptation for adaptive R_m.

    Public API (to be implemented):
        - predict(state)
        - residual(z_m, z_hat)
        - jacobian(state)

    Data contract:
        - z_m is a 3x1 magnetometer measurement in {M}.
        - R_m is a 3x3 covariance in {M}.

    Frames and units:
        - Residual ν is in {M}, units tesla.
        - Raw measurements are not rotated into {B}; prediction is in {M}.

    Determinism and edge cases:
        - Deterministic mapping from state to predicted magnetometer
          measurement.
        - Residual sign convention is ν = z - z_hat.
        - If the magnetic field vector is near zero, reject the update.

    Equations:
        Prediction:
            m_hat_M = R_MB * R_WB * m_W
            ν = z_m - m_hat_M

    Numerical stability notes:
        - Normalize quaternions before forming R_WB.
        - Ensure R_m remains SPD if adapted.

    Suggested unit tests:
        - Residual uses ν = z - z_hat.
        - Prediction matches expected frame chain R_MB * R_WB.
    """

    pass
