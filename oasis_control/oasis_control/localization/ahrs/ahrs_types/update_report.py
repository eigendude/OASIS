################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################


class UpdateReport:
    """Update report definition for AHRS measurement updates.

    Responsibility:
        Document the diagnostics payload produced for each measurement
        update.

    Purpose:
        Provide a stable contract for logging EKF update information and
        diagnosing rejected measurements.

    Inputs/outputs:
        - Inputs: measurement residuals, covariances, and acceptance status.
        - Outputs: structured report for logging and diagnostics.

    Dependencies:
        - Produced by UpdateStep and AhrsEkf.

    Public API (to be implemented):
        - as_dict()
        - summarize()

    Data contract:
        Required fields:
        - t_meas_ns: measurement timestamp in int nanoseconds since an
          arbitrary epoch.
        - measurement_type: enum or string (gyro, accel, mag, zupt, no_turn).
        - z: raw measurement vector.
        - z_hat: predicted measurement vector.
        - nu: residual vector.
        - R: full measurement covariance (never diagonalized).
        - S_hat: optional but recommended predicted innovation covariance
          excluding measurement noise (Ŝ = H P Hᵀ).
        - S: innovation covariance.
        - mahalanobis2: scalar νᵀ S⁻¹ ν.
        - accepted: boolean.
        - rejection_reason: string or empty when accepted.

    Frames and units:
        - nu and z are in the measurement frame ({I} or {M}).
        - Covariances use squared measurement units.

    Determinism and edge cases:
        - Reports faithfully reflect deterministic update decisions.
        - All timestamps are int nanoseconds since an arbitrary epoch.
          The epoch is irrelevant because only differences and exact
          equality are used.
        - rejection_reason must be deterministic for a given failure mode.
        - S not SPD or solver failure must set accepted = False.
        - Rejections are no-ops on state and covariance (no partial update).

    Equations:
        - S_hat = H P Hᵀ (innovation covariance excluding measurement noise).
        - S = S_hat + R.
        - mahalanobis2 = νᵀ S⁻¹ ν.

    Numerical stability notes:
        - Store symmetrized S for reporting.

    Suggested unit tests:
        - accepted flag matches gating decision.
        - rejection_reason populated on failure.
    """

    pass
