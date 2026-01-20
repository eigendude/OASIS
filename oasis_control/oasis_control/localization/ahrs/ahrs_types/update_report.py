################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Update report definition for AHRS measurement updates.

Responsibility:
    Document the diagnostics payload produced for each measurement update.

Inputs/outputs:
    - Inputs: measurement residuals, covariances, and acceptance status.
    - Outputs: structured report for logging and diagnostics.

Dependencies:
    - Produced by UpdateStep and AhrsEkf.

Determinism:
    Reports must faithfully reflect deterministic update decisions.
"""


class UpdateReport:
    """Report structure for a single measurement update.

    Purpose:
        Provide a stable contract for logging EKF update information and
        diagnosing rejected measurements.

    Public API (to be implemented):
        - as_dict()
        - summarize()

    Data contract:
        Required fields:
        - t_meas: measurement timestamp.
        - measurement_type: enum or string (gyro, accel, mag).
        - z: raw measurement vector.
        - z_hat: predicted measurement vector.
        - nu: residual vector.
        - R: measurement covariance.
        - S: innovation covariance.
        - mahalanobis2: scalar νᵀ S⁻¹ ν.
        - accepted: boolean.
        - rejection_reason: string or empty when accepted.

    Frames and units:
        - nu and z are in the measurement frame ({I} or {M}).
        - Covariances use squared measurement units.

    Determinism and edge cases:
        - rejection_reason must be deterministic for a given failure mode.
        - S not SPD or solver failure must set accepted = False.
        - Rejections are no-ops on state and covariance (no partial update).

    Equations:
        - mahalanobis2 = νᵀ S⁻¹ ν.

    Numerical stability notes:
        - Store symmetrized S for reporting.

    Suggested unit tests:
        - accepted flag matches gating decision.
        - rejection_reason populated on failure.
    """

    pass
