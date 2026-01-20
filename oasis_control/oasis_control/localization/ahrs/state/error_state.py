################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

class AhrsErrorState:
    """Error-state representation for the AHRS EKF.

    Responsibility:
        Define the small perturbation state used by the error-state EKF and
        the ordering contract shared with StateMapping and AhrsCovariance.

    Purpose:
        Provide a typed view of the error-state vector used to update the
        mean state and covariance.

    Inputs/outputs:
        - Inputs are error vectors delta_x of length N.
        - Outputs are structured views of delta_x for specific state blocks.

    Dependencies:
        - Used by state_mapping, covariance, and filter update logic.

    Public API (to be implemented):
        - zero()
        - as_vector()
        - from_vector(delta_x)
        - apply_to_state(state)

    Data contract:
        Error-state blocks (see StateMapping for canonical ordering):
        - delta_p, delta_v, delta_theta, delta_omega
        - delta_b_g, delta_b_a
        - delta_A_a (9 elements for 3x3 calibration)
        - delta_xi_BI, delta_xi_BM (6 elements each)
        - delta_g_W, delta_m_W

    Frames and units:
        - delta_p in meters, delta_v in meters per second.
        - delta_theta in radians (tangent-space rotation).
        - delta_omega in rad/s.
        - delta_b_g in rad/s, delta_b_a in m/s^2.
        - delta_xi blocks use meters and radians.
        - delta_g_W in m/s^2, delta_m_W in tesla.

    Determinism and edge cases:
        - Error-state ordering is fixed and deterministic across all modules.
        - Conversion between vector and structured fields must be stable and
          lossless.
        - Delta-theta assumes small-angle approximation.

    Equations:
        - dq ≈ [1, 0.5 * delta_theta] for small-angle updates.
        - SE(3) perturbations apply via Exp(delta_xi) * T.

    Numerical stability notes:
        - Keep delta_theta small; large corrections should be limited by
          gating or reset logic.

    Suggested unit tests:
        - round-trip from_vector/as_vector preserves data.
        - apply_to_state yields expected quaternion updates for small angles.
    """

    pass
