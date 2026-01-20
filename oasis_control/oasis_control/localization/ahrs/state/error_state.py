################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List
from typing import Sequence

from oasis_control.localization.ahrs.state.state_mapping import StateMapping


@dataclass(frozen=True, slots=True)
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

    Public API:
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

    delta_x: List[float]

    @staticmethod
    def zero() -> AhrsErrorState:
        """Return a zero error-state vector."""
        zeros: List[float] = [0.0 for _ in range(StateMapping.dimension())]
        return AhrsErrorState(delta_x=zeros)

    def as_vector(self) -> List[float]:
        """Return a copy of the error-state vector."""
        return list(self.delta_x)

    @staticmethod
    def from_vector(delta_x: Sequence[float]) -> AhrsErrorState:
        """Create an error state from a raw vector."""
        if len(delta_x) != StateMapping.dimension():
            raise ValueError("delta_x must have length 45")
        delta_list: List[float] = [float(value) for value in delta_x]
        value: float
        for value in delta_list:
            if not math.isfinite(value):
                raise ValueError("delta_x contains non-finite value")
        return AhrsErrorState(delta_x=delta_list)

    def delta_p(self) -> List[float]:
        """Return δp as a copy."""
        return self._slice(StateMapping.slice_delta_p())

    def delta_v(self) -> List[float]:
        """Return δv as a copy."""
        return self._slice(StateMapping.slice_delta_v())

    def delta_theta(self) -> List[float]:
        """Return δθ as a copy."""
        return self._slice(StateMapping.slice_delta_theta())

    def delta_omega(self) -> List[float]:
        """Return δω as a copy."""
        return self._slice(StateMapping.slice_delta_omega())

    def delta_b_g(self) -> List[float]:
        """Return δb_g as a copy."""
        return self._slice(StateMapping.slice_delta_b_g())

    def delta_b_a(self) -> List[float]:
        """Return δb_a as a copy."""
        return self._slice(StateMapping.slice_delta_b_a())

    def delta_A_a(self) -> List[float]:
        """Return δA_a as a copy."""
        return self._slice(StateMapping.slice_delta_A_a())

    def delta_xi_BI(self) -> List[float]:
        """Return δξ_BI as a copy."""
        return self._slice(StateMapping.slice_delta_xi_BI())

    def delta_xi_BM(self) -> List[float]:
        """Return δξ_BM as a copy."""
        return self._slice(StateMapping.slice_delta_xi_BM())

    def delta_g_W(self) -> List[float]:
        """Return δg_W as a copy."""
        return self._slice(StateMapping.slice_delta_g_W())

    def delta_m_W(self) -> List[float]:
        """Return δm_W as a copy."""
        return self._slice(StateMapping.slice_delta_m_W())

    def apply_to_state(self, state: AhrsState) -> AhrsState:
        """Return a new mean state with this error applied."""
        return state.apply_error(self.delta_x)

    def _slice(self, slc: slice) -> List[float]:
        """Return a copy of the selected slice."""
        return list(self.delta_x[slc])


from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
