################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Reference anchoring model for mounting calibration."""

from __future__ import annotations

import numpy as np


class AnchorModelError(Exception):
    """Raised when anchor model usage is invalid."""


def _unit_vector(vector: np.ndarray, name: str) -> np.ndarray:
    """Return a unit vector after validation."""
    vec: np.ndarray = np.asarray(vector, dtype=np.float64)
    if vec.shape != (3,):
        raise AnchorModelError(f"{name} must have shape (3,)")
    if not np.all(np.isfinite(vec)):
        raise AnchorModelError(f"{name} must be finite")
    norm: float = float(np.linalg.norm(vec))
    if not np.isfinite(norm) or norm <= 0.0:
        raise AnchorModelError(f"{name} must be non-zero to normalize")
    return vec / norm


class AnchorModel:
    """Capture and access reference gravity/mag directions."""

    def __init__(self) -> None:
        """Initialize the anchor model with no captured reference."""
        self.captured: bool = False
        self.g_ref_W: np.ndarray | None = None
        self.m_ref_W: np.ndarray | None = None
        self.mag_reference_invalid: bool = False

    def reset(self) -> None:
        """Reset the anchor model state."""
        self.captured = False
        self.g_ref_W = None
        self.m_ref_W = None
        self.mag_reference_invalid = False

    def capture(self, *, g_ref_W: np.ndarray, m_ref_W: np.ndarray | None) -> None:
        """Capture reference gravity and magnetometer directions."""
        if self.captured:
            raise AnchorModelError("anchor already captured; reset required")
        self.g_ref_W = _unit_vector(g_ref_W, "g_ref_W")
        if m_ref_W is None:
            self.m_ref_W = None
            self.mag_reference_invalid = True
        else:
            self.m_ref_W = _unit_vector(m_ref_W, "m_ref_W")
            self.mag_reference_invalid = False
        self.captured = True

    def has_mag_reference(self) -> bool:
        """Return True when a valid magnetometer reference exists."""
        return (
            self.captured
            and self.m_ref_W is not None
            and not self.mag_reference_invalid
        )

    def gravity_ref_W(self) -> np.ndarray:
        """Return the captured gravity reference direction."""
        if not self.captured or self.g_ref_W is None:
            raise AnchorModelError("gravity reference not captured")
        return np.array(self.g_ref_W, dtype=np.float64)

    def mag_ref_W(self) -> np.ndarray:
        """Return the captured magnetometer reference direction."""
        if not self.captured:
            raise AnchorModelError("mag reference not captured")
        if self.m_ref_W is None or self.mag_reference_invalid:
            raise AnchorModelError("mag reference is not available")
        return np.array(self.m_ref_W, dtype=np.float64)
