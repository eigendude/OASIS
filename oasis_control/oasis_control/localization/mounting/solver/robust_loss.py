################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Robust loss helpers for mounting calibration."""

from __future__ import annotations

import numpy as np


def huber_weight(sq_norm: float, scale: float) -> float:
    """Return the Huber weight for a squared residual norm."""
    if scale <= 0.0:
        return 1.0
    if sq_norm <= scale * scale:
        return 1.0
    denom: float = float(np.sqrt(sq_norm))
    if denom <= 0.0:
        return 1.0
    return float(scale / denom)


def cauchy_weight(sq_norm: float, scale: float) -> float:
    """Return the Cauchy weight for a squared residual norm."""
    if scale <= 0.0:
        return 1.0
    return float(1.0 / (1.0 + sq_norm / (scale * scale)))


def robust_weight(
    residual: np.ndarray,
    robust_type: str | None,
    scale: float,
) -> float:
    """Return the scalar robust weight for a residual vector."""
    if robust_type is None:
        return 1.0
    robust_name: str = robust_type.lower()
    if robust_name in ("none", "identity", "off"):
        return 1.0
    sq_norm: float = float(np.dot(residual, residual))
    if robust_name == "huber":
        return huber_weight(sq_norm, scale)
    if robust_name == "cauchy":
        return cauchy_weight(sq_norm, scale)
    return 1.0
