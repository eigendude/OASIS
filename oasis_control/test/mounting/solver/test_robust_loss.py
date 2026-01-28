################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for robust loss weights."""

from __future__ import annotations

import numpy as np

from oasis_control.localization.mounting.solver.robust_loss import cauchy_weight
from oasis_control.localization.mounting.solver.robust_loss import huber_weight


def test_huber_weight_limits() -> None:
    """Check Huber weights for small and large residuals."""
    scale: float = 1.0
    small_sq: float = 1e-4
    large_sq: float = 100.0
    small_weight: float = huber_weight(small_sq, scale)
    large_weight: float = huber_weight(large_sq, scale)
    assert np.isclose(small_weight, 1.0, atol=1e-12)
    assert np.isclose(large_weight, scale / np.sqrt(large_sq), atol=1e-12)


def test_cauchy_weight_monotonic() -> None:
    """Check Cauchy weights decrease monotonically with residual size."""
    scale: float = 0.5
    weights: list[float] = [
        cauchy_weight(0.0, scale),
        cauchy_weight(0.1, scale),
        cauchy_weight(1.0, scale),
        cauchy_weight(10.0, scale),
    ]
    assert all(weights[i] >= weights[i + 1] for i in range(len(weights) - 1))
