################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for adaptive noise modeling."""

from __future__ import annotations

import numpy as np
import pytest

from oasis_control.localization.mounting.models.noise_adaptation import (
    DirectionNoiseAdapter,
)
from oasis_control.localization.mounting.models.noise_adaptation import (
    NoiseAdaptationError,
)


def test_noise_adapter_update_moves_r() -> None:
    """Ensure the update nudges R toward the residual covariance."""
    adapter: DirectionNoiseAdapter = DirectionNoiseAdapter(
        R_init=np.eye(3, dtype=np.float64) * 0.1,
        R_min=np.eye(3, dtype=np.float64) * 0.01,
        R_max=np.eye(3, dtype=np.float64) * 1.0,
        alpha=1.0,
    )
    residual: np.ndarray = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    S_hat: np.ndarray = np.zeros((3, 3), dtype=np.float64)

    updated: np.ndarray = adapter.update(r=residual, S_hat=S_hat)

    expected_diag: np.ndarray = np.array([1.0, 0.01, 0.01], dtype=np.float64)
    np.testing.assert_allclose(np.diag(updated), expected_diag)


def test_noise_adapter_clamps_bounds() -> None:
    """Ensure eigenvalue bounds are enforced."""
    adapter: DirectionNoiseAdapter = DirectionNoiseAdapter(
        R_init=np.eye(3, dtype=np.float64) * 0.1,
        R_min=np.eye(3, dtype=np.float64) * 0.05,
        R_max=np.eye(3, dtype=np.float64) * 0.2,
        alpha=1.0,
    )
    residual: np.ndarray = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    S_hat: np.ndarray = np.eye(3, dtype=np.float64) * 1.0

    updated: np.ndarray = adapter.update(r=residual, S_hat=S_hat)

    diag: np.ndarray = np.diag(updated)
    assert np.all(diag >= 0.05)
    assert np.all(diag <= 0.2)


def test_noise_adapter_invalid_alpha() -> None:
    """Ensure invalid alpha values raise errors."""
    with pytest.raises(NoiseAdaptationError):
        DirectionNoiseAdapter(
            R_init=np.eye(3, dtype=np.float64),
            R_min=np.eye(3, dtype=np.float64),
            R_max=np.eye(3, dtype=np.float64),
            alpha=0.0,
        )
