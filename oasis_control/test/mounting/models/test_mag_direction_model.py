################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for magnetometer direction modeling."""

from __future__ import annotations

import numpy as np
import pytest

from oasis_control.localization.mounting.models.mag_direction_model import (
    MagDirectionModelError,
)
from oasis_control.localization.mounting.models.mag_direction_model import (
    direction_covariance_from_raw,
)
from oasis_control.localization.mounting.models.mag_direction_model import (
    direction_residual_cross,
)
from oasis_control.localization.mounting.models.mag_direction_model import normalize


def test_direction_covariance_from_raw() -> None:
    """Validate covariance conversion for a simple case."""
    m_raw: np.ndarray = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    cov_raw: np.ndarray = np.diag([1.0, 2.0, 3.0]).astype(np.float64)

    cov_dir: np.ndarray = direction_covariance_from_raw(
        m_raw_T=m_raw,
        cov_m_raw_T2=cov_raw,
        s_min_T=1e-6,
    )

    expected: np.ndarray = np.diag([0.0, 2.0, 3.0]).astype(np.float64)
    np.testing.assert_allclose(cov_dir, expected)


def test_direction_covariance_minimum_norm() -> None:
    """Ensure too-small magnitudes raise errors."""
    m_raw: np.ndarray = np.array([1e-7, 0.0, 0.0], dtype=np.float64)
    cov_raw: np.ndarray = np.eye(3, dtype=np.float64)

    with pytest.raises(MagDirectionModelError):
        direction_covariance_from_raw(
            m_raw_T=m_raw,
            cov_m_raw_T2=cov_raw,
            s_min_T=1e-6,
        )


def test_direction_residual_cross() -> None:
    """Verify cross-product residual computation."""
    u_meas: np.ndarray = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    u_pred: np.ndarray = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    residual: np.ndarray = direction_residual_cross(u_meas, u_pred)
    np.testing.assert_allclose(residual, np.array([0.0, 0.0, 1.0], dtype=np.float64))


def test_normalize_invalid() -> None:
    """Ensure invalid normalize inputs raise errors."""
    with pytest.raises(MagDirectionModelError):
        normalize(np.array([0.0, 0.0, 0.0], dtype=np.float64))
