################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for covariance utilities."""

from __future__ import annotations

import numpy as np
import pytest

from oasis_control.localization.mounting.state.covariance import Covariance
from oasis_control.localization.mounting.state.covariance import CovarianceError


def test_covariance_zeros() -> None:
    """Ensure zero covariances are valid and symmetric."""
    cov: Covariance = Covariance.zeros(3)
    mat: np.ndarray = cov.as_array()
    assert mat.shape == (3, 3)
    np.testing.assert_allclose(mat, np.zeros((3, 3), dtype=np.float64))


def test_covariance_rejects_non_symmetric() -> None:
    """Ensure non-symmetric covariance matrices raise."""
    mat: np.ndarray = np.array(
        [[1.0, 2.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    with pytest.raises(CovarianceError):
        Covariance(mat)


def test_covariance_psd_checks() -> None:
    """Ensure PSD checks are consistent."""
    cov: Covariance = Covariance.eye(3)
    assert cov.is_psd()
    cov.assert_psd()

    bad: Covariance = Covariance(np.diag(np.array([1.0, -0.5, 2.0], dtype=np.float64)))
    assert not bad.is_psd()
    with pytest.raises(CovarianceError):
        bad.assert_psd()
