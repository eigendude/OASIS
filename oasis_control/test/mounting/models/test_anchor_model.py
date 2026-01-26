################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the anchor model."""

from __future__ import annotations

import numpy as np
import pytest

from oasis_control.localization.mounting.models.anchor_model import AnchorModel
from oasis_control.localization.mounting.models.anchor_model import AnchorModelError


def test_anchor_capture_and_flags() -> None:
    """Verify capture stores references and flags."""
    model: AnchorModel = AnchorModel()
    g_ref: np.ndarray = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    m_ref: np.ndarray = np.array([1.0, 0.0, 0.0], dtype=np.float64)

    model.capture(g_ref_W=g_ref, m_ref_W=m_ref)

    assert model.captured
    assert model.has_mag_reference()
    np.testing.assert_allclose(model.gravity_ref_W(), g_ref)
    np.testing.assert_allclose(model.mag_ref_W(), m_ref)


def test_anchor_capture_without_mag() -> None:
    """Verify capture without mag marks mag reference invalid."""
    model: AnchorModel = AnchorModel()
    g_ref: np.ndarray = np.array([0.0, 1.0, 0.0], dtype=np.float64)

    model.capture(g_ref_W=g_ref, m_ref_W=None)

    assert model.captured
    assert not model.has_mag_reference()
    assert model.mag_reference_invalid


def test_anchor_double_capture_raises() -> None:
    """Ensure capturing twice without reset raises an error."""
    model: AnchorModel = AnchorModel()
    g_ref: np.ndarray = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    model.capture(g_ref_W=g_ref, m_ref_W=None)
    with pytest.raises(AnchorModelError):
        model.capture(g_ref_W=g_ref, m_ref_W=None)


def test_anchor_access_before_capture_raises() -> None:
    """Ensure access before capture raises errors."""
    model: AnchorModel = AnchorModel()
    with pytest.raises(AnchorModelError):
        model.gravity_ref_W()
    with pytest.raises(AnchorModelError):
        model.mag_ref_W()
