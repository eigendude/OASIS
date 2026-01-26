################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for magnetometer packet types."""

from __future__ import annotations

import numpy as np
import pytest

from oasis_control.localization.mounting.mounting_types.mag_packet import MagPacket


def test_mag_packet_helpers() -> None:
    """Verify magnitude and unit direction helpers."""
    packet: MagPacket = MagPacket(
        t_meas_ns=10,
        frame_id="mag",
        m_raw_T=np.array([1.0, 2.0, 2.0], dtype=np.float64),
        cov_m_raw_T2=np.eye(3, dtype=np.float64),
    )
    magnitude: float = packet.magnitude_T()
    assert magnitude == pytest.approx(3.0)
    unit_dir: np.ndarray = packet.unit_direction()
    np.testing.assert_allclose(unit_dir, np.array([1.0, 2.0, 2.0]) / 3.0)


def test_mag_packet_shape_validation() -> None:
    """Ensure invalid shapes raise a ValueError."""
    with pytest.raises(ValueError):
        MagPacket(
            t_meas_ns=0,
            frame_id="mag",
            m_raw_T=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            cov_m_raw_T2=np.eye(2, dtype=np.float64),
        )


def test_mag_packet_finite_validation() -> None:
    """Ensure NaN values are rejected."""
    m_raw_T: np.ndarray = np.array([1.0, np.nan, 0.0], dtype=np.float64)
    with pytest.raises(ValueError):
        MagPacket(
            t_meas_ns=0,
            frame_id="mag",
            m_raw_T=m_raw_T,
            cov_m_raw_T2=np.eye(3, dtype=np.float64),
        )
