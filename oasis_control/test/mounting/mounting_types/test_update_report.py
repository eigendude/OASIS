################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for update report types."""

from __future__ import annotations

import numpy as np
import pytest

from oasis_control.localization.mounting.mounting_types.update_report import (
    UpdateReport,
)


def test_update_report_validation() -> None:
    """Ensure invalid fields raise errors."""
    with pytest.raises(ValueError):
        UpdateReport(
            did_update=True,
            iterations=-1,
            step_norm=None,
            residual_rms=None,
            need_more_tilt=False,
            need_more_yaw=False,
            dropped_segments=0,
            dropped_mags=0,
            message=None,
        )

    with pytest.raises(ValueError):
        UpdateReport(
            did_update=True,
            iterations=1,
            step_norm=float(np.nan),
            residual_rms=None,
            need_more_tilt=False,
            need_more_yaw=False,
            dropped_segments=0,
            dropped_mags=0,
            message=None,
        )


def test_update_report_success() -> None:
    """Ensure a valid report is accepted."""
    report: UpdateReport = UpdateReport(
        did_update=True,
        iterations=3,
        step_norm=0.1,
        residual_rms=0.5,
        need_more_tilt=False,
        need_more_yaw=True,
        dropped_segments=1,
        dropped_mags=0,
        message="ok",
    )
    assert report.did_update
