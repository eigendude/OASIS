################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for diagnostics types."""

from __future__ import annotations

from typing import cast

import pytest

from oasis_control.localization.mounting.mounting_types.diagnostics import Diagnostics
from oasis_control.localization.mounting.mounting_types.update_report import (
    UpdateReport,
)


def test_diagnostics_to_dict() -> None:
    """Ensure diagnostics export to a dictionary."""
    report: UpdateReport = UpdateReport(
        did_update=True,
        iterations=1,
        step_norm=0.1,
        residual_rms=0.2,
        need_more_tilt=False,
        need_more_yaw=False,
        dropped_segments=0,
        dropped_mags=0,
        message=None,
    )
    diag: Diagnostics = Diagnostics(
        t_meas_ns=0,
        pipeline_state="running",
        update_report=report,
        segment_count=2,
        keyframe_count=1,
        diversity_tilt_deg=5.0,
        diversity_yaw_deg=None,
        warnings=("warn",),
        errors=(),
        info=("info",),
        dropped_packets=0,
    )
    data: dict[str, object] = diag.to_dict()
    assert data["pipeline_state"] == "running"
    assert data["update_report"] == report.to_dict()


def test_diagnostics_validation() -> None:
    """Ensure invalid diagnostic fields raise errors."""
    with pytest.raises(ValueError):
        Diagnostics(
            t_meas_ns=0,
            pipeline_state="state",
            update_report=None,
            segment_count=-1,
            keyframe_count=0,
            diversity_tilt_deg=None,
            diversity_yaw_deg=None,
            warnings=(),
            errors=(),
            info=(),
            dropped_packets=0,
        )

    with pytest.raises(ValueError):
        Diagnostics(
            t_meas_ns=0,
            pipeline_state="state",
            update_report=None,
            segment_count=0,
            keyframe_count=0,
            diversity_tilt_deg=None,
            diversity_yaw_deg=None,
            warnings=cast(tuple[str, ...], ("ok", 1)),
            errors=(),
            info=(),
            dropped_packets=0,
        )


def test_diagnostics_has_errors() -> None:
    """Ensure error detection works."""
    diag: Diagnostics = Diagnostics(
        t_meas_ns=0,
        pipeline_state="state",
        update_report=None,
        segment_count=0,
        keyframe_count=0,
        diversity_tilt_deg=None,
        diversity_yaw_deg=None,
        warnings=(),
        errors=("err",),
        info=(),
        dropped_packets=0,
    )
    assert diag.has_errors()
