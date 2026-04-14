################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for helipad landing detection logic."""

from __future__ import annotations

from oasis_control.lego_models.helipad_manager import HelipadMode
from oasis_control.lego_models.helipad_manager import HelipadModeTracker


def _make_tracker() -> HelipadModeTracker:
    return HelipadModeTracker(
        land_threshold_volts=1.0,
        debounce_secs=0.2,
    )


def test_guidance_is_reported_above_threshold() -> None:
    tracker: HelipadModeTracker = _make_tracker()

    mode: HelipadMode = tracker.update_mode(0.0, 2.0)

    assert mode == HelipadMode.GUIDANCE
    assert tracker.landed is False


def test_landing_reports_landed_after_debounce() -> None:
    tracker: HelipadModeTracker = _make_tracker()

    tracker.update_mode(0.0, 2.0)
    pending_mode: HelipadMode = tracker.update_mode(0.1, 0.0)
    landed_mode: HelipadMode = tracker.update_mode(0.35, 0.0)

    assert pending_mode == HelipadMode.GUIDANCE
    assert landed_mode == HelipadMode.LANDED
    assert tracker.landed is True


def test_debounce_ignores_brief_reflection() -> None:
    tracker: HelipadModeTracker = _make_tracker()

    tracker.update_mode(0.0, 2.0)
    tracker.update_mode(0.1, 0.0)
    mode: HelipadMode = tracker.update_mode(0.2, 2.0)

    assert mode == HelipadMode.GUIDANCE
    assert tracker.landed is False


def test_takeoff_reports_guidance_after_debounce() -> None:
    tracker: HelipadModeTracker = _make_tracker()

    tracker.update_mode(0.0, 2.0)
    tracker.update_mode(0.1, 0.0)
    tracker.update_mode(0.35, 0.0)

    pending_mode: HelipadMode = tracker.update_mode(0.45, 2.0)
    guidance_mode: HelipadMode = tracker.update_mode(0.7, 2.0)

    assert pending_mode == HelipadMode.LANDED
    assert guidance_mode == HelipadMode.GUIDANCE
    assert tracker.landed is False
