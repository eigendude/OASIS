################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for person-triggered train cruise."""

from __future__ import annotations

from oasis_control.input.person_cruise import PERSON_CRUISE_LOST_TIMEOUT_SEC
from oasis_control.input.person_cruise import PERSON_CRUISE_RIGHT_THIRD_MIN_X
from oasis_control.input.person_cruise import PersonCruise


def test_right_third_presence_activates_and_loss_deactivates_cruise() -> None:
    person_cruise: PersonCruise = PersonCruise()

    assert (
        person_cruise.update(
            [PERSON_CRUISE_RIGHT_THIRD_MIN_X],
            now_sec=1.0,
            activation_allowed=True,
        )
        is True
    )
    assert person_cruise.active

    assert person_cruise.update([], now_sec=1.25, activation_allowed=True) is None
    assert person_cruise.active

    assert (
        person_cruise.update(
            [],
            now_sec=1.0 + PERSON_CRUISE_LOST_TIMEOUT_SEC,
            activation_allowed=True,
        )
        is False
    )
    assert not person_cruise.active


def test_presence_outside_right_third_does_not_activate_cruise() -> None:
    person_cruise: PersonCruise = PersonCruise()

    assert (
        person_cruise.update(
            [PERSON_CRUISE_RIGHT_THIRD_MIN_X - 0.001],
            now_sec=1.0,
            activation_allowed=True,
        )
        is None
    )
    assert not person_cruise.active


def test_disallowed_or_cancelled_presence_requires_reentry() -> None:
    person_cruise: PersonCruise = PersonCruise()

    assert person_cruise.update([0.9], now_sec=1.0, activation_allowed=False) is None
    assert person_cruise.update([0.9], now_sec=1.1, activation_allowed=True) is None

    assert (
        person_cruise.update(
            [],
            now_sec=1.1 + PERSON_CRUISE_LOST_TIMEOUT_SEC,
            activation_allowed=True,
        )
        is None
    )
    assert person_cruise.update([0.9], now_sec=2.0, activation_allowed=True) is True

    person_cruise.cancel()
    assert not person_cruise.active
    assert person_cruise.update([0.9], now_sec=2.1, activation_allowed=True) is None
