################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for checkerboard-interruption cruise slowdown state."""

from __future__ import annotations

import pytest

from oasis_control.input.checkerboard_slowdown import CheckerboardCruiseSlowdown


def _make_slowdown(enabled: bool = True) -> CheckerboardCruiseSlowdown:
    return CheckerboardCruiseSlowdown(
        enabled=enabled,
        duration_sec=5.0,
        clear_confirm_sec=2.0,
        scale=0.588,
    )


def test_first_false_status_does_not_trigger_slowdown() -> None:
    slowdown: CheckerboardCruiseSlowdown = _make_slowdown()

    interrupted: bool = slowdown.update_checkerboard_status(False, 10.0)

    assert not interrupted
    assert not slowdown.is_active(10.0)


def test_leading_edge_triggers_slowdown_then_waits_for_clear() -> None:
    slowdown: CheckerboardCruiseSlowdown = _make_slowdown()

    assert not slowdown.update_checkerboard_status(True, 9.0)
    assert slowdown.update_checkerboard_status(False, 10.0)

    assert slowdown.is_slowdown_active(14.99)
    assert not slowdown.is_waiting_for_clear(14.99)
    assert not slowdown.is_slowdown_active(15.0)
    assert slowdown.is_waiting_for_clear(15.0)
    assert slowdown.is_active(25.0)


def test_visible_flicker_does_not_rearm_until_continuously_clear() -> None:
    slowdown: CheckerboardCruiseSlowdown = _make_slowdown()

    slowdown.update_checkerboard_status(True, 9.0)
    slowdown.update_checkerboard_status(False, 10.0)
    assert slowdown.is_waiting_for_clear(15.0)

    slowdown.update_checkerboard_status(True, 15.5)
    slowdown.update_checkerboard_status(False, 16.0)
    slowdown.update_checkerboard_status(True, 16.5)
    assert not slowdown.update_checkerboard_status(False, 17.0)
    assert slowdown.is_waiting_for_clear(17.0)
    slowdown.update_checkerboard_status(True, 17.0)
    assert slowdown.is_waiting_for_clear(18.99)
    slowdown.update_checkerboard_status(True, 19.0)

    assert not slowdown.is_waiting_for_clear(19.0)


def test_rearms_after_continuous_visible_clear_period() -> None:
    slowdown: CheckerboardCruiseSlowdown = _make_slowdown()

    slowdown.update_checkerboard_status(True, 9.0)
    slowdown.update_checkerboard_status(False, 10.0)
    assert slowdown.is_waiting_for_clear(15.0)
    slowdown.update_checkerboard_status(True, 16.0)
    slowdown.update_checkerboard_status(True, 18.0)
    assert not slowdown.is_waiting_for_clear(18.0)
    assert slowdown.update_checkerboard_status(False, 19.0)

    assert slowdown.is_slowdown_active(23.99)


def test_does_not_retrigger_while_waiting_for_clear() -> None:
    slowdown: CheckerboardCruiseSlowdown = _make_slowdown()

    slowdown.update_checkerboard_status(True, 9.0)
    slowdown.update_checkerboard_status(False, 10.0)
    assert slowdown.is_waiting_for_clear(15.0)
    slowdown.update_checkerboard_status(True, 15.2)

    assert not slowdown.update_checkerboard_status(False, 15.7)
    assert not slowdown.update_checkerboard_status(True, 16.2)
    assert not slowdown.update_checkerboard_status(False, 16.7)
    assert not slowdown.is_slowdown_active(16.7)
    assert slowdown.is_waiting_for_clear(16.7)


def test_scale_applies_only_during_slowdown_not_waiting_for_clear() -> None:
    slowdown: CheckerboardCruiseSlowdown = _make_slowdown()

    slowdown.update_checkerboard_status(True, 9.0)
    slowdown.update_checkerboard_status(False, 10.0)

    assert slowdown.scale_command(1.0, True, 12.0) == pytest.approx(0.588)
    assert slowdown.scale_command(1.0, True, 16.0) == pytest.approx(1.0)
    slowdown.update_checkerboard_status(True, 16.0)
    slowdown.update_checkerboard_status(True, 18.0)
    assert slowdown.scale_command(1.0, True, 18.0) == pytest.approx(1.0)


def test_scale_command_expires_and_restores_full_command() -> None:
    slowdown: CheckerboardCruiseSlowdown = _make_slowdown()

    assert not slowdown.update_checkerboard_status(True, 9.0)
    assert slowdown.update_checkerboard_status(False, 10.0)

    scaled_command: float = slowdown.scale_command(1.0, True, 12.0)
    assert scaled_command == pytest.approx(0.588)
    assert slowdown.consume_slowdown_activation(12.0)

    restored_command: float = slowdown.scale_command(1.0, True, 15.0)
    assert restored_command == pytest.approx(1.0)
    assert slowdown.consume_slowdown_expiration(15.0)
    assert not slowdown.consume_slowdown_expiration(15.1)


@pytest.mark.parametrize(
    ("cruise_active", "expected_command"),
    [
        (True, 0.588),
        (False, 1.0),
    ],
)
def test_scale_applies_only_when_cruise_is_active(
    cruise_active: bool,
    expected_command: float,
) -> None:
    slowdown: CheckerboardCruiseSlowdown = _make_slowdown()

    slowdown.update_checkerboard_status(True, 10.0)
    slowdown.update_checkerboard_status(False, 11.0)

    command: float = slowdown.scale_command(1.0, cruise_active, 12.0)

    assert command == pytest.approx(expected_command)


def test_cancel_clears_waiting_for_clear() -> None:
    slowdown: CheckerboardCruiseSlowdown = _make_slowdown()

    slowdown.update_checkerboard_status(True, 9.0)
    slowdown.update_checkerboard_status(False, 10.0)
    assert slowdown.is_waiting_for_clear(15.0)
    assert slowdown.cancel()

    assert not slowdown.is_slowdown_active(15.0)
    assert not slowdown.is_waiting_for_clear(15.0)
    assert slowdown.scale_command(1.0, True, 15.0) == pytest.approx(1.0)
    assert not slowdown.consume_slowdown_expiration(15.0)
    assert not slowdown.consume_rearmed(18.0)


def test_disabled_slowdown_does_not_reduce_command() -> None:
    slowdown: CheckerboardCruiseSlowdown = _make_slowdown(enabled=False)

    slowdown.update_checkerboard_status(True, 10.0)
    interrupted: bool = slowdown.update_checkerboard_status(False, 11.0)
    command: float = slowdown.scale_command(1.0, True, 12.0)

    assert not interrupted
    assert command == 1.0


def test_activation_expiration_and_rearmed_events_are_reported_once() -> None:
    slowdown: CheckerboardCruiseSlowdown = _make_slowdown()

    slowdown.update_checkerboard_status(True, 9.0)
    slowdown.update_checkerboard_status(False, 10.0)

    assert slowdown.consume_slowdown_activation(10.0)
    assert not slowdown.consume_slowdown_activation(12.0)
    assert not slowdown.consume_slowdown_expiration(14.0)
    assert slowdown.consume_slowdown_expiration(15.0)
    assert slowdown.consume_waiting_for_clear_activation(15.0)
    assert not slowdown.consume_waiting_for_clear_activation(16.0)
    slowdown.update_checkerboard_status(True, 16.0)
    assert not slowdown.consume_rearmed(17.0)
    slowdown.update_checkerboard_status(True, 18.0)
    assert slowdown.consume_rearmed(18.0)
    assert not slowdown.consume_rearmed(19.0)
