################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for helipad landing guidance logic."""

from __future__ import annotations

import math

from oasis_control.helipad.helipad_controller import HelipadController


def _make_controller() -> HelipadController:
    return HelipadController(
        land_threshold_volts=1.0,
        debounce_secs=0.2,
        guidance_period_secs=2.0,
        max_duty_cycle=0.6,
    )


def test_guidance_alternates_pairs() -> None:
    controller: HelipadController = _make_controller()

    controller.update(0.0, 0.0)
    duty_a_1, duty_b_1 = controller.update(0.5, 0.0)
    duty_a_2, duty_b_2 = controller.update(1.5, 0.0)

    assert math.isclose(duty_a_1, 0.6, rel_tol=1e-6, abs_tol=1e-9)
    assert math.isclose(duty_b_1, 0.0, rel_tol=1e-6, abs_tol=1e-9)
    assert math.isclose(duty_a_2, 0.0, rel_tol=1e-6, abs_tol=1e-9)
    assert math.isclose(duty_b_2, 0.6, rel_tol=1e-6, abs_tol=1e-9)


def test_pwm_bounds_are_clamped() -> None:
    controller: HelipadController = _make_controller()

    controller.update(0.0, 0.0)
    for timestamp_sec in [0.1, 0.4, 0.7, 1.1, 1.6, 1.9]:
        duty_a, duty_b = controller.update(timestamp_sec, 0.0)

        assert 0.0 <= duty_a <= 0.6
        assert 0.0 <= duty_b <= 0.6


def test_landing_turns_both_pairs_off_after_debounce() -> None:
    controller: HelipadController = _make_controller()

    controller.update(0.0, 0.0)
    duty_a_before, duty_b_before = controller.update(0.5, 0.0)
    duty_a_pending, duty_b_pending = controller.update(0.6, 2.0)
    duty_a_after, duty_b_after = controller.update(0.8, 2.0)

    assert duty_a_before > 0.0
    assert math.isclose(duty_b_before, 0.0, rel_tol=1e-6, abs_tol=1e-9)
    assert duty_a_pending > 0.0 or duty_b_pending > 0.0
    assert math.isclose(duty_a_after, 0.0, rel_tol=1e-6, abs_tol=1e-9)
    assert math.isclose(duty_b_after, 0.0, rel_tol=1e-6, abs_tol=1e-9)
    assert controller.landed is True


def test_debounce_ignores_brief_reflection() -> None:
    controller: HelipadController = _make_controller()

    controller.update(0.0, 0.0)
    controller.update(0.5, 0.0)
    controller.update(0.6, 2.0)
    duty_a, duty_b = controller.update(0.75, 0.0)

    assert duty_a > 0.0 or duty_b > 0.0
    assert controller.landed is False


def test_takeoff_restarts_guidance_after_debounce() -> None:
    controller: HelipadController = _make_controller()

    controller.update(0.0, 0.0)
    controller.update(0.5, 0.0)
    controller.update(0.6, 2.0)
    controller.update(0.8, 2.0)

    duty_a_landed, duty_b_landed = controller.update(1.0, 0.0)
    duty_a_restart, duty_b_restart = controller.update(1.21, 0.0)
    duty_a_active, duty_b_active = controller.update(1.46, 0.0)

    assert math.isclose(duty_a_landed, 0.0, rel_tol=1e-6, abs_tol=1e-9)
    assert math.isclose(duty_b_landed, 0.0, rel_tol=1e-6, abs_tol=1e-9)
    assert math.isclose(duty_a_restart, 0.0, rel_tol=1e-6, abs_tol=1e-9)
    assert math.isclose(duty_b_restart, 0.0, rel_tol=1e-6, abs_tol=1e-9)
    assert duty_a_active > 0.0
    assert math.isclose(duty_b_active, 0.0, rel_tol=1e-6, abs_tol=1e-9)
    assert controller.landed is False
