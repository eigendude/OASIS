################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for LEGO station telemetry state handling."""

from __future__ import annotations

import math
from typing import Any

import pytest

from oasis_control.lego_models.station_manager import StationManager


class _Publisher:
    def __init__(self) -> None:
        self.messages: list[Any] = []

    def publish(self, message: Any) -> None:
        self.messages.append(message)


def _make_station_manager(motor_voltage_reversed: bool = False) -> StationManager:
    manager: StationManager = StationManager.__new__(StationManager)

    manager._motor_voltage_a = 0.0
    manager._motor_voltage_a_initialized = True
    manager._motor_voltage_a_timestamp_sec = 1.0
    manager._prev_motor_voltage_a = None
    manager._prev_motor_voltage_a_timestamp_sec = None
    manager._motor_voltage_b = 0.0
    manager._motor_voltage_b_initialized = True
    manager._motor_voltage_b_timestamp_sec = 1.0
    manager._prev_motor_voltage_b = None
    manager._prev_motor_voltage_b_timestamp_sec = None
    manager._motor_voltage = 0.0
    manager._motor_voltage_mu = 0.0
    manager._motor_voltage_base_var = 0.0
    manager._motor_voltage_initialized = False
    manager._motor_voltage_stddev = 0.0
    manager._motor_voltage_reversed = motor_voltage_reversed
    manager._last_motor_pwm_cmd = None

    return manager


def _make_pwm_station_manager() -> tuple[StationManager, _Publisher, _Publisher]:
    manager: StationManager = StationManager.__new__(StationManager)
    pwm_publisher: _Publisher = _Publisher()
    direction_publisher: _Publisher = _Publisher()

    manager._motor_pwm_pin = 5
    manager._motor_dir_pin = 6
    manager._motor_duty_cycle = 0.0
    manager._last_motor_pwm_cmd = None
    manager_any: Any = manager
    manager_any._motor_pwm_cmd_pub = pwm_publisher
    manager_any._motor_dir_cmd_pub = direction_publisher

    return manager, pwm_publisher, direction_publisher


def _update_motor_voltage(
    manager: StationManager,
    measured_motor_voltage: float,
    timestamp_sec: float = 1.0,
) -> float:
    manager._motor_voltage_a = measured_motor_voltage
    manager._motor_voltage_a_timestamp_sec = timestamp_sec
    manager._motor_voltage_b = 0.0
    manager._motor_voltage_b_timestamp_sec = timestamp_sec

    manager._update_motor_voltage()

    return manager.motor_voltage


@pytest.mark.parametrize(
    ("measured_motor_voltage", "expected_motor_voltage"),
    [
        (0.001, 0.0),
        (-0.001, 0.0),
        (0.10, 0.10),
        (-0.10, -0.10),
    ],
)
def test_motor_voltage_canonicalizes_telemetry_zero(
    measured_motor_voltage: float,
    expected_motor_voltage: float,
) -> None:
    manager: StationManager = _make_station_manager()

    motor_voltage: float = _update_motor_voltage(manager, measured_motor_voltage)

    assert motor_voltage == pytest.approx(expected_motor_voltage)
    if expected_motor_voltage == 0.0:
        assert math.copysign(1.0, motor_voltage) == 1.0


def test_reversed_tiny_positive_measured_motor_voltage_reports_positive_zero() -> None:
    manager: StationManager = _make_station_manager(motor_voltage_reversed=True)

    motor_voltage: float = _update_motor_voltage(manager, 0.001)

    assert motor_voltage == 0.0
    assert math.copysign(1.0, motor_voltage) == 1.0


def test_motor_voltage_stddev_uses_canonical_zero() -> None:
    manager: StationManager = _make_station_manager()

    _update_motor_voltage(manager, 0.001, timestamp_sec=1.0)
    _update_motor_voltage(manager, -0.001, timestamp_sec=2.0)

    assert manager.motor_voltage_stddev == 0.0


def test_back_to_back_duplicate_motor_pwm_publishes_once() -> None:
    manager, pwm_publisher, _ = _make_pwm_station_manager()

    manager.set_motor_pwm(0.5, False)
    manager.set_motor_pwm(0.5, False)

    assert len(pwm_publisher.messages) == 1
    assert pwm_publisher.messages[0].digital_pin == 5
    assert pwm_publisher.messages[0].duty_cycle == pytest.approx(0.5)
    assert manager.motor_duty_cycle == pytest.approx(0.5)


def test_motor_direction_side_effect_resets_duplicate_pwm_cache() -> None:
    manager, pwm_publisher, direction_publisher = _make_pwm_station_manager()

    manager.set_motor_pwm(0.5, False)
    manager.set_motor_direction(True)
    manager.set_motor_pwm(0.5, True)

    assert len(pwm_publisher.messages) == 2
    assert len(direction_publisher.messages) == 1
    assert manager.motor_duty_cycle == pytest.approx(-0.5)


def test_opposite_reverse_motor_pwm_republishes_same_magnitude() -> None:
    manager, pwm_publisher, _ = _make_pwm_station_manager()

    manager.set_motor_pwm(0.5, False)
    manager.set_motor_pwm(0.5, True)

    assert len(pwm_publisher.messages) == 2
    assert pwm_publisher.messages[0].duty_cycle == pytest.approx(0.5)
    assert pwm_publisher.messages[1].duty_cycle == pytest.approx(0.5)
    assert manager.motor_duty_cycle == pytest.approx(-0.5)
