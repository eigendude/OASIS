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

import pytest

from oasis_control.lego_models.station_manager import StationManager


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

    return manager


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
