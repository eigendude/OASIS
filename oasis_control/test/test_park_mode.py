################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for reverse train park mode."""

from __future__ import annotations

from typing import cast

import pytest
import rclpy.node

from oasis_control.input.park_mode import TrainParkMode
from oasis_control.input.station_input import MAX_BOOSTED_TRAIN_COMMAND
from oasis_control.input.station_input import MAX_SAFE_MOTOR_DUTY_CYCLE
from oasis_control.input.station_input import StationInput
from oasis_control.lego_models.station_manager import StationManager
from oasis_msgs.msg import AnalogButton as AnalogButtonMsg
from oasis_msgs.msg import DigitalButton as DigitalButtonMsg
from oasis_msgs.msg import PeripheralInput as PeripheralInputMsg


JOYSTICK_ADDRESS: str = "controller"


class _StationManager:
    def __init__(self) -> None:
        self.direction: bool = False
        self.pwm: float = 0.0
        self.pwm_reverse: bool = False

    def set_motor_direction(self, reverse: bool) -> None:
        self.direction = reverse

    def set_motor_pwm(self, pwm: float, reverse: bool) -> None:
        self.pwm = pwm
        self.pwm_reverse = reverse


def _make_station_input(
    park_mode: TrainParkMode,
) -> tuple[StationInput, _StationManager]:
    node: rclpy.node.Node = rclpy.node.Node("station_input_test")
    station_manager: _StationManager = _StationManager()
    station_input: StationInput = StationInput(
        node,
        cast(StationManager, station_manager),
        park_mode,
    )
    station_input._joysticks[JOYSTICK_ADDRESS] = "game.controller.default"

    return station_input, station_manager


def _button(name: str, pressed: bool) -> DigitalButtonMsg:
    button = DigitalButtonMsg()
    button.name = name
    button.pressed = pressed

    return button


def _trigger(name: str, magnitude: float) -> AnalogButtonMsg:
    trigger = AnalogButtonMsg()
    trigger.name = name
    trigger.magnitude = magnitude

    return trigger


def _input_message(
    buttons: list[DigitalButtonMsg],
    analog_buttons: list[AnalogButtonMsg] | None = None,
) -> PeripheralInputMsg:
    message = PeripheralInputMsg()
    message.address = JOYSTICK_ADDRESS
    message.digital_buttons = buttons
    message.analog_buttons = analog_buttons if analog_buttons is not None else []

    return message


def test_start_enables_park_mode() -> None:
    park_mode: TrainParkMode = TrainParkMode(enabled=True, command=0.9)
    station_input, station_manager = _make_station_input(park_mode)

    station_input._on_peripheral_input(_input_message([_button("start", True)]))

    assert park_mode.active
    assert station_input.reverse
    assert station_input.magnitude == pytest.approx(0.9 * MAX_SAFE_MOTOR_DUTY_CYCLE)
    assert station_manager.pwm_reverse
    assert station_manager.pwm == pytest.approx(0.9 * MAX_SAFE_MOTOR_DUTY_CYCLE)


def test_periodic_tick_maintains_park_mode_output() -> None:
    park_mode: TrainParkMode = TrainParkMode(enabled=True, command=0.9)
    station_input, station_manager = _make_station_input(park_mode)

    station_input._on_peripheral_input(_input_message([_button("start", True)]))
    station_manager.pwm = 0.0
    station_manager.pwm_reverse = False

    station_input.update_autonomous_train_control()

    assert park_mode.active
    assert station_input.reverse
    assert station_manager.pwm_reverse
    assert station_manager.pwm == pytest.approx(0.9 * MAX_SAFE_MOTOR_DUTY_CYCLE)


def test_park_mode_waits_for_visible_to_interrupted_edge() -> None:
    park_mode: TrainParkMode = TrainParkMode(enabled=True, command=0.9)
    station_input, station_manager = _make_station_input(park_mode)

    station_input._on_peripheral_input(_input_message([_button("start", True)]))

    station_input.update_checkerboard_status(False)
    assert park_mode.active
    assert station_manager.pwm > 0.0
    station_input.update_checkerboard_status(True)
    assert park_mode.active

    station_input.update_checkerboard_status(False)
    assert not park_mode.active
    assert station_input.magnitude == 0.0
    assert not station_input.reverse
    assert station_manager.pwm == 0.0
    assert not station_manager.pwm_reverse


def test_b_button_cancels_park_mode_and_stops_train() -> None:
    park_mode: TrainParkMode = TrainParkMode(enabled=True, command=0.9)
    station_input, station_manager = _make_station_input(park_mode)

    station_input._on_peripheral_input(_input_message([_button("start", True)]))
    station_input._on_peripheral_input(
        _input_message(
            [_button("b", True)],
            [_trigger("righttrigger", 1.0)],
        )
    )

    assert not park_mode.active
    assert station_input.magnitude == 0.0
    assert station_manager.pwm == 0.0


@pytest.mark.parametrize("button_name", ["a", "y"])
def test_a_and_y_cancel_park_mode(button_name: str) -> None:
    park_mode: TrainParkMode = TrainParkMode(enabled=True, command=0.9)
    station_input, station_manager = _make_station_input(park_mode)

    station_input._on_peripheral_input(_input_message([_button("start", True)]))
    assert park_mode.active

    station_input._on_peripheral_input(_input_message([_button(button_name, True)]))

    assert not park_mode.active

    if button_name == "a":
        assert not station_input.reverse
        assert not station_manager.pwm_reverse
        assert station_manager.pwm != pytest.approx(0.9 * MAX_SAFE_MOTOR_DUTY_CYCLE)


def test_x_boosts_park_mode_without_cancelling() -> None:
    park_mode: TrainParkMode = TrainParkMode(enabled=True, command=0.9)
    station_input, station_manager = _make_station_input(park_mode)

    station_input._on_peripheral_input(_input_message([_button("start", True)]))
    assert park_mode.active

    station_input._on_peripheral_input(_input_message([_button("x", True)]))

    boosted_pwm: float = 0.9 * MAX_BOOSTED_TRAIN_COMMAND * MAX_SAFE_MOTOR_DUTY_CYCLE
    assert park_mode.active
    assert station_input.reverse
    assert station_manager.pwm_reverse
    assert station_manager.pwm == pytest.approx(boosted_pwm)

    station_input._on_peripheral_input(_input_message([_button("x", False)]))

    assert park_mode.active
    assert station_input.reverse
    assert station_manager.pwm_reverse
    assert station_manager.pwm == pytest.approx(0.9 * MAX_SAFE_MOTOR_DUTY_CYCLE)


def test_periodic_tick_preserves_x_boosted_park_mode() -> None:
    park_mode: TrainParkMode = TrainParkMode(enabled=True, command=0.9)
    station_input, station_manager = _make_station_input(park_mode)

    station_input._on_peripheral_input(_input_message([_button("start", True)]))
    station_input._on_peripheral_input(_input_message([_button("x", True)]))
    station_manager.pwm = 0.0
    station_manager.pwm_reverse = False

    station_input.update_autonomous_train_control()

    boosted_pwm: float = 0.9 * MAX_BOOSTED_TRAIN_COMMAND * MAX_SAFE_MOTOR_DUTY_CYCLE
    assert park_mode.active
    assert station_input.reverse
    assert station_manager.pwm_reverse
    assert station_manager.pwm == pytest.approx(boosted_pwm)


def test_disabled_park_mode_ignores_start() -> None:
    park_mode: TrainParkMode = TrainParkMode(enabled=False, command=0.9)
    station_input, station_manager = _make_station_input(park_mode)

    station_input._on_peripheral_input(_input_message([_button("start", True)]))

    assert not park_mode.active
    assert station_input.magnitude == 0.0
    assert station_manager.pwm == 0.0


def test_park_mode_detects_edge_after_activation() -> None:
    park_mode: TrainParkMode = TrainParkMode(enabled=True, command=0.9)

    assert park_mode.activate()
    assert not park_mode.update_checkerboard_status(False)
    assert not park_mode.update_checkerboard_status(True)
    assert park_mode.update_checkerboard_status(False)
