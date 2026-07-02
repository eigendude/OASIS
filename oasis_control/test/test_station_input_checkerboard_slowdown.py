################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for station-input checkerboard slowdown ownership."""

from __future__ import annotations

from typing import cast

import pytest
import rclpy.node

from oasis_control.input.checkerboard_slowdown import CheckerboardCruiseSlowdown
from oasis_control.input.park_mode import TrainParkMode
from oasis_control.input.station_input import MAX_BOOSTED_TRAIN_COMMAND
from oasis_control.input.station_input import MAX_SAFE_MOTOR_DUTY_CYCLE
from oasis_control.input.station_input import StationInput
from oasis_control.lego_models.station_manager import StationManager
from oasis_control.nodes.conductor_manager_telemetrix_node import ConductorManagerNode
from oasis_msgs.msg import AnalogButton as AnalogButtonMsg
from oasis_msgs.msg import DigitalButton as DigitalButtonMsg
from oasis_msgs.msg import PeripheralInput as PeripheralInputMsg


JOYSTICK_ADDRESS: str = "controller"
SLOWDOWN_SCALE: float = 0.65


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


class _Logger:
    def __init__(self) -> None:
        self.info_messages: list[str] = []

    def info(self, message: str) -> None:
        self.info_messages.append(message)


class _ConductorNodeProbe:
    def __init__(self, station_input: StationInput, now_sec: float) -> None:
        self._station_input: StationInput = station_input
        self.now_sec: float = now_sec
        self.logger: _Logger = _Logger()

    def _now_sec(self) -> float:
        return self.now_sec

    def get_logger(self) -> _Logger:
        return self.logger


def _make_slowdown() -> CheckerboardCruiseSlowdown:
    return CheckerboardCruiseSlowdown(
        enabled=True,
        duration_sec=5.0,
        clear_confirm_sec=2.0,
        scale=SLOWDOWN_SCALE,
    )


def _make_station_input() -> tuple[StationInput, _StationManager]:
    node: rclpy.node.Node = rclpy.node.Node("station_input_slowdown_test")
    station_manager: _StationManager = _StationManager()
    station_input: StationInput = StationInput(
        node,
        cast(StationManager, station_manager),
        _make_slowdown(),
        TrainParkMode(enabled=True, command=0.9),
    )
    station_input._joysticks[JOYSTICK_ADDRESS] = "game.controller.default"

    return station_input, station_manager


def _button(name: str, pressed: bool) -> DigitalButtonMsg:
    button: DigitalButtonMsg = DigitalButtonMsg()
    button.name = name
    button.pressed = pressed

    return button


def _trigger(name: str, magnitude: float) -> AnalogButtonMsg:
    trigger: AnalogButtonMsg = AnalogButtonMsg()
    trigger.name = name
    trigger.magnitude = magnitude

    return trigger


def _input_message(
    buttons: list[DigitalButtonMsg],
    analog_buttons: list[AnalogButtonMsg] | None = None,
) -> PeripheralInputMsg:
    message: PeripheralInputMsg = PeripheralInputMsg()
    message.address = JOYSTICK_ADDRESS
    message.digital_buttons = buttons
    message.analog_buttons = analog_buttons if analog_buttons is not None else []

    return message


def _enable_hold_speed(station_input: StationInput) -> None:
    station_input._on_peripheral_input(_input_message([_button("y", True)]))
    station_input._on_peripheral_input(_input_message([_button("y", False)]))


def test_hold_speed_reapplies_full_command_when_slowdown_expires() -> None:
    station_input: StationInput
    station_manager: _StationManager
    station_input, station_manager = _make_station_input()

    _enable_hold_speed(station_input)
    assert station_input.hold_speed

    assert not station_input.update_checkerboard_status(True, 10.0)
    assert station_input.update_checkerboard_status(False, 11.0)

    scaled_pwm: float = MAX_SAFE_MOTOR_DUTY_CYCLE * SLOWDOWN_SCALE
    assert station_manager.pwm == pytest.approx(scaled_pwm)
    assert station_input.consume_checkerboard_slowdown_activation(11.0)

    assert station_input.update_checkerboard_slowdown(16.0)

    assert station_input.hold_speed
    assert station_manager.pwm == pytest.approx(MAX_SAFE_MOTOR_DUTY_CYCLE)
    assert not station_manager.pwm_reverse


def test_periodic_tick_maintains_hold_speed_without_controller_input() -> None:
    station_input: StationInput
    station_manager: _StationManager
    station_input, station_manager = _make_station_input()

    _enable_hold_speed(station_input)
    station_manager.pwm = 0.0

    assert station_input.hold_speed
    assert not station_input.update_autonomous_train_control(10.0)

    assert station_manager.pwm == pytest.approx(MAX_SAFE_MOTOR_DUTY_CYCLE)
    assert not station_manager.pwm_reverse


def test_periodic_tick_restores_full_cruise_after_slowdown() -> None:
    station_input: StationInput
    station_manager: _StationManager
    station_input, station_manager = _make_station_input()

    _enable_hold_speed(station_input)
    assert station_input.hold_speed

    assert not station_input.update_checkerboard_status(True, 10.0)
    assert station_input.update_checkerboard_status(False, 11.0)

    scaled_pwm: float = MAX_SAFE_MOTOR_DUTY_CYCLE * SLOWDOWN_SCALE
    assert station_manager.pwm == pytest.approx(scaled_pwm)
    assert station_input.consume_checkerboard_slowdown_activation(11.0)

    node_probe: _ConductorNodeProbe = _ConductorNodeProbe(station_input, 16.0)
    ConductorManagerNode._update_input_state(cast(ConductorManagerNode, node_probe))

    assert station_input.hold_speed
    assert station_manager.pwm == pytest.approx(MAX_SAFE_MOTOR_DUTY_CYCLE)
    assert not station_manager.pwm_reverse
    assert node_probe.logger.info_messages == [
        "Checkerboard slowdown expired; waiting for train trailing edge"
    ]

    ConductorManagerNode._update_input_state(cast(ConductorManagerNode, node_probe))

    assert node_probe.logger.info_messages == [
        "Checkerboard slowdown expired; waiting for train trailing edge"
    ]


def test_periodic_tick_does_not_reapply_after_manual_cancel() -> None:
    station_input: StationInput
    station_manager: _StationManager
    station_input, station_manager = _make_station_input()

    _enable_hold_speed(station_input)
    assert not station_input.update_checkerboard_status(True, 10.0)
    assert station_input.update_checkerboard_status(False, 11.0)

    scaled_pwm: float = MAX_SAFE_MOTOR_DUTY_CYCLE * SLOWDOWN_SCALE
    assert station_manager.pwm == pytest.approx(scaled_pwm)
    assert station_input.consume_checkerboard_slowdown_activation(11.0)

    manual_command: float = 0.5
    station_input._on_peripheral_input(
        _input_message(
            [_button("a", True)],
            [_trigger("righttrigger", manual_command)],
        )
    )

    manual_pwm: float = MAX_SAFE_MOTOR_DUTY_CYCLE * manual_command
    assert not station_input.hold_speed
    assert station_manager.pwm == pytest.approx(manual_pwm)

    assert not station_input.update_autonomous_train_control(16.0)

    assert not station_input.hold_speed
    assert station_manager.pwm == pytest.approx(manual_pwm)
    assert station_manager.pwm != pytest.approx(MAX_SAFE_MOTOR_DUTY_CYCLE)


def test_periodic_tick_preserves_x_boosted_cruise() -> None:
    station_input: StationInput
    station_manager: _StationManager
    station_input, station_manager = _make_station_input()

    _enable_hold_speed(station_input)
    station_input._on_peripheral_input(_input_message([_button("x", True)]))

    assert station_input.hold_speed
    assert not station_input.update_autonomous_train_control(10.0)

    boosted_pwm: float = MAX_SAFE_MOTOR_DUTY_CYCLE * MAX_BOOSTED_TRAIN_COMMAND
    assert station_manager.pwm == pytest.approx(boosted_pwm)

    station_input._on_peripheral_input(_input_message([_button("x", False)]))

    assert station_input.hold_speed
    assert not station_input.update_autonomous_train_control(11.0)
    assert station_manager.pwm == pytest.approx(MAX_SAFE_MOTOR_DUTY_CYCLE)


def test_x_boost_does_not_cancel_hold_speed() -> None:
    station_input: StationInput
    station_manager: _StationManager
    station_input, station_manager = _make_station_input()

    _enable_hold_speed(station_input)
    assert station_input.hold_speed

    station_input._on_peripheral_input(_input_message([_button("x", True)]))

    boosted_pwm: float = MAX_SAFE_MOTOR_DUTY_CYCLE * MAX_BOOSTED_TRAIN_COMMAND
    assert station_input.hold_speed
    assert station_manager.pwm == pytest.approx(boosted_pwm)

    station_input._on_peripheral_input(_input_message([_button("x", False)]))

    assert station_input.hold_speed
    assert station_manager.pwm == pytest.approx(MAX_SAFE_MOTOR_DUTY_CYCLE)


def test_slowdown_expiration_reapplies_hold_speed_after_x_boost() -> None:
    station_input: StationInput
    station_manager: _StationManager
    station_input, station_manager = _make_station_input()

    _enable_hold_speed(station_input)
    assert not station_input.update_checkerboard_status(True, 10.0)
    assert station_input.update_checkerboard_status(False, 11.0)

    scaled_pwm: float = MAX_SAFE_MOTOR_DUTY_CYCLE * SLOWDOWN_SCALE
    assert station_manager.pwm == pytest.approx(scaled_pwm)
    assert station_input.consume_checkerboard_slowdown_activation(11.0)

    station_input._on_peripheral_input(_input_message([_button("x", True)]))

    boosted_scaled_pwm: float = scaled_pwm * MAX_BOOSTED_TRAIN_COMMAND
    assert station_input.hold_speed
    assert station_manager.pwm == pytest.approx(boosted_scaled_pwm)

    station_input._on_peripheral_input(_input_message([_button("x", False)]))

    assert station_input.hold_speed
    assert station_manager.pwm == pytest.approx(scaled_pwm)

    assert station_input.update_checkerboard_slowdown(16.0)

    assert station_input.hold_speed
    assert station_manager.pwm == pytest.approx(MAX_SAFE_MOTOR_DUTY_CYCLE)
    assert not station_manager.pwm_reverse


def test_slowdown_expiration_does_not_reapply_after_manual_cancel() -> None:
    station_input: StationInput
    station_manager: _StationManager
    station_input, station_manager = _make_station_input()

    _enable_hold_speed(station_input)
    assert not station_input.update_checkerboard_status(True, 10.0)
    assert station_input.update_checkerboard_status(False, 11.0)

    scaled_pwm: float = MAX_SAFE_MOTOR_DUTY_CYCLE * SLOWDOWN_SCALE
    assert station_manager.pwm == pytest.approx(scaled_pwm)
    assert station_input.consume_checkerboard_slowdown_activation(11.0)

    manual_command: float = 0.5
    station_input._on_peripheral_input(
        _input_message(
            [_button("a", True)],
            [_trigger("righttrigger", manual_command)],
        )
    )

    manual_pwm: float = MAX_SAFE_MOTOR_DUTY_CYCLE * manual_command
    assert not station_input.hold_speed
    assert station_manager.pwm == pytest.approx(manual_pwm)

    assert not station_input.update_checkerboard_slowdown(16.0)

    assert not station_input.hold_speed
    assert station_manager.pwm == pytest.approx(manual_pwm)
    assert station_manager.pwm != pytest.approx(MAX_SAFE_MOTOR_DUTY_CYCLE)
