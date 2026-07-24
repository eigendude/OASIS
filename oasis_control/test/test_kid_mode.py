################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for kid-mode train control."""

from __future__ import annotations

from typing import cast

import pytest
import rclpy.node

from oasis_control.input.park_mode import ParkModeLaunchProfile
from oasis_control.input.park_mode import TrainParkMode
from oasis_control.input.station_input import MOTOR_DUTY_CYCLE_PER_VOLT
from oasis_control.input.station_input import NOMINAL_MOTOR_VOLTAGE
from oasis_control.input.station_input import StationInput
from oasis_control.lego_models.station_manager import StationManager
from oasis_msgs.msg import AnalogButton as AnalogButtonMsg
from oasis_msgs.msg import AnalogStick as AnalogStickMsg
from oasis_msgs.msg import BoundingBox as BoundingBoxMsg
from oasis_msgs.msg import CameraScene as CameraSceneMsg
from oasis_msgs.msg import DigitalButton as DigitalButtonMsg
from oasis_msgs.msg import PeripheralInput as PeripheralInputMsg


JOYSTICK_ADDRESS: str = "controller"
NORMAL_DUTY: float = NOMINAL_MOTOR_VOLTAGE * MOTOR_DUTY_CYCLE_PER_VOLT


class _StationManager:
    def __init__(self) -> None:
        self.direction: bool = False
        self.pwm: float = 0.0
        self.events: list[tuple[str, float | bool]] = []

    def set_motor_direction(self, reverse: bool) -> None:
        self.direction = reverse
        self.events.append(("direction", reverse))

    def set_motor_pwm(self, pwm: float, reverse: bool) -> None:
        self.pwm = pwm
        self.events.append(("pwm", pwm))


def _make_station_input() -> tuple[StationInput, TrainParkMode, _StationManager]:
    node: rclpy.node.Node = rclpy.node.Node("kid_mode_test")
    park_mode: TrainParkMode = TrainParkMode(
        True,
        ParkModeLaunchProfile(0.55, 0.2, 0.56, 0.2, 0.2, 0.9, 2.5),
    )
    station_manager: _StationManager = _StationManager()
    station_input: StationInput = StationInput(
        node,
        cast(StationManager, station_manager),
        park_mode,
    )
    station_input._joysticks[JOYSTICK_ADDRESS] = "game.controller.default"
    return station_input, park_mode, station_manager


def _button(name: str, pressed: bool = True) -> DigitalButtonMsg:
    button: DigitalButtonMsg = DigitalButtonMsg()
    button.name = name
    button.pressed = pressed
    return button


def _trigger(name: str, magnitude: float) -> AnalogButtonMsg:
    trigger: AnalogButtonMsg = AnalogButtonMsg()
    trigger.name = name
    trigger.magnitude = magnitude
    return trigger


def _stick(name: str, y: float) -> AnalogStickMsg:
    stick: AnalogStickMsg = AnalogStickMsg()
    stick.name = name
    stick.y = y
    return stick


def _input_message(
    buttons: list[DigitalButtonMsg] | None = None,
    triggers: list[AnalogButtonMsg] | None = None,
    sticks: list[AnalogStickMsg] | None = None,
) -> PeripheralInputMsg:
    message: PeripheralInputMsg = PeripheralInputMsg()
    message.address = JOYSTICK_ADDRESS
    message.digital_buttons = buttons if buttons is not None else []
    message.analog_buttons = triggers if triggers is not None else []
    message.analog_sticks = sticks if sticks is not None else []
    return message


def _toggle_kid_mode(station_input: StationInput) -> None:
    station_input._on_peripheral_input(_input_message([_button("back")]))


def test_select_toggles_only_on_rising_edges_and_stops() -> None:
    station_input, _, station_manager = _make_station_input()
    station_input._apply_train_command(1.0, boost_enabled=False)

    _toggle_kid_mode(station_input)
    assert station_input.kid_mode
    assert station_manager.pwm == 0.0

    _toggle_kid_mode(station_input)
    assert station_input.kid_mode

    station_input._on_peripheral_input(_input_message([_button("back", False)]))
    station_input._on_peripheral_input(
        _input_message(
            [_button("back")],
            sticks=[_stick("leftstick", -1.0)],
        )
    )
    assert not station_input.kid_mode
    assert station_manager.pwm == 0.0


@pytest.mark.parametrize(
    ("stick_name", "stick_y", "expected_command"),
    [
        ("leftstick", -1.0, -1.0),
        ("rightstick", -1.0, -1.0),
        ("leftstick", 1.0, 1.0),
        ("rightstick", 1.0, 1.0),
        ("leftstick", -0.37, -0.37),
        ("rightstick", 0.42, 0.42),
        ("leftstick", -2.0, -1.0),
        ("rightstick", 2.0, 1.0),
    ],
)
def test_each_stick_controls_clamped_normal_range(
    stick_name: str,
    stick_y: float,
    expected_command: float,
) -> None:
    station_input, _, station_manager = _make_station_input()
    _toggle_kid_mode(station_input)
    station_input._on_peripheral_input(
        _input_message(sticks=[_stick(stick_name, stick_y)])
    )

    assert station_manager.pwm == pytest.approx(abs(expected_command) * NORMAL_DUTY)
    assert station_manager.direction == (expected_command < 0.0)


@pytest.mark.parametrize(
    ("left_y", "right_y", "expected_command"),
    [
        (-0.6, 0.6, 0.0),
        (-0.9, 0.4, -0.5),
        (-0.2, 0.8, 0.6),
    ],
)
def test_stick_intents_cancel_by_their_difference(
    left_y: float,
    right_y: float,
    expected_command: float,
) -> None:
    station_input, _, station_manager = _make_station_input()
    _toggle_kid_mode(station_input)
    station_input._on_peripheral_input(
        _input_message(
            sticks=[
                _stick("leftstick", left_y),
                _stick("rightstick", right_y),
            ]
        )
    )

    assert station_manager.pwm == pytest.approx(abs(expected_command) * NORMAL_DUTY)
    assert station_manager.direction == (expected_command < 0.0)


def test_buttons_and_triggers_have_no_effect_in_kid_mode() -> None:
    station_input, park_mode, station_manager = _make_station_input()
    _toggle_kid_mode(station_input)

    station_input._on_peripheral_input(
        _input_message(
            [
                _button("a"),
                _button("b"),
                _button("x"),
                _button("y"),
                _button("start"),
            ],
            [
                _trigger("lefttrigger", 1.0),
                _trigger("righttrigger", 1.0),
            ],
        )
    )

    assert station_manager.pwm == 0.0
    assert not station_input.hold_speed
    assert not park_mode.active
    assert station_input._last_x_button is False


def test_entering_kid_mode_cancels_park_and_cruise() -> None:
    station_input, park_mode, station_manager = _make_station_input()
    assert park_mode.activate(0.0)
    station_input._hold_speed = True

    _toggle_kid_mode(station_input)

    assert not park_mode.active
    assert not station_input.hold_speed
    assert station_manager.pwm == 0.0


def test_camera_scene_cannot_enable_person_cruise() -> None:
    station_input, _, station_manager = _make_station_input()
    _toggle_kid_mode(station_input)
    camera_scene: CameraSceneMsg = CameraSceneMsg()
    bounding_box: BoundingBoxMsg = BoundingBoxMsg()
    bounding_box.x_center = 0.9
    camera_scene.bounding_boxes = [bounding_box]

    station_input.update_camera_scene(camera_scene, 1.0)
    station_input.update_autonomous_train_control()

    assert not station_input.hold_speed
    assert not station_input._person_cruise.active
    assert station_manager.pwm == 0.0
