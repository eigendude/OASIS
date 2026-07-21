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

from oasis_control.input.park_mode import ParkModeLaunchProfile
from oasis_control.input.park_mode import TrainParkMode
from oasis_control.input.station_input import MAX_BOOSTED_TRAIN_COMMAND
from oasis_control.input.station_input import MAX_SAFE_MOTOR_DUTY_CYCLE
from oasis_control.input.station_input import StationInput
from oasis_control.lego_models.station_manager import StationManager
from oasis_control.nodes.conductor_manager_telemetrix_node import (
    DEFAULT_PARK_MODE_PROFILE,
)
from oasis_control.nodes.conductor_manager_telemetrix_node import (
    _validated_park_mode_profile,
)
from oasis_msgs.msg import AnalogButton as AnalogButtonMsg
from oasis_msgs.msg import DigitalButton as DigitalButtonMsg
from oasis_msgs.msg import PeripheralInput as PeripheralInputMsg


JOYSTICK_ADDRESS: str = "controller"


def _profile() -> ParkModeLaunchProfile:
    return ParkModeLaunchProfile(0.55, 0.2, 0.56, 0.2, 0.2, 0.9, 2.5)


class _StationManager:
    def __init__(self) -> None:
        self.direction: bool = False
        self.pwm: float = 0.0
        self.pwm_reverse: bool = False
        self.events: list[tuple[str, float | bool]] = []

    def set_motor_direction(self, reverse: bool) -> None:
        self.direction = reverse
        self.events.append(("direction", reverse))

    def set_motor_pwm(self, pwm: float, reverse: bool) -> None:
        self.pwm = pwm
        self.pwm_reverse = reverse
        self.events.append(("pwm", pwm))


class _TestStationInput(StationInput):
    def __init__(
        self,
        node: rclpy.node.Node,
        station_manager: StationManager,
        park_mode: TrainParkMode,
    ) -> None:
        self.now_sec: float = 0.0
        super().__init__(node, station_manager, park_mode)

    def _now_sec(self) -> float:
        return self.now_sec


def _make_station_input(
    park_mode: TrainParkMode,
) -> tuple[_TestStationInput, _StationManager]:
    node: rclpy.node.Node = rclpy.node.Node("station_input_test")
    station_manager: _StationManager = _StationManager()
    station_input: _TestStationInput = _TestStationInput(
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


def test_start_begins_park_mode_at_zero_in_reverse() -> None:
    park_mode: TrainParkMode = TrainParkMode(True, _profile())
    station_input, station_manager = _make_station_input(park_mode)

    station_input._on_peripheral_input(_input_message([_button("start", True)]))

    assert park_mode.active
    assert station_input.reverse
    assert station_input.magnitude == 0.0
    assert station_manager.pwm_reverse
    assert station_manager.pwm == 0.0
    assert station_manager.events[-2:] == [("direction", True), ("pwm", 0.0)]


def test_piecewise_park_mode_profile() -> None:
    park_mode: TrainParkMode = TrainParkMode(True, _profile())
    station_input, station_manager = _make_station_input(park_mode)

    station_input._on_peripheral_input(_input_message([_button("start", True)]))
    station_input.now_sec = 0.2
    station_input.update_autonomous_train_control()
    assert station_input.magnitude == pytest.approx(0.55 * MAX_SAFE_MOTOR_DUTY_CYCLE)

    station_input.now_sec = 0.4
    station_input.update_autonomous_train_control()
    assert station_input.magnitude == pytest.approx(0.56 * MAX_SAFE_MOTOR_DUTY_CYCLE)

    station_input.now_sec = 0.6
    station_input.update_autonomous_train_control()
    assert station_input.magnitude == pytest.approx(0.56 * MAX_SAFE_MOTOR_DUTY_CYCLE)

    acceleration_samples: tuple[tuple[float, float], ...] = (
        (1.1, 0.628),
        (1.6, 0.696),
        (2.1, 0.764),
        (2.6, 0.832),
        (3.1, 0.90),
    )
    for timestamp_sec, expected_command in acceleration_samples:
        station_input.now_sec = timestamp_sec
        station_input.update_autonomous_train_control()
        assert station_input.magnitude == pytest.approx(
            expected_command * MAX_SAFE_MOTOR_DUTY_CYCLE
        )


def test_repeated_callbacks_at_same_time_do_not_advance_ramp() -> None:
    park_mode: TrainParkMode = TrainParkMode(True, _profile())
    station_input, station_manager = _make_station_input(park_mode)
    station_input._on_peripheral_input(_input_message([_button("start", True)]))
    station_input.now_sec = 0.6

    for _ in range(5):
        station_input.update_autonomous_train_control()

    assert station_manager.pwm == pytest.approx(0.56 * MAX_SAFE_MOTOR_DUTY_CYCLE)


def test_zero_duration_hold_transitions_directly_to_acceleration() -> None:
    profile: ParkModeLaunchProfile = ParkModeLaunchProfile(
        0.55, 0.2, 0.62, 0.4, 0.0, 0.9, 0.5
    )
    park_mode: TrainParkMode = TrainParkMode(True, profile)
    assert park_mode.activate(0.0)

    assert park_mode.limit_command(0.9, 0.6) == pytest.approx(0.62)
    assert park_mode.limit_command(0.9, 0.85) == pytest.approx(0.76)
    assert park_mode.limit_command(0.9, 1.1) == pytest.approx(0.9)


def test_x_boost_does_not_bypass_launch_hold() -> None:
    park_mode: TrainParkMode = TrainParkMode(True, _profile())
    boosted_target: float = 0.9 * MAX_BOOSTED_TRAIN_COMMAND
    assert park_mode.activate(0.0)
    assert park_mode.limit_command(boosted_target, 0.0) == 0.0

    assert park_mode.limit_command(boosted_target, 0.4) == pytest.approx(0.56)
    assert park_mode.limit_command(boosted_target, 0.6) == pytest.approx(0.56)
    assert park_mode.limit_command(boosted_target, 1.1) == pytest.approx(0.628)
    assert park_mode.limit_command(boosted_target, 1.6) == pytest.approx(0.696)
    assert park_mode.limit_command(boosted_target, 2.1) == pytest.approx(0.764)
    assert park_mode.limit_command(boosted_target, 2.6) == pytest.approx(0.832)
    assert park_mode.limit_command(boosted_target, 3.1) == pytest.approx(0.9)


def test_park_mode_waits_for_visible_to_interrupted_edge() -> None:
    park_mode: TrainParkMode = TrainParkMode(True, _profile())
    station_input, station_manager = _make_station_input(park_mode)

    station_input._on_peripheral_input(_input_message([_button("start", True)]))
    station_input.now_sec = 0.5
    station_input.update_autonomous_train_control()

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
    park_mode: TrainParkMode = TrainParkMode(True, _profile())
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
    park_mode: TrainParkMode = TrainParkMode(True, _profile())
    station_input, station_manager = _make_station_input(park_mode)

    station_input._on_peripheral_input(_input_message([_button("start", True)]))
    assert park_mode.active

    station_input._on_peripheral_input(_input_message([_button(button_name, True)]))

    assert not park_mode.active
    assert station_input.magnitude == 0.0
    assert not station_input.reverse
    assert station_manager.pwm == 0.0
    assert not station_manager.pwm_reverse


def test_x_boost_is_slew_limited() -> None:
    park_mode: TrainParkMode = TrainParkMode(True, _profile())
    station_input, station_manager = _make_station_input(park_mode)

    station_input._on_peripheral_input(_input_message([_button("start", True)]))
    station_input.now_sec = 3.1
    station_input.update_autonomous_train_control()
    unboosted_pwm: float = 0.9 * MAX_SAFE_MOTOR_DUTY_CYCLE
    assert station_manager.pwm == pytest.approx(unboosted_pwm)

    station_input._on_peripheral_input(_input_message([_button("x", True)]))
    assert station_manager.pwm == pytest.approx(unboosted_pwm)

    station_input.now_sec = 3.4
    station_input.update_autonomous_train_control()
    expected_slew_command: float = 0.9 + 0.3 * (0.9 - 0.56) / 2.5
    assert station_manager.pwm == pytest.approx(
        expected_slew_command * MAX_SAFE_MOTOR_DUTY_CYCLE
    )

    station_input.now_sec = 4.8
    station_input.update_autonomous_train_control()
    boosted_pwm: float = 0.9 * MAX_BOOSTED_TRAIN_COMMAND * MAX_SAFE_MOTOR_DUTY_CYCLE
    assert station_manager.pwm == pytest.approx(boosted_pwm)


def test_disabled_park_mode_ignores_start() -> None:
    park_mode: TrainParkMode = TrainParkMode(False, _profile())
    station_input, station_manager = _make_station_input(park_mode)

    station_input._on_peripheral_input(_input_message([_button("start", True)]))

    assert not park_mode.active
    assert station_input.magnitude == 0.0
    assert station_manager.pwm == 0.0


def test_park_mode_detects_edge_after_activation() -> None:
    park_mode: TrainParkMode = TrainParkMode(True, _profile())

    assert park_mode.activate(0.0)
    assert not park_mode.update_checkerboard_status(False)
    assert not park_mode.update_checkerboard_status(True)
    assert park_mode.update_checkerboard_status(False)


def test_reactivation_starts_a_new_ramp() -> None:
    park_mode: TrainParkMode = TrainParkMode(True, _profile())
    station_input, station_manager = _make_station_input(park_mode)
    station_input._on_peripheral_input(_input_message([_button("start", True)]))
    station_input.now_sec = 1.0
    station_input.update_autonomous_train_control()
    assert station_manager.pwm > 0.0

    station_input._on_peripheral_input(_input_message([_button("b", True)]))
    station_input._on_peripheral_input(_input_message([_button("start", False)]))
    station_input.now_sec = 2.0
    station_input._on_peripheral_input(_input_message([_button("start", True)]))

    assert park_mode.active
    assert station_input.reverse
    assert station_manager.pwm == 0.0


def test_backward_time_restarts_ramp_safely() -> None:
    park_mode: TrainParkMode = TrainParkMode(True, _profile())
    assert park_mode.activate(10.0)
    assert park_mode.limit_command(0.9, 10.2) == pytest.approx(0.55)

    assert park_mode.limit_command(0.9, 9.0) == 0.0
    assert park_mode.limit_command(0.9, 9.2) == pytest.approx(0.55)


def test_start_from_forward_cruise_runs_complete_reverse_profile() -> None:
    park_mode: TrainParkMode = TrainParkMode(True, _profile())
    station_input, station_manager = _make_station_input(park_mode)
    station_input._on_peripheral_input(
        _input_message(
            [_button("a", True)],
            [_trigger("righttrigger", 1.0)],
        )
    )
    assert not station_input.reverse
    assert station_input.magnitude == pytest.approx(MAX_SAFE_MOTOR_DUTY_CYCLE)

    station_input._on_peripheral_input(_input_message([_button("start", True)]))
    assert station_input.reverse
    assert station_manager.pwm == 0.0
    assert station_manager.events[-2:] == [("direction", True), ("pwm", 0.0)]

    expected_commands: tuple[tuple[float, float], ...] = (
        (0.2, 0.55),
        (0.4, 0.56),
        (0.6, 0.56),
        (1.1, 0.628),
        (1.6, 0.696),
        (2.1, 0.764),
        (2.6, 0.832),
        (3.1, 0.90),
    )
    for timestamp_sec, expected_command in expected_commands:
        station_input.now_sec = timestamp_sec
        station_input.update_autonomous_train_control()
        assert station_manager.pwm == pytest.approx(
            expected_command * MAX_SAFE_MOTOR_DUTY_CYCLE
        )
        assert station_manager.pwm_reverse


@pytest.mark.parametrize(
    "invalid_profile",
    [
        ParkModeLaunchProfile(0.0, 0.2, 0.62, 0.4, 1.75, 0.9, 0.5),
        ParkModeLaunchProfile(float("nan"), 0.2, 0.62, 0.4, 1.75, 0.9, 0.5),
        ParkModeLaunchProfile(0.55, float("inf"), 0.62, 0.4, 1.75, 0.9, 0.5),
        ParkModeLaunchProfile(0.55, 0.2, 0.55, 0.4, 1.75, 0.9, 0.5),
        ParkModeLaunchProfile(0.75, 0.2, 0.62, 0.4, 1.75, 0.9, 0.5),
        ParkModeLaunchProfile(0.55, 0.2, 0.95, 0.4, 1.75, 0.9, 0.5),
        ParkModeLaunchProfile(0.55, 0.2, 0.62, -1.0, 1.75, 0.9, 0.5),
        ParkModeLaunchProfile(0.55, 0.2, 0.62, 0.4, -0.1, 0.9, 0.5),
        ParkModeLaunchProfile(0.55, 0.2, 0.62, 0.4, float("nan"), 0.9, 0.5),
        ParkModeLaunchProfile(0.55, 0.2, 0.62, 0.4, float("inf"), 0.9, 0.5),
        ParkModeLaunchProfile(0.55, 0.2, 0.62, 0.4, 1.75, 1.1, 0.5),
    ],
)
def test_invalid_profile_uses_complete_default(
    invalid_profile: ParkModeLaunchProfile,
) -> None:
    assert _validated_park_mode_profile(invalid_profile) is DEFAULT_PARK_MODE_PROFILE


def test_zero_hold_duration_is_valid() -> None:
    profile: ParkModeLaunchProfile = ParkModeLaunchProfile(
        0.55, 0.2, 0.62, 0.4, 0.0, 0.9, 0.5
    )
    assert _validated_park_mode_profile(profile) is profile
