################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the ROS-side helipad manager."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any
from typing import Callable

import rclpy

from oasis_control.lego_models.helipad_manager import CLIENT_HELIPAD_ATTACH
from oasis_control.lego_models.helipad_manager import CLIENT_SET_ANALOG_MODE
from oasis_control.lego_models.helipad_manager import CLIENT_SET_DIGITAL_MODE
from oasis_control.lego_models.helipad_manager import CLIENT_SET_HELIPAD_MODE
from oasis_control.lego_models.helipad_manager import HelipadManager
from oasis_msgs.msg import AnalogReading as AnalogReadingMsg
from oasis_msgs.msg import HelipadMode as HelipadModeMsg


@dataclass
class _FakeFuture:
    _result: object = object()
    _done_callbacks: list[Callable[[Any], None]] | None = None

    def result(self) -> object:
        return self._result

    def exception(self) -> None:
        return None

    def add_done_callback(self, callback: Callable[[Any], None]) -> None:
        callback(self)


class _FakeClient:
    def __init__(self, name: str) -> None:
        self.name: str = name
        self.wait_calls: int = 0
        self.requests: list[Any] = []

    def wait_for_service(self) -> None:
        self.wait_calls += 1

    def call_async(self, request: Any) -> _FakeFuture:
        self.requests.append(request)
        return _FakeFuture()


class _FakeLogger:
    def debug(self, message: str) -> None:
        del message

    def info(self, message: str) -> None:
        del message

    def warning(self, message: str) -> None:
        del message

    def error(self, message: str) -> None:
        del message


class _FakeClockNow:
    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds: int = nanoseconds


class _FakeClock:
    def __init__(self) -> None:
        self.nanoseconds: int = 0

    def now(self) -> _FakeClockNow:
        return _FakeClockNow(self.nanoseconds)


class _FakeNode:
    def __init__(self) -> None:
        self.clients: dict[str, _FakeClient] = {}
        self.logger: _FakeLogger = _FakeLogger()
        self.clock: _FakeClock = _FakeClock()
        self.publisher_calls: int = 0
        self.timer_calls: int = 0

    def create_client(self, srv_type: Any, srv_name: str) -> _FakeClient:
        del srv_type
        client: _FakeClient = _FakeClient(srv_name)
        self.clients[srv_name] = client
        return client

    def create_subscription(
        self,
        msg_type: Any,
        topic: str,
        callback: Callable[[Any], None],
        qos_profile: Any,
    ) -> object:
        del msg_type
        del topic
        del callback
        del qos_profile
        return object()

    def create_publisher(self, msg_type: Any, topic: str, qos_profile: Any) -> object:
        del msg_type
        del topic
        del qos_profile
        self.publisher_calls += 1
        return object()

    def create_timer(
        self, timer_period_sec: float, callback: Callable[[], None]
    ) -> object:
        del timer_period_sec
        del callback
        self.timer_calls += 1
        return object()

    def get_logger(self) -> _FakeLogger:
        return self.logger

    def get_clock(self) -> _FakeClock:
        return self.clock


def _make_reading(analog_pin: int, analog_value: float) -> AnalogReadingMsg:
    reading: AnalogReadingMsg = AnalogReadingMsg()
    reading.analog_pin = analog_pin
    reading.analog_value = analog_value
    reading.reference_voltage = 5.0
    return reading


def test_initialize_uses_services_and_not_pwm_animation(monkeypatch: Any) -> None:
    monkeypatch.setattr(rclpy, "spin_until_future_complete", lambda node, future: None)

    node: _FakeNode = _FakeNode()
    manager: HelipadManager = HelipadManager(node, 2, 10, 11)

    assert manager.initialize() is True

    assert node.publisher_calls == 0
    assert node.timer_calls == 0
    assert node.clients[CLIENT_HELIPAD_ATTACH].wait_calls == 1
    assert node.clients[CLIENT_SET_ANALOG_MODE].wait_calls == 1
    assert node.clients[CLIENT_SET_DIGITAL_MODE].wait_calls == 1
    assert node.clients[CLIENT_SET_HELIPAD_MODE].wait_calls == 1
    assert len(node.clients[CLIENT_HELIPAD_ATTACH].requests) == 1
    assert len(node.clients[CLIENT_SET_ANALOG_MODE].requests) == 1
    assert len(node.clients[CLIENT_SET_DIGITAL_MODE].requests) == 2
    assert len(node.clients[CLIENT_SET_HELIPAD_MODE].requests) == 0


def test_guidance_mode_is_sent_once_for_repeated_active_readings(
    monkeypatch: Any,
) -> None:
    monkeypatch.setattr(rclpy, "spin_until_future_complete", lambda node, future: None)

    node: _FakeNode = _FakeNode()
    manager: HelipadManager = HelipadManager(node, 2, 10, 11)

    assert manager.initialize() is True

    monkeypatch.setattr(
        rclpy,
        "spin_until_future_complete",
        lambda node, future: (_ for _ in ()).throw(
            AssertionError("callback path must not spin")
        ),
    )

    node.clock.nanoseconds = 0
    manager._on_analog_reading(_make_reading(2, 1.0))

    node.clock.nanoseconds = int(0.1 * 1e9)
    manager._on_analog_reading(_make_reading(2, 1.0))

    mode_requests: list[Any] = node.clients[CLIENT_SET_HELIPAD_MODE].requests

    assert len(mode_requests) == 1
    assert mode_requests[0].mode == HelipadModeMsg.GUIDANCE


def test_landed_mode_is_sent_after_debounce(monkeypatch: Any) -> None:
    monkeypatch.setattr(rclpy, "spin_until_future_complete", lambda node, future: None)

    node: _FakeNode = _FakeNode()
    manager: HelipadManager = HelipadManager(node, 2, 10, 11)

    assert manager.initialize() is True

    node.clock.nanoseconds = 0
    manager._on_analog_reading(_make_reading(2, 1.0))

    node.clock.nanoseconds = int(0.05 * 1e9)
    manager._on_analog_reading(_make_reading(2, 0.0))

    node.clock.nanoseconds = int(0.30 * 1e9)
    manager._on_analog_reading(_make_reading(2, 0.0))

    mode_requests: list[Any] = node.clients[CLIENT_SET_HELIPAD_MODE].requests

    assert [request.mode for request in mode_requests] == [
        HelipadModeMsg.GUIDANCE,
        HelipadModeMsg.LANDED,
    ]


def test_analog_reading_during_initialize_does_not_set_mode(monkeypatch: Any) -> None:
    monkeypatch.setattr(rclpy, "spin_until_future_complete", lambda node, future: None)

    node: _FakeNode = _FakeNode()
    manager: HelipadManager = HelipadManager(node, 2, 10, 11)
    manager._initializing = True

    node.clock.nanoseconds = 0
    manager._on_analog_reading(_make_reading(2, 1.0))

    assert len(node.clients[CLIENT_SET_HELIPAD_MODE].requests) == 0
