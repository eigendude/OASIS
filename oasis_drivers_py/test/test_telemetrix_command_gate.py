################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import List

import pytest

from oasis_drivers.telemetrix.command_gate import TelemetrixCommandGate
from oasis_drivers.telemetrix.telemetrix_config_cache import TelemetrixConfigCache
from oasis_drivers.telemetrix.telemetrix_config_store import TelemetrixConfigStore


class _Logger:
    def info(self, message: str) -> None:
        pass

    def warning(self, message: str) -> None:
        pass


class _BlockingHelipadBridge:
    def __init__(
        self, replay_started: threading.Event, replay_release: threading.Event
    ):
        self.commands: List[tuple[object, ...]] = []
        self._replay_started: threading.Event = replay_started
        self._replay_release: threading.Event = replay_release

    def configure_effect(
        self,
        effect_kind: int,
        instance_id: int,
        analog_pins: List[int],
        digital_pins: List[int],
        pwm_pins: List[int],
        config_values: List[float],
    ) -> None:
        self._replay_started.set()
        if not self._replay_release.wait(timeout=2.0):
            raise RuntimeError("test replay release timed out")
        self.commands.append(
            (
                "configure",
                effect_kind,
                instance_id,
                analog_pins,
                digital_pins,
                pwm_pins,
                config_values,
            )
        )

    def set_effect(
        self, effect_kind: int, instance_id: int, mode: int, values: List[float]
    ) -> None:
        self.commands.append(("mode", effect_kind, instance_id, mode, values))


def _helipad_cache(path: Path) -> TelemetrixConfigCache:
    cache: TelemetrixConfigCache = TelemetrixConfigCache(TelemetrixConfigStore(path))
    cache.record_helipad_attach(2, 5, 6)
    cache.record_helipad_mode(1)
    return cache


def test_endpoints_are_created_before_restoration_worker_starts() -> None:
    source: str = (
        Path(__file__).parents[1] / "oasis_drivers/nodes/telemetrix_bridge_node.py"
    ).read_text(encoding="utf-8")
    constructor: str = source[
        source.index("    def __init__") : source.index("    def _on_ping_timer")
    ]

    assert constructor.index("self.create_subscription(") < constructor.index(
        'self._start_reconnect_thread(reason="initialization")'
    )
    assert constructor.index("self.create_service(") < constructor.index(
        'self._start_reconnect_thread(reason="initialization")'
    )
    assert "initialize_with_retries()" not in constructor


def test_command_arriving_during_real_replay_runs_once_afterward(
    tmp_path: Path,
) -> None:
    gate: TelemetrixCommandGate = TelemetrixCommandGate()
    cache: TelemetrixConfigCache = _helipad_cache(tmp_path / "telemetrix.json")
    replay_started: threading.Event = threading.Event()
    replay_release: threading.Event = threading.Event()
    bridge: _BlockingHelipadBridge = _BlockingHelipadBridge(
        replay_started, replay_release
    )
    live_count: int = 0

    def live_command() -> None:
        nonlocal live_count
        live_count += 1
        bridge.set_effect(1, 0, 2, [])

    def restore() -> None:
        cache.replay(bridge, _Logger())  # type: ignore[arg-type]
        gate.restore_complete()

    restore_thread: threading.Thread = threading.Thread(target=restore)
    restore_thread.start()
    assert replay_started.wait(timeout=1.0)
    gate.submit(live_command, wait=False)
    assert live_count == 0

    replay_release.set()
    restore_thread.join(timeout=2.0)

    assert not restore_thread.is_alive()
    assert live_count == 1
    assert bridge.commands[-1] == ("mode", 1, 0, 2, [])


def test_service_during_replay_waits_for_populated_response() -> None:
    gate: TelemetrixCommandGate = TelemetrixCommandGate()
    response: dict[str, int] = {}
    returned: list[object] = []

    def execute() -> dict[str, int]:
        response["value"] = 42
        return response

    service_thread: threading.Thread = threading.Thread(
        target=lambda: returned.append(gate.submit(execute, wait=True))
    )
    service_thread.start()
    service_thread.join(timeout=0.02)
    assert service_thread.is_alive()
    assert response == {}

    gate.restore_complete()
    service_thread.join(timeout=1.0)

    assert returned == [{"value": 42}]


def test_deferred_implementations_do_not_requeue_and_preserve_order() -> None:
    gate: TelemetrixCommandGate = TelemetrixCommandGate()
    commands: List[int] = []

    for value in (1, 2, 3):

        def append_value(command_value: int = value) -> None:
            commands.append(command_value)

        gate.submit(append_value, wait=False)

    applied_count, errors = gate.restore_complete()

    assert applied_count == 3
    assert errors == []
    assert commands == [1, 2, 3]


def test_command_after_readiness_executes_immediately() -> None:
    gate: TelemetrixCommandGate = TelemetrixCommandGate()
    gate.restore_complete()
    commands: List[int] = []

    result: object | None = gate.submit(lambda: commands.append(1), wait=False)

    assert result is None
    assert commands == [1]


def test_deferral_waits_for_active_command_and_queues_new_command() -> None:
    gate: TelemetrixCommandGate = TelemetrixCommandGate()
    gate.restore_complete()
    active_started: threading.Event = threading.Event()
    active_release: threading.Event = threading.Event()
    deferral_started: threading.Event = threading.Event()
    deferral_complete: threading.Event = threading.Event()
    commands: List[str] = []

    def active_command() -> None:
        active_started.set()
        if not active_release.wait(timeout=2.0):
            raise RuntimeError("test active command release timed out")
        commands.append("active")

    def defer() -> None:
        deferral_started.set()
        gate.defer_commands()
        deferral_complete.set()

    active_thread: threading.Thread = threading.Thread(
        target=lambda: gate.submit(active_command, wait=False)
    )
    active_thread.start()
    assert active_started.wait(timeout=1.0)

    defer_thread: threading.Thread = threading.Thread(target=defer)
    defer_thread.start()
    assert deferral_started.wait(timeout=1.0)
    assert not deferral_complete.wait(timeout=0.02)
    deadline: float = time.monotonic() + 1.0
    while gate.ready and time.monotonic() < deadline:
        pass
    assert not gate.ready

    gate.submit(lambda: commands.append("deferred"), wait=False)
    assert commands == []

    active_release.set()
    active_thread.join(timeout=1.0)
    defer_thread.join(timeout=1.0)
    assert not active_thread.is_alive()
    assert not defer_thread.is_alive()
    assert commands == ["active"]

    applied_count, errors = gate.restore_complete()

    assert applied_count == 1
    assert errors == []
    assert commands == ["active", "deferred"]


def test_deferral_barrier_is_released_when_active_command_raises() -> None:
    gate: TelemetrixCommandGate = TelemetrixCommandGate()
    gate.restore_complete()
    active_started: threading.Event = threading.Event()
    active_release: threading.Event = threading.Event()
    command_errors: List[Exception] = []

    def failing_command() -> None:
        active_started.set()
        if not active_release.wait(timeout=2.0):
            raise RuntimeError("test active command release timed out")
        raise RuntimeError("active command failed")

    def submit() -> None:
        try:
            gate.submit(failing_command, wait=False)
        except Exception as exc:
            command_errors.append(exc)

    active_thread: threading.Thread = threading.Thread(target=submit)
    active_thread.start()
    assert active_started.wait(timeout=1.0)

    defer_thread: threading.Thread = threading.Thread(target=gate.defer_commands)
    defer_thread.start()
    defer_thread.join(timeout=0.02)
    assert defer_thread.is_alive()

    active_release.set()
    active_thread.join(timeout=1.0)
    defer_thread.join(timeout=1.0)

    assert not defer_thread.is_alive()
    assert len(command_errors) == 1
    assert str(command_errors[0]) == "active command failed"
    assert not gate.ready


def test_shutdown_releases_waiting_service_and_rejects_new_commands() -> None:
    gate: TelemetrixCommandGate = TelemetrixCommandGate()
    service_errors: List[Exception] = []

    def submit_service() -> None:
        try:
            gate.submit(lambda: {"value": 42}, wait=True)
        except Exception as exc:
            service_errors.append(exc)

    service_thread: threading.Thread = threading.Thread(target=submit_service)
    service_thread.start()
    service_thread.join(timeout=0.02)
    assert service_thread.is_alive()

    failed_count: int = gate.fail_deferred(RuntimeError("bridge shutdown"))
    service_thread.join(timeout=1.0)

    assert failed_count == 1
    assert not service_thread.is_alive()
    assert len(service_errors) == 1
    assert str(service_errors[0]) == "bridge shutdown"
    with pytest.raises(RuntimeError, match="bridge shutdown"):
        gate.submit(lambda: None, wait=False)


def test_repeated_ready_defer_restore_cycles() -> None:
    gate: TelemetrixCommandGate = TelemetrixCommandGate()
    commands: List[int] = []

    for value in range(3):

        def append_immediate(command_value: int = value) -> None:
            commands.append(command_value)

        def append_deferred(command_value: int = value) -> None:
            commands.append(command_value + 10)

        gate.restore_complete()
        assert gate.ready
        gate.submit(append_immediate, False)
        gate.defer_commands()
        assert not gate.ready
        gate.submit(append_deferred, False)
        applied_count, errors = gate.restore_complete()
        assert applied_count == 1
        assert errors == []

    assert commands == [0, 10, 1, 11, 2, 12]


def test_failed_initialization_keeps_commands_deferred_until_reconnect() -> None:
    gate: TelemetrixCommandGate = TelemetrixCommandGate()
    commands: List[str] = []

    gate.submit(lambda: commands.append("live"), wait=False)
    assert commands == []
    assert not gate.ready

    gate.defer_commands()
    gate.restore_complete()

    assert gate.ready
    assert commands == ["live"]


def test_replay_retry_and_command_failure_do_not_leave_gate_blocked() -> None:
    gate: TelemetrixCommandGate = TelemetrixCommandGate()
    commands: List[str] = []

    gate.submit(lambda: commands.append("after retry"), wait=False)
    try:
        raise RuntimeError("replay failed")
    except RuntimeError:
        pass

    assert not gate.ready
    assert commands == []

    gate.restore_complete()
    gate.defer_commands()

    def fail() -> None:
        raise RuntimeError("command failed")

    gate.submit(fail, wait=False)
    applied_count, errors = gate.restore_complete()

    assert applied_count == 1
    assert len(errors) == 1
    assert gate.ready
    assert commands == ["after retry"]


def test_cache_reflects_final_deferred_state(tmp_path: Path) -> None:
    cache_path: Path = tmp_path / "telemetrix.json"
    cache: TelemetrixConfigCache = _helipad_cache(cache_path)
    gate: TelemetrixCommandGate = TelemetrixCommandGate()

    gate.submit(lambda: cache.record_helipad_attach(3, 7, 8), wait=False)
    gate.submit(lambda: cache.record_helipad_mode(2), wait=False)
    gate.restore_complete()

    document: object = json.loads(cache_path.read_text(encoding="utf-8"))
    assert isinstance(document, dict)
    assert document["helipad_config"] == [3, 7, 8]
    assert document["helipad_mode"] == 2
