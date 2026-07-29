################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Serialize live commands behind Telemetrix state restoration."""

from __future__ import annotations

import threading
from collections import deque
from collections.abc import Callable
from typing import Deque


class _DeferredCommand:
    def __init__(self, command: Callable[[], object]) -> None:
        self.command: Callable[[], object] = command
        self.completed: threading.Event = threading.Event()
        self.result: object | None = None
        self.error: Exception | None = None


class TelemetrixCommandGate:
    """Defer commands until the Telemetrix bridge is ready."""

    def __init__(self) -> None:
        self._condition: threading.Condition = threading.Condition()
        self._ready: bool = False
        self._active_count: int = 0
        self._stopped_error: Exception | None = None
        self._commands: Deque[_DeferredCommand] = deque()

    @property
    def ready(self) -> bool:
        """Return whether commands execute immediately."""

        with self._condition:
            return self._ready

    def defer_commands(self) -> None:
        """Defer new commands and wait for admitted commands to finish."""

        with self._condition:
            self._ready = False
            while self._active_count > 0:
                self._condition.wait()

    def submit(self, command: Callable[[], object], wait: bool) -> object | None:
        """Execute a ready command or defer it until restoration completes."""

        deferred: _DeferredCommand | None = None
        execute_immediately: bool = False
        with self._condition:
            if self._stopped_error is not None:
                raise self._stopped_error
            if not self._ready:
                deferred = _DeferredCommand(command)
                self._commands.append(deferred)
            else:
                self._active_count += 1
                execute_immediately = True

        if execute_immediately:
            try:
                return command()
            finally:
                with self._condition:
                    self._active_count -= 1
                    self._condition.notify_all()

        assert deferred is not None
        if not wait:
            return None

        deferred.completed.wait()
        if deferred.error is not None:
            raise deferred.error
        return deferred.result

    def restore_complete(self) -> tuple[int, list[Exception]]:
        """Execute deferred work in arrival order, then enter ready state."""

        applied_count: int = 0
        errors: list[Exception] = []

        while True:
            with self._condition:
                if self._stopped_error is not None:
                    break
                if not self._commands:
                    self._ready = True
                    break
                deferred: _DeferredCommand = self._commands.popleft()
                self._active_count += 1

            try:
                deferred.result = deferred.command()
            except Exception as exc:
                deferred.error = exc
                errors.append(exc)
            finally:
                applied_count += 1
                deferred.completed.set()
                with self._condition:
                    self._active_count -= 1
                    self._condition.notify_all()

        return applied_count, errors

    def fail_deferred(self, error: Exception) -> int:
        """Stop the gate and fail all deferred commands."""

        failed_count: int = 0
        with self._condition:
            self._ready = False
            if self._stopped_error is None:
                self._stopped_error = error
            stopped_error: Exception = self._stopped_error
            commands: list[_DeferredCommand] = list(self._commands)
            self._commands.clear()

            for deferred in commands:
                deferred.error = stopped_error
                deferred.completed.set()
                failed_count += 1

            while self._active_count > 0:
                self._condition.wait()

        return failed_count
