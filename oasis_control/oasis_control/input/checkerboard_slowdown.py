################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Checkerboard-triggered cruise slowdown state."""

from __future__ import annotations

from enum import Enum
from enum import auto
from typing import Optional


class CheckerboardSlowdownState(Enum):
    """Checkerboard slowdown trigger state."""

    ARMED = auto()
    SLOWDOWN = auto()
    WAITING_FOR_CLEAR = auto()


class CheckerboardCruiseSlowdown:
    """
    Tracks checkerboard-triggered slowdown state for train cruise mode.

    A visible -> interrupted transition marks the train's leading edge and starts
    a slowdown window.

    After slowdown, the state waits for the checkerboard to stay visible for the
    configured clear-confirmation time before re-arming.

    This avoids retriggers from gaps between cars. The class only tracks state
    and scales caller-provided cruise commands.
    """

    def __init__(
        self,
        enabled: bool,
        duration_sec: float,
        clear_confirm_sec: float,
        scale: float,
    ) -> None:
        """
        Initialize slowdown configuration.
        """
        self._enabled: bool = enabled
        self._duration_sec: float = duration_sec
        self._clear_confirm_sec: float = clear_confirm_sec
        self._scale: float = scale
        self._state: CheckerboardSlowdownState = CheckerboardSlowdownState.ARMED
        self._have_checkerboard_status: bool = False
        self._last_checkerboard_visible: bool = False
        self._slowdown_until_time_sec: Optional[float] = None
        self._clear_since_sec: Optional[float] = None
        self._was_slowdown_active: bool = False
        self._was_waiting_for_clear: bool = False

    @property
    def checkerboard_visible(self) -> Optional[bool]:
        return (
            self._last_checkerboard_visible if self._have_checkerboard_status else None
        )

    def update_checkerboard_status(
        self,
        checkerboard_visible: bool,
        now_sec: float,
    ) -> bool:
        self.update_time(now_sec)

        if not self._have_checkerboard_status:
            self._have_checkerboard_status = True
            self._last_checkerboard_visible = checkerboard_visible
            return False

        interrupted: bool = (
            self._enabled
            and self._state == CheckerboardSlowdownState.ARMED
            and self._last_checkerboard_visible
            and not checkerboard_visible
        )

        if interrupted:
            self._slowdown_until_time_sec = now_sec + self._duration_sec
            self._clear_since_sec = None
            self._was_slowdown_active = False
            self._was_waiting_for_clear = False

            if self._duration_sec > 0.0:
                self._state = CheckerboardSlowdownState.SLOWDOWN
            else:
                self._state = CheckerboardSlowdownState.WAITING_FOR_CLEAR

        elif self._state == CheckerboardSlowdownState.WAITING_FOR_CLEAR:
            if checkerboard_visible:
                if self._clear_since_sec is None:
                    self._clear_since_sec = now_sec
                elif now_sec - self._clear_since_sec >= self._clear_confirm_sec:
                    self._state = CheckerboardSlowdownState.ARMED
                    self._slowdown_until_time_sec = None
                    self._clear_since_sec = None
            else:
                self._clear_since_sec = None

        self._last_checkerboard_visible = checkerboard_visible

        return interrupted

    def update_time(self, now_sec: float) -> None:
        if not self._enabled:
            self._state = CheckerboardSlowdownState.ARMED
            self._slowdown_until_time_sec = None
            self._clear_since_sec = None
            return

        if self._state == CheckerboardSlowdownState.SLOWDOWN:
            slowdown_until_time_sec: Optional[float] = self._slowdown_until_time_sec
            if (
                slowdown_until_time_sec is not None
                and now_sec >= slowdown_until_time_sec
            ):
                self._state = CheckerboardSlowdownState.WAITING_FOR_CLEAR

    def is_slowdown_active(self, now_sec: float) -> bool:
        self.update_time(now_sec)

        if self._state != CheckerboardSlowdownState.SLOWDOWN:
            return False

        if self._slowdown_until_time_sec is None:
            return False

        return now_sec < self._slowdown_until_time_sec

    def is_waiting_for_clear(self, now_sec: float) -> bool:
        self.update_time(now_sec)

        return self._state == CheckerboardSlowdownState.WAITING_FOR_CLEAR

    def is_active(self, now_sec: float) -> bool:
        return self.is_slowdown_active(now_sec) or self.is_waiting_for_clear(now_sec)

    def slowdown_remaining_sec(self, now_sec: float) -> float:
        if not self.is_slowdown_active(now_sec):
            return 0.0

        slowdown_until_time_sec: Optional[float] = self._slowdown_until_time_sec
        if slowdown_until_time_sec is None:
            return 0.0

        return max(0.0, slowdown_until_time_sec - now_sec)

    def cancel(self) -> bool:
        was_active: bool = (
            self._state == CheckerboardSlowdownState.SLOWDOWN
            or self._state == CheckerboardSlowdownState.WAITING_FOR_CLEAR
        )

        self._state = CheckerboardSlowdownState.ARMED
        self._slowdown_until_time_sec = None
        self._clear_since_sec = None
        self._was_slowdown_active = False
        self._was_waiting_for_clear = False

        return was_active

    def consume_slowdown_activation(self, now_sec: float) -> bool:
        active: bool = self.is_slowdown_active(now_sec)
        activated: bool = active and not self._was_slowdown_active
        self._was_slowdown_active = active

        return activated

    def consume_slowdown_expiration(self, now_sec: float) -> bool:
        active: bool = self.is_slowdown_active(now_sec)
        expired: bool = self._was_slowdown_active and not active
        self._was_slowdown_active = active

        return expired

    def consume_waiting_for_clear_activation(self, now_sec: float) -> bool:
        active: bool = self.is_waiting_for_clear(now_sec)
        activated: bool = active and not self._was_waiting_for_clear
        self._was_waiting_for_clear = active

        return activated

    def consume_rearmed(self, now_sec: float) -> bool:
        active: bool = self.is_waiting_for_clear(now_sec)
        rearmed: bool = self._was_waiting_for_clear and not active
        self._was_waiting_for_clear = active

        return rearmed

    def scale_command(
        self,
        command: float,
        cruise_active: bool,
        now_sec: float,
    ) -> float:
        self.update_time(now_sec)

        if cruise_active and self.is_slowdown_active(now_sec):
            return command * self._scale

        return command
