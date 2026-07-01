################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Checkerboard-triggered train park-mode state."""

from __future__ import annotations


class TrainParkMode:
    """
    Tracks reverse park-mode state until a checkerboard interruption edge.

    Park mode drives the train backward while watching for the train end to
    interrupt the checkerboard. The first checkerboard message after activation
    initializes edge detection only, so activation while already interrupted
    does not immediately park the train.
    """

    def __init__(self, enabled: bool, command: float) -> None:
        """
        Initialize park-mode configuration.
        """
        self._enabled: bool = enabled
        self._command: float = command
        self._active: bool = False
        self._have_checkerboard_status: bool = False
        self._last_checkerboard_visible: bool = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def command(self) -> float:
        return self._command

    @property
    def active(self) -> bool:
        return self._active

    def activate(self) -> bool:
        if not self._enabled:
            return False

        self._active = True
        self._have_checkerboard_status = False
        self._last_checkerboard_visible = False

        return True

    def cancel(self) -> bool:
        was_active: bool = self._active

        self._active = False
        self._have_checkerboard_status = False
        self._last_checkerboard_visible = False

        return was_active

    def update_checkerboard_status(self, checkerboard_visible: bool) -> bool:
        if not self._active:
            return False

        if not self._have_checkerboard_status:
            self._have_checkerboard_status = True
            self._last_checkerboard_visible = checkerboard_visible
            return False

        parked: bool = self._last_checkerboard_visible and not checkerboard_visible
        self._last_checkerboard_visible = checkerboard_visible

        if parked:
            self._active = False

        return parked
