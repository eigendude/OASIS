################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""ROS-independent state management for the Kodi train HUD."""

from __future__ import annotations


class TrainHudManager:
    """Manage train HUD visibility state from button edges."""

    def __init__(self) -> None:
        """Initialize the SLAM view as hidden and the up button as released."""

        self._show_slam: bool = False
        self._up_pressed: bool = False

    @property
    def show_slam(self) -> bool:
        """Return whether the train HUD should show the SLAM view."""

        return self._show_slam

    def update_up_button(self, pressed: bool) -> bool | None:
        """Return the changed SLAM visibility on a distinct up-button press."""

        changed_state: bool | None = None
        if pressed and not self._up_pressed:
            self._show_slam = not self._show_slam
            changed_state = self._show_slam

        self._up_pressed = pressed
        return changed_state
