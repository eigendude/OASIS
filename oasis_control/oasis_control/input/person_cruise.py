################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Person-presence state for automatic train cruise."""

from collections.abc import Iterable
from typing import Optional


# Normalized minimum box center X coordinate considered right-third presence
PERSON_CRUISE_RIGHT_THIRD_MIN_X: float = 2.0 / 3.0

# Seconds to wait after the last right-third person before ending cruise
PERSON_CRUISE_LOST_TIMEOUT_SEC: float = 0.1


class PersonCruise:
    """Track right-third person presence and cruise activation."""

    def __init__(self) -> None:
        """Initialize inactive person cruise state."""
        self._active: bool = False
        self._last_person_right_third_sec: Optional[float] = None

    @property
    def active(self) -> bool:
        """Return whether person cruise is active."""
        return self._active

    def update(
        self,
        person_x_centers: Iterable[float],
        now_sec: float,
        activation_allowed: bool,
    ) -> Optional[bool]:
        """Return the new active state when presence changes it."""
        person_right_third: bool = any(
            x_center >= PERSON_CRUISE_RIGHT_THIRD_MIN_X for x_center in person_x_centers
        )

        if person_right_third:
            person_entered_right_third: bool = self._last_person_right_third_sec is None
            self._last_person_right_third_sec = now_sec

            if person_entered_right_third and activation_allowed:
                self._active = True
                return True

            return None

        last_seen_sec: Optional[float] = self._last_person_right_third_sec
        if last_seen_sec is None:
            return None

        lost_duration_sec: float = now_sec - last_seen_sec
        if lost_duration_sec < PERSON_CRUISE_LOST_TIMEOUT_SEC:
            return None

        self._last_person_right_third_sec = None
        if not self._active:
            return None

        self._active = False
        return False

    def cancel(self) -> None:
        """Cancel cruise without resetting tracked person presence."""
        self._active = False
