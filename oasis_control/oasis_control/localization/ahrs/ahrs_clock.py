################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
Clock abstraction for AHRS localization
"""

from __future__ import annotations

from oasis_control.localization.ahrs.ahrs_conversions import seconds_from_ahrs
from oasis_control.localization.ahrs.ahrs_types import AhrsTime


class AhrsClock:
    """
    Clock abstraction for retrieving AHRS time
    """

    def now(self) -> AhrsTime:
        """
        Return the current AHRS time
        """

        raise NotImplementedError

    def is_future_stamp(self, t_meas: AhrsTime, epsilon_sec: float) -> bool:
        """
        True when the measurement stamp is ahead of the wall clock
        """

        now: AhrsTime = self.now()
        now_sec: float = seconds_from_ahrs(now)
        meas_sec: float = seconds_from_ahrs(t_meas)

        return meas_sec > now_sec + epsilon_sec
