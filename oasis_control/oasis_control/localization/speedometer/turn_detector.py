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

import math

from oasis_control.localization.speedometer.contracts import TurnDetection


################################################################################
# Turn detection
################################################################################


class TurnDetector:
    """
    Minimal turn detector used only to gate online forward-axis learning.
    """

    def __init__(self, *, turn_rate_threshold_rads: float) -> None:
        """Initialize the placeholder detector."""

        self._turn_rate_threshold_rads: float = max(
            0.0, float(turn_rate_threshold_rads)
        )

    def update(self, *, yaw_rate_rads: float) -> TurnDetection:
        """
        Return a simple thresholded turn decision.
        """

        finite_yaw_rate_rads: float = (
            float(yaw_rate_rads) if math.isfinite(yaw_rate_rads) else 0.0
        )
        return TurnDetection(
            turn_detected=(abs(finite_yaw_rate_rads) >= self._turn_rate_threshold_rads),
            yaw_rate_rads=finite_yaw_rate_rads,
            threshold_rads=self._turn_rate_threshold_rads,
        )
