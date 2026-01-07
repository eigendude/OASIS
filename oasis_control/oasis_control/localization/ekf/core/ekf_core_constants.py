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
Shared constants for EKF core logic
"""

from __future__ import annotations

from oasis_control.localization.ekf.ekf_types import EkfEventType


_EVENT_ORDER: dict[EkfEventType, int] = {
    EkfEventType.IMU: 0,
    EkfEventType.CAMERA_INFO: 1,
    EkfEventType.MAG: 2,
    EkfEventType.APRILTAG: 3,
}

# Nanoseconds per second for time conversions
NS_PER_S: int = 1_000_000_000
