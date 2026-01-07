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
Validation helpers for EKF IMU packet handling
"""

from __future__ import annotations

from typing import Optional

from oasis_control.localization.ekf.ekf_types import EkfTime


def imu_calibration_stamp_reject_reason(
    imu_stamp: EkfTime, calibration_stamp: Optional[EkfTime]
) -> Optional[str]:
    if calibration_stamp is None:
        return None
    if imu_stamp != calibration_stamp:
        return "imu_calibration_stamp_mismatch"
    return None
