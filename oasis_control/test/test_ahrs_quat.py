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

from oasis_control.localization.ahrs.ahrs_quat import quat_from_rotvec_wxyz


def test_quat_from_rotvec_small_angle_normalizes() -> None:
    rotvec: list[float] = [1.0e-13, -2.0e-13, 0.5e-13]
    quat: list[float] = quat_from_rotvec_wxyz(rotvec)

    norm: float = math.sqrt(
        quat[0] * quat[0]
        + quat[1] * quat[1]
        + quat[2] * quat[2]
        + quat[3] * quat[3]
    )

    assert math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-12)
