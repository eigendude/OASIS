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


def test_quat_from_rotvec_small_angle_normalized() -> None:
    rotvec: list[float] = [1.0e-14, -2.0e-14, 3.0e-14]
    q_wxyz: list[float] = quat_from_rotvec_wxyz(rotvec)
    norm: float = math.sqrt(
        q_wxyz[0] * q_wxyz[0]
        + q_wxyz[1] * q_wxyz[1]
        + q_wxyz[2] * q_wxyz[2]
        + q_wxyz[3] * q_wxyz[3]
    )

    assert math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-12)
