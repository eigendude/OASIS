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
Utility helpers for EKF core logic
"""

from __future__ import annotations

import math

from oasis_control.localization.ekf.ekf_types import EkfMatrix


class EkfCoreUtilsMixin:
    def _zero_matrix(self, dim: int) -> EkfMatrix:
        return EkfMatrix(rows=dim, cols=dim, data=[0.0] * (dim * dim))

    def _nan_list(self, length: int) -> list[float]:
        return [math.nan] * length

    def _wrap_angle(self, angle: float) -> float:
        wrapped: float = (angle + math.pi) % (2.0 * math.pi) - math.pi
        return wrapped
