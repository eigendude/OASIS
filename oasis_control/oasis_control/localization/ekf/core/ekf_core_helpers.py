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
Shared helpers for the EKF core
"""

from __future__ import annotations

import math

from oasis_control.localization.ekf.ekf_types import EkfMatrix


class _EkfCoreHelpersMixin:
    def _zero_matrix(self, dim: int) -> EkfMatrix:
        return EkfMatrix(rows=dim, cols=dim, data=[0.0] * (dim * dim))

    def _nan_list(self, length: int) -> list[float]:
        return [math.nan] * length
