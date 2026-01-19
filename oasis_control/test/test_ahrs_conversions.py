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

import pytest

from oasis_control.localization.ahrs.ahrs_conversions import ahrs_time_from_pair
from oasis_control.localization.ahrs.ahrs_conversions import cov3x3_from_seq
from oasis_control.localization.ahrs.ahrs_conversions import normalize_ahrs_time
from oasis_control.localization.ahrs.ahrs_types import AhrsTime


def test_cov3x3_from_seq_rejects_length() -> None:
    with pytest.raises(ValueError):
        cov3x3_from_seq([0.0] * 8)


def test_cov3x3_from_seq_rejects_nan() -> None:
    cov: list[float] = [0.0] * 8 + [math.nan]

    with pytest.raises(ValueError):
        cov3x3_from_seq(cov)


def test_normalize_ahrs_time_carries_nanoseconds() -> None:
    t_raw: AhrsTime = AhrsTime(sec=123, nanosec=1_500_000_000)

    normalized: AhrsTime = normalize_ahrs_time(t_raw)

    assert normalized.sec == 124
    assert normalized.nanosec == 500_000_000


def test_ahrs_time_from_pair_normalizes_input() -> None:
    normalized: AhrsTime = ahrs_time_from_pair(sec=10, nanosec=2_000_000_001)

    assert normalized.sec == 12
    assert normalized.nanosec == 1
