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

from oasis_control.localization.ahrs.ahrs_bootstrap import _quat_from_two_unit_vectors
from oasis_control.localization.ahrs.ahrs_quat import quat_rotate_wxyz


def _norm4(q_wxyz: list[float]) -> float:
    q0: float = q_wxyz[0]
    q1: float = q_wxyz[1]
    q2: float = q_wxyz[2]
    q3: float = q_wxyz[3]
    return math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)


def test_quat_from_two_unit_vectors_parallel() -> None:
    from_unit: list[float] = [0.0, 0.0, -1.0]
    to_unit: list[float] = [0.0, 0.0, -1.0]

    quat: list[float] = _quat_from_two_unit_vectors(from_unit, to_unit)

    assert quat == pytest.approx([1.0, 0.0, 0.0, 0.0])


def test_quat_from_two_unit_vectors_opposite() -> None:
    from_unit: list[float] = [1.0, 0.0, 0.0]
    to_unit: list[float] = [-1.0, 0.0, 0.0]

    quat: list[float] = _quat_from_two_unit_vectors(from_unit, to_unit)
    norm: float = _norm4(quat)
    rotated: list[float] = quat_rotate_wxyz(quat, from_unit)

    assert norm == pytest.approx(1.0, rel=1.0e-6, abs=1.0e-6)
    assert rotated == pytest.approx(to_unit, rel=1.0e-6, abs=1.0e-6)


def test_quat_from_two_unit_vectors_general() -> None:
    from_unit: list[float] = [0.0, 0.0, -1.0]
    raw: list[float] = [0.3, 0.4, -0.5]
    raw_norm: float = math.sqrt(sum(value * value for value in raw))
    to_unit: list[float] = [value / raw_norm for value in raw]

    quat: list[float] = _quat_from_two_unit_vectors(from_unit, to_unit)
    rotated: list[float] = quat_rotate_wxyz(quat, from_unit)

    assert rotated == pytest.approx(to_unit, rel=1.0e-6, abs=1.0e-6)
