################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for quaternion and vector helpers used by the speedometer."""

from __future__ import annotations

import math

from oasis_control.localization.common.algebra.quat import normalize_quaternion_xyzw
from oasis_control.localization.common.algebra.quat import quaternion_to_rotation_matrix
from oasis_control.localization.common.algebra.quat import rotate_vector
from oasis_control.localization.common.algebra.quat import transpose_matrix


def test_normalize_quaternion_accepts_non_unit_input() -> None:
    normalized: tuple[float, float, float, float] | None = normalize_quaternion_xyzw(
        (0.0, 0.0, 0.0, 2.0)
    )

    assert normalized == (0.0, 0.0, 0.0, 1.0)


def test_normalize_quaternion_rejects_invalid_input() -> None:
    assert normalize_quaternion_xyzw((0.0, 0.0, 0.0, 0.0)) is None
    assert normalize_quaternion_xyzw((0.0, 0.0, 0.0, math.nan)) is None


def test_quaternion_rotation_and_vector_helpers() -> None:
    quaternion: tuple[float, float, float, float] = (
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    )
    rotation: tuple[tuple[float, float, float], ...] = quaternion_to_rotation_matrix(
        quaternion
    )

    rotated: tuple[float, float, float] = rotate_vector(
        rotation,
        (1.0, 0.0, 0.0),
    )
    restored: tuple[float, float, float] = rotate_vector(
        transpose_matrix(rotation),
        rotated,
    )

    assert math.isclose(rotated[0], 0.0, abs_tol=1.0e-9)
    assert math.isclose(rotated[1], 1.0, abs_tol=1.0e-9)
    assert math.isclose(restored[0], 1.0, abs_tol=1.0e-9)
    assert math.isclose(restored[1], 0.0, abs_tol=1.0e-9)
