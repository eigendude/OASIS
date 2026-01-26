################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for diversity metrics."""

from __future__ import annotations

import numpy as np

from oasis_control.localization.mounting.models.diversity_metrics import (
    gravity_max_angle_deg,
)
from oasis_control.localization.mounting.models.diversity_metrics import (
    mag_proj_max_angle_deg,
)
from oasis_control.localization.mounting.mounting_types import Keyframe


def _keyframe_with_dirs(
    keyframe_id: int,
    gravity_dir: np.ndarray,
    mag_dir: np.ndarray | None = None,
) -> Keyframe:
    """Create a keyframe with specified mean directions."""
    gravity_vec: np.ndarray = np.asarray(gravity_dir, dtype=np.float64)
    gravity_unit: np.ndarray = gravity_vec / float(np.linalg.norm(gravity_vec))
    if mag_dir is None:
        mag_mean: np.ndarray | None = None
        mag_cov: np.ndarray | None = None
        mag_weight: int = 0
    else:
        mag_vec: np.ndarray = np.asarray(mag_dir, dtype=np.float64)
        mag_mean = mag_vec / float(np.linalg.norm(mag_vec))
        mag_cov = np.zeros((3, 3), dtype=np.float64)
        mag_weight = 1
    return Keyframe(
        keyframe_id=keyframe_id,
        gravity_mean_dir_I=gravity_unit,
        gravity_cov_dir_I=np.zeros((3, 3), dtype=np.float64),
        gravity_weight=1,
        mag_mean_dir_M=mag_mean,
        mag_cov_dir_M=mag_cov,
        mag_weight=mag_weight,
        segment_count=1,
    )


def test_gravity_max_angle() -> None:
    """Ensure gravity angle metric returns expected values."""
    keyframes: list[Keyframe] = [
        _keyframe_with_dirs(0, np.array([1.0, 0.0, 0.0], dtype=np.float64)),
        _keyframe_with_dirs(1, np.array([0.0, 1.0, 0.0], dtype=np.float64)),
        _keyframe_with_dirs(2, np.array([-1.0, 0.0, 0.0], dtype=np.float64)),
    ]
    angle: float | None = gravity_max_angle_deg(keyframes)
    assert angle is not None
    assert angle == 180.0


def test_mag_proj_max_angle() -> None:
    """Ensure mag angle metric returns expected values."""
    keyframes: list[Keyframe] = [
        _keyframe_with_dirs(
            0,
            np.array([0.0, 0.0, 1.0], dtype=np.float64),
            mag_dir=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        ),
        _keyframe_with_dirs(
            1,
            np.array([0.0, 0.0, 1.0], dtype=np.float64),
            mag_dir=np.array([0.0, 1.0, 0.0], dtype=np.float64),
        ),
    ]
    angle: float | None = mag_proj_max_angle_deg(keyframes)
    assert angle is not None
    assert angle == 90.0


def test_diversity_none_cases() -> None:
    """Ensure metrics return None when insufficient keyframes exist."""
    keyframe: Keyframe = _keyframe_with_dirs(
        0,
        np.array([1.0, 0.0, 0.0], dtype=np.float64),
    )
    assert gravity_max_angle_deg([keyframe]) is None
    assert mag_proj_max_angle_deg([keyframe]) is None
