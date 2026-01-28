################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Diversity metrics for mounting calibration keyframes."""

from __future__ import annotations

import numpy as np

from oasis_control.localization.mounting.mounting_types import Keyframe


class DiversityMetricsError(Exception):
    """Raised when diversity metric inputs are invalid."""


def _angle_deg(u: np.ndarray, v: np.ndarray) -> float:
    """Return the angle between two unit vectors in degrees."""
    u_vec: np.ndarray = np.asarray(u, dtype=np.float64)
    v_vec: np.ndarray = np.asarray(v, dtype=np.float64)
    if u_vec.shape != (3,) or v_vec.shape != (3,):
        raise DiversityMetricsError("vectors must have shape (3,)")
    if not np.all(np.isfinite(u_vec)) or not np.all(np.isfinite(v_vec)):
        raise DiversityMetricsError("vectors must be finite")
    dot: float = float(np.dot(u_vec, v_vec))
    dot = float(np.clip(dot, -1.0, 1.0))
    return float(np.degrees(np.arccos(dot)))


def _max_pairwise_angle(vectors: list[np.ndarray]) -> float:
    """Return the maximum pairwise angle between vectors."""
    max_angle: float = 0.0
    for i, vec_i in enumerate(vectors):
        for vec_j in vectors[i + 1 :]:
            angle: float = _angle_deg(vec_i, vec_j)
            max_angle = max(max_angle, angle)
    return max_angle


def gravity_max_angle_deg(
    keyframes: list[Keyframe] | tuple[Keyframe, ...],
) -> float | None:
    """Return the max gravity-direction separation in degrees."""
    gravity_dirs: list[np.ndarray] = [
        keyframe.gravity_unit_mean_dir_I()
        for keyframe in keyframes
        if keyframe.gravity_weight > 0
    ]
    if len(gravity_dirs) < 2:
        return None
    return _max_pairwise_angle(gravity_dirs)


def mag_proj_max_angle_deg(
    keyframes: list[Keyframe] | tuple[Keyframe, ...],
) -> float | None:
    """Return the max mag-direction separation in degrees."""
    mag_dirs: list[np.ndarray] = [
        keyframe.mag_unit_mean_dir_M()
        for keyframe in keyframes
        if keyframe.mag_weight > 0
    ]
    if len(mag_dirs) < 2:
        return None
    return _max_pairwise_angle(mag_dirs)
