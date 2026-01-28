################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
"""Stability tracking for mounting transform rotations."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Callable
from typing import Deque

import numpy as np
from numpy.typing import NDArray

from oasis_control.localization.mounting.math_utils.linalg import SO3
from oasis_control.localization.mounting.timing.time_base import sec_to_ns


_NS_PER_SEC: float = 1e9
_ROT_DET_TOL: float = 1e-6
_ROT_ORTH_TOL: float = 1e-6


class StabilityError(Exception):
    """Raised when stability tracking encounters invalid data."""


@dataclass(frozen=True)
class StabilityStatus:
    """Status of the rotation stability tracker.

    Attributes:
        is_stable: True when the estimate is stable enough
        delta_theta_BI_rad: Max rotation change for R_BI over the window in radians
        delta_theta_BM_rad: Max rotation change for R_BM over the window in radians
        window_duration_sec: Duration covered by the current window in seconds
    """

    is_stable: bool
    delta_theta_BI_rad: float
    delta_theta_BM_rad: float
    window_duration_sec: float


@dataclass(frozen=True)
class _RotationSample:
    """Rotation samples tracked in the stability window."""

    t_ns: int
    R_BI: NDArray[np.float64]
    R_BM: NDArray[np.float64]


class RotationStabilityTracker:
    """Track mounting rotation stability over a sliding time window."""

    def __init__(
        self,
        *,
        stable_window_sec: float,
        stable_rot_thresh_rad: float,
        require_need_more_tilt_false: bool = True,
        require_need_more_yaw_false: bool = True,
    ) -> None:
        """Initialize the tracker with stability thresholds."""
        if not np.isfinite(stable_window_sec) or stable_window_sec <= 0.0:
            raise StabilityError("stable_window_sec must be positive")
        if not np.isfinite(stable_rot_thresh_rad) or stable_rot_thresh_rad <= 0.0:
            raise StabilityError("stable_rot_thresh_rad must be positive")
        if not isinstance(require_need_more_tilt_false, bool):
            raise StabilityError("require_need_more_tilt_false must be a bool")
        if not isinstance(require_need_more_yaw_false, bool):
            raise StabilityError("require_need_more_yaw_false must be a bool")
        self._stable_window_sec: float = float(stable_window_sec)
        self._stable_rot_thresh_rad: float = float(stable_rot_thresh_rad)
        self._require_need_more_tilt_false: bool = require_need_more_tilt_false
        self._require_need_more_yaw_false: bool = require_need_more_yaw_false
        self._samples: Deque[_RotationSample] = deque()
        self._last_t_ns: int | None = None

    def reset(self) -> None:
        """Reset the tracker to its initial empty state."""
        self._samples.clear()
        self._last_t_ns = None

    def update(
        self,
        *,
        t_ns: int,
        R_BI: NDArray[np.float64],
        R_BM: NDArray[np.float64],
        need_more_tilt: bool,
        need_more_yaw: bool,
        mag_used_for_yaw: bool,
    ) -> StabilityStatus:
        """Update the tracker with a new rotation sample."""
        if not isinstance(t_ns, int) or isinstance(t_ns, bool):
            raise StabilityError("t_ns must be an int")
        if t_ns < 0:
            raise StabilityError("t_ns must be non-negative")
        if self._last_t_ns is not None and t_ns < self._last_t_ns:
            raise StabilityError("t_ns must be non-decreasing")
        if not isinstance(need_more_tilt, bool):
            raise StabilityError("need_more_tilt must be a bool")
        if not isinstance(need_more_yaw, bool):
            raise StabilityError("need_more_yaw must be a bool")
        if not isinstance(mag_used_for_yaw, bool):
            raise StabilityError("mag_used_for_yaw must be a bool")

        R_BI_mat: NDArray[np.float64] = _validate_rotation(R_BI, "R_BI")
        R_BM_mat: NDArray[np.float64] = _validate_rotation(R_BM, "R_BM")
        self._samples.append(_RotationSample(t_ns=t_ns, R_BI=R_BI_mat, R_BM=R_BM_mat))
        self._last_t_ns = t_ns

        window_ns: int = sec_to_ns(self._stable_window_sec)
        while self._samples and (t_ns - self._samples[0].t_ns) > window_ns:
            self._samples.popleft()

        oldest_t_ns: int = self._samples[0].t_ns
        window_duration_sec: float = (t_ns - oldest_t_ns) / _NS_PER_SEC

        delta_theta_BI_rad: float = _max_rotation_delta(
            self._samples,
            lambda sample: sample.R_BI,
        )
        delta_theta_BM_rad: float = _max_rotation_delta(
            self._samples,
            lambda sample: sample.R_BM,
        )

        window_filled: bool = (t_ns - oldest_t_ns) >= window_ns
        is_stable: bool = window_filled
        if is_stable:
            if delta_theta_BI_rad >= self._stable_rot_thresh_rad:
                is_stable = False
            elif delta_theta_BM_rad >= self._stable_rot_thresh_rad:
                is_stable = False
        if self._require_need_more_tilt_false and need_more_tilt:
            is_stable = False
        if mag_used_for_yaw and self._require_need_more_yaw_false and need_more_yaw:
            is_stable = False

        return StabilityStatus(
            is_stable=is_stable,
            delta_theta_BI_rad=delta_theta_BI_rad,
            delta_theta_BM_rad=delta_theta_BM_rad,
            window_duration_sec=window_duration_sec,
        )


def _max_rotation_delta(
    samples: Deque[_RotationSample],
    selector: Callable[[_RotationSample], NDArray[np.float64]],
) -> float:
    """Return the maximum rotation delta relative to the newest sample."""
    newest: _RotationSample = samples[-1]
    R_new: NDArray[np.float64] = selector(newest)
    max_angle: float = 0.0
    for sample in samples:
        R_old: NDArray[np.float64] = selector(sample)
        R_rel: NDArray[np.float64] = R_new @ R_old.T
        angle: float = float(SO3.angle(R_rel))
        if angle > max_angle:
            max_angle = angle
    return max_angle


def _validate_rotation(matrix: NDArray[np.float64], name: str) -> NDArray[np.float64]:
    """Validate that an array is a proper rotation matrix."""
    mat: NDArray[np.float64] = np.asarray(matrix, dtype=np.float64)
    if mat.shape != (3, 3):
        raise StabilityError(f"{name} must be shape (3, 3)")
    if not np.all(np.isfinite(mat)):
        raise StabilityError(f"{name} must be finite")
    det: float = float(np.linalg.det(mat))
    if abs(det - 1.0) > _ROT_DET_TOL:
        raise StabilityError(f"{name} must have determinant 1")
    ident: NDArray[np.float64] = mat @ mat.T
    if not np.allclose(ident, np.eye(3), rtol=0.0, atol=_ROT_ORTH_TOL):
        raise StabilityError(f"{name} must be orthonormal")
    return mat
