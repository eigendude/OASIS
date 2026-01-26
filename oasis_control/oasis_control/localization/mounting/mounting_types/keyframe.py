################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Keyframe aggregation types for mounting calibration."""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import replace
from typing import Any

import numpy as np

from oasis_control.localization.mounting.mounting_types.steady_segment import (
    SteadySegment,
)


@dataclass(frozen=True)
class Keyframe:
    """Running aggregate of steady segments.

    Attributes:
        keyframe_id: Unique keyframe identifier
        gravity_mean_dir_I: Mean of unit gravity directions in the IMU frame
        gravity_cov_dir_I: Covariance of gravity directions in the IMU frame
        gravity_weight: Number of segments contributing to gravity statistics
        mag_mean_dir_M: Mean of unit magnetic directions in the mag frame
        mag_cov_dir_M: Covariance of magnetic directions in the mag frame
        mag_weight: Number of segments contributing to mag statistics
        segment_count: Total number of segments assigned to the keyframe
    """

    keyframe_id: int
    gravity_mean_dir_I: np.ndarray
    gravity_cov_dir_I: np.ndarray
    gravity_weight: int
    mag_mean_dir_M: np.ndarray | None
    mag_cov_dir_M: np.ndarray | None
    mag_weight: int
    segment_count: int

    def __post_init__(self) -> None:
        """Validate keyframe fields and coerce arrays."""
        if not isinstance(self.keyframe_id, int) or isinstance(self.keyframe_id, bool):
            raise ValueError("keyframe_id must be an int")
        if not isinstance(self.gravity_weight, int) or isinstance(
            self.gravity_weight, bool
        ):
            raise ValueError("gravity_weight must be an int")
        if self.gravity_weight < 0:
            raise ValueError("gravity_weight must be non-negative")
        if not isinstance(self.mag_weight, int) or isinstance(self.mag_weight, bool):
            raise ValueError("mag_weight must be an int")
        if self.mag_weight < 0:
            raise ValueError("mag_weight must be non-negative")
        if not isinstance(self.segment_count, int) or isinstance(
            self.segment_count, bool
        ):
            raise ValueError("segment_count must be an int")
        if self.segment_count < 0:
            raise ValueError("segment_count must be non-negative")

        gravity_mean_dir_I: np.ndarray = _as_float_array(
            self.gravity_mean_dir_I,
            "gravity_mean_dir_I",
            (3,),
        )
        gravity_cov_dir_I: np.ndarray = _as_float_array(
            self.gravity_cov_dir_I,
            "gravity_cov_dir_I",
            (3, 3),
        )
        if self.gravity_weight > 0:
            _require_non_zero(gravity_mean_dir_I, "gravity_mean_dir_I")

        object.__setattr__(self, "gravity_mean_dir_I", gravity_mean_dir_I)
        object.__setattr__(self, "gravity_cov_dir_I", gravity_cov_dir_I)

        if self.mag_mean_dir_M is None:
            if self.mag_cov_dir_M is not None or self.mag_weight != 0:
                raise ValueError("mag stats must be empty when mag_mean_dir_M is None")
            object.__setattr__(self, "mag_cov_dir_M", None)
        else:
            mag_mean_dir_M: np.ndarray = _as_float_array(
                self.mag_mean_dir_M,
                "mag_mean_dir_M",
                (3,),
            )
            mag_cov_dir_M: np.ndarray = _as_float_array(
                self.mag_cov_dir_M,
                "mag_cov_dir_M",
                (3, 3),
            )
            if self.mag_weight > 0:
                _require_non_zero(mag_mean_dir_M, "mag_mean_dir_M")
            object.__setattr__(self, "mag_mean_dir_M", mag_mean_dir_M)
            object.__setattr__(self, "mag_cov_dir_M", mag_cov_dir_M)

    def update_with_segment(self, segment: SteadySegment) -> Keyframe:
        """Return a new keyframe updated with a steady segment."""
        gravity_dir_I: np.ndarray = segment.gravity_up_dir_I()
        mag_dir_M: np.ndarray | None
        if segment.m_mean_T is None:
            mag_dir_M = None
        else:
            mag_dir_M = _unit_vector(segment.m_mean_T, "m_mean_T")
        return self.update_with_directions(gravity_dir_I, mag_dir_M)

    def update_with_directions(
        self,
        gravity_dir_I: np.ndarray,
        mag_dir_M: np.ndarray | None = None,
    ) -> Keyframe:
        """Return a new keyframe updated with direction observations.

        The input directions are normalized before they are aggregated.
        """
        gravity_dir_I_unit: np.ndarray = _unit_vector(gravity_dir_I, "gravity_dir_I")
        gravity_mean_dir_I: np.ndarray
        gravity_cov_dir_I: np.ndarray
        gravity_weight: int
        (
            gravity_mean_dir_I,
            gravity_cov_dir_I,
            gravity_weight,
        ) = _update_dir_stats(
            self.gravity_mean_dir_I,
            self.gravity_cov_dir_I,
            self.gravity_weight,
            gravity_dir_I_unit,
        )

        mag_mean_dir_M: np.ndarray | None = self.mag_mean_dir_M
        mag_cov_dir_M: np.ndarray | None = self.mag_cov_dir_M
        mag_weight: int = self.mag_weight
        if mag_dir_M is not None:
            mag_dir_M_unit: np.ndarray = _unit_vector(mag_dir_M, "mag_dir_M")
            if mag_mean_dir_M is None or mag_cov_dir_M is None or mag_weight == 0:
                mag_mean_dir_M = mag_dir_M_unit
                mag_cov_dir_M = np.zeros((3, 3), dtype=np.float64)
                mag_weight = 1
            else:
                mag_mean_dir_M, mag_cov_dir_M, mag_weight = _update_dir_stats(
                    mag_mean_dir_M,
                    mag_cov_dir_M,
                    mag_weight,
                    mag_dir_M_unit,
                )

        return replace(
            self,
            gravity_mean_dir_I=gravity_mean_dir_I,
            gravity_cov_dir_I=gravity_cov_dir_I,
            gravity_weight=gravity_weight,
            mag_mean_dir_M=mag_mean_dir_M,
            mag_cov_dir_M=mag_cov_dir_M,
            mag_weight=mag_weight,
            segment_count=self.segment_count + 1,
        )

    def gravity_unit_mean_dir_I(self) -> np.ndarray:
        """Return the unit-length mean gravity direction."""
        return _unit_vector(self.gravity_mean_dir_I, "gravity_mean_dir_I")

    def mag_unit_mean_dir_M(self) -> np.ndarray:
        """Return the unit-length mean magnetic direction."""
        if self.mag_mean_dir_M is None:
            raise ValueError("mag_mean_dir_M is not available")
        return _unit_vector(self.mag_mean_dir_M, "mag_mean_dir_M")


def _update_dir_stats(
    mean_dir: np.ndarray,
    cov_dir: np.ndarray,
    weight: int,
    direction: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, int]:
    """Update mean and covariance for direction statistics."""
    if weight == 0:
        return direction, np.zeros((3, 3), dtype=np.float64), 1

    new_weight: int = weight + 1
    delta: np.ndarray = direction - mean_dir
    mean_new: np.ndarray = mean_dir + delta / float(new_weight)
    delta2: np.ndarray = direction - mean_new
    m2_old: np.ndarray = cov_dir * float(weight)
    m2_new: np.ndarray = m2_old + np.outer(delta, delta2)
    cov_new: np.ndarray = m2_new / float(new_weight)

    return mean_new, cov_new, new_weight


def _as_float_array(value: Any, name: str, shape: tuple[int, ...]) -> np.ndarray:
    """Coerce a value to a float64 numpy array with a specific shape."""
    array: np.ndarray = np.asarray(value, dtype=np.float64)
    if array.shape != shape:
        raise ValueError(f"{name} must have shape {shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain finite values")
    return array


def _require_non_zero(vector: np.ndarray, name: str) -> None:
    """Require a non-zero vector when weight is positive."""
    norm: float = float(np.linalg.norm(vector))
    if not np.isfinite(norm) or norm <= 0.0:
        raise ValueError(f"{name} must be non-zero when weight is positive")


def _unit_vector(vector: np.ndarray, name: str) -> np.ndarray:
    """Return a unit vector derived from the input."""
    norm: float = float(np.linalg.norm(vector))
    if not np.isfinite(norm) or norm <= 0.0:
        raise ValueError(f"{name} must be non-zero to normalize")
    return vector / norm
