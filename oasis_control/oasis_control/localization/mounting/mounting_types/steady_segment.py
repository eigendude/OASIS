################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Steady-segment types for mounting calibration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class SteadySegment:
    """Steady segment summary emitted by the detector.

    Attributes:
        t_start_ns: Segment start time in nanoseconds
        t_end_ns: Segment end time in nanoseconds
        t_meas_ns: Representative timestamp for the segment
        imu_frame_id: IMU frame associated with the segment
        mag_frame_id: Magnetometer frame, if magnetometer data was present
        a_mean_mps2: Mean accelerometer sample in m/s^2
        cov_a: Accelerometer covariance in (m/s^2)^2
        m_mean_T: Mean magnetometer sample in tesla when available
        cov_m: Magnetometer covariance in tesla^2 when available
        sample_count: Number of samples included in the segment
        duration_ns: Segment duration in nanoseconds
    """

    t_start_ns: int
    t_end_ns: int
    t_meas_ns: int
    imu_frame_id: str
    mag_frame_id: str | None
    a_mean_mps2: np.ndarray
    cov_a: np.ndarray
    m_mean_T: np.ndarray | None
    cov_m: np.ndarray | None
    sample_count: int
    duration_ns: int

    def __post_init__(self) -> None:
        """Validate segment fields and coerce arrays."""
        if not isinstance(self.t_start_ns, int) or isinstance(self.t_start_ns, bool):
            raise ValueError("t_start_ns must be an int")
        if not isinstance(self.t_end_ns, int) or isinstance(self.t_end_ns, bool):
            raise ValueError("t_end_ns must be an int")
        if not isinstance(self.t_meas_ns, int) or isinstance(self.t_meas_ns, bool):
            raise ValueError("t_meas_ns must be an int")
        if self.t_end_ns < self.t_start_ns:
            raise ValueError("t_end_ns must be >= t_start_ns")
        if not (self.t_start_ns <= self.t_meas_ns <= self.t_end_ns):
            raise ValueError("t_meas_ns must be within segment bounds")
        if not isinstance(self.imu_frame_id, str):
            raise ValueError("imu_frame_id must be a str")
        if self.mag_frame_id is not None and not isinstance(self.mag_frame_id, str):
            raise ValueError("mag_frame_id must be a str or None")
        if not isinstance(self.sample_count, int) or isinstance(
            self.sample_count, bool
        ):
            raise ValueError("sample_count must be an int")
        if self.sample_count <= 0:
            raise ValueError("sample_count must be positive")
        if not isinstance(self.duration_ns, int) or isinstance(self.duration_ns, bool):
            raise ValueError("duration_ns must be an int")
        if self.duration_ns != self.t_end_ns - self.t_start_ns:
            raise ValueError("duration_ns must match t_end_ns - t_start_ns")

        a_mean_mps2: np.ndarray = _as_float_array(
            self.a_mean_mps2,
            "a_mean_mps2",
            (3,),
        )
        cov_a: np.ndarray = _as_float_array(self.cov_a, "cov_a", (3, 3))

        object.__setattr__(self, "a_mean_mps2", a_mean_mps2)
        object.__setattr__(self, "cov_a", cov_a)

        if self.mag_frame_id is None:
            if self.m_mean_T is not None or self.cov_m is not None:
                raise ValueError("m_mean_T and cov_m must be None without mag_frame_id")
            object.__setattr__(self, "m_mean_T", None)
            object.__setattr__(self, "cov_m", None)
        else:
            if self.m_mean_T is None or self.cov_m is None:
                raise ValueError("m_mean_T and cov_m are required with mag_frame_id")
            m_mean_T: np.ndarray = _as_float_array(self.m_mean_T, "m_mean_T", (3,))
            cov_m: np.ndarray = _as_float_array(self.cov_m, "cov_m", (3, 3))
            object.__setattr__(self, "m_mean_T", m_mean_T)
            object.__setattr__(self, "cov_m", cov_m)

    def gravity_up_dir_I(self) -> np.ndarray:
        """Return the unit gravity-up direction in the IMU frame."""
        return _unit_vector(-self.a_mean_mps2, "a_mean_mps2")


def _as_float_array(value: Any, name: str, shape: tuple[int, ...]) -> np.ndarray:
    """Coerce a value to a float64 numpy array with a specific shape."""
    array: np.ndarray = np.asarray(value, dtype=np.float64)
    if array.shape != shape:
        raise ValueError(f"{name} must have shape {shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain finite values")
    return array


def _unit_vector(vector: np.ndarray, name: str) -> np.ndarray:
    """Return a unit vector derived from the input."""
    norm: float = float(np.linalg.norm(vector))
    if not np.isfinite(norm) or norm <= 0.0:
        raise ValueError(f"{name} must be non-zero to normalize")
    return vector / norm
