################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Snapshot of current mounting calibration estimates."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class ResultSnapshot:
    """Immutable bundle of the current mounting calibration estimate.

    Attributes:
        t_meas_ns: Timestamp of the snapshot in nanoseconds
        frame_base: Base frame name
        frame_imu: IMU frame name
        frame_mag: Magnetometer frame name when available
        R_BI: Estimated base-to-IMU rotation matrix, unitless
        R_BM: Estimated base-to-mag rotation matrix, unitless, when available
        cov_rot_BI: Rotation covariance for R_BI in tangent space
        cov_rot_BM: Rotation covariance for R_BM in tangent space
        b_a_mps2: Accelerometer bias estimate in m/s^2
        A_a: Accelerometer scale/misalignment estimate
        accel_param_cov: Covariance for [b_a, vec(A_a)] in (m/s^2)^2
        b_g_rads: Gyroscope bias estimate in rad/s
        gyro_bias_cov: Covariance of gyroscope bias in (rad/s)^2
        b_m_T: Magnetometer bias estimate in tesla when available
        R_m: Adaptive magnetometer direction-residual noise covariance (unitless^2
            ≈ rad^2) when available
        segment_count: Number of steady segments processed
        keyframe_count: Number of keyframes in the estimate
        diversity_tilt_deg: Tilt diversity in degrees when available
        diversity_yaw_deg: Yaw diversity in degrees when available
        is_stable: Indicates whether the estimate is considered stable
        quality_score: Aggregate quality score when available
    """

    t_meas_ns: int
    frame_base: str
    frame_imu: str
    frame_mag: str | None
    R_BI: NDArray[np.float64]
    R_BM: NDArray[np.float64] | None
    cov_rot_BI: np.ndarray
    cov_rot_BM: np.ndarray | None
    b_a_mps2: np.ndarray
    A_a: np.ndarray
    accel_param_cov: np.ndarray
    b_g_rads: np.ndarray
    gyro_bias_cov: np.ndarray
    b_m_T: np.ndarray | None
    R_m: np.ndarray | None
    segment_count: int
    keyframe_count: int
    diversity_tilt_deg: float | None
    diversity_yaw_deg: float | None
    is_stable: bool
    quality_score: float | None

    def __post_init__(self) -> None:
        """Validate snapshot fields and coerce arrays."""
        if not isinstance(self.t_meas_ns, int) or isinstance(self.t_meas_ns, bool):
            raise ValueError("t_meas_ns must be an int")
        if not isinstance(self.frame_base, str):
            raise ValueError("frame_base must be a str")
        if not isinstance(self.frame_imu, str):
            raise ValueError("frame_imu must be a str")
        if self.frame_mag is not None and not isinstance(self.frame_mag, str):
            raise ValueError("frame_mag must be a str or None")
        if not isinstance(self.segment_count, int) or isinstance(
            self.segment_count, bool
        ):
            raise ValueError("segment_count must be an int")
        if self.segment_count < 0:
            raise ValueError("segment_count must be non-negative")
        if not isinstance(self.keyframe_count, int) or isinstance(
            self.keyframe_count, bool
        ):
            raise ValueError("keyframe_count must be an int")
        if self.keyframe_count < 0:
            raise ValueError("keyframe_count must be non-negative")
        if not isinstance(self.is_stable, bool):
            raise ValueError("is_stable must be a bool")

        R_BI: NDArray[np.float64] = _as_rotation_matrix(self.R_BI, "R_BI")
        cov_rot_BI: np.ndarray = _as_float_array(self.cov_rot_BI, "cov_rot_BI", (3, 3))
        b_a_mps2: np.ndarray = _as_float_array(self.b_a_mps2, "b_a_mps2", (3,))
        A_a: np.ndarray = _as_float_array(self.A_a, "A_a", (3, 3))
        accel_param_cov: np.ndarray = _as_float_array(
            self.accel_param_cov,
            "accel_param_cov",
            (12, 12),
        )
        b_g_rads: np.ndarray = _as_float_array(self.b_g_rads, "b_g_rads", (3,))
        gyro_bias_cov: np.ndarray = _as_float_array(
            self.gyro_bias_cov,
            "gyro_bias_cov",
            (3, 3),
        )

        object.__setattr__(self, "R_BI", R_BI)
        object.__setattr__(self, "cov_rot_BI", cov_rot_BI)
        object.__setattr__(self, "b_a_mps2", b_a_mps2)
        object.__setattr__(self, "A_a", A_a)
        object.__setattr__(self, "accel_param_cov", accel_param_cov)
        object.__setattr__(self, "b_g_rads", b_g_rads)
        object.__setattr__(self, "gyro_bias_cov", gyro_bias_cov)

        if self.quality_score is not None:
            _require_finite(self.quality_score, "quality_score")
        if self.diversity_tilt_deg is not None:
            _require_finite(self.diversity_tilt_deg, "diversity_tilt_deg")
        if self.diversity_yaw_deg is not None:
            _require_finite(self.diversity_yaw_deg, "diversity_yaw_deg")

        if self.frame_mag is None:
            if (
                self.R_BM is not None
                or self.cov_rot_BM is not None
                or self.b_m_T is not None
                or self.R_m is not None
            ):
                raise ValueError("mag fields must be None when frame_mag is None")
            object.__setattr__(self, "R_BM", None)
            object.__setattr__(self, "cov_rot_BM", None)
            object.__setattr__(self, "b_m_T", None)
            object.__setattr__(self, "R_m", None)
        else:
            if self.R_BM is None:
                raise ValueError("R_BM is required when frame_mag is set")
            R_BM: NDArray[np.float64] = _as_rotation_matrix(self.R_BM, "R_BM")
            object.__setattr__(self, "R_BM", R_BM)
            if self.cov_rot_BM is None:
                raise ValueError("mag covariances are required when frame_mag is set")
            cov_rot_BM: np.ndarray = _as_float_array(
                self.cov_rot_BM,
                "cov_rot_BM",
                (3, 3),
            )
            object.__setattr__(self, "cov_rot_BM", cov_rot_BM)

            if self.b_m_T is not None:
                b_m_T: np.ndarray = _as_float_array(self.b_m_T, "b_m_T", (3,))
                object.__setattr__(self, "b_m_T", b_m_T)
            if self.R_m is not None:
                R_m: np.ndarray = _as_float_array(self.R_m, "R_m", (3, 3))
                object.__setattr__(self, "R_m", R_m)


def _as_rotation_matrix(value: Any, name: str) -> NDArray[np.float64]:
    """Coerce a value to a float64 rotation matrix."""
    array: NDArray[np.float64] = np.asarray(value, dtype=np.float64)
    if array.shape != (3, 3):
        raise ValueError(f"{name} must have shape (3, 3)")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain finite values")
    return array


def _as_float_array(
    value: Any,
    name: str,
    shape: tuple[int, ...],
) -> NDArray[np.float64]:
    """Coerce a value to a float64 numpy array with a specific shape."""
    array: NDArray[np.float64] = np.asarray(value, dtype=np.float64)
    if array.shape != shape:
        raise ValueError(f"{name} must have shape {shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain finite values")
    return array


def _require_finite(value: float, name: str) -> None:
    """Ensure a scalar value is finite."""
    if not np.isfinite(value):
        raise ValueError(f"{name} must be finite")
