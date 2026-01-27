################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Windowed steady-state detector for mounting calibration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.mounting_types import MagPacket
from oasis_control.localization.mounting.mounting_types import SteadySegment


class SteadyDetectorError(Exception):
    """Raised when the steady detector encounters invalid data."""


def _as_float_array(value: np.ndarray, name: str, shape: tuple[int, ...]) -> np.ndarray:
    """Coerce an array to float64 with a required shape."""
    array: np.ndarray = np.asarray(value, dtype=np.float64)
    if array.shape != shape:
        raise SteadyDetectorError(f"{name} must have shape {shape}")
    if not np.all(np.isfinite(array)):
        raise SteadyDetectorError(f"{name} must contain finite values")
    return array


def _covariance(samples: Iterable[np.ndarray]) -> np.ndarray:
    """Return the covariance of a sequence of 3D samples."""
    data_list: list[np.ndarray] = [
        np.asarray(sample, dtype=np.float64) for sample in samples
    ]
    if not data_list:
        raise SteadyDetectorError("covariance requires at least one sample")
    data: np.ndarray = np.stack(data_list, axis=0)
    mean: np.ndarray = np.mean(data, axis=0)
    centered: np.ndarray = data - mean
    cov: np.ndarray = (centered.T @ centered) / float(data.shape[0])
    return 0.5 * (cov + cov.T)


@dataclass(frozen=True)
class _ImuSample:
    """Internal representation of an IMU sample."""

    t_ns: int
    omega_corr_rads: np.ndarray
    a_corr_mps2: np.ndarray


@dataclass(frozen=True)
class _MagSample:
    """Internal representation of a mag sample."""

    t_ns: int
    frame_id: str
    m_T: np.ndarray


class SteadyDetector:
    """Detect steady intervals using a time-based window."""

    def __init__(self, params: MountingParams) -> None:
        """Create a steady detector from mounting parameters."""
        self._params: MountingParams = params
        self._steady_ns: int = int(round(params.steady.steady_sec * 1e9))
        if self._steady_ns <= 0:
            raise SteadyDetectorError("steady_sec must be positive")
        if params.steady.window_type != "sliding":
            raise SteadyDetectorError("Only sliding window_type is supported")
        self._imu_samples: list[_ImuSample] = []
        self._mag_samples: list[_MagSample] = []
        self._last_t_ns: int | None = None
        self._last_mag_t_ns: int | None = None
        self._was_steady: bool = False
        self._steady_enter_ns: int | None = None
        self._has_emitted: bool = False

    def reset(self) -> None:
        """Reset internal buffers and emission state."""
        self._imu_samples = []
        self._mag_samples = []
        self._last_t_ns = None
        self._last_mag_t_ns = None
        self._was_steady = False
        self._steady_enter_ns = None
        self._has_emitted = False

    def window_is_steady(self) -> bool:
        """Return True when the current window satisfies steady criteria."""
        return self._window_is_steady()

    def push(
        self,
        *,
        t_ns: int,
        omega_corr_rads: np.ndarray,
        a_corr_mps2: np.ndarray,
        imu_frame_id: str,
        mag: MagPacket | None = None,
    ) -> SteadySegment | None:
        """Push a corrected IMU sample and return a steady segment if emitted."""
        if not isinstance(t_ns, int) or isinstance(t_ns, bool):
            raise SteadyDetectorError("t_ns must be an int")
        if self._last_t_ns is not None and t_ns <= self._last_t_ns:
            raise SteadyDetectorError("t_ns must be strictly increasing")
        if not isinstance(imu_frame_id, str):
            raise SteadyDetectorError("imu_frame_id must be a str")

        omega_corr: np.ndarray = _as_float_array(
            omega_corr_rads, "omega_corr_rads", (3,)
        )
        a_corr: np.ndarray = _as_float_array(a_corr_mps2, "a_corr_mps2", (3,))

        self._last_t_ns = t_ns
        self._imu_samples.append(
            _ImuSample(t_ns=t_ns, omega_corr_rads=omega_corr, a_corr_mps2=a_corr)
        )

        if mag is not None:
            if self._last_mag_t_ns is not None and mag.t_meas_ns <= self._last_mag_t_ns:
                raise SteadyDetectorError("mag.t_meas_ns must be strictly increasing")
            if not isinstance(mag.frame_id, str):
                raise SteadyDetectorError("mag.frame_id must be a str")
            m_T: np.ndarray = _as_float_array(mag.m_raw_T, "mag.m_raw_T", (3,))
            self._mag_samples.append(
                _MagSample(t_ns=mag.t_meas_ns, frame_id=mag.frame_id, m_T=m_T)
            )
            self._last_mag_t_ns = mag.t_meas_ns

        self._trim_buffers(t_ns)

        is_steady: bool = self._window_is_steady()
        if not is_steady:
            self._was_steady = False
            self._steady_enter_ns = None
            self._has_emitted = False
            return None

        if not self._was_steady:
            self._was_steady = True
            self._steady_enter_ns = t_ns
            self._has_emitted = False
        elif self._steady_enter_ns is None:
            self._steady_enter_ns = t_ns

        if self._has_emitted:
            return None

        if t_ns - self._steady_enter_ns < self._steady_ns:
            return None

        segment: SteadySegment = self._build_segment(imu_frame_id)
        self._has_emitted = True
        return segment

    def _trim_buffers(self, t_ns: int) -> None:
        """Trim samples outside the current steady window."""
        window_start: int = t_ns - self._steady_ns
        self._imu_samples = [
            sample for sample in self._imu_samples if sample.t_ns >= window_start
        ]
        self._mag_samples = [
            sample for sample in self._mag_samples if sample.t_ns >= window_start
        ]

    def _window_is_steady(self) -> bool:
        """Return True when the current window satisfies steady criteria."""
        if not self._imu_samples:
            return False
        window_span_ns: int = self._imu_samples[-1].t_ns - self._imu_samples[0].t_ns
        if window_span_ns < self._steady_ns:
            return False
        omega_samples: list[np.ndarray] = [
            sample.omega_corr_rads for sample in self._imu_samples
        ]
        a_samples: list[np.ndarray] = [
            sample.a_corr_mps2 for sample in self._imu_samples
        ]

        omega_mean: np.ndarray = np.mean(np.stack(omega_samples, axis=0), axis=0)
        a_mean: np.ndarray = np.mean(np.stack(a_samples, axis=0), axis=0)
        omega_cov: np.ndarray = _covariance(omega_samples)
        a_cov: np.ndarray = _covariance(a_samples)

        omega_mean_norm: float = float(np.linalg.norm(omega_mean))
        a_mean_norm: float = float(np.linalg.norm(a_mean))

        omega_mean_thresh: float | None = self._params.steady.omega_mean_thresh
        if omega_mean_thresh is not None and omega_mean_norm >= omega_mean_thresh:
            return False

        omega_cov_thresh: float | None = self._params.steady.omega_cov_thresh
        if (
            omega_cov_thresh is not None
            and float(np.trace(omega_cov)) >= omega_cov_thresh
        ):
            return False

        a_cov_thresh: float | None = self._params.steady.a_cov_thresh
        if a_cov_thresh is not None and float(np.trace(a_cov)) >= a_cov_thresh:
            return False

        a_norm_min: float | None = self._params.steady.a_norm_min
        if a_norm_min is not None and a_mean_norm <= a_norm_min:
            return False

        a_norm_max: float | None = self._params.steady.a_norm_max
        if a_norm_max is not None and a_mean_norm >= a_norm_max:
            return False

        return True

    def _build_segment(self, imu_frame_id: str) -> SteadySegment:
        """Build a SteadySegment from the current window."""
        samples: list[_ImuSample] = list(self._imu_samples)
        if not samples:
            raise SteadyDetectorError("no samples available for segment")
        t_start_ns: int = samples[0].t_ns
        t_end_ns: int = samples[-1].t_ns
        t_meas_ns: int = (t_start_ns + t_end_ns) // 2
        a_samples: list[np.ndarray] = [sample.a_corr_mps2 for sample in samples]
        a_mean: np.ndarray = np.mean(np.stack(a_samples, axis=0), axis=0)
        cov_a: np.ndarray = _covariance(a_samples)

        mag_samples: list[_MagSample] = [
            sample
            for sample in self._mag_samples
            if t_start_ns <= sample.t_ns <= t_end_ns
        ]
        mag_frame_id: str | None
        m_mean: np.ndarray | None
        cov_m: np.ndarray | None
        if mag_samples:
            mag_frame_id = mag_samples[0].frame_id
            if any(sample.frame_id != mag_frame_id for sample in mag_samples):
                raise SteadyDetectorError("mag_frame_id must be consistent")
            m_samples: list[np.ndarray] = [sample.m_T for sample in mag_samples]
            m_mean = np.mean(np.stack(m_samples, axis=0), axis=0)
            cov_m = _covariance(m_samples)
        else:
            mag_frame_id = None
            m_mean = None
            cov_m = None

        return SteadySegment(
            t_start_ns=t_start_ns,
            t_end_ns=t_end_ns,
            t_meas_ns=t_meas_ns,
            imu_frame_id=imu_frame_id,
            mag_frame_id=mag_frame_id,
            a_mean_mps2=a_mean,
            cov_a=cov_a,
            m_mean_T=m_mean,
            cov_m=cov_m,
            sample_count=len(samples),
            duration_ns=t_end_ns - t_start_ns,
        )
