################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""IMU packet types for mounting calibration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class ImuCalibrationPrior:
    """One-shot calibration prior for IMU nuisance parameters.

    Attributes:
        valid: Indicates whether the prior should be used
        frame_id: Frame name associated with the prior
        b_a_mps2: Accelerometer bias prior in m/s^2
        A_a: Accelerometer scale/misalignment matrix
        b_g_rads: Gyroscope bias prior in rad/s
        cov_a_params: Covariance of [b_a, vec(A_a)] in (m/s^2)^2
        cov_b_g: Covariance of gyroscope bias in (rad/s)^2
    """

    valid: bool
    frame_id: str
    b_a_mps2: np.ndarray
    A_a: np.ndarray
    b_g_rads: np.ndarray
    cov_a_params: np.ndarray | None
    cov_b_g: np.ndarray | None

    def __post_init__(self) -> None:
        """Validate calibration prior fields."""
        if not isinstance(self.valid, bool):
            raise ValueError("valid must be a bool")
        if not isinstance(self.frame_id, str):
            raise ValueError("frame_id must be a str")

        b_a_mps2: np.ndarray = _as_float_array(self.b_a_mps2, "b_a_mps2", (3,))
        A_a: np.ndarray = _as_float_array(self.A_a, "A_a", (3, 3))
        b_g_rads: np.ndarray = _as_float_array(self.b_g_rads, "b_g_rads", (3,))

        object.__setattr__(self, "b_a_mps2", b_a_mps2)
        object.__setattr__(self, "A_a", A_a)
        object.__setattr__(self, "b_g_rads", b_g_rads)

        if self.cov_a_params is None:
            cov_a_params: np.ndarray | None = None
        else:
            cov_a_params = _as_float_array(
                self.cov_a_params,
                "cov_a_params",
                (12, 12),
            )
        if self.cov_b_g is None:
            cov_b_g: np.ndarray | None = None
        else:
            cov_b_g = _as_float_array(self.cov_b_g, "cov_b_g", (3, 3))

        object.__setattr__(self, "cov_a_params", cov_a_params)
        object.__setattr__(self, "cov_b_g", cov_b_g)


@dataclass(frozen=True)
class ImuPacket:
    """Raw IMU packet with optional calibration prior."""

    t_meas_ns: int
    frame_id: str
    omega_raw_rads: np.ndarray
    cov_omega_raw: np.ndarray
    a_raw_mps2: np.ndarray
    cov_a_raw: np.ndarray
    calibration: ImuCalibrationPrior | None = None

    def __post_init__(self) -> None:
        """Validate packet fields and coerce arrays."""
        if not isinstance(self.t_meas_ns, int) or isinstance(self.t_meas_ns, bool):
            raise ValueError("t_meas_ns must be an int")
        if not isinstance(self.frame_id, str):
            raise ValueError("frame_id must be a str")

        omega_raw_rads: np.ndarray = _as_float_array(
            self.omega_raw_rads,
            "omega_raw_rads",
            (3,),
        )
        cov_omega_raw: np.ndarray = _as_float_array(
            self.cov_omega_raw,
            "cov_omega_raw",
            (3, 3),
        )
        a_raw_mps2: np.ndarray = _as_float_array(self.a_raw_mps2, "a_raw_mps2", (3,))
        cov_a_raw: np.ndarray = _as_float_array(
            self.cov_a_raw,
            "cov_a_raw",
            (3, 3),
        )

        object.__setattr__(self, "omega_raw_rads", omega_raw_rads)
        object.__setattr__(self, "cov_omega_raw", cov_omega_raw)
        object.__setattr__(self, "a_raw_mps2", a_raw_mps2)
        object.__setattr__(self, "cov_a_raw", cov_a_raw)

    def has_calibration(self) -> bool:
        """Return True when a calibration prior is available."""
        return self.calibration is not None


def _as_float_array(value: Any, name: str, shape: tuple[int, ...]) -> np.ndarray:
    """Coerce a value to a float64 numpy array with a specific shape."""
    array: np.ndarray = np.asarray(value, dtype=np.float64)
    if array.shape != shape:
        raise ValueError(f"{name} must have shape {shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain finite values")
    return array
