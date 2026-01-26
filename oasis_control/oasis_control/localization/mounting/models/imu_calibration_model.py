################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""IMU calibration model helpers for mounting calibration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from oasis_control.localization.mounting.mounting_types import ImuCalibrationPrior


class ImuCalibrationModelError(Exception):
    """Raised when IMU calibration model validation fails."""


def _as_float_array(value: Any, name: str, shape: tuple[int, ...]) -> np.ndarray:
    """Return a float64 numpy array with a required shape."""
    array: np.ndarray = np.asarray(value, dtype=np.float64)
    if array.shape != shape:
        raise ImuCalibrationModelError(f"{name} must have shape {shape}")
    if not np.all(np.isfinite(array)):
        raise ImuCalibrationModelError(f"{name} must contain finite values")
    return array


@dataclass(frozen=True)
class ImuNuisanceState:
    """Nuisance calibration state for IMU measurements.

    Attributes:
        b_a_mps2: Accelerometer bias in m/s^2
        A_a: Accelerometer scale/misalignment matrix
        b_g_rads: Gyroscope bias in rad/s
    """

    b_a_mps2: np.ndarray
    A_a: np.ndarray
    b_g_rads: np.ndarray

    def __post_init__(self) -> None:
        """Validate nuisance state fields."""
        b_a_mps2: np.ndarray = _as_float_array(self.b_a_mps2, "b_a_mps2", (3,))
        A_a: np.ndarray = _as_float_array(self.A_a, "A_a", (3, 3))
        b_g_rads: np.ndarray = _as_float_array(self.b_g_rads, "b_g_rads", (3,))

        object.__setattr__(self, "b_a_mps2", b_a_mps2)
        object.__setattr__(self, "A_a", A_a)
        object.__setattr__(self, "b_g_rads", b_g_rads)

    def correct_accel(self, a_raw_mps2: np.ndarray) -> np.ndarray:
        """Return calibrated accelerometer data using the nuisance state."""
        a_raw: np.ndarray = _as_float_array(a_raw_mps2, "a_raw_mps2", (3,))
        return self.A_a @ (a_raw - self.b_a_mps2)

    def correct_gyro(self, omega_raw_rads: np.ndarray) -> np.ndarray:
        """Return calibrated gyro data using the nuisance state."""
        omega_raw: np.ndarray = _as_float_array(omega_raw_rads, "omega_raw_rads", (3,))
        return omega_raw - self.b_g_rads


class ImuCalibrationModel:
    """Factory helpers for IMU nuisance state initialization."""

    @staticmethod
    def default_state() -> ImuNuisanceState:
        """Return the default nuisance state with zero biases."""
        return ImuNuisanceState(
            b_a_mps2=np.zeros(3, dtype=np.float64),
            A_a=np.eye(3, dtype=np.float64),
            b_g_rads=np.zeros(3, dtype=np.float64),
        )

    @staticmethod
    def state_from_prior(prior: ImuCalibrationPrior) -> ImuNuisanceState:
        """Create a nuisance state from a one-shot calibration prior."""
        if not prior.valid:
            return ImuCalibrationModel.default_state()
        return ImuNuisanceState(
            b_a_mps2=prior.b_a_mps2,
            A_a=prior.A_a,
            b_g_rads=prior.b_g_rads,
        )
