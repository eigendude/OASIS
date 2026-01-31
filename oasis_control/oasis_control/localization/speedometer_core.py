################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Pure Python wrapper around the forward velocity estimator."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
from numpy.typing import NDArray

from oasis_control.localization.velocity_estimator import VelocityEstimator
from oasis_control.localization.velocity_estimator import VelocityEstimatorConfig


_FLOAT_ARRAY = NDArray[np.float64]


@dataclass
class SpeedometerCoreConfig:
    """Configuration values for the speedometer core."""

    # Forward speed process noise in (m/s)^2 per second. Used in
    # prediction; expected >= 0.0.
    process_noise_v: float = 0.4

    # Bias process noise in (m/s^2)^2 per second. Used in prediction;
    # expected >= 0.0.
    process_noise_b: float = 0.04

    # Initial forward speed sigma in m/s. Used to seed covariance; expected
    # >= 0.0.
    initial_speed_sigma: float = 0.5

    # Initial acceleration bias sigma in m/s^2. Used to seed covariance;
    # expected >= 0.0.
    initial_bias_sigma: float = 0.2

    # Maximum allowed dt in seconds before clamping to zero. Used to
    # reject large gaps; expected > 0.0.
    max_dt_sec: float = 0.2

    # Minimum allowed dt in seconds before clamping to zero. Used to
    # reject negative dt; expected >= 0.0.
    min_dt_sec: float = 0.0

    # Minimum ZUPT variance in (m/s)^2 to avoid divide-by-zero. Used in
    # update; expected > 0.0.
    zupt_var_floor: float = 1e-12


class SpeedometerCore:
    """ROS-agnostic wrapper for forward velocity estimation."""

    def __init__(self, config: SpeedometerCoreConfig) -> None:
        self._config: SpeedometerCoreConfig = config
        self._estimator: VelocityEstimator = VelocityEstimator(
            self._build_estimator_config(config)
        )

    def set_bias_priors(self, bias_mean: float, bias_var: float) -> None:
        """Apply bias priors if the values are finite and positive."""

        bias_mean_val: float = float(bias_mean)
        bias_var_val: float = float(bias_var)
        if not _is_finite(bias_mean_val):
            return
        if not _is_finite(bias_var_val) or bias_var_val <= 0.0:
            return
        self._estimator.set_priors(bias_mean_val, bias_var_val)

    def predict(
        self,
        timestamp_sec: float,
        a_f_mps2: float,
        a_f_var: Optional[float] = None,
    ) -> dict[str, object]:
        """Propagate the estimator using forward acceleration."""

        return self._estimator.predict(timestamp_sec, a_f_mps2, a_f_var)

    def apply_zupt(self, timestamp_sec: float, zupt_var: float) -> dict[str, object]:
        """Apply a zero-velocity measurement update."""

        zupt_var_val: float = float(zupt_var)
        if not _is_finite(zupt_var_val) or zupt_var_val <= 0.0:
            zupt_var_val = self._config.zupt_var_floor
        return self._estimator.update_zupt(timestamp_sec, zupt_var_val)

    def snapshot(self) -> dict[str, object]:
        """Return the latest estimator diagnostics snapshot."""

        return self._estimator.get_diagnostics_snapshot()

    def get_velocity_mps(self) -> float:
        """Return the estimated forward speed in m/s."""

        return self._estimator.get_velocity_mps()

    def get_covariance(self) -> _FLOAT_ARRAY:
        """Return a copy of the state covariance matrix."""

        return self._estimator.get_covariance()

    def _build_estimator_config(
        self, config: SpeedometerCoreConfig
    ) -> VelocityEstimatorConfig:
        return VelocityEstimatorConfig(
            process_noise_v=config.process_noise_v,
            process_noise_b=config.process_noise_b,
            initial_speed_sigma=config.initial_speed_sigma,
            initial_bias_sigma=config.initial_bias_sigma,
            max_dt_sec=config.max_dt_sec,
            min_dt_sec=config.min_dt_sec,
            zupt_var_floor=config.zupt_var_floor,
        )


def _is_finite(value: float) -> bool:
    return math.isfinite(value)
