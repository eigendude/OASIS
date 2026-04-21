################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque
from typing import Optional

from oasis_control.localization.common.algebra.quat import Matrix3
from oasis_control.localization.common.algebra.quat import Quaternion
from oasis_control.localization.common.measurements.gravity_observable_attitude import (
    GravityObservableAttitudeVariance,
)
from oasis_control.localization.common.measurements.gravity_observable_attitude import (
    gravity_covariance_to_roll_pitch_variance_rad2,
)


# Units: s
# Meaning: recent non-turning yaw history used to measure HUD-visible heading
# stability
DEFAULT_YAW_JITTER_WINDOW_SEC: float = 2.0

# Units: deg/s
# Meaning: heading-rate threshold above which recent yaw spread is treated as
# active turning instead of jitter
DEFAULT_YAW_JITTER_TURN_RATE_THRESHOLD_DEGPS: float = 10.0


@dataclass(frozen=True)
class AttitudeUncertaintyEstimate:
    """
    Orientation covariance estimate for the AHRS output contract.

    Fields:
        orientation_covariance_rad2: 3x3 roll/pitch/yaw covariance in
            `base_link` when gravity-observable roll/pitch uncertainty is
            available
        orientation_covariance_unknown: true when gravity-derived roll/pitch
            uncertainty is unavailable
    """

    orientation_covariance_rad2: Optional[Matrix3]
    orientation_covariance_unknown: bool


@dataclass(frozen=True)
class _YawSample:
    """
    One unwrapped yaw observation used by the recent jitter estimator.
    """

    timestamp_ns: int
    unwrapped_yaw_rad: float


class RecentYawJitterEstimator:
    """
    Estimate recent yaw jitter from unwrapped yaw samples.

    The estimator tracks a short rolling window of heading samples, unwraps the
    `[-pi, pi]` discontinuity, and reports a population standard deviation in
    radians. Fast turns are treated as intentional motion instead of jitter, so
    the last stable estimate is held while the rolling window resets.
    """

    def __init__(
        self,
        *,
        window_sec: float = DEFAULT_YAW_JITTER_WINDOW_SEC,
        turn_rate_threshold_degps: float = (
            DEFAULT_YAW_JITTER_TURN_RATE_THRESHOLD_DEGPS
        ),
    ) -> None:
        self._window_ns: int = int(window_sec * 1.0e9)
        self._turn_rate_threshold_rads: float = math.radians(turn_rate_threshold_degps)
        self._has_last_yaw: bool = False
        self._last_timestamp_ns: int = 0
        self._last_wrapped_yaw_rad: float = 0.0
        self._last_unwrapped_yaw_rad: float = 0.0
        self._yaw_jitter_stddev_rad: float = 0.0
        self._recent_samples: Deque[_YawSample] = deque()

    def update(self, *, timestamp_ns: int, yaw_rad: float) -> float:
        """
        Update the estimator and return the latest yaw jitter standard
        deviation.
        """

        if not math.isfinite(yaw_rad):
            return self._yaw_jitter_stddev_rad

        if not self._has_last_yaw:
            self._has_last_yaw = True
            self._last_timestamp_ns = timestamp_ns
            self._last_wrapped_yaw_rad = yaw_rad
            self._last_unwrapped_yaw_rad = yaw_rad
            self._recent_samples.append(
                _YawSample(timestamp_ns=timestamp_ns, unwrapped_yaw_rad=yaw_rad)
            )
            self._yaw_jitter_stddev_rad = 0.0
            return self._yaw_jitter_stddev_rad

        delta_timestamp_ns: int = timestamp_ns - self._last_timestamp_ns
        if delta_timestamp_ns <= 0:
            return self._yaw_jitter_stddev_rad

        delta_yaw_rad: float = _normalize_angle_rad(
            yaw_rad - self._last_wrapped_yaw_rad
        )
        unwrapped_yaw_rad: float = self._last_unwrapped_yaw_rad + delta_yaw_rad

        self._last_timestamp_ns = timestamp_ns
        self._last_wrapped_yaw_rad = yaw_rad
        self._last_unwrapped_yaw_rad = unwrapped_yaw_rad

        delta_time_sec: float = float(delta_timestamp_ns) / 1.0e9
        yaw_rate_rads: float = abs(delta_yaw_rad) / delta_time_sec

        self._trim_window(newest_timestamp_ns=timestamp_ns)
        if yaw_rate_rads > self._turn_rate_threshold_rads:
            self._recent_samples.clear()
            self._recent_samples.append(
                _YawSample(
                    timestamp_ns=timestamp_ns,
                    unwrapped_yaw_rad=unwrapped_yaw_rad,
                )
            )
            return self._yaw_jitter_stddev_rad

        self._recent_samples.append(
            _YawSample(
                timestamp_ns=timestamp_ns,
                unwrapped_yaw_rad=unwrapped_yaw_rad,
            )
        )
        self._trim_window(newest_timestamp_ns=timestamp_ns)
        self._yaw_jitter_stddev_rad = _population_stddev_rad(
            tuple(sample.unwrapped_yaw_rad for sample in self._recent_samples)
        )
        return self._yaw_jitter_stddev_rad

    @property
    def yaw_jitter_stddev_rad(self) -> float:
        return self._yaw_jitter_stddev_rad

    def _trim_window(self, *, newest_timestamp_ns: int) -> None:
        oldest_allowed_timestamp_ns: int = newest_timestamp_ns - self._window_ns
        while (
            len(self._recent_samples) > 0
            and self._recent_samples[0].timestamp_ns < oldest_allowed_timestamp_ns
        ):
            self._recent_samples.popleft()


class AhrsOrientationUncertaintyEstimator:
    """
    Build the published AHRS orientation covariance.

    Roll and pitch variance come from the gravity-observable attitude
    primitive. Yaw variance comes from recent yaw jitter on the published
    attitude stream.
    """

    def __init__(self) -> None:
        self._yaw_jitter_estimator: RecentYawJitterEstimator = (
            RecentYawJitterEstimator()
        )

    def update(
        self,
        *,
        timestamp_ns: int,
        orientation_xyzw: Quaternion,
        gravity_mps2: Optional[tuple[float, float, float]],
        gravity_covariance_mps2_2: Optional[Matrix3],
    ) -> AttitudeUncertaintyEstimate:
        """
        Update the uncertainty estimate for one published AHRS sample.
        """

        yaw_stddev_rad: float = self._yaw_jitter_estimator.update(
            timestamp_ns=timestamp_ns,
            yaw_rad=_yaw_from_quaternion_xyzw(orientation_xyzw),
        )

        roll_pitch_variance: GravityObservableAttitudeVariance = (
            gravity_covariance_to_roll_pitch_variance_rad2(
                gravity_mps2=(
                    gravity_mps2 if gravity_mps2 is not None else (0.0, 0.0, 0.0)
                ),
                gravity_covariance_mps2_2=gravity_covariance_mps2_2,
            )
        )
        if (
            roll_pitch_variance.roll_variance_rad2 <= 0.0
            or roll_pitch_variance.pitch_variance_rad2 <= 0.0
        ):
            return AttitudeUncertaintyEstimate(
                orientation_covariance_rad2=None,
                orientation_covariance_unknown=True,
            )

        yaw_variance_rad2: float = yaw_stddev_rad * yaw_stddev_rad
        return AttitudeUncertaintyEstimate(
            orientation_covariance_rad2=(
                (roll_pitch_variance.roll_variance_rad2, 0.0, 0.0),
                (0.0, roll_pitch_variance.pitch_variance_rad2, 0.0),
                (0.0, 0.0, yaw_variance_rad2),
            ),
            orientation_covariance_unknown=False,
        )


def _yaw_from_quaternion_xyzw(quaternion_xyzw: Quaternion) -> float:
    """
    Extract yaw from a ROS xyzw quaternion in radians.
    """

    x_value: float = quaternion_xyzw[0]
    y_value: float = quaternion_xyzw[1]
    z_value: float = quaternion_xyzw[2]
    w_value: float = quaternion_xyzw[3]

    return math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value),
    )


def _normalize_angle_rad(angle_rad: float) -> float:
    """
    Wrap an angle into the `[-pi, pi]` interval.
    """

    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _population_stddev_rad(samples_rad: tuple[float, ...]) -> float:
    """
    Return the population standard deviation of finite radian samples.
    """

    if len(samples_rad) < 2:
        return 0.0

    mean_rad: float = sum(samples_rad) / float(len(samples_rad))
    variance_rad2: float = sum(
        (sample_rad - mean_rad) * (sample_rad - mean_rad) for sample_rad in samples_rad
    ) / float(len(samples_rad))
    return math.sqrt(max(0.0, variance_rad2))
