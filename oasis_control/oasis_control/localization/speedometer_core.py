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
from dataclasses import dataclass
from typing import Iterable
from typing import Optional


################################################################################
# Speedometer estimator
################################################################################


# Units: m/s^2
# Meaning: reject degenerate acceleration and gravity vectors
MIN_VECTOR_NORM_MPS2: float = 1.0e-6

# Units: m/s^2
# Meaning: tiny floor to keep published speed variance strictly positive
MIN_SPEED_VARIANCE_MPS2: float = 1.0e-6

# Units: m^2/s^3
# Meaning: ZUPT variance below this threshold is treated as stationary
STATIONARY_ZUPT_THRESHOLD_MPS2: float = 1.0


@dataclass(frozen=True)
class SpeedometerConfig:
    """
    Tunable parameters for the HUD-focused speed estimator.

    Fields:
        axis_learning_accel_threshold_mps2: minimum gravity-removed
            acceleration magnitude used as motion-axis evidence
        axis_learning_max_gyro_threshold_rads: maximum gyro magnitude accepted
            during axis learning, or a nonpositive value to disable the gate
        axis_learning_min_samples: minimum accepted motion samples before axis
            lock-in
        axis_learning_min_confidence: minimum alignment confidence in [0, 1]
            from the sign-aligned acceleration accumulator
        unlocked_speed_std_mps: published 1-sigma speed uncertainty before the
            motion axis is learned and the estimator becomes informative
        default_forward_accel_std_mps2: fallback 1-sigma acceleration noise
            used when IMU covariance is unknown
        process_bias_walk_std_mps2: 1-sigma bias random walk per sqrt(second)
        initial_speed_std_mps: initial 1-sigma uncertainty on signed speed
        initial_bias_std_mps2: initial 1-sigma uncertainty on acceleration
            bias projected onto the learned axis
        max_predict_timestamp_jitter_sec: maximum tolerated backward IMU
            timestamp jitter that is dropped as a no-op instead of rejected
    """

    axis_learning_accel_threshold_mps2: float = 0.35
    axis_learning_max_gyro_threshold_rads: float = 0.75
    axis_learning_min_samples: int = 30
    axis_learning_min_confidence: float = 0.70
    unlocked_speed_std_mps: float = 8.0
    default_forward_accel_std_mps2: float = 0.6
    process_bias_walk_std_mps2: float = 0.02
    initial_speed_std_mps: float = 5.0
    initial_bias_std_mps2: float = 0.4
    max_predict_timestamp_jitter_sec: float = 0.003


@dataclass
class SpeedometerState:
    """
    Mutable estimator state.

    Fields:
        speed_mps: signed speed estimate along the learned boot-relative axis
        accel_bias_mps2: signed projected acceleration bias along that axis
        speed_variance_mps2: variance of the signed speed state
        speed_bias_covariance_mps3: covariance between speed and bias states
        bias_variance_mps2_2: variance of the projected acceleration bias
        last_predict_timestamp_sec: last accepted IMU timestamp
        learned_motion_axis_imu: learned unit motion axis in IMU coordinates,
            up to sign
        axis_sample_count: accepted motion-evidence sample count
        axis_evidence_direction_sum: sign-aligned sum of normalized dynamic
            acceleration directions used for axis lock-in
        axis_confidence: current consistency score in [0, 1]
        stationary_hint: last ZUPT-derived stationary hint
    """

    speed_mps: float
    accel_bias_mps2: float
    speed_variance_mps2: float
    speed_bias_covariance_mps3: float
    bias_variance_mps2_2: float
    last_predict_timestamp_sec: Optional[float]
    learned_motion_axis_imu: Optional[tuple[float, float, float]]
    axis_sample_count: int
    axis_evidence_direction_sum: tuple[float, float, float]
    axis_confidence: float
    stationary_hint: bool


@dataclass(frozen=True)
class SpeedometerEstimate:
    """
    HUD-facing speed estimate.

    Fields:
        speed_mps: nonnegative scalar speed magnitude for HUD display
        speed_variance_mps2: variance of the published scalar speed magnitude
        axis_learned: true once the boot-relative motion axis has locked
        motion_axis_imu: learned unit motion axis in IMU coordinates, or None
            before lock-in
        axis_confidence: current axis-learning confidence in [0, 1]
    """

    speed_mps: float
    speed_variance_mps2: float
    axis_learned: bool
    motion_axis_imu: Optional[tuple[float, float, float]]
    axis_confidence: float


class SpeedometerCore:
    """
    Estimate HUD speed magnitude from IMU and ZUPT measurements.

    The estimator learns a fixed motion axis in the IMU frame after boot from
    gravity-removed acceleration samples. Once the axis is available, it tracks
    a signed speed and projected acceleration bias along that axis, then
    publishes the unsigned magnitude for the HUD.
    """

    def __init__(self, config: SpeedometerConfig) -> None:
        """Initialize the speedometer core."""

        self._config: SpeedometerConfig = config
        self._state: SpeedometerState = SpeedometerState(
            speed_mps=0.0,
            accel_bias_mps2=0.0,
            speed_variance_mps2=max(
                MIN_SPEED_VARIANCE_MPS2, config.initial_speed_std_mps**2
            ),
            speed_bias_covariance_mps3=0.0,
            bias_variance_mps2_2=max(
                MIN_SPEED_VARIANCE_MPS2, config.initial_bias_std_mps2**2
            ),
            last_predict_timestamp_sec=None,
            learned_motion_axis_imu=None,
            axis_sample_count=0,
            axis_evidence_direction_sum=(0.0, 0.0, 0.0),
            axis_confidence=0.0,
            stationary_hint=False,
        )

    @property
    def state(self) -> SpeedometerState:
        """Return the mutable estimator state."""

        return self._state

    def handle_imu_sample(
        self,
        timestamp_sec: float,
        linear_accel_mps2: Iterable[float],
        angular_velocity_rads: Iterable[float],
        linear_accel_covariance_mps2_2: Optional[Iterable[Iterable[float]]] = None,
    ) -> Optional[SpeedometerEstimate]:
        """
        Consume one IMU sample and return the current HUD estimate.
        """

        if not math.isfinite(timestamp_sec):
            return None

        linear_accel_vector_mps2: Optional[tuple[float, float, float]] = _vector3(
            linear_accel_mps2
        )
        if linear_accel_vector_mps2 is None:
            return None

        angular_velocity_vector_rads: Optional[tuple[float, float, float]] = _vector3(
            angular_velocity_rads
        )
        if angular_velocity_vector_rads is None:
            return None

        timestamp_status: str = _classify_predict_timestamp(
            previous_timestamp_sec=self._state.last_predict_timestamp_sec,
            timestamp_sec=timestamp_sec,
            max_backward_jitter_sec=(self._config.max_predict_timestamp_jitter_sec),
        )
        if timestamp_status == "reject":
            return None
        if timestamp_status == "drop":
            return self.get_estimate()

        dt_sec: Optional[float] = _compute_dt_sec(
            previous_timestamp_sec=self._state.last_predict_timestamp_sec,
            timestamp_sec=timestamp_sec,
        )
        self._state.last_predict_timestamp_sec = timestamp_sec

        if self._state.learned_motion_axis_imu is None:
            self._update_axis_learning(
                dynamic_accel_mps2=linear_accel_vector_mps2,
                angular_velocity_rads=angular_velocity_vector_rads,
            )

        if dt_sec is None or dt_sec <= 0.0:
            return self.get_estimate()

        if self._state.learned_motion_axis_imu is None:
            return self.get_estimate()

        motion_axis_imu: tuple[float, float, float] = (
            self._state.learned_motion_axis_imu
        )
        projected_accel_mps2: float = _dot(linear_accel_vector_mps2, motion_axis_imu)
        accel_variance_mps2_2: float = self._project_accel_variance_mps2_2(
            motion_axis_imu=motion_axis_imu,
            linear_accel_covariance_mps2_2=linear_accel_covariance_mps2_2,
        )
        self._predict_state(
            dt_sec=dt_sec,
            projected_accel_mps2=projected_accel_mps2,
            accel_variance_mps2_2=accel_variance_mps2_2,
        )
        return self.get_estimate()

    def handle_zupt(
        self, timestamp_sec: float, zero_velocity_variance_mps2: float
    ) -> Optional[SpeedometerEstimate]:
        """
        Consume a zero-velocity measurement and return the current HUD estimate.
        """

        if not math.isfinite(timestamp_sec):
            return None

        measurement_variance_mps2: float = float(zero_velocity_variance_mps2)
        if (
            not math.isfinite(measurement_variance_mps2)
            or measurement_variance_mps2 <= 0.0
        ):
            return self.get_estimate()

        self._state.stationary_hint = (
            measurement_variance_mps2 <= STATIONARY_ZUPT_THRESHOLD_MPS2
        )

        innovation_variance_mps2: float = (
            self._state.speed_variance_mps2 + measurement_variance_mps2
        )
        if innovation_variance_mps2 <= 0.0:
            return self.get_estimate()

        kalman_gain_speed: float = (
            self._state.speed_variance_mps2 / innovation_variance_mps2
        )
        kalman_gain_bias: float = (
            self._state.speed_bias_covariance_mps3 / innovation_variance_mps2
        )
        innovation_mps: float = -self._state.speed_mps

        self._state.speed_mps += kalman_gain_speed * innovation_mps
        self._state.accel_bias_mps2 += kalman_gain_bias * innovation_mps

        updated_speed_variance_mps2: float = (
            1.0 - kalman_gain_speed
        ) * self._state.speed_variance_mps2
        updated_speed_bias_covariance_mps3: float = (
            1.0 - kalman_gain_speed
        ) * self._state.speed_bias_covariance_mps3
        updated_bias_variance_mps2_2: float = (
            self._state.bias_variance_mps2_2
            - kalman_gain_bias * self._state.speed_bias_covariance_mps3
        )

        self._state.speed_variance_mps2 = max(
            MIN_SPEED_VARIANCE_MPS2, updated_speed_variance_mps2
        )
        self._state.speed_bias_covariance_mps3 = updated_speed_bias_covariance_mps3
        self._state.bias_variance_mps2_2 = max(
            MIN_SPEED_VARIANCE_MPS2, updated_bias_variance_mps2_2
        )
        return self.get_estimate()

    def get_estimate(self) -> SpeedometerEstimate:
        """
        Return the current HUD estimate.
        """

        return SpeedometerEstimate(
            speed_mps=(
                abs(self._state.speed_mps)
                if self._state.learned_motion_axis_imu is not None
                else 0.0
            ),
            speed_variance_mps2=self._published_speed_variance_mps2(),
            axis_learned=self._state.learned_motion_axis_imu is not None,
            motion_axis_imu=self._state.learned_motion_axis_imu,
            axis_confidence=float(self._state.axis_confidence),
        )

    def _published_speed_variance_mps2(self) -> float:
        """
        Return the HUD-facing speed variance with an honest pre-lock policy.
        """

        if self._state.learned_motion_axis_imu is None:
            return max(
                MIN_SPEED_VARIANCE_MPS2,
                self._config.unlocked_speed_std_mps**2,
            )

        return max(MIN_SPEED_VARIANCE_MPS2, self._state.speed_variance_mps2)

    def _update_axis_learning(
        self,
        dynamic_accel_mps2: tuple[float, float, float],
        angular_velocity_rads: tuple[float, float, float],
    ) -> None:
        """
        Accumulate motion evidence for the boot-relative longitudinal axis.
        """

        dynamic_norm_mps2: float = _norm(dynamic_accel_mps2)
        if dynamic_norm_mps2 < self._config.axis_learning_accel_threshold_mps2:
            return

        if self._state.stationary_hint:
            return

        gyro_limit_rads: float = self._config.axis_learning_max_gyro_threshold_rads
        if gyro_limit_rads > 0.0 and _norm(angular_velocity_rads) > gyro_limit_rads:
            return

        candidate_direction: tuple[float, float, float] = (
            dynamic_accel_mps2[0] / dynamic_norm_mps2,
            dynamic_accel_mps2[1] / dynamic_norm_mps2,
            dynamic_accel_mps2[2] / dynamic_norm_mps2,
        )
        evidence_direction_sum: tuple[float, float, float] = (
            self._state.axis_evidence_direction_sum
        )

        if _dot(evidence_direction_sum, candidate_direction) < 0.0:
            candidate_direction = (
                -candidate_direction[0],
                -candidate_direction[1],
                -candidate_direction[2],
            )

        evidence_direction_sum = _add_vectors(
            evidence_direction_sum,
            candidate_direction,
        )
        self._state.axis_evidence_direction_sum = evidence_direction_sum
        self._state.axis_sample_count += 1

        accumulator_norm: float = _norm(evidence_direction_sum)
        self._state.axis_confidence = accumulator_norm / float(
            self._state.axis_sample_count
        )

        if self._state.axis_sample_count < self._config.axis_learning_min_samples:
            return

        if self._state.axis_confidence < self._config.axis_learning_min_confidence:
            return

        learned_axis_imu: Optional[tuple[float, float, float]] = _normalized_vector(
            evidence_direction_sum
        )
        if learned_axis_imu is None:
            return

        self._state.learned_motion_axis_imu = learned_axis_imu

    def _predict_state(
        self,
        dt_sec: float,
        projected_accel_mps2: float,
        accel_variance_mps2_2: float,
    ) -> None:
        """
        Propagate the 1D speed and bias filter with one IMU sample.
        """

        self._state.speed_mps += dt_sec * (
            projected_accel_mps2 - self._state.accel_bias_mps2
        )

        previous_speed_variance_mps2: float = self._state.speed_variance_mps2
        previous_speed_bias_covariance_mps3: float = (
            self._state.speed_bias_covariance_mps3
        )
        previous_bias_variance_mps2_2: float = self._state.bias_variance_mps2_2

        accel_process_variance_mps2: float = (
            dt_sec * dt_sec * max(MIN_SPEED_VARIANCE_MPS2, accel_variance_mps2_2)
        )

        bias_walk_variance_mps2_2: float = dt_sec * (
            self._config.process_bias_walk_std_mps2**2
        )

        self._state.speed_variance_mps2 = max(
            MIN_SPEED_VARIANCE_MPS2,
            previous_speed_variance_mps2
            - 2.0 * dt_sec * previous_speed_bias_covariance_mps3
            + dt_sec * dt_sec * previous_bias_variance_mps2_2
            + accel_process_variance_mps2,
        )
        self._state.speed_bias_covariance_mps3 = (
            previous_speed_bias_covariance_mps3 - dt_sec * previous_bias_variance_mps2_2
        )
        self._state.bias_variance_mps2_2 = max(
            MIN_SPEED_VARIANCE_MPS2,
            previous_bias_variance_mps2_2 + bias_walk_variance_mps2_2,
        )

    def _project_accel_variance_mps2_2(
        self,
        motion_axis_imu: tuple[float, float, float],
        linear_accel_covariance_mps2_2: Optional[Iterable[Iterable[float]]],
    ) -> float:
        """
        Project the IMU acceleration covariance onto the learned motion axis.
        """

        covariance_rows_mps2_2: Optional[tuple[tuple[float, float, float], ...]] = (
            _covariance3x3(linear_accel_covariance_mps2_2)
        )
        if covariance_rows_mps2_2 is None:
            # ROS-style negative covariance diagonals mean unknown/unavailable,
            # so unusable IMU covariance falls back to the configured default.
            return self._config.default_forward_accel_std_mps2**2

        projected_variance_mps2_2: float = 0.0
        row_index: int
        for row_index in range(3):
            col_index: int
            for col_index in range(3):
                projected_variance_mps2_2 += (
                    motion_axis_imu[row_index]
                    * covariance_rows_mps2_2[row_index][col_index]
                    * motion_axis_imu[col_index]
                )

        if projected_variance_mps2_2 <= 0.0 or not math.isfinite(
            projected_variance_mps2_2
        ):
            return self._config.default_forward_accel_std_mps2**2
        return projected_variance_mps2_2


def _compute_dt_sec(
    previous_timestamp_sec: Optional[float], timestamp_sec: float
) -> Optional[float]:
    """
    Return IMU sample spacing in seconds or None for the first sample.
    """

    if previous_timestamp_sec is None:
        return None

    dt_sec: float = timestamp_sec - previous_timestamp_sec
    if not math.isfinite(dt_sec) or dt_sec <= 0.0:
        return None
    return dt_sec


def _classify_predict_timestamp(
    previous_timestamp_sec: Optional[float],
    timestamp_sec: float,
    max_backward_jitter_sec: float,
) -> str:
    """
    Classify an IMU timestamp as accepted, quietly dropped, or rejected.
    """

    if previous_timestamp_sec is None:
        return "accept"

    dt_sec: float = timestamp_sec - previous_timestamp_sec
    if not math.isfinite(dt_sec):
        return "reject"

    if dt_sec >= 0.0:
        return "accept"

    if dt_sec >= -max(0.0, max_backward_jitter_sec):
        return "drop"

    return "reject"


def _vector3(values: Iterable[float]) -> Optional[tuple[float, float, float]]:
    """
    Convert an iterable to a finite 3D tuple.
    """

    try:
        result: tuple[float, ...] = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return None

    if len(result) != 3:
        return None

    if not all(math.isfinite(value) for value in result):
        return None

    return (result[0], result[1], result[2])


def _covariance3x3(
    covariance_rows: Optional[Iterable[Iterable[float]]],
) -> Optional[tuple[tuple[float, float, float], ...]]:
    """
    Validate a usable 3x3 covariance matrix.

    Negative diagonal entries are treated as ROS-style unknown/unavailable
    covariance markers, so the whole matrix is rejected.
    """

    if covariance_rows is None:
        return None

    try:
        rows: tuple[tuple[float, ...], ...] = tuple(
            tuple(float(value) for value in row) for row in covariance_rows
        )
    except (TypeError, ValueError):
        return None

    if len(rows) != 3:
        return None

    row: tuple[float, ...]
    row_index: int
    for row_index, row in enumerate(rows):
        if len(row) != 3:
            return None
        if not all(math.isfinite(value) for value in row):
            return None
        if row[row_index] < 0.0:
            return None

    return (
        (rows[0][0], rows[0][1], rows[0][2]),
        (rows[1][0], rows[1][1], rows[1][2]),
        (rows[2][0], rows[2][1], rows[2][2]),
    )


def _normalized_vector(
    vector: Iterable[float],
) -> Optional[tuple[float, float, float]]:
    """
    Return a normalized 3D vector or None for degenerate inputs.
    """

    values: Optional[tuple[float, float, float]] = _vector3(vector)
    if values is None:
        return None

    norm: float = _norm(values)
    if norm < MIN_VECTOR_NORM_MPS2:
        return None

    return (
        values[0] / norm,
        values[1] / norm,
        values[2] / norm,
    )


def _dot(lhs: Iterable[float], rhs: Iterable[float]) -> float:
    """
    Return the 3D dot product.
    """

    return sum(float(left) * float(right) for left, right in zip(lhs, rhs))


def _norm(vector: Iterable[float]) -> float:
    """
    Return the Euclidean norm of a 3D vector.
    """

    return math.sqrt(_dot(vector, vector))


def _add_vectors(
    lhs: tuple[float, float, float], rhs: tuple[float, float, float]
) -> tuple[float, float, float]:
    """
    Add two 3D vectors.
    """

    return (
        lhs[0] + rhs[0],
        lhs[1] + rhs[1],
        lhs[2] + rhs[2],
    )
