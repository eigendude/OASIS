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
from typing import Optional

from oasis_control.localization.speedometer.contracts import ForwardAxisState
from oasis_control.localization.speedometer.contracts import ForwardTwistConfig
from oasis_control.localization.speedometer.contracts import ForwardTwistEstimate
from oasis_control.localization.speedometer.contracts import LearningState
from oasis_control.localization.speedometer.contracts import PersistenceRecord
from oasis_control.localization.speedometer.contracts import ZuptMeasurement
from oasis_control.localization.speedometer.forward_yaw_persistence import (
    ForwardYawPersistence,
)
from oasis_control.localization.speedometer.turn_detector import TurnDetector


################################################################################
# Forward twist estimator
################################################################################


class ForwardTwistEstimator:
    """
    Placeholder forward-speed estimator with candidate and committed yaw state.

    This skeleton intentionally keeps the math simple. The public contract is
    present so later work can replace the placeholder learning and covariance
    model without rewriting the ROS integration layer.
    """

    def __init__(
        self,
        *,
        config: ForwardTwistConfig,
        persistence: ForwardYawPersistence,
        turn_detector: TurnDetector,
    ) -> None:
        """Initialize the skeleton estimator."""

        self._config: ForwardTwistConfig = config
        self._persistence: ForwardYawPersistence = persistence
        self._turn_detector: TurnDetector = turn_detector

        self._candidate_forward_yaw_rad: float = 0.0
        self._committed_forward_yaw_rad: float = 0.0
        self._candidate_sample_count: int = 0
        self._committed_sample_count: int = 0
        self._forward_speed_mps: float = 0.0
        self._forward_speed_variance_mps2: float = max(
            self._config.min_forward_speed_variance_mps2,
            1.0,
        )
        self._last_turn_detected: bool = False
        self._last_stationary_flag: Optional[bool] = None
        self._last_timestamp_ns: int = 0

    def update_imu(
        self,
        *,
        timestamp_ns: int,
        orientation_xyzw: tuple[float, float, float, float],
        angular_velocity_rads: tuple[float, float, float],
        linear_acceleration_mps2: tuple[float, float, float],
    ) -> ForwardTwistEstimate:
        """
        Update the placeholder runtime state from one IMU sample.
        """

        self._last_timestamp_ns = int(timestamp_ns)

        turn_detection = self._turn_detector.update(
            yaw_rate_rads=float(angular_velocity_rads[2])
        )
        self._last_turn_detected = turn_detection.turn_detected

        measured_yaw_rad: float = _yaw_from_quaternion_xyzw(orientation_xyzw)
        if not turn_detection.turn_detected:
            self._candidate_forward_yaw_rad = _wrap_angle_rad(
                (1.0 - self._config.candidate_learning_gain)
                * self._candidate_forward_yaw_rad
                + self._config.candidate_learning_gain * measured_yaw_rad
            )
            self._candidate_sample_count += 1

            if self._candidate_sample_count > self._committed_sample_count:
                self._committed_forward_yaw_rad = _wrap_angle_rad(
                    (1.0 - self._config.committed_learning_gain)
                    * self._committed_forward_yaw_rad
                    + self._config.committed_learning_gain
                    * self._candidate_forward_yaw_rad
                )
                self._committed_sample_count += 1

        # TODO: Replace this projection with real forward-speed estimation from
        # mounted IMU dynamics, ZUPT drift control, and a principled covariance
        # model that does not trust upstream IMU covariance as the output model
        self._forward_speed_mps = _project_forward_speed_placeholder(
            linear_acceleration_mps2=linear_acceleration_mps2,
            forward_yaw_rad=self._committed_forward_yaw_rad,
        )
        self._forward_speed_variance_mps2 = max(
            self._config.min_forward_speed_variance_mps2,
            abs(self._forward_speed_mps) * 0.1 + 0.05,
        )

        return self.get_estimate()

    def update_zupt(self, *, measurement: ZuptMeasurement) -> ForwardTwistEstimate:
        """
        Apply a placeholder ZUPT correction to the runtime scalar speed.
        """

        self._last_timestamp_ns = int(measurement.timestamp_ns)
        self._last_stationary_flag = measurement.stationary_flag

        if measurement.stationary_flag is True:
            self._forward_speed_mps = 0.0
        else:
            self._forward_speed_mps *= 0.5

        self._forward_speed_variance_mps2 = max(
            self._config.min_forward_speed_variance_mps2,
            float(measurement.zero_velocity_variance_mps2),
        )
        return self.get_estimate()

    def store_committed_forward_yaw(self, *, hostname: str) -> None:
        """
        Persist the committed forward yaw using the repo convention.
        """

        self._persistence.store(
            record=PersistenceRecord(
                hostname=hostname,
                forward_yaw_rad=self._committed_forward_yaw_rad,
            )
        )

    def get_estimate(self) -> ForwardTwistEstimate:
        """
        Return the current placeholder estimate.
        """

        forward_axis_xyz: tuple[float, float, float] = (
            math.cos(self._committed_forward_yaw_rad),
            math.sin(self._committed_forward_yaw_rad),
            0.0,
        )
        return ForwardTwistEstimate(
            timestamp_ns=self._last_timestamp_ns,
            forward_speed_mps=self._forward_speed_mps,
            forward_speed_variance_mps2=self._forward_speed_variance_mps2,
            forward_axis=ForwardAxisState(
                forward_yaw_rad=self._committed_forward_yaw_rad,
                forward_axis_xyz=forward_axis_xyz,
                learned=self._committed_sample_count > 0,
            ),
            learning_state=LearningState(
                candidate_forward_yaw_rad=self._candidate_forward_yaw_rad,
                committed_forward_yaw_rad=self._committed_forward_yaw_rad,
                candidate_sample_count=self._candidate_sample_count,
                committed_sample_count=self._committed_sample_count,
                learning_gated_by_turn=self._last_turn_detected,
            ),
            turn_detected=self._last_turn_detected,
        )


def _yaw_from_quaternion_xyzw(
    quaternion_xyzw: tuple[float, float, float, float],
) -> float:
    """
    Return a basic Z-yaw placeholder from an XYZW quaternion.
    """

    x_component: float = float(quaternion_xyzw[0])
    y_component: float = float(quaternion_xyzw[1])
    z_component: float = float(quaternion_xyzw[2])
    w_component: float = float(quaternion_xyzw[3])

    numerator: float = 2.0 * (w_component * z_component + x_component * y_component)
    denominator: float = 1.0 - 2.0 * (
        y_component * y_component + z_component * z_component
    )
    return math.atan2(numerator, denominator)


def _project_forward_speed_placeholder(
    *,
    linear_acceleration_mps2: tuple[float, float, float],
    forward_yaw_rad: float,
) -> float:
    """
    Return a tiny bounded placeholder speed proxy from body acceleration.
    """

    forward_axis_xy: tuple[float, float] = (
        math.cos(forward_yaw_rad),
        math.sin(forward_yaw_rad),
    )
    projected_accel_mps2: float = forward_axis_xy[0] * float(
        linear_acceleration_mps2[0]
    ) + forward_axis_xy[1] * float(linear_acceleration_mps2[1])
    return max(-5.0, min(5.0, projected_accel_mps2 * 0.1))


def _wrap_angle_rad(angle_rad: float) -> float:
    """
    Wrap one angle to [-pi, pi).
    """

    wrapped_angle_rad: float = math.fmod(float(angle_rad) + math.pi, 2.0 * math.pi)
    if wrapped_angle_rad < 0.0:
        wrapped_angle_rad += 2.0 * math.pi
    return wrapped_angle_rad - math.pi
