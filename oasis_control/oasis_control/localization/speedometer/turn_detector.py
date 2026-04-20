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

from oasis_control.localization.speedometer.contracts import TurnDetection


################################################################################
# Turn detection
################################################################################


MIN_HORIZONTAL_ACCEL_MPS2: float = 1.0e-6


class TurnDetector:
    """
    Fused turn detector used only to gate forward-axis learning.

    The detector combines direct yaw-rate evidence with a lightweight check for
    changing horizontal acceleration direction. Straight forward and reverse
    motion should look sign-consistent after alignment, while curved motion
    tends to rotate the horizontal acceleration direction relative to the
    recent reference.
    """

    def __init__(
        self,
        *,
        turn_rate_threshold_rads: float,
        turn_accel_threshold_mps2: float,
        turn_direction_alignment_threshold: float,
    ) -> None:
        """Initialize the fused learning gate."""

        self._turn_rate_threshold_rads: float = max(
            0.0, float(turn_rate_threshold_rads)
        )
        self._turn_accel_threshold_mps2: float = max(
            0.0, float(turn_accel_threshold_mps2)
        )
        self._turn_direction_alignment_threshold: float = min(
            1.0,
            max(-1.0, float(turn_direction_alignment_threshold)),
        )
        self._reference_direction_xy: Optional[tuple[float, float]] = None

    def update(
        self,
        *,
        angular_velocity_rads: tuple[float, float, float],
        linear_acceleration_mps2: tuple[float, float, float],
    ) -> TurnDetection:
        """
        Classify the current IMU sample as turning or straight enough to learn.
        """

        yaw_rate_rads: float = _finite_value(float(angular_velocity_rads[2]))
        horizontal_accel_xy: tuple[float, float] = (
            _finite_value(float(linear_acceleration_mps2[0])),
            _finite_value(float(linear_acceleration_mps2[1])),
        )
        horizontal_accel_mps2: float = math.hypot(
            horizontal_accel_xy[0], horizontal_accel_xy[1]
        )
        reference_alignment: float = 1.0

        candidate_direction_xy: Optional[tuple[float, float]] = None
        if horizontal_accel_mps2 >= max(
            MIN_HORIZONTAL_ACCEL_MPS2, self._turn_accel_threshold_mps2
        ):
            candidate_direction_xy = (
                horizontal_accel_xy[0] / horizontal_accel_mps2,
                horizontal_accel_xy[1] / horizontal_accel_mps2,
            )
            reference_alignment = self._compute_alignment(
                candidate_direction_xy=candidate_direction_xy
            )
            self._update_reference(candidate_direction_xy=candidate_direction_xy)

        accel_turn_detected: bool = (
            candidate_direction_xy is not None
            and abs(yaw_rate_rads) >= 0.5 * self._turn_rate_threshold_rads
            and reference_alignment < self._turn_direction_alignment_threshold
        )
        yaw_turn_detected: bool = abs(yaw_rate_rads) >= self._turn_rate_threshold_rads

        return TurnDetection(
            turn_detected=(yaw_turn_detected or accel_turn_detected),
            yaw_rate_rads=yaw_rate_rads,
            horizontal_accel_mps2=horizontal_accel_mps2,
            reference_alignment=reference_alignment,
            threshold_rads=self._turn_rate_threshold_rads,
        )

    def _compute_alignment(
        self, *, candidate_direction_xy: tuple[float, float]
    ) -> float:
        """Return sign-aligned agreement with the running reference."""

        if self._reference_direction_xy is None:
            return 1.0

        raw_alignment: float = (
            self._reference_direction_xy[0] * candidate_direction_xy[0]
            + self._reference_direction_xy[1] * candidate_direction_xy[1]
        )
        return abs(max(-1.0, min(1.0, raw_alignment)))

    def _update_reference(self, *, candidate_direction_xy: tuple[float, float]) -> None:
        """Update the sign-aligned running acceleration-direction reference."""

        if self._reference_direction_xy is None:
            self._reference_direction_xy = candidate_direction_xy
            return

        aligned_direction_xy: tuple[float, float] = candidate_direction_xy
        if (
            self._reference_direction_xy[0] * candidate_direction_xy[0]
            + self._reference_direction_xy[1] * candidate_direction_xy[1]
        ) < 0.0:
            aligned_direction_xy = (
                -candidate_direction_xy[0],
                -candidate_direction_xy[1],
            )

        blended_reference_xy: tuple[float, float] = (
            0.8 * self._reference_direction_xy[0] + 0.2 * aligned_direction_xy[0],
            0.8 * self._reference_direction_xy[1] + 0.2 * aligned_direction_xy[1],
        )
        blended_norm: float = math.hypot(
            blended_reference_xy[0], blended_reference_xy[1]
        )
        if blended_norm < MIN_HORIZONTAL_ACCEL_MPS2:
            return

        self._reference_direction_xy = (
            blended_reference_xy[0] / blended_norm,
            blended_reference_xy[1] / blended_norm,
        )


def _finite_value(value: float) -> float:
    """Return one finite scalar, falling back to zero when invalid."""

    return float(value) if math.isfinite(value) else 0.0
