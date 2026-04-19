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

from oasis_control.localization.common.algebra.covariance import (
    UNKNOWN_ORIENTATION_COVARIANCE,
)
from oasis_control.localization.common.algebra.covariance import (
    flatten_matrix3_row_major,
)
from oasis_control.localization.common.algebra.quat import normalize_quaternion_xyzw
from oasis_control.localization.common.algebra.quat import quaternion_to_rotation_matrix
from oasis_control.localization.common.algebra.quat import rotate_vector


################################################################################
# AHRS tilt estimator
################################################################################


# Units: rad^2
# Meaning: large yaw variance because tilt output intentionally suppresses yaw
YAW_VARIANCE_RAD2: float = 1.0e6

# Meaning: world down direction expressed in world coordinates
WORLD_DOWN_DIRECTION: tuple[float, float, float] = (0.0, 0.0, -1.0)


@dataclass(frozen=True)
class AhrsTiltEstimate:
    """
    Roll/pitch-only view of mounted AHRS orientation.

    Fields:
        roll_rad: body roll angle in radians in `base_link`
        pitch_rad: body pitch angle in radians in `base_link`
        quaternion_xyzw: roll/pitch-only quaternion in ROS xyzw order
        orientation_covariance: sensor_msgs/Imu 3x3 row-major covariance
    """

    roll_rad: float
    pitch_rad: float
    quaternion_xyzw: tuple[float, float, float, float]
    orientation_covariance: list[float]


class AhrsTiltEstimator:
    """
    Extract roll/pitch tilt from mounted AHRS orientation.

    The input quaternion is expected to already describe the body in
    `base_link`. Tilt is derived from the world down vector expressed in body
    coordinates, which depends only on body up/down direction and is therefore
    invariant to arbitrary yaw heading.
    """

    def update(
        self,
        *,
        orientation_xyzw: Iterable[float],
        orientation_covariance_rad2: Optional[Iterable[Iterable[float]]],
        orientation_covariance_unknown: bool,
    ) -> Optional[AhrsTiltEstimate]:
        """
        Convert one mounted AHRS orientation sample into a tilt estimate.
        """

        quaternion_xyzw: Optional[tuple[float, float, float, float]] = (
            normalize_quaternion_xyzw(orientation_xyzw)
        )
        if quaternion_xyzw is None:
            return None

        # Meaning: world down direction expressed in base_link coordinates
        # Why yaw-invariant: rotating world down into body removes dependence
        # on heading and keeps only the body tilt relative to gravity
        down_direction_body: tuple[float, float, float] = rotate_vector(
            quaternion_to_rotation_matrix(quaternion_xyzw),
            WORLD_DOWN_DIRECTION,
        )

        roll_rad: float
        pitch_rad: float
        roll_rad, pitch_rad = _roll_pitch_from_down_direction_body(down_direction_body)

        return AhrsTiltEstimate(
            roll_rad=roll_rad,
            pitch_rad=pitch_rad,
            quaternion_xyzw=_quaternion_from_roll_pitch_yaw(
                roll_rad=roll_rad,
                pitch_rad=pitch_rad,
                yaw_rad=0.0,
            ),
            orientation_covariance=_make_tilt_orientation_covariance(
                orientation_covariance_rad2=orientation_covariance_rad2,
                orientation_covariance_unknown=orientation_covariance_unknown,
            ),
        )


def _make_tilt_orientation_covariance(
    *,
    orientation_covariance_rad2: Optional[Iterable[Iterable[float]]],
    orientation_covariance_unknown: bool,
) -> list[float]:
    """
    Preserve roll/pitch covariance while marking yaw as intentionally unknown.
    """

    if orientation_covariance_unknown:
        return list(UNKNOWN_ORIENTATION_COVARIANCE)

    if orientation_covariance_rad2 is None:
        return list(UNKNOWN_ORIENTATION_COVARIANCE)

    try:
        covariance_rows_rad2: tuple[tuple[float, ...], ...] = tuple(
            tuple(float(value) for value in row) for row in orientation_covariance_rad2
        )
    except (TypeError, ValueError):
        return list(UNKNOWN_ORIENTATION_COVARIANCE)

    if len(covariance_rows_rad2) != 3:
        return list(UNKNOWN_ORIENTATION_COVARIANCE)

    row_values_rad2: tuple[float, ...]
    for row_values_rad2 in covariance_rows_rad2:
        if len(row_values_rad2) != 3:
            return list(UNKNOWN_ORIENTATION_COVARIANCE)

        if not all(math.isfinite(value) for value in row_values_rad2):
            return list(UNKNOWN_ORIENTATION_COVARIANCE)

    return flatten_matrix3_row_major(
        (
            (
                covariance_rows_rad2[0][0],
                covariance_rows_rad2[0][1],
                0.0,
            ),
            (
                covariance_rows_rad2[1][0],
                covariance_rows_rad2[1][1],
                0.0,
            ),
            (
                0.0,
                0.0,
                YAW_VARIANCE_RAD2,
            ),
        )
    )


def _roll_pitch_from_down_direction_body(
    down_direction_body: tuple[float, float, float],
) -> tuple[float, float]:
    """
    Convert body-frame world down direction into roll and pitch.
    """

    down_x: float = down_direction_body[0]
    down_y: float = down_direction_body[1]
    down_z: float = down_direction_body[2]

    # Units: rad
    # Meaning: roll from the lateral down component relative to body Z
    roll_rad: float = math.atan2(down_y, -down_z)

    # Units: rad
    # Meaning: pitch from the forward down component relative to the YZ-plane
    pitch_rad: float = math.atan2(
        -down_x,
        math.sqrt(down_y * down_y + down_z * down_z),
    )

    return (roll_rad, pitch_rad)


def _quaternion_from_roll_pitch_yaw(
    *,
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> tuple[float, float, float, float]:
    """
    Convert roll, pitch, yaw into a quaternion in ROS xyzw order.
    """

    half_roll_rad: float = roll_rad * 0.5
    half_pitch_rad: float = pitch_rad * 0.5
    half_yaw_rad: float = yaw_rad * 0.5

    sin_half_roll: float = math.sin(half_roll_rad)
    cos_half_roll: float = math.cos(half_roll_rad)
    sin_half_pitch: float = math.sin(half_pitch_rad)
    cos_half_pitch: float = math.cos(half_pitch_rad)
    sin_half_yaw: float = math.sin(half_yaw_rad)
    cos_half_yaw: float = math.cos(half_yaw_rad)

    return (
        sin_half_roll * cos_half_pitch * cos_half_yaw
        - cos_half_roll * sin_half_pitch * sin_half_yaw,
        cos_half_roll * sin_half_pitch * cos_half_yaw
        + sin_half_roll * cos_half_pitch * sin_half_yaw,
        cos_half_roll * cos_half_pitch * sin_half_yaw
        - sin_half_roll * sin_half_pitch * cos_half_yaw,
        cos_half_roll * cos_half_pitch * cos_half_yaw
        + sin_half_roll * sin_half_pitch * sin_half_yaw,
    )
