################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
Types and helpers for AHRS localization
"""

import enum
from dataclasses import dataclass
from typing import Literal
from typing import Optional
from typing import Union


@dataclass(frozen=True)
class AhrsTime:
    """
    AHRS timestamp stored as seconds and nanoseconds

    Fields:
        sec: Whole seconds since the AHRS time reference
        nanosec: Sub-second remainder in nanoseconds [0, 1e9)
    """

    sec: int
    nanosec: int


AhrsFrameId = Literal["world", "odom", "base_link"]


@dataclass(frozen=True)
class AhrsFrameTransform:
    """
    Rigid transform between semantic frames

    Fields:
        parent_frame: Semantic parent frame
        child_frame: Semantic child frame
        translation_m: Translation in meters, XYZ order
        rotation_wxyz: Quaternion rotation in wxyz order, unit length
    """

    parent_frame: AhrsFrameId
    child_frame: AhrsFrameId
    translation_m: list[float]
    rotation_wxyz: list[float]


@dataclass(frozen=True)
class AhrsFrameOutputs:
    """
    Frame outputs derived from the canonical odom state

    Fields:
        t_odom_base: Transform from odom to base
        t_world_odom: Transform from world to odom
        t_world_base: Transform from world to base derived by composition
    """

    t_odom_base: AhrsFrameTransform
    t_world_odom: AhrsFrameTransform
    t_world_base: AhrsFrameTransform


class AhrsEventType(enum.Enum):
    """
    Enumerates the kinds of time-ordered AHRS events

    Attributes:
        IMU: Inertial sample or IMU packet event
        MAG: Magnetometer measurement event
    """

    IMU = "imu"
    MAG = "mag"


@dataclass(frozen=True)
class ImuCalibrationData:
    """
    Calibration prior data for IMU bias and scale parameters

    Fields:
        valid: True when the calibration payload is safe to apply
        frame_id: IMU frame identifier expected to match the IMU sample frame
        accel_bias_mps2: Accelerometer bias in m/s^2, XYZ order
        accel_a: 3x3 accel scale/misalignment matrix, row-major
        accel_param_cov: 12x12 covariance of accel calibration params
        gyro_bias_rps: Gyroscope bias in rad/s, XYZ order
        gyro_bias_cov: 3x3 covariance of gyro bias params, row-major
        gravity_mps2: Gravity magnitude assumed during calibration in m/s^2
        fit_sample_count: Number of samples used in the calibration fit
        rms_residual_mps2: RMS gravity residual of the calibration in m/s^2
        temperature_c: Mean calibration temperature in deg C
        temperature_var_c2: Temperature variance in (deg C)^2
    """

    valid: bool
    frame_id: str
    accel_bias_mps2: list[float]
    accel_a: list[float]
    accel_param_cov: list[float]
    gyro_bias_rps: list[float]
    gyro_bias_cov: list[float]
    gravity_mps2: float
    fit_sample_count: int
    rms_residual_mps2: float
    temperature_c: float
    temperature_var_c2: float


@dataclass(frozen=True)
class ImuSample:
    """
    IMU sample with raw motion data

    Fields:
        frame_id: IMU frame identifier for this sample
        angular_velocity_rps: Angular velocity in rad/s, XYZ order
        linear_acceleration_mps2: Linear acceleration in m/s^2, XYZ order
        angular_velocity_cov: 3x3 gyro covariance, row-major
        linear_acceleration_cov: 3x3 accel covariance, row-major
    """

    frame_id: str
    angular_velocity_rps: list[float]
    linear_acceleration_mps2: list[float]
    angular_velocity_cov: list[float]
    linear_acceleration_cov: list[float]


@dataclass(frozen=True)
class AhrsImuPacket:
    """
    IMU packet event payload

    Fields:
        imu: Raw IMU measurement sample
        calibration: IMU calibration prior tied to the IMU frame
    """

    imu: ImuSample
    calibration: ImuCalibrationData


@dataclass(frozen=True)
class MagSample:
    """
    Magnetometer sample data

    Fields:
        frame_id: Magnetometer frame identifier
        magnetic_field_t: Magnetic field vector in tesla, XYZ order
        magnetic_field_cov: 3x3 covariance in tesla^2, row-major
    """

    frame_id: str
    magnetic_field_t: list[float]
    magnetic_field_cov: list[float]


AhrsEventPayload = Union[
    AhrsImuPacket,
    MagSample,
]


@dataclass(frozen=True)
class AhrsEvent:
    """
    Time-stamped AHRS event in the common filter timeline

    Fields:
        t_meas: Measurement timestamp
        event_type: Event category for dispatching updates
        payload: Event data payload for the selected type
    """

    t_meas: AhrsTime
    event_type: AhrsEventType
    payload: AhrsEventPayload


@dataclass(frozen=True)
class AhrsMatrix:
    """
    Row-major matrix data used by update reporting

    Fields:
        rows: Number of rows in the matrix
        cols: Number of columns in the matrix
        data: Row-major matrix entries
    """

    rows: int
    cols: int
    data: list[float]


@dataclass(frozen=True)
class AhrsUpdateData:
    """
    Generic update report payload

    Fields:
        sensor: Sensor name such as magnetic_field or apriltags
        frame_id: Measurement frame identifier
        t_meas: Measurement timestamp
        (TODO)
    """

    sensor: str
    frame_id: str
    t_meas: AhrsTime
    # TODO


@dataclass(frozen=True)
class AhrsOutputs:
    """
    Outputs produced by AHRS event processing

    Fields:
        odom_time: Timestamp for publishing the odometry frame output
        world_odom_time: Timestamp for publishing the world odometry output
        frame_transforms: Frame transforms derived from the AHRS state
        mag_update: Update report payload for magnetometer updates
    """

    odom_time: Optional[AhrsTime]
    world_odom_time: Optional[AhrsTime]
    frame_transforms: Optional[AhrsFrameOutputs]
    mag_update: Optional[AhrsUpdateData]
