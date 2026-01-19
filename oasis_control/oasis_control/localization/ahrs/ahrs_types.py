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
class AhrsSe3Transform:
    """
    Rigid transform between arbitrary frames

    Fields:
        parent_frame: Parent frame name for the transform
        child_frame: Child frame name for the transform
        translation_m: Translation in meters, XYZ order
        rotation_wxyz: Quaternion rotation in wxyz order, unit length
    """

    parent_frame: str
    child_frame: str
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
        topic: Topic name that produced the event
        frame_id: Frame identifier from the event header
        event_type: Event category for dispatching updates
        payload: Event data payload for the selected type
    """

    t_meas: AhrsTime
    topic: str
    frame_id: str
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
        sensor: Sensor name such as gyro, accel, or mag
        frame_id: Measurement frame identifier
        t_meas: Measurement timestamp
        accepted: True when the update was accepted by the filter
        reject_reason: Optional reason string when the update is rejected
        z: Measurement vector in sensor units, as received
        z_hat: Predicted measurement vector in sensor units
        nu: Innovation vector (z - z_hat) in sensor units
        r: Measurement covariance in sensor units, row-major
        s_hat: Predicted innovation covariance, row-major
        s: Innovation covariance after conditioning, row-major
        maha_d2: Mahalanobis distance squared for gating
        gate_threshold: Gate threshold for the Mahalanobis distance
    """

    sensor: str
    frame_id: str
    t_meas: AhrsTime
    accepted: bool
    reject_reason: Optional[str]
    z: list[float]
    z_hat: list[float]
    nu: list[float]
    r: AhrsMatrix
    s_hat: AhrsMatrix
    s: AhrsMatrix
    maha_d2: float
    gate_threshold: float


@dataclass(frozen=True)
class AhrsStateData:
    """
    Filter state output payload for AhrsState publication

    Fields:
        t_filter: Filter frontier timestamp for this state
        state_seq: Monotonic state sequence identifier
        initialized: True when the filter has a valid initial state
        world_frame_id: World frame identifier for the state
        odom_frame_id: Odometry frame identifier for the state
        body_frame_id: Body frame identifier for the state
        p_wb_m: Position of the body in world in meters, XYZ order
        v_wb_mps: Velocity of the body in world in m/s, XYZ order
        q_wb_wxyz: Orientation of the body in world, quaternion wxyz order
        b_g_rps: Gyro bias in rad/s, XYZ order
        b_a_mps2: Accel bias in m/s^2, XYZ order
        a_a: Accel scale/misalignment matrix, 3x3 row-major
        t_bi: IMU -> body transform (T_BI)
        t_bm: Magnetometer -> body transform (T_BM)
        g_w_mps2: Gravity vector in world in m/s^2, XYZ order
        m_w_t: Magnetic field vector in world in tesla, XYZ order
        error_state_names: Names of error-state entries in the covariance
        p_cov: Full error-state covariance, row-major
    """

    t_filter: AhrsTime
    state_seq: int
    initialized: bool
    world_frame_id: str
    odom_frame_id: str
    body_frame_id: str
    p_wb_m: list[float]
    v_wb_mps: list[float]
    q_wb_wxyz: list[float]
    b_g_rps: list[float]
    b_a_mps2: list[float]
    a_a: list[float]
    t_bi: AhrsSe3Transform
    t_bm: AhrsSe3Transform
    g_w_mps2: list[float]
    m_w_t: list[float]
    error_state_names: list[str]
    p_cov: AhrsMatrix


@dataclass(frozen=True)
class AhrsDiagnosticsData:
    """
    Diagnostics output payload for AhrsDiagnostics publication

    Fields:
        t_filter: Filter frontier timestamp if an output was published
        diag_seq: Monotonic diagnostics sequence identifier
        buffer_node_count: Number of time nodes currently buffered
        buffer_span_sec: Time span of buffered events in seconds
        replay_happened: True when a replay was triggered
        dropped_missing_stamp: Count of updates dropped for missing stamps
        dropped_future_stamp: Count of updates dropped for future stamps
        dropped_too_old: Count of updates dropped for being too old
        dropped_nan_cov: Count of updates dropped for NaN covariance entries
        dropped_imu_gap: Count of updates dropped for IMU gap detection
        dropped_clock_jump_reset: Count of updates dropped for clock jump reset
        reset_count: Number of filter resets performed
        last_reset_reason: Latest reset reason description
    """

    t_filter: Optional[AhrsTime]
    diag_seq: int
    buffer_node_count: int
    buffer_span_sec: float
    replay_happened: bool
    dropped_missing_stamp: int
    dropped_future_stamp: int
    dropped_too_old: int
    dropped_nan_cov: int
    dropped_imu_gap: int
    dropped_clock_jump_reset: int
    reset_count: int
    last_reset_reason: str


@dataclass(frozen=True)
class AhrsOutputs:
    """
    Outputs produced by AHRS event processing

    Fields:
        frontier_advanced: True when the filter frontier advanced on this event
        t_filter: Frontier timestamp for the current estimate outputs
        frame_transforms: Frame transforms derived from the AHRS state
        state: State output payload for AhrsState publication
        diagnostics: Diagnostics output payload for AhrsDiagnostics publication
        gyro_update: Update report payload for gyro updates
        accel_update: Update report payload for accel updates
        mag_update: Update report payload for magnetometer updates
    """

    frontier_advanced: bool
    t_filter: Optional[AhrsTime]
    frame_transforms: Optional[AhrsFrameOutputs]
    state: Optional[AhrsStateData]
    diagnostics: Optional[AhrsDiagnosticsData]
    gyro_update: Optional[AhrsUpdateData]
    accel_update: Optional[AhrsUpdateData]
    mag_update: Optional[AhrsUpdateData]
