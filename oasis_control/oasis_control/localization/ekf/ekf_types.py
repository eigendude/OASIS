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
Types and helpers for EKF localization
"""

import enum
from dataclasses import dataclass
from typing import Literal
from typing import Optional
from typing import Union


# Nanoseconds per second for converting EKF timestamps
_NS_PER_S: int = 1_000_000_000


@dataclass(frozen=True)
class EkfTime:
    """
    EKF timestamp stored as seconds and nanoseconds

    Fields:
        sec: Whole seconds since the EKF time reference
        nanosec: Sub-second remainder in nanoseconds [0, 1e9)
    """

    sec: int
    nanosec: int


FrameId = Literal["world", "odom", "base"]


@dataclass(frozen=True)
class EkfFrameTransform:
    """
    Rigid transform between semantic frames

    Fields:
        parent_frame: Semantic parent frame
        child_frame: Semantic child frame
        translation_m: Translation in meters, XYZ order
        rotation_wxyz: Quaternion rotation in wxyz order, unit length
    """

    parent_frame: FrameId
    child_frame: FrameId
    translation_m: list[float]
    rotation_wxyz: list[float]


@dataclass(frozen=True)
class EkfFrameOutputs:
    """
    Frame outputs derived from the canonical odom state

    Fields:
        t_odom_base: Transform from odom to base
        t_world_odom: Transform from world to odom
        t_world_base: Transform from world to base derived by composition
    """

    t_odom_base: EkfFrameTransform
    t_world_odom: EkfFrameTransform
    t_world_base: EkfFrameTransform


def to_ns(t: EkfTime) -> int:
    return t.sec * _NS_PER_S + t.nanosec


def from_ns(ns: int) -> EkfTime:
    sec, nanosec = divmod(ns, _NS_PER_S)
    return EkfTime(sec=sec, nanosec=nanosec)


def to_seconds(t: EkfTime) -> float:
    return float(t.sec) + float(t.nanosec) / _NS_PER_S


def from_seconds(seconds: float) -> EkfTime:
    total_ns: int = int(round(seconds * _NS_PER_S))
    return from_ns(total_ns)


class EkfEventType(enum.Enum):
    """
    Enumerates the kinds of time-ordered EKF events

    Attributes:
        IMU: Inertial sample or IMU packet event
        MAG: Magnetometer measurement event
        APRILTAG: AprilTag detection array event
        CAMERA_INFO: Camera calibration cache event
    """

    IMU = "imu"
    MAG = "mag"
    APRILTAG = "apriltag"
    CAMERA_INFO = "camera_info"


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
class EkfImuPacket:
    """
    IMU packet event payload

    Fields:
        imu: Raw IMU measurement sample
        calibration: Optional calibration prior tied to the IMU frame
    """

    imu: ImuSample
    calibration: Optional[ImuCalibrationData]


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


@dataclass(frozen=True)
class AprilTagDetection:
    """
    One AprilTag detection with pixel corner data

    Fields:
        family: AprilTag family name such as tag36h11
        tag_id: Identifier within the family
        det_index_in_msg: Index in the incoming detection array
        corners_px: Pixel corners in [x0, y0, x1, y1, x2, y2, x3, y3] order
        pose_world_xyz_yaw: Optional [px, py, pz, yaw] in world frame
        decision_margin: Detector decision margin for gating
        homography: 3x3 homography matrix, row-major
    """

    family: str
    tag_id: int
    det_index_in_msg: int
    corners_px: list[float]
    pose_world_xyz_yaw: Optional[list[float]]
    decision_margin: float
    homography: list[float]


@dataclass(frozen=True)
class AprilTagDetectionArrayData:
    """
    AprilTag detection array payload

    Fields:
        frame_id: Camera frame identifier for pixel coordinates
        detections: List of per-tag detection data
    """

    frame_id: str
    detections: list[AprilTagDetection]


@dataclass(frozen=True)
class CameraInfoData:
    """
    Cached camera intrinsic calibration data

    Fields:
        frame_id: Camera frame identifier
        width: Image width in pixels
        height: Image height in pixels
        distortion_model: Distortion model name
        d: Distortion coefficients
        k: 3x3 intrinsic matrix, row-major
        r: 3x3 rectification matrix, row-major
        p: 3x4 projection matrix, row-major
    """

    frame_id: str
    width: int
    height: int
    distortion_model: str
    d: list[float]
    k: list[float]
    r: list[float]
    p: list[float]


EkfEventPayload = Union[
    EkfImuPacket,
    MagSample,
    AprilTagDetectionArrayData,
    CameraInfoData,
]


@dataclass(frozen=True)
class EkfEvent:
    """
    Time-stamped EKF event in the common filter timeline

    Fields:
        t_meas: Measurement timestamp
        event_type: Event category for dispatching updates
        payload: Event data payload for the selected type
    """

    t_meas: EkfTime
    event_type: EkfEventType
    payload: EkfEventPayload


@dataclass(frozen=True)
class EkfMatrix:
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
class EkfUpdateData:
    """
    Generic update report payload

    Fields:
        sensor: Sensor name such as magnetic_field or apriltags
        frame_id: Measurement frame identifier
        t_meas: Measurement timestamp
        accepted: True when the update is accepted
        reject_reason: Reason for rejection when not accepted
        z_dim: Dimension of the measurement vector
        z: Measurement vector values
        z_hat: Predicted measurement vector values
        nu: Innovation vector values
        r: Measurement noise covariance used for the update
        s_hat: Predicted innovation covariance without measurement noise
        s: Total innovation covariance including measurement noise
        maha_d2: Squared Mahalanobis distance of the innovation
        gate_d2_threshold: Squared gating threshold used for acceptance
        reproj_rms_px: Root mean square reprojection error in pixels
    """

    sensor: str
    frame_id: str
    t_meas: EkfTime
    accepted: bool
    reject_reason: str
    z_dim: int
    z: list[float]
    z_hat: list[float]
    nu: list[float]
    r: EkfMatrix
    s_hat: EkfMatrix
    s: EkfMatrix
    maha_d2: float
    gate_d2_threshold: float
    reproj_rms_px: float


@dataclass(frozen=True)
class EkfAprilTagDetectionUpdate:
    """
    Update report payload for a single AprilTag detection

    Fields:
        family: AprilTag family name
        tag_id: AprilTag identifier within the family
        det_index_in_msg: Index in the incoming detection array
        update: Embedded update report payload
    """

    family: str
    tag_id: int
    det_index_in_msg: int
    update: EkfUpdateData


@dataclass(frozen=True)
class EkfAprilTagUpdateData:
    """
    Update report payload for an AprilTag detection array

    Fields:
        t_meas: Measurement timestamp
        frame_id: Camera frame identifier for the detection array
        detections: Per-detection update data payloads
    """

    t_meas: EkfTime
    frame_id: str
    detections: list[EkfAprilTagDetectionUpdate]


@dataclass(frozen=True)
class EkfOutputs:
    """
    Outputs produced by EKF event processing

    Fields:
        odom_time: Timestamp for publishing the odometry frame output
        world_odom_time: Timestamp for publishing the world odometry output
        frame_transforms: Frame transforms derived from the EKF state
        mag_update: Update report payload for magnetometer updates
        apriltag_update: Update report payload for AprilTag updates
    """

    odom_time: Optional[EkfTime]
    world_odom_time: Optional[EkfTime]
    frame_transforms: Optional[EkfFrameOutputs]
    mag_update: Optional[EkfUpdateData]
    apriltag_update: Optional[EkfAprilTagUpdateData]
