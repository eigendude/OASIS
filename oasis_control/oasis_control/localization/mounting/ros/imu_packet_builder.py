################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Helpers for building mounting packets from IMU-related messages"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.math_utils.validation import (
    reshape_covariance as _reshape_covariance,
)
from oasis_control.localization.mounting.math_utils.validation import (
    reshape_matrix as _reshape_matrix,
)
from oasis_control.localization.mounting.mounting_types import ImuCalibrationPrior
from oasis_control.localization.mounting.mounting_types import ImuPacket
from oasis_control.localization.mounting.mounting_types import MagPacket


if TYPE_CHECKING:
    from builtin_interfaces.msg import Time as TimeMsg
    from sensor_msgs.msg import Imu as ImuMsg
    from sensor_msgs.msg import MagneticField as MagneticFieldMsg

    from oasis_msgs.msg import ImuCalibration as ImuCalibrationMsg


def _time_to_ns(stamp: TimeMsg) -> int:
    sec_ns: int = int(stamp.sec) * int(1e9)

    return sec_ns + int(stamp.nanosec)


def build_imu_packet(
    imu_msg: ImuMsg,
    cal_msg: ImuCalibrationMsg,
    *,
    params: MountingParams,
) -> ImuPacket:
    imu_frame: str = imu_msg.header.frame_id
    t_ns: int = _time_to_ns(imu_msg.header.stamp)

    omega_raw: np.ndarray = np.array(
        [
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z,
        ],
        dtype=np.float64,
    )
    accel_raw: np.ndarray = np.array(
        [
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z,
        ],
        dtype=np.float64,
    )

    b_a_mps2: np.ndarray = np.array(
        [
            cal_msg.accel_bias.x,
            cal_msg.accel_bias.y,
            cal_msg.accel_bias.z,
        ],
        dtype=np.float64,
    )
    A_a: np.ndarray = _reshape_matrix(cal_msg.accel_a, (3, 3), "accel_a")
    b_g_rads: np.ndarray = np.array(
        [
            cal_msg.gyro_bias.x,
            cal_msg.gyro_bias.y,
            cal_msg.gyro_bias.z,
        ],
        dtype=np.float64,
    )

    cov_a_params: np.ndarray = _reshape_covariance(
        cal_msg.accel_param_cov,
        (12, 12),
        "accel_param_cov",
    )
    cov_b_g: np.ndarray = _reshape_covariance(
        cal_msg.gyro_bias_cov,
        (3, 3),
        "gyro_bias_cov",
    )

    omega_cov_default: np.ndarray = np.diag(params.imu.omega_cov_diag)
    accel_cov_default: np.ndarray = np.diag(params.imu.accel_cov_diag)

    cov_omega_raw: np.ndarray = _reshape_covariance(
        imu_msg.angular_velocity_covariance,
        (3, 3),
        "angular_velocity_covariance",
        fallback=omega_cov_default,
    )
    cov_accel_raw: np.ndarray = _reshape_covariance(
        imu_msg.linear_acceleration_covariance,
        (3, 3),
        "linear_acceleration_covariance",
        fallback=accel_cov_default,
    )

    calibration: ImuCalibrationPrior = ImuCalibrationPrior(
        valid=bool(cal_msg.valid),
        frame_id=imu_frame,
        b_a_mps2=b_a_mps2,
        A_a=A_a,
        b_g_rads=b_g_rads,
        cov_a_params=cov_a_params,
        cov_b_g=cov_b_g,
    )

    return ImuPacket(
        t_meas_ns=t_ns,
        frame_id=imu_frame,
        omega_raw_rads=omega_raw,
        cov_omega_raw=cov_omega_raw,
        a_raw_mps2=accel_raw,
        cov_a_raw=cov_accel_raw,
        calibration=calibration,
    )


def build_mag_packet(message: MagneticFieldMsg) -> MagPacket:
    t_ns: int = _time_to_ns(message.header.stamp)
    m_raw: np.ndarray = np.array(
        [
            message.magnetic_field.x,
            message.magnetic_field.y,
            message.magnetic_field.z,
        ],
        dtype=np.float64,
    )
    cov_m_raw: np.ndarray = _reshape_covariance(
        message.magnetic_field_covariance,
        (3, 3),
        "magnetic_field_covariance",
    )

    return MagPacket(
        t_meas_ns=t_ns,
        frame_id=message.header.frame_id,
        m_raw_T=m_raw,
        cov_m_raw_T2=cov_m_raw,
    )
