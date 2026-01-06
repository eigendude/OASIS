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

import numpy as np


################################################################################
# Madgwick AHRS
################################################################################


# Units: rad^2/s. Meaning: small-angle process noise power added as
# Q = PROCESS_NOISE_THETA_RAD2_PER_S * dt in the discrete-time covariance
PROCESS_NOISE_THETA_RAD2_PER_S: float = math.radians(0.5) ** 2

# Units: rad^2/s^2. Meaning: conservative default gyro variance when
# covariance is unknown, roughly (0.5 deg/s)^2 per axis
DEFAULT_GYRO_VAR_RAD2_PER_S2: float = math.radians(0.5) ** 2

# Units: unitless. Meaning: low-pass gain for the magnetic reference update
MAG_REF_ALPHA: float = 0.05


class MadgwickAhrs:
    """
    Minimal Madgwick AHRS implementation for 6-axis and 9-axis IMU data.

    Block diagram (signals map to variable names below):

        gyro (gx, gy, gz)
             |
             v
        [gyro quaternion rate]
        q_dot_gyro = 0.5 * q ⊗ [0, gx, gy, gz]
             |
             |                     accel (ax, ay, az)
             |                          |
             |                          v
             |                    [normalize]
             |                          |
             |                          v
             |                    [gravity residuals]
             |                    f1..f3 from (q, ax, ay, az)
             |                          |
             |                          v
             |                   [gradient step]
             |                   s1..s4 = grad(f1..f3)
             |                          |
             |                          v
             |                    [beta gain]
             |                    beta * s1..s4
             |                          |
             |                          v
             |                    (-) correction
             |                          |
             |                          v
             +-------------------->(+) sum = q_dot
                                   |
                                   v
                              [integrate dt]
                              q_next = q + q_dot * dt
                                   |
                                   v
                              [normalize]
                              _quaternion

        magnetometer path (update only):

            mag (mx, my, mz) -> [normalize] -> hx, hy, b_x, b_z
                                           -> [mag residuals] f4..f6
                                           -> [gradient step] s1..s4

    Mapping notes:
      - q1..q4 correspond to (w, x, y, z) but are stored as
        _quaternion = [x, y, z, w], so q2, q3, q4, q1 read from
        _quaternion[0..3] in that order
      - f1..f3 are gravity residuals, f4..f6 are magnetic residuals
      - s1..s4 are the normalized gradient direction used as the
        correction term scaled by beta
      - q_dot1..q_dot4 are the quaternion derivatives used in the
        integrator
    """

    def __init__(self, beta: float = 0.1) -> None:
        """
        Initialize the filter state.

        Args:
            beta: Filter gain that trades gyro integration against correction
        """

        self._beta: float = float(beta)
        self._quaternion: np.ndarray = np.array(
            [0.0, 0.0, 0.0, 1.0], dtype=np.float64
        )
        self._p_theta_body: np.ndarray = np.zeros((3, 3), dtype=np.float64)
        self._covariance_valid: bool = False
        self._mag_ref_world_unit: Optional[np.ndarray] = None

    @property
    def quaternion(self) -> tuple[float, float, float, float]:
        """
        Get the current quaternion as (x, y, z, w).
        """

        return (
            float(self._quaternion[0]),
            float(self._quaternion[1]),
            float(self._quaternion[2]),
            float(self._quaternion[3]),
        )

    @property
    def theta_covariance(self) -> np.ndarray:
        """
        Get the current 3x3 small-angle covariance in the body frame.
        """

        return np.array(self._p_theta_body, dtype=np.float64, copy=True)

    @property
    def has_theta_covariance(self) -> bool:
        """
        Return True once a covariance propagation has occurred.
        """

        return self._covariance_valid

    def reset(self) -> None:
        """
        Reset the filter orientation to the identity quaternion.
        """

        self._quaternion = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        self._p_theta_body = np.zeros((3, 3), dtype=np.float64)
        self._covariance_valid = False
        self._mag_ref_world_unit = None

    def update_imu(
        self,
        gx: float,
        gy: float,
        gz: float,
        ax: float,
        ay: float,
        az: float,
        dt_s: float,
        cov_gyro_body: Optional[np.ndarray] = None,
        cov_accel_body: Optional[np.ndarray] = None,
    ) -> None:
        """
        Update the filter with gyro and accelerometer data.

        Args:
            gx: Angular velocity about x-axis in rad/s.
            gy: Angular velocity about y-axis in rad/s.
            gz: Angular velocity about z-axis in rad/s.
            ax: Accelerometer x-axis measurement in m/s^2.
            ay: Accelerometer y-axis measurement in m/s^2.
            az: Accelerometer z-axis measurement in m/s^2.
            dt_s: Time delta in seconds.
            cov_gyro_body: Gyro covariance (3x3) in (rad/s)^2.
            cov_accel_body: Accel covariance (3x3) in (m/s^2)^2.
        """

        dt: float = max(0.0, float(dt_s))
        if dt <= 0.0:
            return

        ax_raw: float = float(ax)
        ay_raw: float = float(ay)
        az_raw: float = float(az)

        accel_norm: float = math.sqrt(ax_raw * ax_raw + ay_raw * ay_raw + az_raw * az_raw)
        if accel_norm <= 0.0:
            self.update_imu(
                gx=gx,
                gy=gy,
                gz=gz,
                ax=ax,
                ay=ay,
                az=az,
                dt_s=dt,
                cov_gyro_body=cov_gyro_body,
                cov_accel_body=cov_accel_body,
            )
            return

        ax = ax_raw / accel_norm
        ay = ay_raw / accel_norm
        az = az_raw / accel_norm

        q1: float
        q2: float
        q3: float
        q4: float
        q2, q3, q4, q1 = (
            float(self._quaternion[0]),
            float(self._quaternion[1]),
            float(self._quaternion[2]),
            float(self._quaternion[3]),
        )

        # Units: unitless. Meaning: 1/2 factor from rotation matrix terms
        half: float = 0.5

        # Units: unitless. Meaning: double-angle scale in gradient terms
        two: float = 2.0

        # Units: unitless. Meaning: 4x factor from derivative expansion
        four: float = 4.0

        f1: float = two * (q2 * q4 - q1 * q3) - ax
        f2: float = two * (q1 * q2 + q3 * q4) - ay
        f3: float = two * (half - q2 * q2 - q3 * q3) - az

        s1: float = -two * q3 * f1 + two * q2 * f2
        s2: float = two * q4 * f1 + two * q1 * f2 - four * q2 * f3
        s3: float = -two * q1 * f1 + two * q4 * f2 - four * q3 * f3
        s4: float = two * q2 * f1 + two * q3 * f2

        s_norm: float = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
        if s_norm > 0.0:
            s1 /= s_norm
            s2 /= s_norm
            s3 /= s_norm
            s4 /= s_norm

        # Units: unitless. Meaning: half factor in quaternion derivative terms
        half_rate: float = 0.5

        q_dot1: float = -half_rate * (q2 * gx + q3 * gy + q4 * gz) - self._beta * s1
        q_dot2: float = half_rate * (q1 * gx + q3 * gz - q4 * gy) - self._beta * s2
        q_dot3: float = half_rate * (q1 * gy - q2 * gz + q4 * gx) - self._beta * s3
        q_dot4: float = half_rate * (q1 * gz + q2 * gy - q3 * gx) - self._beta * s4

        self._quaternion = np.array(
            [q2 + q_dot2 * dt, q3 + q_dot3 * dt, q4 + q_dot4 * dt, q1 + q_dot1 * dt],
            dtype=np.float64,
        )
        self._normalize_quaternion()
        self._update_accel_covariance(
            ax=ax_raw,
            ay=ay_raw,
            az=az_raw,
            cov_accel_body=cov_accel_body,
        )

    def update(
        self,
        gx: float,
        gy: float,
        gz: float,
        ax: float,
        ay: float,
        az: float,
        mx: float,
        my: float,
        mz: float,
        dt_s: float,
        cov_gyro_body: Optional[np.ndarray] = None,
        cov_accel_body: Optional[np.ndarray] = None,
        cov_mag_body: Optional[np.ndarray] = None,
    ) -> None:
        """
        Update the filter with gyro, accelerometer, and magnetometer data.

        Args:
            gx: Angular velocity about x-axis in rad/s.
            gy: Angular velocity about y-axis in rad/s.
            gz: Angular velocity about z-axis in rad/s.
            ax: Accelerometer x-axis measurement in m/s^2.
            ay: Accelerometer y-axis measurement in m/s^2.
            az: Accelerometer z-axis measurement in m/s^2.
            mx: Magnetometer x-axis measurement in tesla.
            my: Magnetometer y-axis measurement in tesla.
            mz: Magnetometer z-axis measurement in tesla.
            dt_s: Time delta in seconds.
            cov_gyro_body: Gyro covariance (3x3) in (rad/s)^2.
            cov_accel_body: Accel covariance (3x3) in (m/s^2)^2.
            cov_mag_body: Mag covariance (3x3) in tesla^2.

        See the class-level diagram for the full signal flow.
        """

        dt: float = max(0.0, float(dt_s))
        if dt <= 0.0:
            return

        self._propagate_covariance(dt, cov_gyro_body)

        ax_raw: float = float(ax)
        ay_raw: float = float(ay)
        az_raw: float = float(az)

        accel_norm: float = math.sqrt(ax_raw * ax_raw + ay_raw * ay_raw + az_raw * az_raw)
        if accel_norm <= 0.0:
            self._integrate_gyro(gx, gy, gz, dt)
            return

        mx_raw: float = float(mx)
        my_raw: float = float(my)
        mz_raw: float = float(mz)

        mag_norm: float = math.sqrt(mx_raw * mx_raw + my_raw * my_raw + mz_raw * mz_raw)
        if mag_norm <= 0.0:
            self.update_imu(
                gx=gx,
                gy=gy,
                gz=gz,
                ax=ax,
                ay=ay,
                az=az,
                dt_s=dt,
                cov_gyro_body=cov_gyro_body,
                cov_accel_body=cov_accel_body,
            )
            return

        self._propagate_covariance(dt, cov_gyro_body)

        ax = ax_raw / accel_norm
        ay = ay_raw / accel_norm
        az = az_raw / accel_norm

        mx = mx_raw / mag_norm
        my = my_raw / mag_norm
        mz = mz_raw / mag_norm

        q1: float
        q2: float
        q3: float
        q4: float
        q2, q3, q4, q1 = (
            float(self._quaternion[0]),
            float(self._quaternion[1]),
            float(self._quaternion[2]),
            float(self._quaternion[3]),
        )

        # Units: unitless. Meaning: 1/2 factor from rotation matrix terms
        half: float = 0.5

        # Units: unitless. Meaning: double-angle scale in gradient terms
        two: float = 2.0

        # Units: unitless. Meaning: 4x factor from derivative expansion
        four: float = 4.0

        hx: float = (
            two * mx * (half - q3 * q3 - q4 * q4)
            + two * my * (q2 * q3 - q1 * q4)
            + two * mz * (q2 * q4 + q1 * q3)
        )
        hy: float = (
            two * mx * (q2 * q3 + q1 * q4)
            + two * my * (half - q2 * q2 - q4 * q4)
            + two * mz * (q3 * q4 - q1 * q2)
        )

        # Units: unitless. Meaning: horizontal magnetic field magnitude
        b_x: float = math.sqrt(hx * hx + hy * hy)

        b_z: float = (
            two * mx * (q2 * q4 - q1 * q3)
            + two * my * (q3 * q4 + q1 * q2)
            + two * mz * (half - q2 * q2 - q3 * q3)
        )

        f1: float = two * (q2 * q4 - q1 * q3) - ax
        f2: float = two * (q1 * q2 + q3 * q4) - ay
        f3: float = two * (half - q2 * q2 - q3 * q3) - az
        f4: float = (
            two * b_x * (half - q3 * q3 - q4 * q4)
            + two * b_z * (q2 * q4 - q1 * q3)
            - mx
        )
        f5: float = (
            two * b_x * (q2 * q3 - q1 * q4) + two * b_z * (q1 * q2 + q3 * q4) - my
        )
        f6: float = (
            two * b_x * (q1 * q3 + q2 * q4)
            + two * b_z * (half - q2 * q2 - q3 * q3)
            - mz
        )

        s1: float = (
            -two * q3 * f1
            + two * q2 * f2
            - two * b_z * q3 * f4
            + (-two * b_x * q4 + two * b_z * q2) * f5
            + two * b_x * q3 * f6
        )
        s2: float = (
            two * q4 * f1
            + two * q1 * f2
            - four * q2 * f3
            + two * b_z * q4 * f4
            + (two * b_x * q3 + two * b_z * q1) * f5
            + (two * b_x * q4 - four * b_z * q2) * f6
        )
        s3: float = (
            -two * q1 * f1
            + two * q4 * f2
            - four * q3 * f3
            + (-four * b_x * q3 - two * b_z * q1) * f4
            + (two * b_x * q2 + two * b_z * q4) * f5
            + (two * b_x * q1 - four * b_z * q3) * f6
        )
        s4: float = (
            two * q2 * f1
            + two * q3 * f2
            + (-four * b_x * q4 + two * b_z * q2) * f4
            + (-two * b_x * q1 + two * b_z * q3) * f5
            + two * b_x * q2 * f6
        )

        s_norm: float = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
        if s_norm > 0.0:
            s1 /= s_norm
            s2 /= s_norm
            s3 /= s_norm
            s4 /= s_norm

        # Units: unitless. Meaning: half factor in quaternion derivative terms
        half_rate: float = 0.5

        q_dot1: float = -half_rate * (q2 * gx + q3 * gy + q4 * gz) - self._beta * s1
        q_dot2: float = half_rate * (q1 * gx + q3 * gz - q4 * gy) - self._beta * s2
        q_dot3: float = half_rate * (q1 * gy - q2 * gz + q4 * gx) - self._beta * s3
        q_dot4: float = half_rate * (q1 * gz + q2 * gy - q3 * gx) - self._beta * s4

        self._quaternion = np.array(
            [q2 + q_dot2 * dt, q3 + q_dot3 * dt, q4 + q_dot4 * dt, q1 + q_dot1 * dt],
            dtype=np.float64,
        )
        self._normalize_quaternion()
        self._update_accel_covariance(
            ax=ax_raw,
            ay=ay_raw,
            az=az_raw,
            cov_accel_body=cov_accel_body,
        )
        self._update_mag_reference(mx=mx_raw, my=my_raw, mz=mz_raw)
        self._update_mag_covariance(
            mx=mx_raw,
            my=my_raw,
            mz=mz_raw,
            cov_mag_body=cov_mag_body,
        )

    def _integrate_gyro(self, gx: float, gy: float, gz: float, dt: float) -> None:
        q1: float
        q2: float
        q3: float
        q4: float
        q2, q3, q4, q1 = (
            float(self._quaternion[0]),
            float(self._quaternion[1]),
            float(self._quaternion[2]),
            float(self._quaternion[3]),
        )

        # Units: unitless. Meaning: half factor in quaternion derivative terms
        half_rate: float = 0.5

        q_dot1: float = -half_rate * (q2 * gx + q3 * gy + q4 * gz)
        q_dot2: float = half_rate * (q1 * gx + q3 * gz - q4 * gy)
        q_dot3: float = half_rate * (q1 * gy - q2 * gz + q4 * gx)
        q_dot4: float = half_rate * (q1 * gz + q2 * gy - q3 * gx)

        self._quaternion = np.array(
            [q2 + q_dot2 * dt, q3 + q_dot3 * dt, q4 + q_dot4 * dt, q1 + q_dot1 * dt],
            dtype=np.float64,
        )
        self._normalize_quaternion()

    def _normalize_quaternion(self) -> None:
        norm: float = float(np.linalg.norm(self._quaternion))
        if norm <= 0.0:
            self._quaternion = np.array(
                [0.0, 0.0, 0.0, 1.0], dtype=np.float64
            )
            return

        self._quaternion /= norm

    def _normalized_quaternion(self) -> np.ndarray:
        quaternion: np.ndarray = np.array(self._quaternion, dtype=np.float64, copy=True)
        norm: float = float(np.linalg.norm(quaternion))
        if norm <= 0.0:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

        return quaternion / norm

    def _propagate_covariance(
        self, dt: float, cov_gyro_body: Optional[np.ndarray]
    ) -> None:
        cov_gyro: np.ndarray = self._gyro_covariance(cov_gyro_body)

        dt_sq: float = dt * dt

        # Units: rad^2. Meaning: process covariance from white-noise model
        q_process: float = PROCESS_NOISE_THETA_RAD2_PER_S * dt

        q_process_matrix: np.ndarray = np.eye(3, dtype=np.float64) * q_process

        self._p_theta_body = self._p_theta_body + dt_sq * cov_gyro + q_process_matrix
        self._p_theta_body = _symmetrize(self._p_theta_body)
        self._covariance_valid = True

    def _gyro_covariance(self, cov_gyro_body: Optional[np.ndarray]) -> np.ndarray:
        cov_body: Optional[np.ndarray] = _as_covariance(cov_gyro_body)
        if cov_body is None:
            default_cov: np.ndarray = (
                np.eye(3, dtype=np.float64) * DEFAULT_GYRO_VAR_RAD2_PER_S2
            )
            return default_cov

        return _symmetrize(cov_body)

    def _update_accel_covariance(
        self,
        ax: float,
        ay: float,
        az: float,
        cov_accel_body: Optional[np.ndarray],
    ) -> None:
        cov_body: Optional[np.ndarray] = _as_covariance(cov_accel_body)
        if cov_body is None:
            return
        cov_body = _symmetrize(cov_body)

        accel: np.ndarray = np.array([ax, ay, az], dtype=np.float64)
        accel_norm: float = float(np.linalg.norm(accel))
        if accel_norm <= 0.0:
            return

        a_meas_unit: np.ndarray = accel / accel_norm

        quaternion: np.ndarray = self._normalized_quaternion()
        rot_world_to_body: np.ndarray = _rotation_matrix_world_to_body(quaternion)

        g_world_unit: np.ndarray = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        a_pred_body_unit: np.ndarray = rot_world_to_body @ g_world_unit

        self._update_direction_covariance(
            measured_unit=a_meas_unit,
            predicted_unit=a_pred_body_unit,
            measurement_vector=accel,
            cov_body=cov_body,
        )

    def _update_mag_reference(self, mx: float, my: float, mz: float) -> None:
        mag: np.ndarray = np.array([mx, my, mz], dtype=np.float64)
        mag_norm: float = float(np.linalg.norm(mag))
        if mag_norm <= 0.0:
            return

        m_body_unit: np.ndarray = mag / mag_norm
        quaternion: np.ndarray = self._normalized_quaternion()
        rot_body_to_world: np.ndarray = _rotation_matrix_body_to_world(quaternion)

        m_world_meas_unit: np.ndarray = rot_body_to_world @ m_body_unit
        m_world_meas_unit = _normalize_vector(m_world_meas_unit)

        if self._mag_ref_world_unit is None:
            self._mag_ref_world_unit = m_world_meas_unit
            return

        # Units: unitless. Meaning: low-pass blend in world frame
        alpha: float = MAG_REF_ALPHA

        blended: np.ndarray = (
            (1.0 - alpha) * self._mag_ref_world_unit + alpha * m_world_meas_unit
        )
        self._mag_ref_world_unit = _normalize_vector(blended)

    def _update_mag_covariance(
        self,
        mx: float,
        my: float,
        mz: float,
        cov_mag_body: Optional[np.ndarray],
    ) -> None:
        cov_body: Optional[np.ndarray] = _as_covariance(cov_mag_body)
        if cov_body is None:
            return
        cov_body = _symmetrize(cov_body)

        if self._mag_ref_world_unit is None:
            return

        mag: np.ndarray = np.array([mx, my, mz], dtype=np.float64)
        mag_norm: float = float(np.linalg.norm(mag))
        if mag_norm <= 0.0:
            return

        m_body_unit: np.ndarray = mag / mag_norm
        quaternion: np.ndarray = self._normalized_quaternion()
        rot_world_to_body: np.ndarray = _rotation_matrix_world_to_body(quaternion)

        m_pred_body_unit: np.ndarray = rot_world_to_body @ self._mag_ref_world_unit

        self._update_direction_covariance(
            measured_unit=m_body_unit,
            predicted_unit=m_pred_body_unit,
            measurement_vector=mag,
            cov_body=cov_body,
        )

    def _update_direction_covariance(
        self,
        measured_unit: np.ndarray,
        predicted_unit: np.ndarray,
        measurement_vector: np.ndarray,
        cov_body: np.ndarray,
    ) -> None:
        predicted_unit = _normalize_vector(predicted_unit)
        measured_unit = _normalize_vector(measured_unit)

        residual: np.ndarray = measured_unit - predicted_unit

        # Units: unitless. Meaning: residual uses r = z - h(x) for EKF
        _ = residual

        # Units: unitless. Meaning: small-angle error rotates vector by
        # -[v]_x * theta, hence the negative sign in H
        h_matrix: np.ndarray = -_skew(predicted_unit)

        measurement_norm: float = float(np.linalg.norm(measurement_vector))
        if measurement_norm <= 0.0:
            return

        # Units: 1/units. Meaning: Jacobian of normalize(v) with respect to v
        inv_norm: float = 1.0 / measurement_norm
        inv_norm_cubed: float = inv_norm / (measurement_norm * measurement_norm)

        outer: np.ndarray = np.outer(measurement_vector, measurement_vector)
        j_norm: np.ndarray = (
            np.eye(3, dtype=np.float64) * inv_norm - outer * inv_norm_cubed
        )

        # Units: unitless. Meaning: covariance of the normalized direction
        r_meas: np.ndarray = j_norm @ cov_body @ j_norm.T
        r_meas = _symmetrize(r_meas)

        p_matrix: np.ndarray = self._p_theta_body
        s_matrix: np.ndarray = h_matrix @ p_matrix @ h_matrix.T + r_meas
        s_inv: np.ndarray = np.linalg.inv(s_matrix)

        k_matrix: np.ndarray = p_matrix @ h_matrix.T @ s_inv
        identity: np.ndarray = np.eye(3, dtype=np.float64)
        kh: np.ndarray = k_matrix @ h_matrix

        # Units: unitless. Meaning: Joseph-form covariance update for symmetry
        self._p_theta_body = (
            (identity - kh) @ p_matrix @ (identity - kh).T + k_matrix @ r_meas @ k_matrix.T
        )
        self._p_theta_body = _symmetrize(self._p_theta_body)


def _as_covariance(covariance: Optional[np.ndarray]) -> Optional[np.ndarray]:
    if covariance is None:
        return None

    cov_array: np.ndarray = np.asarray(covariance, dtype=np.float64)
    if cov_array.shape != (3, 3):
        return None

    return cov_array


def _symmetrize(matrix: np.ndarray) -> np.ndarray:
    return 0.5 * (matrix + matrix.T)


def _normalize_vector(vector: np.ndarray) -> np.ndarray:
    norm: float = float(np.linalg.norm(vector))
    if norm <= 0.0:
        return np.array([0.0, 0.0, 0.0], dtype=np.float64)

    return vector / norm


def _skew(vector: np.ndarray) -> np.ndarray:
    vx: float = float(vector[0])
    vy: float = float(vector[1])
    vz: float = float(vector[2])

    return np.array(
        [[0.0, -vz, vy], [vz, 0.0, -vx], [-vy, vx, 0.0]], dtype=np.float64
    )


def _rotation_matrix_world_to_body(quaternion: np.ndarray) -> np.ndarray:
    x: float = float(quaternion[0])
    y: float = float(quaternion[1])
    z: float = float(quaternion[2])
    w: float = float(quaternion[3])

    xx: float = x * x
    yy: float = y * y
    zz: float = z * z

    xy: float = x * y
    xz: float = x * z
    yz: float = y * z

    wx: float = w * x
    wy: float = w * y
    wz: float = w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy)],
            [2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx)],
            [2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _rotation_matrix_body_to_world(quaternion: np.ndarray) -> np.ndarray:
    return _rotation_matrix_world_to_body(quaternion).T
