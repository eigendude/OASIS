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

import numpy as np


################################################################################
# Madgwick AHRS
################################################################################


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
        self._quaternion: np.ndarray = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

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

    def reset(self) -> None:
        """
        Reset the filter orientation to the identity quaternion.
        """

        self._quaternion = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

    def update_imu(
        self,
        gx: float,
        gy: float,
        gz: float,
        ax: float,
        ay: float,
        az: float,
        dt_s: float,
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
        """

        dt: float = max(0.0, float(dt_s))
        if dt <= 0.0:
            return

        accel_norm: float = math.sqrt(ax * ax + ay * ay + az * az)
        if accel_norm <= 0.0:
            self._integrate_gyro(gx, gy, gz, dt)
            return

        ax /= accel_norm
        ay /= accel_norm
        az /= accel_norm

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
            dtype=float,
        )
        self._normalize_quaternion()

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

        See the class-level diagram for the full signal flow.
        """

        dt: float = max(0.0, float(dt_s))
        if dt <= 0.0:
            return

        accel_norm: float = math.sqrt(ax * ax + ay * ay + az * az)
        if accel_norm <= 0.0:
            self._integrate_gyro(gx, gy, gz, dt)
            return

        mag_norm: float = math.sqrt(mx * mx + my * my + mz * mz)
        if mag_norm <= 0.0:
            self.update_imu(gx, gy, gz, ax, ay, az, dt)
            return

        ax /= accel_norm
        ay /= accel_norm
        az /= accel_norm

        mx /= mag_norm
        my /= mag_norm
        mz /= mag_norm

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
            dtype=float,
        )
        self._normalize_quaternion()

    def _integrate_gyro(self, gx: float, gy: float, gz: float, dt: float) -> None:
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
            dtype=float,
        )
        self._normalize_quaternion()

    def _normalize_quaternion(self) -> None:
        norm: float = float(np.linalg.norm(self._quaternion))
        if norm <= 0.0:
            self._quaternion = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
            return

        self._quaternion /= norm
