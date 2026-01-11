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
IMU-driven process model for EKF propagation
"""

from __future__ import annotations

import math

import numpy as np

from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_state import EkfState
from oasis_control.localization.ekf.se3 import normalize_quaternion
from oasis_control.localization.ekf.se3 import quat_from_rotvec
from oasis_control.localization.ekf.se3 import quat_multiply
from oasis_control.localization.ekf.se3 import quat_to_rotation_matrix


# Seconds per nanosecond for time conversions
_NS_TO_S: float = 1.0e-9


class ImuProcessModel:
    """
    IMU-driven propagation and process noise model
    """

    def __init__(self, config: EkfConfig) -> None:
        self._config: EkfConfig = config

    def propagate_nominal(
        self,
        state: EkfState,
        *,
        omega_raw_rps: np.ndarray,
        accel_raw_mps2: np.ndarray,
        dt_ns: int,
    ) -> None:
        """
        Propagate the nominal state with IMU inputs

        Args:
            state: EKF nominal state to update in-place
            omega_raw_rps: Angular velocity in the IMU frame (rad/s)
            accel_raw_mps2: Linear acceleration in the IMU frame (m/s^2)
            dt_ns: Time step in nanoseconds
        """

        self._assert_positive_ns("dt_ns", dt_ns)
        dt_s: float = float(dt_ns) * _NS_TO_S

        omega_raw: np.ndarray = np.asarray(omega_raw_rps, dtype=float).reshape(3)
        accel_raw: np.ndarray = np.asarray(accel_raw_mps2, dtype=float).reshape(3)
        self._assert_finite_array("omega_raw_rps", omega_raw)
        self._assert_finite_array("accel_raw_mps2", accel_raw)

        gyro_bias: np.ndarray = np.asarray(
            state.imu_bias_gyro_rps, dtype=float
        ).reshape(3)
        accel_bias: np.ndarray = np.asarray(
            state.imu_bias_accel_mps2, dtype=float
        ).reshape(3)
        accel_a: np.ndarray = np.asarray(state.imu_accel_a, dtype=float).reshape(3, 3)
        self._assert_finite_array("imu_bias_gyro_rps", gyro_bias)
        self._assert_finite_array("imu_bias_accel_mps2", accel_bias)
        self._assert_finite_array("imu_accel_a", accel_a)

        omega_corr: np.ndarray = omega_raw - gyro_bias
        accel_corr: np.ndarray = accel_a @ (accel_raw - accel_bias)

        delta_rot: np.ndarray = omega_corr * dt_s
        delta_quat: np.ndarray = quat_from_rotvec(delta_rot)
        quat_ob: np.ndarray = quat_multiply(state.pose_ob.rotation_wxyz, delta_quat)
        quat_ob = normalize_quaternion(quat_ob)
        state.pose_ob.rotation_wxyz = quat_ob

        rot_odom_from_body: np.ndarray = quat_to_rotation_matrix(quat_ob)
        accel_odom_mps2: np.ndarray = rot_odom_from_body @ accel_corr

        gravity_mps2: float = self._config.gravity_mps2
        accel_odom_mps2 = accel_odom_mps2 - np.array(
            [0.0, 0.0, gravity_mps2], dtype=float
        )

        vel: np.ndarray = state.vel_o_mps + accel_odom_mps2 * dt_s
        pos: np.ndarray = state.pose_ob.translation_m + vel * dt_s
        state.pose_ob.translation_m = pos
        state.vel_o_mps = vel

    def discrete_process_noise(self, state: EkfState, *, dt_ns: int) -> np.ndarray:
        """
        Build discrete-time process noise for the packed error-state

        Returns the v0 closed-form discrete-time Q over dt_ns that does not
        depend on max_dt_ns or substepping

        Args:
            state: EKF state for indexing
            dt_ns: Time step in nanoseconds

        Returns:
            Symmetric process noise matrix Q in packed error-state coordinates
        """

        self._assert_positive_ns("dt_ns", dt_ns)
        dt_s: float = float(dt_ns) * _NS_TO_S

        total_dim: int = state.index.total_dim
        q: np.ndarray = np.zeros((total_dim, total_dim), dtype=float)

        accel_noise_var: float = float(self._config.accel_noise_var)
        gyro_noise_var: float = float(self._config.gyro_noise_var)
        self._assert_finite_scalar("accel_noise_var", accel_noise_var)
        self._assert_finite_scalar("gyro_noise_var", gyro_noise_var)

        # Position variance in m^2 from white accel noise (dt^4 / 4)
        pos_noise: float = 0.25 * accel_noise_var * (dt_s**4)

        # Position/velocity covariance in m^2/s from white accel noise (dt^3 / 2)
        pos_vel_noise: float = 0.5 * accel_noise_var * (dt_s**3)

        # Velocity variance in (m/s)^2 from white accel noise (dt^2)
        vel_noise: float = accel_noise_var * (dt_s**2)

        # Angle variance in rad^2 from white gyro noise (dt^2)
        ang_noise: float = gyro_noise_var * (dt_s**2)

        pose_start: int = state.index.pose.start
        rot_start: int = pose_start + 3
        vel_start: int = state.index.velocity.start
        for axis in range(3):
            pos_index: int = pose_start + axis
            vel_index: int = vel_start + axis
            ang_index: int = rot_start + axis
            q[pos_index, pos_index] += pos_noise
            q[pos_index, vel_index] += pos_vel_noise
            q[vel_index, pos_index] += pos_vel_noise
            q[vel_index, vel_index] += vel_noise
            q[ang_index, ang_index] += ang_noise
        return q

    def substep_count(self, dt_ns: int, max_dt_ns: int) -> int:
        """
        Compute the number of substeps for a time interval

        Args:
            dt_ns: Total time step in nanoseconds
            max_dt_ns: Max allowed step size in nanoseconds

        Returns:
            Number of substeps N >= 1
        """

        self._assert_positive_ns("dt_ns", dt_ns)
        self._assert_positive_ns("max_dt_ns", max_dt_ns)

        steps: int = (dt_ns + max_dt_ns - 1) // max_dt_ns
        return max(1, steps)

    def propagate_nominal_substepped(
        self,
        state: EkfState,
        *,
        omega_raw_rps: np.ndarray,
        accel_raw_mps2: np.ndarray,
        dt_ns: int,
        max_dt_ns: int,
    ) -> None:
        """
        Propagate the nominal state with optional substeps
        """

        steps: int = self.substep_count(dt_ns, max_dt_ns)
        base_step_ns: int = dt_ns // steps
        remainder_ns: int = dt_ns - base_step_ns * steps
        # Propagation uses previous-sample inputs (ZOH) to ensure replay is
        # order-invariant
        for step_index in range(steps):
            step_dt_ns: int = base_step_ns + (1 if step_index < remainder_ns else 0)
            self.propagate_nominal(
                state,
                omega_raw_rps=omega_raw_rps,
                accel_raw_mps2=accel_raw_mps2,
                dt_ns=step_dt_ns,
            )

    def discrete_process_noise_substepped(
        self, state: EkfState, *, dt_ns: int, max_dt_ns: int
    ) -> np.ndarray:
        """
        Integrate process noise over substeps
        """

        steps: int = self.substep_count(dt_ns, max_dt_ns)
        base_step_ns: int = dt_ns // steps
        remainder_ns: int = dt_ns - base_step_ns * steps
        q_total: np.ndarray = np.zeros(
            (state.index.total_dim, state.index.total_dim), dtype=float
        )
        for step_index in range(steps):
            step_dt_ns: int = base_step_ns + (1 if step_index < remainder_ns else 0)
            q_total = q_total + self.discrete_process_noise(state, dt_ns=step_dt_ns)
        return q_total

    def _assert_finite_array(self, name: str, value: np.ndarray) -> None:
        if not np.all(np.isfinite(value)):
            raise ValueError(f"{name} must be finite")

    def _assert_finite_scalar(self, name: str, value: float) -> None:
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")

    def _assert_positive_ns(self, name: str, value: int) -> None:
        if value <= 0:
            raise ValueError(f"{name} must be positive")
