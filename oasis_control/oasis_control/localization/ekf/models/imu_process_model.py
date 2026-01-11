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

    def propagate_nominal_and_covariance_substepped(
        self,
        state: EkfState,
        covariance: np.ndarray,
        *,
        omega_raw_rps: np.ndarray,
        accel_raw_mps2: np.ndarray,
        dt_ns: int,
        max_dt_ns: int,
    ) -> np.ndarray:
        """
        Propagate the nominal state and covariance with optional substeps

        Returns the updated covariance
        """

        steps: int = self.substep_count(dt_ns, max_dt_ns)
        base_step_ns: int = dt_ns // steps
        remainder_ns: int = dt_ns - base_step_ns * steps
        omega_raw: np.ndarray = np.asarray(omega_raw_rps, dtype=float).reshape(3)
        accel_raw: np.ndarray = np.asarray(accel_raw_mps2, dtype=float).reshape(3)
        self._assert_finite_array("omega_raw_rps", omega_raw)
        self._assert_finite_array("accel_raw_mps2", accel_raw)

        p: np.ndarray = covariance
        for step_index in range(steps):
            step_dt_ns: int = base_step_ns + (1 if step_index < remainder_ns else 0)
            step_dt_s: float = float(step_dt_ns) * _NS_TO_S
            f_mat, g_mat, q_c = self._continuous_dynamics_matrices(
                state,
                omega_raw_rps=omega_raw,
                accel_raw_mps2=accel_raw,
            )
            phi: np.ndarray = self._discrete_state_transition(f_mat, step_dt_s)
            q_d: np.ndarray = self._discrete_process_noise(f_mat, g_mat, q_c, step_dt_s)
            p = phi @ p @ phi.T + q_d
            p = 0.5 * (p + p.T)
            self.propagate_nominal(
                state,
                omega_raw_rps=omega_raw,
                accel_raw_mps2=accel_raw,
                dt_ns=step_dt_ns,
            )
        return p

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

    def _continuous_dynamics_matrices(
        self,
        state: EkfState,
        *,
        omega_raw_rps: np.ndarray,
        accel_raw_mps2: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
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

        omega_corr: np.ndarray = omega_raw_rps - gyro_bias
        accel_corr: np.ndarray = accel_a @ (accel_raw_mps2 - accel_bias)
        rot_odom_from_body: np.ndarray = quat_to_rotation_matrix(
            state.pose_ob.rotation_wxyz
        )
        skew_omega: np.ndarray = self._skew_symmetric(omega_corr)
        skew_accel: np.ndarray = self._skew_symmetric(accel_corr)

        total_dim: int = state.index.total_dim
        f_mat: np.ndarray = np.zeros((total_dim, total_dim), dtype=float)
        g_mat: np.ndarray = np.zeros((total_dim, 12), dtype=float)
        q_c: np.ndarray = np.zeros((12, 12), dtype=float)

        pose_start: int = state.index.pose.start
        rot_start: int = pose_start + 3
        vel_start: int = state.index.velocity.start
        accel_bias_start: int = state.index.accel_bias.start
        gyro_bias_start: int = state.index.gyro_bias.start

        f_mat[pose_start : pose_start + 3, vel_start : vel_start + 3] = np.eye(
            3, dtype=float
        )
        f_mat[vel_start : vel_start + 3, rot_start : rot_start + 3] = (
            -rot_odom_from_body @ skew_accel
        )
        f_mat[
            vel_start : vel_start + 3,
            accel_bias_start : accel_bias_start + 3,
        ] = (
            -rot_odom_from_body @ accel_a
        )
        f_mat[rot_start : rot_start + 3, rot_start : rot_start + 3] = -skew_omega
        f_mat[rot_start : rot_start + 3, gyro_bias_start : gyro_bias_start + 3] = (
            -np.eye(3, dtype=float)
        )

        g_mat[rot_start : rot_start + 3, 0:3] = -np.eye(3, dtype=float)
        g_mat[vel_start : vel_start + 3, 3:6] = -rot_odom_from_body
        g_mat[gyro_bias_start : gyro_bias_start + 3, 6:9] = np.eye(3, dtype=float)
        g_mat[accel_bias_start : accel_bias_start + 3, 9:12] = np.eye(3, dtype=float)

        accel_noise_var: float = float(self._config.accel_noise_var)
        gyro_noise_var: float = float(self._config.gyro_noise_var)
        self._assert_finite_scalar("accel_noise_var", accel_noise_var)
        self._assert_finite_scalar("gyro_noise_var", gyro_noise_var)

        # Units: (rad/s)^2/Hz. Meaning: continuous gyro noise variance
        q_c[0:3, 0:3] = np.eye(3, dtype=float) * gyro_noise_var

        # Units: (m/s^2)^2/Hz. Meaning: continuous accel noise variance
        q_c[3:6, 3:6] = np.eye(3, dtype=float) * accel_noise_var
        return f_mat, g_mat, q_c

    def _discrete_state_transition(self, f_mat: np.ndarray, dt_s: float) -> np.ndarray:
        total_dim: int = f_mat.shape[0]
        identity: np.ndarray = np.eye(total_dim, dtype=float)
        f_dt: np.ndarray = f_mat * dt_s
        phi: np.ndarray = identity + f_dt + 0.5 * (f_dt @ f_dt)
        return phi

    def _discrete_process_noise(
        self,
        f_mat: np.ndarray,
        g_mat: np.ndarray,
        q_c: np.ndarray,
        dt_s: float,
    ) -> np.ndarray:
        q_cont: np.ndarray = g_mat @ q_c @ g_mat.T
        f_q: np.ndarray = f_mat @ q_cont
        q_f: np.ndarray = q_cont @ f_mat.T

        dt_2: float = dt_s * dt_s

        # Units: seconds. Meaning: first-order integration weight
        term_1: np.ndarray = q_cont * dt_s

        # Units: seconds^2. Meaning: second-order integration weight
        term_2: np.ndarray = 0.5 * (f_q + q_f) * dt_2

        dt_3: float = dt_2 * dt_s

        # Units: seconds^3. Meaning: third-order integration weight
        term_3: np.ndarray = (
            (1.0 / 6.0) * (f_mat @ f_q + q_f @ f_mat.T + f_mat @ q_f) * dt_3
        )
        q_d: np.ndarray = term_1 + term_2 + term_3
        return 0.5 * (q_d + q_d.T)

    def _skew_symmetric(self, vec: np.ndarray) -> np.ndarray:
        x: float = float(vec[0])
        y: float = float(vec[1])
        z: float = float(vec[2])
        return np.array(
            [
                [0.0, -z, y],
                [z, 0.0, -x],
                [-y, x, 0.0],
            ],
            dtype=float,
        )

    def _assert_finite_array(self, name: str, value: np.ndarray) -> None:
        if not np.all(np.isfinite(value)):
            raise ValueError(f"{name} must be finite")

    def _assert_finite_scalar(self, name: str, value: float) -> None:
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")

    def _assert_positive_ns(self, name: str, value: int) -> None:
        if value <= 0:
            raise ValueError(f"{name} must be positive")
