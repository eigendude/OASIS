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
from typing import Optional

import numpy as np


################################################################################
# Data model
################################################################################


STATE_LEARNING: str = "learning"
STATE_CONVERGED: str = "converged"
STATE_LOCKED: str = "locked"


@dataclass(frozen=True)
class LearnerConfig:
    """
    Configuration for IMU mount learning

    Attributes:
        process_var_rad2_per_s: Process variance for the mount estimate in
            rad^2/s. This controls how quickly the learner can adapt when the
            platform is moving
        gravity_roll_pitch_var_rad2: Gravity-based roll/pitch observation
            variance in rad^2
        gravity_yaw_var_rad2: Gravity-based yaw observation variance in rad^2.
            Large because gravity does not directly observe yaw
        heading_var_reference_rad2: Reference heading variance in rad^2 used to
            scale yaw updates. Large incoming heading variance reduces yaw
            learning gain
        converged_stddev_rad: Angular standard deviation threshold for the
            converged state in radians
        lock_stddev_rad: Angular standard deviation threshold for the locked
            state in radians
        lock_min_duration_s: Minimum time in seconds that convergence and
            stationarity must hold before entering locked
        stationary_accel_delta_mps2: Maximum allowed | ||a|| - g | in m/s^2 for
            stationarity checks
        stationary_gyro_norm_rads: Maximum angular-rate norm in rad/s for
            stationarity checks
    """

    process_var_rad2_per_s: float = 1.0e-4
    gravity_roll_pitch_var_rad2: float = 2.5e-3
    gravity_yaw_var_rad2: float = 1.0e6
    heading_var_reference_rad2: float = 5.0e-3
    converged_stddev_rad: float = math.radians(3.0)
    lock_stddev_rad: float = math.radians(1.5)
    lock_min_duration_s: float = 2.0
    stationary_accel_delta_mps2: float = 0.4
    stationary_gyro_norm_rads: float = 0.12


@dataclass(frozen=True)
class LearnerInput:
    """
    Input sample for one learner update

    Attributes:
        imu_orientation_xyzw: Orientation quaternion [x, y, z, w] from
            sensor_msgs/Imu.orientation
        imu_orientation_cov: Full 3x3 orientation covariance from
            sensor_msgs/Imu.orientation_covariance in row-major order
        gravity_mps2: Optional gravity vector in IMU frame in m/s^2
        accel_mps2: Optional acceleration vector in IMU frame in m/s^2
        angular_velocity_rads: Optional angular velocity vector in IMU frame
            in rad/s
        dt_s: Time delta in seconds since the previous learner update
    """

    imu_orientation_xyzw: np.ndarray
    imu_orientation_cov: np.ndarray
    gravity_mps2: Optional[np.ndarray]
    accel_mps2: Optional[np.ndarray]
    angular_velocity_rads: Optional[np.ndarray]
    dt_s: float


@dataclass(frozen=True)
class LearnerOutput:
    """
    Current learner output state

    Attributes:
        mount_quat_xyzw: Learned quaternion [x, y, z, w] for T_BI where parent
            is body and child is IMU
        mount_cov_rad2: Full 3x3 covariance for small-angle mount error in
            radians^2
        tilt_rpy_rad: Tilt [roll, pitch, yaw] in radians. Yaw is always zero to
            keep this topic gravity-referenced instead of full heading
        state: One of learning, converged, or locked
        yaw_update_weight: 0..1 gain-like factor used for yaw update strength
            from heading covariance
    """

    mount_quat_xyzw: np.ndarray
    mount_cov_rad2: np.ndarray
    tilt_rpy_rad: np.ndarray
    state: str
    yaw_update_weight: float


################################################################################
# Core learner
################################################################################


class ImuMountLearner:
    """
    Learn a fixed IMU-to-body transform from fused IMU orientation and gravity

    TF convention:
        T_BI is published with parent body_frame and child imu_frame
        The stored quaternion rotates IMU-frame vectors into body-frame vectors:
            v_B = R_BI * v_I

    Covariance handling:
        The learner keeps a full 3x3 covariance matrix over small-angle mount
        error and performs full-matrix Kalman-style updates. Incoming IMU
        orientation covariance is consumed as a full matrix. Yaw correction
        strength is scaled by heading covariance without diagonalizing known
        full covariances
    """

    def __init__(self, config: LearnerConfig) -> None:
        self._config: LearnerConfig = config

        self._mount_quat_xyzw: np.ndarray = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self._mount_cov_rad2: np.ndarray = np.eye(3, dtype=float) * 0.25

        self._state: str = STATE_LEARNING
        self._lock_candidate_start_s: Optional[float] = None

    def update(self, sample: LearnerInput, timestamp_s: float) -> LearnerOutput:
        dt_s: float = max(0.0, float(sample.dt_s))

        process_q: np.ndarray = np.eye(3, dtype=float) * (
            self._config.process_var_rad2_per_s * dt_s
        )
        pred_cov: np.ndarray = self._sanitize_covariance(
            self._mount_cov_rad2 + process_q
        )

        q_wi: np.ndarray = self._normalize_quaternion(sample.imu_orientation_xyzw)

        # IMU fused orientation measurement for mount: q_BI = q_IW when the
        # body reference is initialized level in the world frame
        q_meas_bi: np.ndarray = self._quat_conjugate(q_wi)

        imu_cov: np.ndarray = self._sanitize_covariance(sample.imu_orientation_cov)

        yaw_var_rad2: float = float(imu_cov[2, 2])
        yaw_update_weight: float = self._yaw_weight_from_variance(yaw_var_rad2)
        imu_cov_weighted: np.ndarray = self._scale_yaw_covariance(
            imu_cov,
            yaw_update_weight,
        )

        self._mount_quat_xyzw, pred_cov = self._apply_measurement_update(
            self._mount_quat_xyzw,
            pred_cov,
            q_meas_bi,
            imu_cov_weighted,
        )

        if sample.gravity_mps2 is not None:
            gravity_vec: np.ndarray = np.array(sample.gravity_mps2, dtype=float)
            gravity_norm: float = float(np.linalg.norm(gravity_vec))

            if gravity_norm > 1.0e-6:
                q_gravity: np.ndarray = self._quat_from_two_vectors(
                    gravity_vec,
                    np.array([0.0, 0.0, -1.0], dtype=float),
                )

                gravity_cov: np.ndarray = np.diag(
                    [
                        self._config.gravity_roll_pitch_var_rad2,
                        self._config.gravity_roll_pitch_var_rad2,
                        self._config.gravity_yaw_var_rad2,
                    ]
                )

                accel_for_gate: Optional[np.ndarray] = None
                if sample.accel_mps2 is not None:
                    accel_for_gate = np.array(sample.accel_mps2, dtype=float)

                gravity_cov = self._inflate_gravity_covariance(
                    gravity_cov,
                    accel_for_gate,
                )

                self._mount_quat_xyzw, pred_cov = self._apply_measurement_update(
                    self._mount_quat_xyzw,
                    pred_cov,
                    q_gravity,
                    gravity_cov,
                )

        self._mount_cov_rad2 = self._sanitize_covariance(pred_cov)

        tilt_rpy_rad: np.ndarray = self._estimate_tilt(
            self._mount_quat_xyzw,
            sample.gravity_mps2,
            q_wi,
        )

        self._update_state(
            timestamp_s=timestamp_s,
            accel_mps2=sample.accel_mps2,
            gyro_rads=sample.angular_velocity_rads,
        )

        return LearnerOutput(
            mount_quat_xyzw=self._mount_quat_xyzw.copy(),
            mount_cov_rad2=self._mount_cov_rad2.copy(),
            tilt_rpy_rad=tilt_rpy_rad,
            state=self._state,
            yaw_update_weight=yaw_update_weight,
        )

    def current_state(self) -> str:
        return self._state

    def mount_covariance(self) -> np.ndarray:
        return self._mount_cov_rad2.copy()

    def _apply_measurement_update(
        self,
        quat_state_xyzw: np.ndarray,
        covariance: np.ndarray,
        quat_measurement_xyzw: np.ndarray,
        measurement_covariance: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        residual_quat: np.ndarray = self._quat_multiply(
            self._quat_conjugate(quat_state_xyzw),
            quat_measurement_xyzw,
        )
        residual_vec: np.ndarray = self._quat_log(residual_quat)

        s_mat: np.ndarray = covariance + measurement_covariance
        s_inv: np.ndarray = np.linalg.pinv(s_mat)

        kalman_gain: np.ndarray = covariance @ s_inv
        delta_vec: np.ndarray = kalman_gain @ residual_vec

        correction_quat: np.ndarray = self._quat_exp(delta_vec)
        updated_quat: np.ndarray = self._quat_multiply(quat_state_xyzw, correction_quat)
        updated_quat = self._normalize_quaternion(updated_quat)

        eye3: np.ndarray = np.eye(3, dtype=float)
        joseph_left: np.ndarray = eye3 - kalman_gain
        updated_cov: np.ndarray = (
            joseph_left @ covariance @ joseph_left.T
            + kalman_gain @ measurement_covariance @ kalman_gain.T
        )

        return updated_quat, self._sanitize_covariance(updated_cov)

    def _estimate_tilt(
        self,
        mount_quat_xyzw: np.ndarray,
        gravity_mps2: Optional[np.ndarray],
        imu_orientation_xyzw: np.ndarray,
    ) -> np.ndarray:
        if gravity_mps2 is not None:
            gravity_vec: np.ndarray = np.array(gravity_mps2, dtype=float)
            gravity_norm: float = float(np.linalg.norm(gravity_vec))

            if gravity_norm > 1.0e-6:
                gravity_body: np.ndarray = self._quat_rotate_vector(
                    mount_quat_xyzw,
                    gravity_vec,
                )
                roll_rad: float
                pitch_rad: float
                roll_rad, pitch_rad = self._gravity_to_roll_pitch(gravity_body)
                return np.array([roll_rad, pitch_rad, 0.0], dtype=float)

        q_ib: np.ndarray = self._quat_conjugate(mount_quat_xyzw)
        q_wb: np.ndarray = self._quat_multiply(imu_orientation_xyzw, q_ib)
        roll_rad_fallback: float
        pitch_rad_fallback: float
        yaw_rad_fallback: float
        roll_rad_fallback, pitch_rad_fallback, yaw_rad_fallback = self._quat_to_rpy(
            q_wb
        )

        return np.array([roll_rad_fallback, pitch_rad_fallback, 0.0], dtype=float)

    def _update_state(
        self,
        timestamp_s: float,
        accel_mps2: Optional[np.ndarray],
        gyro_rads: Optional[np.ndarray],
    ) -> None:
        trace_cov: float = float(np.trace(self._mount_cov_rad2))

        # Units: rad. Meaning: average 1-sigma angular uncertainty
        angle_std_rad: float = math.sqrt(max(trace_cov / 3.0, 0.0))

        converged: bool = angle_std_rad <= self._config.converged_stddev_rad
        lock_quality: bool = angle_std_rad <= self._config.lock_stddev_rad

        stationary: bool = self._is_stationary(accel_mps2, gyro_rads)

        if lock_quality and stationary:
            if self._lock_candidate_start_s is None:
                self._lock_candidate_start_s = timestamp_s
        else:
            self._lock_candidate_start_s = None

        can_lock: bool = False
        if self._lock_candidate_start_s is not None:
            can_lock = (
                timestamp_s - self._lock_candidate_start_s
            ) >= self._config.lock_min_duration_s

        if can_lock:
            self._state = STATE_LOCKED
            return

        if converged:
            self._state = STATE_CONVERGED
        else:
            self._state = STATE_LEARNING

    def _is_stationary(
        self,
        accel_mps2: Optional[np.ndarray],
        gyro_rads: Optional[np.ndarray],
    ) -> bool:
        accel_ok: bool = True
        if accel_mps2 is not None:
            accel_norm: float = float(np.linalg.norm(np.array(accel_mps2, dtype=float)))
            accel_ok = (
                abs(accel_norm - 9.80665) <= self._config.stationary_accel_delta_mps2
            )

        gyro_ok: bool = True
        if gyro_rads is not None:
            gyro_norm: float = float(np.linalg.norm(np.array(gyro_rads, dtype=float)))
            gyro_ok = gyro_norm <= self._config.stationary_gyro_norm_rads

        return accel_ok and gyro_ok

    def _yaw_weight_from_variance(self, yaw_variance_rad2: float) -> float:
        if not math.isfinite(yaw_variance_rad2) or yaw_variance_rad2 <= 0.0:
            return 0.0

        ref_var: float = max(self._config.heading_var_reference_rad2, 1.0e-12)

        # Unitless. Meaning: confidence-like scaling for yaw update gain
        weight: float = ref_var / (ref_var + yaw_variance_rad2)
        return float(min(max(weight, 0.0), 1.0))

    def _scale_yaw_covariance(
        self,
        covariance: np.ndarray,
        yaw_weight: float,
    ) -> np.ndarray:
        min_weight: float = 1.0e-3
        bounded_weight: float = min(max(yaw_weight, min_weight), 1.0)

        # Unitless. Meaning: inflation scale for yaw row/column
        yaw_scale: float = math.sqrt(1.0 / bounded_weight)

        scale_matrix: np.ndarray = np.diag([1.0, 1.0, yaw_scale])
        scaled_covariance: np.ndarray = scale_matrix @ covariance @ scale_matrix.T

        return self._sanitize_covariance(scaled_covariance)

    def _inflate_gravity_covariance(
        self,
        covariance: np.ndarray,
        accel_mps2: Optional[np.ndarray],
    ) -> np.ndarray:
        if accel_mps2 is None:
            return covariance

        accel_norm: float = float(np.linalg.norm(accel_mps2))
        accel_error: float = abs(accel_norm - 9.80665)

        # Unitless. Meaning: confidence inflation when non-gravity acceleration
        # contaminates tilt observations
        inflation: float = 1.0 + min((accel_error / 0.5) * (accel_error / 0.5), 40.0)

        inflated: np.ndarray = covariance.copy()
        inflated[0, 0] = float(inflated[0, 0] * inflation)
        inflated[1, 1] = float(inflated[1, 1] * inflation)
        return self._sanitize_covariance(inflated)

    def _sanitize_covariance(self, covariance: np.ndarray) -> np.ndarray:
        cov: np.ndarray = np.array(covariance, dtype=float).reshape(3, 3)
        cov = 0.5 * (cov + cov.T)

        diag_epsilon: float = 1.0e-12
        cov[0, 0] = max(float(cov[0, 0]), diag_epsilon)
        cov[1, 1] = max(float(cov[1, 1]), diag_epsilon)
        cov[2, 2] = max(float(cov[2, 2]), diag_epsilon)

        eigvals: np.ndarray
        eigvecs: np.ndarray
        eigvals, eigvecs = np.linalg.eigh(cov)
        eigvals = np.maximum(eigvals, diag_epsilon)
        cov_psd: np.ndarray = eigvecs @ np.diag(eigvals) @ eigvecs.T

        return 0.5 * (cov_psd + cov_psd.T)

    def _gravity_to_roll_pitch(self, gravity_vec: np.ndarray) -> tuple[float, float]:
        gx: float = float(gravity_vec[0])
        gy: float = float(gravity_vec[1])
        gz: float = float(gravity_vec[2])

        roll_rad: float = math.atan2(gy, gz)
        pitch_rad: float = math.atan2(-gx, math.sqrt(gy * gy + gz * gz))

        return roll_rad, pitch_rad

    def _normalize_quaternion(self, quat_xyzw: np.ndarray) -> np.ndarray:
        q: np.ndarray = np.array(quat_xyzw, dtype=float).reshape(4)
        norm_q: float = float(np.linalg.norm(q))
        if norm_q <= 0.0 or not math.isfinite(norm_q):
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        return q / norm_q

    def _quat_conjugate(self, quat_xyzw: np.ndarray) -> np.ndarray:
        x: float = float(quat_xyzw[0])
        y: float = float(quat_xyzw[1])
        z: float = float(quat_xyzw[2])
        w: float = float(quat_xyzw[3])

        return np.array([-x, -y, -z, w], dtype=float)

    def _quat_multiply(self, q1_xyzw: np.ndarray, q2_xyzw: np.ndarray) -> np.ndarray:
        x1: float = float(q1_xyzw[0])
        y1: float = float(q1_xyzw[1])
        z1: float = float(q1_xyzw[2])
        w1: float = float(q1_xyzw[3])

        x2: float = float(q2_xyzw[0])
        y2: float = float(q2_xyzw[1])
        z2: float = float(q2_xyzw[2])
        w2: float = float(q2_xyzw[3])

        x: float = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y: float = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z: float = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        w: float = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

        return np.array([x, y, z, w], dtype=float)

    def _quat_rotate_vector(
        self, quat_xyzw: np.ndarray, vector_xyz: np.ndarray
    ) -> np.ndarray:
        q: np.ndarray = self._normalize_quaternion(quat_xyzw)
        q_vec: np.ndarray = np.array([vector_xyz[0], vector_xyz[1], vector_xyz[2], 0.0])
        rotated: np.ndarray = self._quat_multiply(
            self._quat_multiply(q, q_vec),
            self._quat_conjugate(q),
        )
        return rotated[0:3]

    def _quat_from_two_vectors(
        self, from_vec: np.ndarray, to_vec: np.ndarray
    ) -> np.ndarray:
        from_norm: float = float(np.linalg.norm(from_vec))
        to_norm: float = float(np.linalg.norm(to_vec))

        if from_norm <= 1.0e-12 or to_norm <= 1.0e-12:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

        from_unit: np.ndarray = from_vec / from_norm
        to_unit: np.ndarray = to_vec / to_norm

        cross: np.ndarray = np.cross(from_unit, to_unit)
        dot_val: float = float(np.dot(from_unit, to_unit))

        if dot_val < -0.999999:
            axis: np.ndarray = np.cross(
                from_unit, np.array([1.0, 0.0, 0.0], dtype=float)
            )
            if float(np.linalg.norm(axis)) < 1.0e-6:
                axis = np.cross(from_unit, np.array([0.0, 1.0, 0.0], dtype=float))
            axis = axis / float(np.linalg.norm(axis))
            return np.array([axis[0], axis[1], axis[2], 0.0], dtype=float)

        quat_xyzw: np.ndarray = np.array(
            [cross[0], cross[1], cross[2], 1.0 + dot_val],
            dtype=float,
        )

        return self._normalize_quaternion(quat_xyzw)

    def _quat_log(self, quat_xyzw: np.ndarray) -> np.ndarray:
        q: np.ndarray = self._normalize_quaternion(quat_xyzw)

        q_vec: np.ndarray = q[0:3]
        q_w: float = float(q[3])

        vec_norm: float = float(np.linalg.norm(q_vec))
        if vec_norm <= 1.0e-12:
            return np.zeros(3, dtype=float)

        angle: float = 2.0 * math.atan2(vec_norm, q_w)
        axis: np.ndarray = q_vec / vec_norm

        return axis * angle

    def _quat_exp(self, rotation_vec_rad: np.ndarray) -> np.ndarray:
        angle: float = float(np.linalg.norm(rotation_vec_rad))

        if angle <= 1.0e-12:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

        axis: np.ndarray = rotation_vec_rad / angle
        half_angle: float = 0.5 * angle
        sin_half: float = math.sin(half_angle)
        cos_half: float = math.cos(half_angle)

        quat_xyzw: np.ndarray = np.array(
            [axis[0] * sin_half, axis[1] * sin_half, axis[2] * sin_half, cos_half],
            dtype=float,
        )

        return self._normalize_quaternion(quat_xyzw)

    def _quat_to_rpy(self, quat_xyzw: np.ndarray) -> tuple[float, float, float]:
        x: float = float(quat_xyzw[0])
        y: float = float(quat_xyzw[1])
        z: float = float(quat_xyzw[2])
        w: float = float(quat_xyzw[3])

        sinr_cosp: float = 2.0 * (w * x + y * z)
        cosr_cosp: float = 1.0 - 2.0 * (x * x + y * y)
        roll_rad: float = math.atan2(sinr_cosp, cosr_cosp)

        sinp: float = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch_rad: float = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch_rad = math.asin(sinp)

        siny_cosp: float = 2.0 * (w * z + x * y)
        cosy_cosp: float = 1.0 - 2.0 * (y * y + z * z)
        yaw_rad: float = math.atan2(siny_cosp, cosy_cosp)

        return roll_rad, pitch_rad, yaw_rad
