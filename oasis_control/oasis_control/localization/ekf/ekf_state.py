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
State container and indexing for EKF localization
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from typing import Optional

import numpy as np

from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_types import ImuCalibrationData
from oasis_control.localization.ekf.se3 import pose_plus
from oasis_control.localization.ekf.se3 import quat_to_rpy


# Default accelerometer bias sigma in m/s^2
_DEFAULT_ACCEL_BIAS_SIGMA_MPS2: float = 1.0

# Default accelerometer scale/misalignment sigma, unitless
_DEFAULT_ACCEL_A_SIGMA: float = 1.0

# Default gyroscope bias sigma in rad/s
_DEFAULT_GYRO_BIAS_SIGMA_RPS: float = 0.1


@dataclass(frozen=True)
class TagKey:
    """
    Immutable key identifying a tag landmark

    Fields:
        family: AprilTag family name such as tag36h11
        tag_id: Identifier within the family
    """

    family: str
    tag_id: int


@dataclass
class Pose3:
    """
    Pose with translation and quaternion rotation

    Fields:
        translation_m: Translation in meters, XYZ order
        rotation_wxyz: Quaternion in wxyz order, unit length
    """

    translation_m: np.ndarray
    rotation_wxyz: np.ndarray

    def copy(self) -> Pose3:
        return Pose3(self.translation_m.copy(), self.rotation_wxyz.copy())


@dataclass
class EkfStateIndex:
    """
    Slice bookkeeping for packed EKF error-state vectors

    Fields:
        pose: Pose error slice (delta translation, delta rotation)
        velocity: Velocity error slice
        accel_bias: Accelerometer bias error slice
        accel_a: Accelerometer scale/misalignment error slice
        gyro_bias: Gyroscope bias error slice
        extrinsic_bi: Body-to-IMU extrinsic error slice
        extrinsic_bm: Body-to-mag extrinsic error slice
        extrinsic_bc: Body-to-camera extrinsic error slice
        landmarks: Tag landmark slices in insertion order
        total_dim: Total packed error-state dimension
    """

    pose: slice
    velocity: slice
    accel_bias: slice
    accel_a: slice
    gyro_bias: slice
    extrinsic_bi: slice
    extrinsic_bm: slice
    extrinsic_bc: slice
    landmarks: dict[TagKey, slice]
    total_dim: int

    def copy(self) -> EkfStateIndex:
        return EkfStateIndex(
            pose=self.pose,
            velocity=self.velocity,
            accel_bias=self.accel_bias,
            accel_a=self.accel_a,
            gyro_bias=self.gyro_bias,
            extrinsic_bi=self.extrinsic_bi,
            extrinsic_bm=self.extrinsic_bm,
            extrinsic_bc=self.extrinsic_bc,
            landmarks=dict(self.landmarks),
            total_dim=self.total_dim,
        )


class EkfState:
    """
    Container for nominal state and packed error-state covariance

    The pose and velocity are expressed in the local odometry frame
    """

    def __init__(self, config: EkfConfig) -> None:
        self._config: EkfConfig = config
        self.pose_ob: Pose3 = self._identity_pose()
        self.vel_o_mps: np.ndarray = np.zeros(3, dtype=float)
        self.imu_bias_accel_mps2: np.ndarray = np.zeros(3, dtype=float)
        self.imu_accel_a: np.ndarray = np.eye(3, dtype=float)
        self.imu_bias_gyro_rps: np.ndarray = np.zeros(3, dtype=float)
        self.extrinsic_bi: Pose3 = self._identity_pose()
        self.extrinsic_bm: Pose3 = self._identity_pose()
        self.extrinsic_bc: Pose3 = self._identity_pose()
        self._landmarks: OrderedDict[TagKey, Pose3] = OrderedDict()
        anchor_key: TagKey = TagKey(
            family=config.tag_anchor_family, tag_id=config.tag_anchor_id
        )
        self._landmarks[anchor_key] = self._identity_pose()
        self._index: EkfStateIndex = self._build_index(self._landmarks)
        self._p: np.ndarray = self._build_initial_covariance()
        self._error_state: np.ndarray = np.zeros(self._index.total_dim, dtype=float)

    @property
    def covariance(self) -> np.ndarray:
        return self._p

    @covariance.setter
    def covariance(self, value: np.ndarray) -> None:
        self._p = value

    @property
    def error_state(self) -> np.ndarray:
        return self._error_state

    @property
    def index(self) -> EkfStateIndex:
        return self._index

    def pack_error_state(self) -> np.ndarray:
        return self._error_state.copy()

    def unpack_error_state(self, delta: np.ndarray) -> None:
        self._error_state = np.asarray(delta, dtype=float).reshape(
            self._index.total_dim
        )

    def copy(self) -> EkfState:
        new_state: EkfState = EkfState.__new__(EkfState)
        new_state._config = self._config
        new_state.pose_ob = self.pose_ob.copy()
        new_state.vel_o_mps = self.vel_o_mps.copy()
        new_state.imu_bias_accel_mps2 = self.imu_bias_accel_mps2.copy()
        new_state.imu_accel_a = self.imu_accel_a.copy()
        new_state.imu_bias_gyro_rps = self.imu_bias_gyro_rps.copy()
        new_state.extrinsic_bi = self.extrinsic_bi.copy()
        new_state.extrinsic_bm = self.extrinsic_bm.copy()
        new_state.extrinsic_bc = self.extrinsic_bc.copy()
        new_state._landmarks = OrderedDict(
            (key, pose.copy()) for key, pose in self._landmarks.items()
        )
        new_state._index = self._index.copy()
        new_state._p = self._p.copy()
        new_state._error_state = self._error_state.copy()
        return new_state

    def reset(self) -> None:
        self.pose_ob = self._identity_pose()
        self.vel_o_mps = np.zeros(3, dtype=float)
        self.imu_bias_accel_mps2 = np.zeros(3, dtype=float)
        self.imu_accel_a = np.eye(3, dtype=float)
        self.imu_bias_gyro_rps = np.zeros(3, dtype=float)
        self.extrinsic_bi = self._identity_pose()
        self.extrinsic_bm = self._identity_pose()
        self.extrinsic_bc = self._identity_pose()
        self._landmarks = OrderedDict()
        anchor_key: TagKey = TagKey(
            family=self._config.tag_anchor_family,
            tag_id=self._config.tag_anchor_id,
        )
        self._landmarks[anchor_key] = self._identity_pose()
        self._index = self._build_index(self._landmarks)
        self._p = self._build_initial_covariance()
        self._error_state = np.zeros(self._index.total_dim, dtype=float)

    def apply_delta(self, delta: np.ndarray) -> None:
        delta_vec: np.ndarray = np.asarray(delta, dtype=float).reshape(
            self._index.total_dim
        )
        pose_delta: np.ndarray = delta_vec[self._index.pose]
        self.pose_ob.translation_m, self.pose_ob.rotation_wxyz = pose_plus(
            self.pose_ob.translation_m,
            self.pose_ob.rotation_wxyz,
            pose_delta,
        )
        self.vel_o_mps = self.vel_o_mps + delta_vec[self._index.velocity]
        self.imu_bias_accel_mps2 = (
            self.imu_bias_accel_mps2 + delta_vec[self._index.accel_bias]
        )
        accel_a_delta: np.ndarray = delta_vec[self._index.accel_a].reshape(3, 3)
        self.imu_accel_a = self.imu_accel_a + accel_a_delta
        self.imu_bias_gyro_rps = (
            self.imu_bias_gyro_rps + delta_vec[self._index.gyro_bias]
        )
        self.extrinsic_bi.translation_m, self.extrinsic_bi.rotation_wxyz = pose_plus(
            self.extrinsic_bi.translation_m,
            self.extrinsic_bi.rotation_wxyz,
            delta_vec[self._index.extrinsic_bi],
        )
        self.extrinsic_bm.translation_m, self.extrinsic_bm.rotation_wxyz = pose_plus(
            self.extrinsic_bm.translation_m,
            self.extrinsic_bm.rotation_wxyz,
            delta_vec[self._index.extrinsic_bm],
        )
        self.extrinsic_bc.translation_m, self.extrinsic_bc.rotation_wxyz = pose_plus(
            self.extrinsic_bc.translation_m,
            self.extrinsic_bc.rotation_wxyz,
            delta_vec[self._index.extrinsic_bc],
        )
        for tag_key, tag_slice in self._index.landmarks.items():
            pose: Pose3 = self._landmarks[tag_key]
            pose.translation_m, pose.rotation_wxyz = pose_plus(
                pose.translation_m,
                pose.rotation_wxyz,
                delta_vec[tag_slice],
            )

        self._error_state = np.zeros(self._index.total_dim, dtype=float)

    def apply_imu_calibration(self, calibration: ImuCalibrationData) -> None:
        accel_bias: np.ndarray = np.asarray(
            calibration.accel_bias_mps2, dtype=float
        ).reshape(3)
        accel_a: np.ndarray = np.asarray(calibration.accel_a, dtype=float).reshape(3, 3)
        gyro_bias: np.ndarray = np.asarray(
            calibration.gyro_bias_rps, dtype=float
        ).reshape(3)
        self.imu_bias_accel_mps2 = accel_bias
        self.imu_accel_a = accel_a
        self.imu_bias_gyro_rps = gyro_bias

        accel_param_cov: np.ndarray = np.asarray(
            calibration.accel_param_cov, dtype=float
        ).reshape(12, 12)
        gyro_bias_cov: np.ndarray = np.asarray(
            calibration.gyro_bias_cov, dtype=float
        ).reshape(3, 3)

        accel_indices: list[int] = list(
            range(self._index.accel_bias.start, self._index.accel_bias.stop)
        ) + list(range(self._index.accel_a.start, self._index.accel_a.stop))
        self._p[np.ix_(accel_indices, accel_indices)] = accel_param_cov
        self._p[self._index.gyro_bias, self._index.gyro_bias] = gyro_bias_cov

    def ensure_landmark(self, tag_key: TagKey) -> bool:
        if tag_key in self._landmarks:
            return False
        self._landmarks[tag_key] = self._identity_pose()
        old_dim: int = self._index.total_dim
        self._index = self._build_index(self._landmarks)
        new_dim: int = self._index.total_dim
        new_cov: np.ndarray = np.zeros((new_dim, new_dim), dtype=float)
        new_cov[:old_dim, :old_dim] = self._p
        prior: np.ndarray = self._landmark_prior_covariance()
        new_cov[old_dim:new_dim, old_dim:new_dim] = prior
        self._p = new_cov
        new_error_state: np.ndarray = np.zeros(new_dim, dtype=float)
        new_error_state[:old_dim] = self._error_state
        self._error_state = new_error_state
        return True

    def landmark_pose(self, tag_key: TagKey) -> Optional[Pose3]:
        pose: Optional[Pose3] = self._landmarks.get(tag_key)
        if pose is None:
            return None
        return pose.copy()

    def set_landmark_pose(self, tag_key: TagKey, pose: Pose3) -> None:
        if tag_key not in self._landmarks:
            raise KeyError(f"Unknown landmark {tag_key}")
        self._landmarks[tag_key] = pose.copy()

    def landmark_slice(self, tag_key: TagKey) -> Optional[slice]:
        return self._index.landmarks.get(tag_key)

    def legacy_state(self) -> np.ndarray:
        rpy: np.ndarray = quat_to_rpy(self.pose_ob.rotation_wxyz)
        return np.concatenate(
            (
                self.pose_ob.translation_m,
                self.vel_o_mps,
                rpy,
            ),
            axis=0,
        )

    def lift_legacy_jacobian(self, legacy_h: np.ndarray) -> np.ndarray:
        if legacy_h.shape[1] != 9:
            raise ValueError("Expected legacy Jacobian with 9 columns")
        total_dim: int = self._index.total_dim
        lifted: np.ndarray = np.zeros((legacy_h.shape[0], total_dim), dtype=float)
        pose_start: int = self._index.pose.start
        rot_start: int = pose_start + 3
        lifted[:, pose_start : pose_start + 3] = legacy_h[:, 0:3]
        lifted[:, self._index.velocity] = legacy_h[:, 3:6]
        lifted[:, rot_start : rot_start + 3] = legacy_h[:, 6:9]
        return lifted

    def _identity_pose(self) -> Pose3:
        return Pose3(
            translation_m=np.zeros(3, dtype=float),
            rotation_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        )

    def _build_index(self, landmarks: OrderedDict[TagKey, Pose3]) -> EkfStateIndex:
        cursor: int = 0
        pose_slice: slice = slice(cursor, cursor + 6)
        cursor += 6
        velocity_slice: slice = slice(cursor, cursor + 3)
        cursor += 3
        accel_bias_slice: slice = slice(cursor, cursor + 3)
        cursor += 3
        accel_a_slice: slice = slice(cursor, cursor + 9)
        cursor += 9
        gyro_bias_slice: slice = slice(cursor, cursor + 3)
        cursor += 3
        extrinsic_bi_slice: slice = slice(cursor, cursor + 6)
        cursor += 6
        extrinsic_bm_slice: slice = slice(cursor, cursor + 6)
        cursor += 6
        extrinsic_bc_slice: slice = slice(cursor, cursor + 6)
        cursor += 6
        landmark_slices: dict[TagKey, slice] = {}
        for key in landmarks:
            landmark_slices[key] = slice(cursor, cursor + 6)
            cursor += 6
        return EkfStateIndex(
            pose=pose_slice,
            velocity=velocity_slice,
            accel_bias=accel_bias_slice,
            accel_a=accel_a_slice,
            gyro_bias=gyro_bias_slice,
            extrinsic_bi=extrinsic_bi_slice,
            extrinsic_bm=extrinsic_bm_slice,
            extrinsic_bc=extrinsic_bc_slice,
            landmarks=landmark_slices,
            total_dim=cursor,
        )

    def _build_initial_covariance(self) -> np.ndarray:
        total_dim: int = self._index.total_dim
        covariance: np.ndarray = np.zeros((total_dim, total_dim), dtype=float)
        covariance[self._index.pose.start, self._index.pose.start] = (
            self._config.pos_var
        )
        covariance[self._index.pose.start + 1, self._index.pose.start + 1] = (
            self._config.pos_var
        )
        covariance[self._index.pose.start + 2, self._index.pose.start + 2] = (
            self._config.pos_var
        )
        covariance[self._index.pose.start + 3, self._index.pose.start + 3] = (
            self._config.ang_var
        )
        covariance[self._index.pose.start + 4, self._index.pose.start + 4] = (
            self._config.ang_var
        )
        covariance[self._index.pose.start + 5, self._index.pose.start + 5] = (
            self._config.ang_var
        )
        covariance[self._index.velocity, self._index.velocity] = (
            np.eye(3, dtype=float) * self._config.vel_var
        )
        covariance[self._index.accel_bias, self._index.accel_bias] = np.eye(
            3, dtype=float
        ) * (_DEFAULT_ACCEL_BIAS_SIGMA_MPS2**2)
        covariance[self._index.accel_a, self._index.accel_a] = np.eye(
            9, dtype=float
        ) * (_DEFAULT_ACCEL_A_SIGMA**2)
        covariance[self._index.gyro_bias, self._index.gyro_bias] = np.eye(
            3, dtype=float
        ) * (_DEFAULT_GYRO_BIAS_SIGMA_RPS**2)
        extrinsic_prior: np.ndarray = self._extrinsic_prior_covariance()
        covariance[self._index.extrinsic_bi, self._index.extrinsic_bi] = extrinsic_prior
        covariance[self._index.extrinsic_bm, self._index.extrinsic_bm] = extrinsic_prior
        covariance[self._index.extrinsic_bc, self._index.extrinsic_bc] = extrinsic_prior
        for tag_key, tag_slice in self._index.landmarks.items():
            _ = tag_key
            covariance[tag_slice, tag_slice] = self._landmark_prior_covariance()
        return covariance

    def _extrinsic_prior_covariance(self) -> np.ndarray:
        sigma_t: float = self._config.extrinsic_prior_sigma_t_m
        sigma_rot: float = self._config.extrinsic_prior_sigma_rot_rad
        return self._pose_prior_covariance(sigma_t, sigma_rot)

    def _landmark_prior_covariance(self) -> np.ndarray:
        sigma_t: float = self._config.tag_landmark_prior_sigma_t_m
        sigma_rot: float = self._config.tag_landmark_prior_sigma_rot_rad
        return self._pose_prior_covariance(sigma_t, sigma_rot)

    def _pose_prior_covariance(self, sigma_t: float, sigma_rot: float) -> np.ndarray:
        cov: np.ndarray = np.zeros((6, 6), dtype=float)
        cov[0:3, 0:3] = np.eye(3, dtype=float) * (sigma_t**2)
        cov[3:6, 3:6] = np.eye(3, dtype=float) * (sigma_rot**2)
        return cov
