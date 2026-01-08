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
EKF state container with nominal pose and tangent-space covariance
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict
from typing import List
from typing import Tuple

import numpy as np

from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_types import ImuCalibrationData
from oasis_control.localization.ekf.se3 import pose_plus
from oasis_control.localization.ekf.se3 import quat_normalize
from oasis_control.localization.ekf.se3 import so3_log


_BASE_STATE_DIM: int = 9
_IMU_PARAM_DIM: int = 15
_EXTRINSIC_DIM: int = 18
_LANDMARK_DIM: int = 6

# Default accelerometer bias sigma in m/s^2
_DEFAULT_ACCEL_BIAS_SIGMA_MPS2: float = 1.0

# Default accelerometer scale sigma, unitless
_DEFAULT_ACCEL_SCALE_SIGMA: float = 0.1

# Default gyroscope bias sigma in rad/s
_DEFAULT_GYRO_BIAS_SIGMA_RPS: float = 0.1


@dataclass(frozen=True)
class TagKey:
    """
    AprilTag identifier used for landmark keys

    Fields:
        family: AprilTag family name such as tag36h11
        tag_id: AprilTag identifier within the family
    """

    family: str
    tag_id: int


@dataclass
class PoseState:
    """
    Pose stored as translation and quaternion

    Fields:
        translation_m: Translation in meters, XYZ order
        rotation_wxyz: Unit quaternion in [w, x, y, z] order
    """

    translation_m: np.ndarray
    rotation_wxyz: np.ndarray

    def copy(self) -> PoseState:
        translation_m: np.ndarray = np.array(self.translation_m, dtype=float)
        rotation_wxyz: np.ndarray = np.array(self.rotation_wxyz, dtype=float)
        return PoseState(translation_m=translation_m, rotation_wxyz=rotation_wxyz)

    @staticmethod
    def identity() -> PoseState:
        translation_m: np.ndarray = np.zeros(3, dtype=float)
        rotation_wxyz: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        return PoseState(translation_m=translation_m, rotation_wxyz=rotation_wxyz)


@dataclass
class ImuCalibrationParams:
    """
    In-state IMU calibration parameters

    Fields:
        accel_bias_mps2: Accelerometer bias in m/s^2, XYZ order
        accel_scale: Accelerometer scale/misalignment 3x3 matrix, row-major
        gyro_bias_rps: Gyroscope bias in rad/s, XYZ order
    """

    accel_bias_mps2: np.ndarray
    accel_scale: np.ndarray
    gyro_bias_rps: np.ndarray

    def copy(self) -> ImuCalibrationParams:
        accel_bias_mps2: np.ndarray = np.array(self.accel_bias_mps2, dtype=float)
        accel_scale: np.ndarray = np.array(self.accel_scale, dtype=float)
        gyro_bias_rps: np.ndarray = np.array(self.gyro_bias_rps, dtype=float)
        return ImuCalibrationParams(
            accel_bias_mps2=accel_bias_mps2,
            accel_scale=accel_scale,
            gyro_bias_rps=gyro_bias_rps,
        )

    @staticmethod
    def zeros() -> ImuCalibrationParams:
        accel_bias_mps2: np.ndarray = np.zeros(3, dtype=float)
        accel_scale: np.ndarray = np.eye(3, dtype=float)
        gyro_bias_rps: np.ndarray = np.zeros(3, dtype=float)
        return ImuCalibrationParams(
            accel_bias_mps2=accel_bias_mps2,
            accel_scale=accel_scale,
            gyro_bias_rps=gyro_bias_rps,
        )


@dataclass
class ExtrinsicsState:
    """
    In-state extrinsic calibration poses

    Fields:
        t_bi: Pose from body to IMU frame
        t_bm: Pose from body to magnetometer frame
        t_bc: Pose from body to camera frame
    """

    t_bi: PoseState
    t_bm: PoseState
    t_bc: PoseState

    def copy(self) -> ExtrinsicsState:
        return ExtrinsicsState(
            t_bi=self.t_bi.copy(),
            t_bm=self.t_bm.copy(),
            t_bc=self.t_bc.copy(),
        )

    @staticmethod
    def identity() -> ExtrinsicsState:
        return ExtrinsicsState(
            t_bi=PoseState.identity(),
            t_bm=PoseState.identity(),
            t_bc=PoseState.identity(),
        )


@dataclass(frozen=True)
class _StateIndices:
    """
    Slice bookkeeping for the packed error-state vector

    Fields:
        pos: Position error slice in meters, XYZ order
        vel: Velocity error slice in m/s, XYZ order
        rot: Rotation error slice in radians, XYZ order
        imu: IMU parameter error slice
        accel_bias: Accel bias error slice in m/s^2, XYZ order
        accel_scale: Accel scale error slice in row-major 3x3 order
        gyro_bias: Gyro bias error slice in rad/s, XYZ order
        extrinsics: Extrinsic pose error slice
        extrinsic_bi: Body-to-IMU extrinsic error slice
        extrinsic_bm: Body-to-mag extrinsic error slice
        extrinsic_bc: Body-to-camera extrinsic error slice
        landmark_start: Starting index for landmark error blocks
    """

    pos: slice
    vel: slice
    rot: slice
    imu: slice
    accel_bias: slice
    accel_scale: slice
    gyro_bias: slice
    extrinsics: slice
    extrinsic_bi: slice
    extrinsic_bm: slice
    extrinsic_bc: slice
    landmark_start: int


class EkfState:
    """
    EKF state with nominal values, error-state vector, and covariance
    """

    def __init__(self, config: EkfConfig) -> None:
        self._config: EkfConfig = config
        self._pose_wb: PoseState = PoseState.identity()
        self._velocity_w_mps: np.ndarray = np.zeros(3, dtype=float)
        self._imu_params: ImuCalibrationParams = ImuCalibrationParams.zeros()
        self._extrinsics: ExtrinsicsState = ExtrinsicsState.identity()
        self._landmarks: Dict[TagKey, PoseState] = {}
        self._landmark_order: List[TagKey] = []
        self._landmark_index: Dict[TagKey, int] = {}

        indices: _StateIndices = self._build_indices()
        self._indices: _StateIndices = indices

        base_dim: int = indices.landmark_start
        self._delta_x: np.ndarray = np.zeros(base_dim, dtype=float)
        self._p: np.ndarray = np.zeros((base_dim, base_dim), dtype=float)
        self._initialize_priors(config)

        anchor_key: TagKey = TagKey(
            family=config.tag_anchor_family, tag_id=config.tag_anchor_id
        )
        self.ensure_landmark(
            anchor_key,
            PoseState.identity(),
            config.tag_landmark_prior_sigma_t_m,
            config.tag_landmark_prior_sigma_rot_rad,
        )

    @property
    def pose_wb(self) -> PoseState:
        return self._pose_wb

    @property
    def velocity_w_mps(self) -> np.ndarray:
        return self._velocity_w_mps

    @velocity_w_mps.setter
    def velocity_w_mps(self, value: np.ndarray) -> None:
        self._velocity_w_mps = value

    @property
    def imu_params(self) -> ImuCalibrationParams:
        return self._imu_params

    @property
    def extrinsics(self) -> ExtrinsicsState:
        return self._extrinsics

    @property
    def covariance(self) -> np.ndarray:
        return self._p

    @covariance.setter
    def covariance(self, value: np.ndarray) -> None:
        self._p = value

    @property
    def error_state(self) -> np.ndarray:
        return self._delta_x

    @property
    def indices(self) -> _StateIndices:
        return self._indices

    @property
    def error_state_dim(self) -> int:
        return int(self._delta_x.shape[0])

    def reset_with_priors(self, config: EkfConfig) -> None:
        self._config = config
        self._pose_wb = PoseState.identity()
        self._velocity_w_mps = np.zeros(3, dtype=float)
        self._imu_params = ImuCalibrationParams.zeros()
        self._extrinsics = ExtrinsicsState.identity()
        self._landmarks = {}
        self._landmark_order = []
        self._landmark_index = {}

        indices: _StateIndices = self._build_indices()
        self._indices = indices
        base_dim: int = indices.landmark_start
        self._delta_x = np.zeros(base_dim, dtype=float)
        self._p = np.zeros((base_dim, base_dim), dtype=float)
        self._initialize_priors(config)

        anchor_key: TagKey = TagKey(
            family=config.tag_anchor_family, tag_id=config.tag_anchor_id
        )
        self.ensure_landmark(
            anchor_key,
            PoseState.identity(),
            config.tag_landmark_prior_sigma_t_m,
            config.tag_landmark_prior_sigma_rot_rad,
        )

    def copy(self) -> EkfState:
        clone: EkfState = EkfState(self._config)
        clone._pose_wb = self._pose_wb.copy()
        clone._velocity_w_mps = np.array(self._velocity_w_mps, dtype=float)
        clone._imu_params = self._imu_params.copy()
        clone._extrinsics = self._extrinsics.copy()
        clone._landmarks = {key: pose.copy() for key, pose in self._landmarks.items()}
        clone._landmark_order = list(self._landmark_order)
        clone._landmark_index = dict(self._landmark_index)
        clone._indices = self._indices
        clone._delta_x = np.array(self._delta_x, dtype=float)
        clone._p = np.array(self._p, dtype=float)
        return clone

    def pack_nominal(self) -> np.ndarray:
        pos_vector: np.ndarray = np.array(self._pose_wb.translation_m, dtype=float)
        rot_vector: np.ndarray = so3_log(self._pose_wb.rotation_wxyz)
        vel_vector: np.ndarray = np.array(self._velocity_w_mps, dtype=float)
        imu_vector: np.ndarray = self._imu_to_vector(self._imu_params)
        extrinsic_vector: np.ndarray = self._extrinsics_to_vector(self._extrinsics)
        landmark_vector: np.ndarray = self._landmarks_to_vector()
        return np.concatenate(
            [
                pos_vector,
                vel_vector,
                rot_vector,
                imu_vector,
                extrinsic_vector,
                landmark_vector,
            ]
        ).astype(float)

    def apply_error_state(self, delta_x: np.ndarray) -> None:
        if delta_x.shape[0] != self.error_state_dim:
            raise ValueError("delta_x size does not match error state")

        indices: _StateIndices = self._indices
        pos_delta: np.ndarray = delta_x[indices.pos]
        vel_delta: np.ndarray = delta_x[indices.vel]
        rot_delta: np.ndarray = delta_x[indices.rot]

        updated_translation: np.ndarray
        updated_rotation: np.ndarray
        updated_translation, updated_rotation = pose_plus(
            self._pose_wb.translation_m,
            self._pose_wb.rotation_wxyz,
            np.concatenate([pos_delta, rot_delta]),
        )
        self._pose_wb.translation_m = updated_translation
        self._pose_wb.rotation_wxyz = quat_normalize(updated_rotation)
        self._velocity_w_mps = self._velocity_w_mps + vel_delta

        accel_bias_delta: np.ndarray = delta_x[indices.accel_bias]
        accel_scale_delta: np.ndarray = delta_x[indices.accel_scale]
        gyro_bias_delta: np.ndarray = delta_x[indices.gyro_bias]

        self._imu_params.accel_bias_mps2 = (
            self._imu_params.accel_bias_mps2 + accel_bias_delta
        )
        self._imu_params.accel_scale = (
            self._imu_params.accel_scale + accel_scale_delta.reshape(3, 3)
        )
        self._imu_params.gyro_bias_rps = self._imu_params.gyro_bias_rps + gyro_bias_delta

        extrinsic_delta: np.ndarray = delta_x[indices.extrinsics]
        self._extrinsics = self._apply_extrinsic_delta(self._extrinsics, extrinsic_delta)

        if delta_x.shape[0] > indices.landmark_start:
            self._apply_landmark_delta(delta_x[indices.landmark_start :])

        self._delta_x = np.zeros(self.error_state_dim, dtype=float)

    def set_imu_calibration(self, calibration: ImuCalibrationData) -> None:
        accel_bias_mps2: np.ndarray = np.asarray(
            calibration.accel_bias_mps2, dtype=float
        )
        accel_scale: np.ndarray = np.asarray(calibration.accel_a, dtype=float).reshape(
            3, 3
        )
        gyro_bias_rps: np.ndarray = np.asarray(
            calibration.gyro_bias_rps, dtype=float
        )

        self._imu_params.accel_bias_mps2 = accel_bias_mps2
        self._imu_params.accel_scale = accel_scale
        self._imu_params.gyro_bias_rps = gyro_bias_rps

        accel_cov: np.ndarray = np.asarray(
            calibration.accel_param_cov, dtype=float
        ).reshape(12, 12)
        gyro_cov: np.ndarray = np.asarray(
            calibration.gyro_bias_cov, dtype=float
        ).reshape(3, 3)

        indices: _StateIndices = self._indices
        self._p[indices.imu, indices.imu] = 0.0
        self._p[indices.accel_bias, indices.accel_bias] = accel_cov[0:3, 0:3]
        self._p[indices.accel_bias, indices.accel_scale] = accel_cov[0:3, 3:12]
        self._p[indices.accel_scale, indices.accel_bias] = accel_cov[3:12, 0:3]
        self._p[indices.accel_scale, indices.accel_scale] = accel_cov[3:12, 3:12]
        self._p[indices.gyro_bias, indices.gyro_bias] = gyro_cov

    def ensure_landmark(
        self,
        key: TagKey,
        pose: PoseState,
        sigma_t_m: float,
        sigma_rot_rad: float,
    ) -> Tuple[bool, slice]:
        if key in self._landmark_index:
            index: int = self._landmark_index[key]
            start: int = self._indices.landmark_start + index * _LANDMARK_DIM
            return False, slice(start, start + _LANDMARK_DIM)

        self._landmarks[key] = pose
        self._landmark_order.append(key)
        index: int = len(self._landmark_order) - 1
        self._landmark_index[key] = index

        prior_block: np.ndarray = self._pose_prior_block(sigma_t_m, sigma_rot_rad)
        self._expand_covariance(prior_block)
        start: int = self._indices.landmark_start + index * _LANDMARK_DIM

        return True, slice(start, start + _LANDMARK_DIM)

    def landmark_pose(self, key: TagKey) -> PoseState:
        return self._landmarks[key]

    def landmark_order(self) -> List[TagKey]:
        return list(self._landmark_order)

    def _initialize_priors(self, config: EkfConfig) -> None:
        indices: _StateIndices = self._indices
        pos_var: float = config.pos_var
        vel_var: float = config.vel_var
        ang_var: float = config.ang_var

        pos_diag: np.ndarray = np.array([pos_var, pos_var, pos_var], dtype=float)
        vel_diag: np.ndarray = np.array([vel_var, vel_var, vel_var], dtype=float)
        ang_diag: np.ndarray = np.array([ang_var, ang_var, ang_var], dtype=float)

        self._p[indices.pos, indices.pos] = np.diag(pos_diag)
        self._p[indices.vel, indices.vel] = np.diag(vel_diag)
        self._p[indices.rot, indices.rot] = np.diag(ang_diag)

        accel_bias_sigma: float = _DEFAULT_ACCEL_BIAS_SIGMA_MPS2
        accel_scale_sigma: float = _DEFAULT_ACCEL_SCALE_SIGMA
        gyro_bias_sigma: float = _DEFAULT_GYRO_BIAS_SIGMA_RPS

        accel_bias_cov: np.ndarray = np.diag(
            np.array([accel_bias_sigma] * 3, dtype=float) ** 2
        )
        accel_scale_cov: np.ndarray = np.diag(
            np.array([accel_scale_sigma] * 9, dtype=float) ** 2
        )
        gyro_bias_cov: np.ndarray = np.diag(
            np.array([gyro_bias_sigma] * 3, dtype=float) ** 2
        )

        self._p[indices.accel_bias, indices.accel_bias] = accel_bias_cov
        self._p[indices.accel_scale, indices.accel_scale] = accel_scale_cov
        self._p[indices.gyro_bias, indices.gyro_bias] = gyro_bias_cov

        extrinsic_sigma_t_m: float = config.extrinsic_prior_sigma_t_m
        extrinsic_sigma_rot_rad: float = config.extrinsic_prior_sigma_rot_rad
        extrinsic_cov: np.ndarray = self._pose_prior_block(
            extrinsic_sigma_t_m, extrinsic_sigma_rot_rad
        )

        self._p[indices.extrinsic_bi, indices.extrinsic_bi] = extrinsic_cov
        self._p[indices.extrinsic_bm, indices.extrinsic_bm] = extrinsic_cov
        self._p[indices.extrinsic_bc, indices.extrinsic_bc] = extrinsic_cov

    def _build_indices(self) -> _StateIndices:
        pos_slice: slice = slice(0, 3)
        vel_slice: slice = slice(3, 6)
        rot_slice: slice = slice(6, 9)

        imu_start: int = _BASE_STATE_DIM
        imu_end: int = imu_start + _IMU_PARAM_DIM
        accel_bias_slice: slice = slice(imu_start, imu_start + 3)
        accel_scale_slice: slice = slice(imu_start + 3, imu_start + 12)
        gyro_bias_slice: slice = slice(imu_start + 12, imu_end)
        imu_slice: slice = slice(imu_start, imu_end)

        extrinsic_start: int = imu_end
        extrinsic_end: int = extrinsic_start + _EXTRINSIC_DIM
        extrinsic_bi_slice: slice = slice(extrinsic_start, extrinsic_start + 6)
        extrinsic_bm_slice: slice = slice(extrinsic_start + 6, extrinsic_start + 12)
        extrinsic_bc_slice: slice = slice(extrinsic_start + 12, extrinsic_end)
        extrinsics_slice: slice = slice(extrinsic_start, extrinsic_end)

        landmark_start: int = extrinsic_end

        return _StateIndices(
            pos=pos_slice,
            vel=vel_slice,
            rot=rot_slice,
            imu=imu_slice,
            accel_bias=accel_bias_slice,
            accel_scale=accel_scale_slice,
            gyro_bias=gyro_bias_slice,
            extrinsics=extrinsics_slice,
            extrinsic_bi=extrinsic_bi_slice,
            extrinsic_bm=extrinsic_bm_slice,
            extrinsic_bc=extrinsic_bc_slice,
            landmark_start=landmark_start,
        )

    def _pose_to_vector(self, pose: PoseState) -> np.ndarray:
        rot_vec: np.ndarray = so3_log(pose.rotation_wxyz)
        return np.concatenate([pose.translation_m, rot_vec]).astype(float)

    def _imu_to_vector(self, imu_params: ImuCalibrationParams) -> np.ndarray:
        accel_bias: np.ndarray = np.array(imu_params.accel_bias_mps2, dtype=float)
        accel_scale: np.ndarray = np.array(imu_params.accel_scale, dtype=float).reshape(
            -1
        )
        gyro_bias: np.ndarray = np.array(imu_params.gyro_bias_rps, dtype=float)
        return np.concatenate([accel_bias, accel_scale, gyro_bias]).astype(float)

    def _extrinsics_to_vector(self, extrinsics: ExtrinsicsState) -> np.ndarray:
        t_bi: np.ndarray = self._pose_to_vector(extrinsics.t_bi)
        t_bm: np.ndarray = self._pose_to_vector(extrinsics.t_bm)
        t_bc: np.ndarray = self._pose_to_vector(extrinsics.t_bc)
        return np.concatenate([t_bi, t_bm, t_bc]).astype(float)

    def _landmarks_to_vector(self) -> np.ndarray:
        vectors: List[np.ndarray] = []
        for key in self._landmark_order:
            pose: PoseState = self._landmarks[key]
            vectors.append(self._pose_to_vector(pose))
        if not vectors:
            return np.zeros(0, dtype=float)
        return np.concatenate(vectors).astype(float)

    def _expand_covariance(self, block: np.ndarray) -> None:
        old_dim: int = int(self._p.shape[0])
        new_dim: int = old_dim + _LANDMARK_DIM
        expanded: np.ndarray = np.zeros((new_dim, new_dim), dtype=float)
        expanded[0:old_dim, 0:old_dim] = self._p
        expanded[old_dim:new_dim, old_dim:new_dim] = block
        self._p = expanded
        self._delta_x = np.zeros(new_dim, dtype=float)

    def _pose_prior_block(self, sigma_t_m: float, sigma_rot_rad: float) -> np.ndarray:
        translation_var: np.ndarray = np.array([sigma_t_m] * 3, dtype=float) ** 2
        rotation_var: np.ndarray = np.array([sigma_rot_rad] * 3, dtype=float) ** 2
        return np.diag(np.concatenate([translation_var, rotation_var]))

    def _apply_extrinsic_delta(
        self, extrinsics: ExtrinsicsState, delta: np.ndarray
    ) -> ExtrinsicsState:
        t_bi_delta: np.ndarray = delta[0:6]
        t_bm_delta: np.ndarray = delta[6:12]
        t_bc_delta: np.ndarray = delta[12:18]

        t_bi_t: np.ndarray
        t_bi_q: np.ndarray
        t_bi_t, t_bi_q = pose_plus(
            extrinsics.t_bi.translation_m,
            extrinsics.t_bi.rotation_wxyz,
            t_bi_delta,
        )
        t_bm_t: np.ndarray
        t_bm_q: np.ndarray
        t_bm_t, t_bm_q = pose_plus(
            extrinsics.t_bm.translation_m,
            extrinsics.t_bm.rotation_wxyz,
            t_bm_delta,
        )
        t_bc_t: np.ndarray
        t_bc_q: np.ndarray
        t_bc_t, t_bc_q = pose_plus(
            extrinsics.t_bc.translation_m,
            extrinsics.t_bc.rotation_wxyz,
            t_bc_delta,
        )

        return ExtrinsicsState(
            t_bi=PoseState(translation_m=t_bi_t, rotation_wxyz=t_bi_q),
            t_bm=PoseState(translation_m=t_bm_t, rotation_wxyz=t_bm_q),
            t_bc=PoseState(translation_m=t_bc_t, rotation_wxyz=t_bc_q),
        )

    def _apply_landmark_delta(self, delta: np.ndarray) -> None:
        index: int
        key: TagKey
        for index, key in enumerate(self._landmark_order):
            start: int = index * _LANDMARK_DIM
            end: int = start + _LANDMARK_DIM
            pose_delta: np.ndarray = delta[start:end]
            pose: PoseState = self._landmarks[key]
            updated_t: np.ndarray
            updated_q: np.ndarray
            updated_t, updated_q = pose_plus(
                pose.translation_m, pose.rotation_wxyz, pose_delta
            )
            self._landmarks[key] = PoseState(
                translation_m=updated_t, rotation_wxyz=updated_q
            )
