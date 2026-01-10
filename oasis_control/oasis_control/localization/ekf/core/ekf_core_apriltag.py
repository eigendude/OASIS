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
AprilTag update helpers for the EKF core
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from oasis_control.localization.ekf.ekf_state import EkfState
from oasis_control.localization.ekf.ekf_state import Pose3
from oasis_control.localization.ekf.ekf_state import TagKey
from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import AprilTagDetectionArrayData
from oasis_control.localization.ekf.ekf_types import EkfAprilTagDetectionUpdate
from oasis_control.localization.ekf.ekf_types import EkfAprilTagUpdateData
from oasis_control.localization.ekf.ekf_types import EkfMatrix
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import EkfUpdateData
from oasis_control.localization.ekf.models.apriltag_measurement_model import (
    SUPPORTED_DISTORTION_MODELS,
)
from oasis_control.localization.ekf.se3 import pose_compose
from oasis_control.localization.ekf.se3 import pose_inverse


# Pixels standard deviation for AprilTag corner noise
_SIGMA_PX_BASE: float = 1.0

# Maximum RMS reprojection error accepted, pixels
_REPROJ_RMS_MAX_PX: float = 5.0

# Homography condition number gating threshold, unitless
_HOMOGRAPHY_COND_MAX: float = 1.0e8

# Decision margin epsilon to avoid division by zero, unitless
_DECISION_MARGIN_EPS: float = 1.0e-3

# Meters step used for translation numerical derivatives
_POS_EPS_M: float = 1.0e-4

# Radians step used for rotation numerical derivatives
_ROT_EPS_RAD: float = 1.0e-4

# Base jitter for stabilizing the innovation covariance
_SINGULAR_JITTER_BASE: float = 1.0e-12

# Default measurement vector dimension for tag corners
_Z_DIM: int = 8


@dataclass(frozen=True)
class _ApriltagLinearization:
    detection: AprilTagDetection
    tag_key: TagKey
    z: Optional[np.ndarray]
    z_hat: Optional[np.ndarray]
    residual: Optional[np.ndarray]
    h: Optional[np.ndarray]
    r: Optional[np.ndarray]
    reproj_rms_px: float
    reject_reason: Optional[str]


class _EkfCoreApriltagMixin:
    def update_with_apriltags(
        self, apriltag_data: AprilTagDetectionArrayData, t_meas: EkfTime
    ) -> EkfAprilTagUpdateData:
        """
        Apply the AprilTag update model
        """

        if self._camera_info is None:
            return self.build_rejected_apriltag_update(
                apriltag_data, t_meas, "No camera_info cached"
            )

        if apriltag_data.frame_id != self._camera_info.frame_id:
            return self.build_rejected_apriltag_update(
                apriltag_data, t_meas, "AprilTag frame mismatch"
            )

        distortion_model: str = self._camera_info.distortion_model.lower()
        if distortion_model not in SUPPORTED_DISTORTION_MODELS:
            return self.build_rejected_apriltag_update(
                apriltag_data, t_meas, "Unsupported distortion model"
            )

        anchor_key: TagKey = TagKey(
            family=self._config.tag_anchor_family,
            tag_id=self._config.tag_anchor_id,
        )
        self._state.ensure_landmark(anchor_key)

        x_lin_state: EkfState = self._state.copy()
        x_lin_state.ensure_landmark(anchor_key)
        world_odom_lin: Pose3 = self._world_odom.copy()

        detections_sorted: list[AprilTagDetection] = sorted(
            apriltag_data.detections, key=lambda det: (det.family, det.tag_id)
        )

        linearizations: list[_ApriltagLinearization] = []
        for detection in detections_sorted:
            tag_key: TagKey = TagKey(family=detection.family, tag_id=detection.tag_id)
            linearization: _ApriltagLinearization = self._linearize_apriltag_detection(
                detection,
                tag_key,
                x_lin_state,
                world_odom_lin,
            )
            linearizations.append(linearization)

        detections: list[EkfAprilTagDetectionUpdate] = []
        for linearization in linearizations:
            detection_update: EkfAprilTagDetectionUpdate = (
                self._apply_apriltag_update(
                    linearization,
                    apriltag_data.frame_id,
                    t_meas,
                )
            )
            detections.append(detection_update)

        return EkfAprilTagUpdateData(
            t_meas=t_meas,
            frame_id=apriltag_data.frame_id,
            detections=detections,
        )

    def build_rejected_apriltag_detection(
        self,
        detection: AprilTagDetection,
        frame_id: str,
        t_meas: EkfTime,
        *,
        reject_reason: str = "TODO: AprilTag update not implemented",
        z: Optional[np.ndarray] = None,
        z_hat: Optional[np.ndarray] = None,
        residual: Optional[np.ndarray] = None,
        r: Optional[np.ndarray] = None,
        s_hat: Optional[np.ndarray] = None,
        s: Optional[np.ndarray] = None,
        maha_d2: float = 0.0,
        gate_d2_threshold: float = 0.0,
        reproj_rms_px: float = 0.0,
    ) -> EkfAprilTagDetectionUpdate:
        z_values: list[float] = [] if z is None else z.tolist()
        z_dim: int = len(z_values) if z_values else _Z_DIM
        if not z_values:
            z_values = self._nan_list(z_dim)
        z_hat_values: list[float] = (
            self._nan_list(z_dim) if z_hat is None else z_hat.tolist()
        )
        nu_values: list[float] = (
            self._nan_list(z_dim) if residual is None else residual.tolist()
        )
        zero_matrix: EkfMatrix = self._zero_matrix(z_dim)
        r_matrix: EkfMatrix = (
            zero_matrix
            if r is None
            else EkfMatrix(rows=z_dim, cols=z_dim, data=r.flatten().tolist())
        )
        s_hat_matrix: EkfMatrix = (
            zero_matrix
            if s_hat is None
            else EkfMatrix(rows=z_dim, cols=z_dim, data=s_hat.flatten().tolist())
        )
        s_matrix: EkfMatrix = (
            zero_matrix
            if s is None
            else EkfMatrix(rows=z_dim, cols=z_dim, data=s.flatten().tolist())
        )
        update: EkfUpdateData = EkfUpdateData(
            sensor="apriltags",
            frame_id=frame_id,
            t_meas=t_meas,
            accepted=False,
            reject_reason=reject_reason,
            z_dim=z_dim,
            z=z_values,
            z_hat=z_hat_values,
            nu=nu_values,
            r=r_matrix,
            s_hat=s_hat_matrix,
            s=s_matrix,
            maha_d2=maha_d2,
            gate_d2_threshold=gate_d2_threshold,
            reproj_rms_px=reproj_rms_px,
        )
        return EkfAprilTagDetectionUpdate(
            family=detection.family,
            tag_id=detection.tag_id,
            det_index_in_msg=detection.det_index_in_msg,
            update=update,
        )

    def build_rejected_apriltag_update(
        self,
        apriltag_data: AprilTagDetectionArrayData,
        t_meas: EkfTime,
        reason: str,
    ) -> EkfAprilTagUpdateData:
        detections_sorted: list[AprilTagDetection] = sorted(
            apriltag_data.detections, key=lambda det: (det.family, det.tag_id)
        )
        detections: list[EkfAprilTagDetectionUpdate] = []
        for detection in detections_sorted:
            detections.append(
                self.build_rejected_apriltag_detection(
                    detection, apriltag_data.frame_id, t_meas, reject_reason=reason
                )
            )
        return EkfAprilTagUpdateData(
            t_meas=t_meas,
            frame_id=apriltag_data.frame_id,
            detections=detections,
        )

    def _linearize_apriltag_detection(
        self,
        detection: AprilTagDetection,
        tag_key: TagKey,
        x_lin_state: EkfState,
        world_odom_lin: Pose3,
    ) -> _ApriltagLinearization:
        homography_status: Optional[str] = self._validate_homography(detection)
        if homography_status is not None:
            return _ApriltagLinearization(
                detection=detection,
                tag_key=tag_key,
                z=None,
                z_hat=None,
                residual=None,
                h=None,
                r=None,
                reproj_rms_px=0.0,
                reject_reason=homography_status,
            )

        z: Optional[np.ndarray] = self._measurement_vector(detection)
        if z is None:
            return _ApriltagLinearization(
                detection=detection,
                tag_key=tag_key,
                z=None,
                z_hat=None,
                residual=None,
                h=None,
                r=None,
                reproj_rms_px=0.0,
                reject_reason="invalid_corners",
            )

        initialized: bool = self._ensure_tag_initialized(
            detection,
            tag_key,
            x_lin_state,
            world_odom_lin,
        )
        if not initialized:
            return _ApriltagLinearization(
                detection=detection,
                tag_key=tag_key,
                z=z,
                z_hat=None,
                residual=None,
                h=None,
                r=None,
                reproj_rms_px=0.0,
                reject_reason="landmark_init_failed",
            )

        z_hat: Optional[np.ndarray] = self._predict_tag_corners(
            x_lin_state,
            world_odom_lin,
            tag_key,
        )
        if z_hat is None:
            return _ApriltagLinearization(
                detection=detection,
                tag_key=tag_key,
                z=z,
                z_hat=None,
                residual=None,
                h=None,
                r=None,
                reproj_rms_px=0.0,
                reject_reason="projection_failed",
            )

        residual: np.ndarray = z - z_hat
        reproj_rms_px: float = self._reprojection_rms(residual)
        if reproj_rms_px > _REPROJ_RMS_MAX_PX:
            return _ApriltagLinearization(
                detection=detection,
                tag_key=tag_key,
                z=z,
                z_hat=z_hat,
                residual=residual,
                h=None,
                r=None,
                reproj_rms_px=reproj_rms_px,
                reject_reason="reprojection_rms",
            )

        h: Optional[np.ndarray] = self._finite_difference_jacobian(
            x_lin_state,
            world_odom_lin,
            tag_key,
        )
        if h is None:
            return _ApriltagLinearization(
                detection=detection,
                tag_key=tag_key,
                z=z,
                z_hat=z_hat,
                residual=residual,
                h=None,
                r=None,
                reproj_rms_px=reproj_rms_px,
                reject_reason="jacobian_failed",
            )

        sigma_px: float = self._sigma_px(detection.decision_margin)
        r: np.ndarray = np.eye(_Z_DIM, dtype=float) * (sigma_px**2)

        return _ApriltagLinearization(
            detection=detection,
            tag_key=tag_key,
            z=z,
            z_hat=z_hat,
            residual=residual,
            h=h,
            r=r,
            reproj_rms_px=reproj_rms_px,
            reject_reason=None,
        )

    def _apply_apriltag_update(
        self,
        linearization: _ApriltagLinearization,
        frame_id: str,
        t_meas: EkfTime,
    ) -> EkfAprilTagDetectionUpdate:
        detection: AprilTagDetection = linearization.detection
        if linearization.reject_reason is not None:
            return self.build_rejected_apriltag_detection(
                detection,
                frame_id,
                t_meas,
                reject_reason=linearization.reject_reason,
                z=linearization.z,
                z_hat=linearization.z_hat,
                residual=linearization.residual,
                r=linearization.r,
                reproj_rms_px=linearization.reproj_rms_px,
            )

        z: np.ndarray = np.asarray(linearization.z, dtype=float).reshape(_Z_DIM)
        z_hat: np.ndarray = np.asarray(linearization.z_hat, dtype=float).reshape(_Z_DIM)
        residual: np.ndarray = np.asarray(linearization.residual, dtype=float).reshape(
            _Z_DIM
        )
        h: np.ndarray = np.asarray(linearization.h, dtype=float).reshape(
            _Z_DIM, self._state.index.total_dim
        )
        r: np.ndarray = np.asarray(linearization.r, dtype=float).reshape(_Z_DIM, _Z_DIM)

        s_hat: np.ndarray = h @ self._state.covariance @ h.T
        s: np.ndarray = s_hat + r
        s = 0.5 * (s + s.T)
        scale: float = max(1.0, float(np.max(np.abs(np.diag(s)))))
        jitter: float = _SINGULAR_JITTER_BASE * scale
        s = s + jitter * np.eye(s.shape[0], dtype=float)
        l: Optional[np.ndarray]
        try:
            l = np.linalg.cholesky(s)
        except np.linalg.LinAlgError:
            l = None
            for factor in (1e-10, 1e-8, 1e-6, 1e-4):
                s_try: np.ndarray = 0.5 * (s + s.T) + (factor * scale) * np.eye(
                    s.shape[0], dtype=float
                )
                try:
                    l = np.linalg.cholesky(s_try)
                    s = s_try
                    break
                except np.linalg.LinAlgError:
                    continue
            else:
                return self.build_rejected_apriltag_detection(
                    detection,
                    frame_id,
                    t_meas,
                    reject_reason="singular S",
                    z=z,
                    z_hat=z_hat,
                    residual=residual,
                    r=r,
                    s_hat=s_hat,
                    s=s,
                    reproj_rms_px=linearization.reproj_rms_px,
                )

        y: np.ndarray = np.linalg.solve(l, residual)
        x: np.ndarray = np.linalg.solve(l.T, y)
        maha_d2: float = float(residual.T @ x)
        gate_d2_threshold: float = self._config.apriltag_gate_d2
        if gate_d2_threshold > 0.0 and maha_d2 > gate_d2_threshold:
            return self.build_rejected_apriltag_detection(
                detection,
                frame_id,
                t_meas,
                reject_reason="mahalanobis_gate",
                z=z,
                z_hat=z_hat,
                residual=residual,
                r=r,
                s_hat=s_hat,
                s=s,
                maha_d2=maha_d2,
                gate_d2_threshold=gate_d2_threshold,
                reproj_rms_px=linearization.reproj_rms_px,
            )

        ph_t: np.ndarray = self._state.covariance @ h.T
        tmp: np.ndarray = np.linalg.solve(l, ph_t.T)
        s_inv_ph_t: np.ndarray = np.linalg.solve(l.T, tmp)
        k_gain: np.ndarray = s_inv_ph_t.T

        delta: np.ndarray = k_gain @ residual
        self._state.apply_delta(delta)
        identity: np.ndarray = np.eye(self._state.index.total_dim, dtype=float)
        temp: np.ndarray = identity - k_gain @ h
        self._state.covariance = (
            temp @ self._state.covariance @ temp.T + k_gain @ r @ k_gain.T
        )

        update: EkfUpdateData = EkfUpdateData(
            sensor="apriltags",
            frame_id=frame_id,
            t_meas=t_meas,
            accepted=True,
            reject_reason="",
            z_dim=_Z_DIM,
            z=z.tolist(),
            z_hat=z_hat.tolist(),
            nu=residual.tolist(),
            r=EkfMatrix(rows=_Z_DIM, cols=_Z_DIM, data=r.flatten().tolist()),
            s_hat=EkfMatrix(rows=_Z_DIM, cols=_Z_DIM, data=s_hat.flatten().tolist()),
            s=EkfMatrix(rows=_Z_DIM, cols=_Z_DIM, data=s.flatten().tolist()),
            maha_d2=maha_d2,
            gate_d2_threshold=gate_d2_threshold,
            reproj_rms_px=linearization.reproj_rms_px,
        )
        return EkfAprilTagDetectionUpdate(
            family=detection.family,
            tag_id=detection.tag_id,
            det_index_in_msg=detection.det_index_in_msg,
            update=update,
        )

    def _measurement_vector(self, detection: AprilTagDetection) -> Optional[np.ndarray]:
        if len(detection.corners_px) != _Z_DIM:
            return None
        return np.asarray(detection.corners_px, dtype=float).reshape(_Z_DIM)

    def _validate_homography(self, detection: AprilTagDetection) -> Optional[str]:
        if len(detection.homography) != 9:
            return "invalid_homography"
        homography: np.ndarray = np.asarray(detection.homography, dtype=float).reshape(
            3, 3
        )
        try:
            cond: float = float(np.linalg.cond(homography))
        except np.linalg.LinAlgError:
            return "homography_singular"
        if not math.isfinite(cond) or cond > _HOMOGRAPHY_COND_MAX:
            return "homography_singular"
        return None

    def _ensure_tag_initialized(
        self,
        detection: AprilTagDetection,
        tag_key: TagKey,
        x_lin_state: EkfState,
        world_odom_lin: Pose3,
    ) -> bool:
        existing_pose: Optional[Pose3] = x_lin_state.landmark_pose(tag_key)
        if existing_pose is not None:
            return True

        estimate: Optional[tuple[np.ndarray, np.ndarray]] = (
            self._apriltag_model.estimate_tag_pose_c(
                detection,
                self._config.tag_size_m,
            )
        )
        if estimate is None:
            return False

        translation_c: np.ndarray
        rotation_c: np.ndarray
        translation_c, rotation_c = estimate

        world_camera: Pose3 = self._world_camera_pose(x_lin_state, world_odom_lin)
        t_world_tag: np.ndarray
        q_world_tag: np.ndarray
        t_world_tag, q_world_tag = pose_compose(
            world_camera.translation_m,
            world_camera.rotation_wxyz,
            translation_c,
            rotation_c,
        )

        tag_pose: Pose3 = Pose3(
            translation_m=t_world_tag,
            rotation_wxyz=q_world_tag,
        )
        self._state.ensure_landmark(tag_key)
        self._state.set_landmark_pose(tag_key, tag_pose)
        x_lin_state.ensure_landmark(tag_key)
        x_lin_state.set_landmark_pose(tag_key, tag_pose)
        return True

    def _predict_tag_corners(
        self,
        state: EkfState,
        world_odom: Pose3,
        tag_key: TagKey,
    ) -> Optional[np.ndarray]:
        tag_pose: Optional[Pose3] = state.landmark_pose(tag_key)
        if tag_pose is None:
            return None

        world_camera: Pose3 = self._world_camera_pose(state, world_odom)
        t_camera_world: np.ndarray
        q_camera_world: np.ndarray
        t_camera_world, q_camera_world = pose_inverse(
            world_camera.translation_m,
            world_camera.rotation_wxyz,
        )
        t_camera_tag: np.ndarray
        q_camera_tag: np.ndarray
        t_camera_tag, q_camera_tag = pose_compose(
            t_camera_world,
            q_camera_world,
            tag_pose.translation_m,
            tag_pose.rotation_wxyz,
        )
        return self._apriltag_model.project_tag_corners(
            t_camera_tag,
            q_camera_tag,
            self._config.tag_size_m,
        )

    def _world_camera_pose(self, state: EkfState, world_odom: Pose3) -> Pose3:
        t_world_base: np.ndarray
        q_world_base: np.ndarray
        t_world_base, q_world_base = pose_compose(
            world_odom.translation_m,
            world_odom.rotation_wxyz,
            state.pose_ob.translation_m,
            state.pose_ob.rotation_wxyz,
        )
        t_camera_base: np.ndarray
        q_camera_base: np.ndarray
        t_camera_base, q_camera_base = pose_inverse(
            state.extrinsic_bc.translation_m,
            state.extrinsic_bc.rotation_wxyz,
        )
        t_world_camera: np.ndarray
        q_world_camera: np.ndarray
        t_world_camera, q_world_camera = pose_compose(
            t_world_base,
            q_world_base,
            t_camera_base,
            q_camera_base,
        )
        return Pose3(translation_m=t_world_camera, rotation_wxyz=q_world_camera)

    def _finite_difference_jacobian(
        self,
        state: EkfState,
        world_odom: Pose3,
        tag_key: TagKey,
    ) -> Optional[np.ndarray]:
        total_dim: int = state.index.total_dim
        h_full: np.ndarray = np.zeros((_Z_DIM, total_dim), dtype=float)

        pose_slice: slice = state.index.pose
        extrinsic_slice: slice = state.index.extrinsic_bc
        landmark_slice: Optional[slice] = state.landmark_slice(tag_key)
        if landmark_slice is None:
            return None

        slices: list[slice] = [pose_slice, extrinsic_slice, landmark_slice]
        for state_slice in slices:
            for offset in range(state_slice.start, state_slice.stop):
                local_index: int = offset - state_slice.start
                eps: float = _POS_EPS_M if local_index < 3 else _ROT_EPS_RAD

                delta_plus: np.ndarray = np.zeros(total_dim, dtype=float)
                delta_plus[offset] = eps
                state_plus: EkfState = state.copy()
                state_plus.apply_delta(delta_plus)
                z_plus: Optional[np.ndarray] = self._predict_tag_corners(
                    state_plus,
                    world_odom,
                    tag_key,
                )

                delta_minus: np.ndarray = np.zeros(total_dim, dtype=float)
                delta_minus[offset] = -eps
                state_minus: EkfState = state.copy()
                state_minus.apply_delta(delta_minus)
                z_minus: Optional[np.ndarray] = self._predict_tag_corners(
                    state_minus,
                    world_odom,
                    tag_key,
                )

                if z_plus is None or z_minus is None:
                    return None

                h_full[:, offset] = (z_plus - z_minus) / (2.0 * eps)

        return h_full

    def _sigma_px(self, decision_margin: float) -> float:
        margin: float = max(_DECISION_MARGIN_EPS, float(decision_margin))
        scale: float = max(1.0, 1.0 / margin)
        return _SIGMA_PX_BASE * scale

    def _reprojection_rms(self, residual: np.ndarray) -> float:
        mean_sq: float = float(np.mean(np.asarray(residual, dtype=float) ** 2))
        return math.sqrt(mean_sq)
