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
AprilTag update handling for the EKF core
"""

from __future__ import annotations

import math
from typing import Optional
from typing import cast

import numpy as np

from oasis_control.localization.ekf.core.ekf_core_state import EkfCoreStateMixin
from oasis_control.localization.ekf.core.ekf_core_utils import EkfCoreUtilsMixin
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_state import EkfState
from oasis_control.localization.ekf.ekf_state import EkfStateIndex
from oasis_control.localization.ekf.ekf_state import Pose3
from oasis_control.localization.ekf.ekf_state import TagKey
from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import AprilTagDetectionArrayData
from oasis_control.localization.ekf.ekf_types import CameraInfoData
from oasis_control.localization.ekf.ekf_types import EkfAprilTagDetectionUpdate
from oasis_control.localization.ekf.ekf_types import EkfAprilTagUpdateData
from oasis_control.localization.ekf.ekf_types import EkfMatrix
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import EkfUpdateData
from oasis_control.localization.ekf.models.apriltag_measurement_model import (
    SUPPORTED_DISTORTION_MODELS,
)
from oasis_control.localization.ekf.models.apriltag_measurement_model import (
    AprilTagMeasurementModel,
)
from oasis_control.localization.ekf.se3 import pose_compose
from oasis_control.localization.ekf.se3 import pose_inverse
from oasis_control.localization.ekf.se3 import pose_minus
from oasis_control.localization.ekf.se3 import pose_plus


# Nominal pixel noise used to scale AprilTag measurement variance, pixels
APRILTAG_REPROJ_NOISE_PX: float = 1.0

# Translation finite-difference step for AprilTag Jacobians, meters
_APRILTAG_JAC_EPS_M: float = 1.0e-4

# Rotation finite-difference step for AprilTag Jacobians, radians
_APRILTAG_JAC_EPS_RAD: float = 1.0e-4


class EkfCoreAprilTagMixin(EkfCoreStateMixin, EkfCoreUtilsMixin):
    _apriltag_model: AprilTagMeasurementModel
    _camera_info: Optional[CameraInfoData]
    _config: EkfConfig
    _state: EkfState

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

        detections_sorted: list[AprilTagDetection] = sorted(
            apriltag_data.detections, key=lambda det: (det.family, det.tag_id)
        )
        x_lin: np.ndarray = self._world_base_legacy_state()
        detections: list[EkfAprilTagDetectionUpdate] = []
        for detection in detections_sorted:
            # Linearize from a fixed state snapshot for deterministic ordering
            tag_key: TagKey = TagKey(family=detection.family, tag_id=detection.tag_id)
            self._state.ensure_landmark(tag_key)
            detection_update: EkfAprilTagDetectionUpdate = (
                self._update_with_apriltag_detection(
                    detection, apriltag_data.frame_id, t_meas, x_lin
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
        z_dim: int = len(z_values) if z_values else 4
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

    def _update_with_apriltag_detection(
        self,
        detection: AprilTagDetection,
        frame_id: str,
        t_meas: EkfTime,
        x_lin: np.ndarray,
    ) -> EkfAprilTagDetectionUpdate:
        z_initial: Optional[np.ndarray] = self._apriltag_model.pose_measurement(
            detection
        )
        if z_initial is None:
            return self.build_rejected_apriltag_detection(detection, frame_id, t_meas)

        tag_pose_c: np.ndarray = np.array(
            [
                z_initial[0],
                z_initial[1],
                z_initial[2],
                0.0,
                0.0,
                z_initial[3],
            ],
            dtype=float,
        )

        # AprilTag updates are driven by reprojection of the detected corners so
        # the refined pose aligns with the full image-space measurement
        refined_pose_c: Optional[np.ndarray] = (
            self._apriltag_model.refine_pose_from_corners(
                tag_pose_c, detection, self._config.tag_size_m
            )
        )
        if refined_pose_c is None:
            return self.build_rejected_apriltag_detection(detection, frame_id, t_meas)

        reprojection: Optional[
            tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]
        ] = self._apriltag_model.reprojection_residuals(
            refined_pose_c, detection, self._config.tag_size_m
        )
        if reprojection is None:
            return self.build_rejected_apriltag_detection(detection, frame_id, t_meas)

        residual_corners: np.ndarray = reprojection[2]
        reproj_rms_px: float = math.sqrt(
            float(residual_corners.T @ residual_corners) / residual_corners.size
        )

        gate_reproj_px: float = self._config.apriltag_reproj_rms_gate_px
        if gate_reproj_px > 0.0 and reproj_rms_px > gate_reproj_px:
            return self.build_rejected_apriltag_detection(
                detection,
                frame_id,
                t_meas,
                reject_reason="reprojection_gate",
                reproj_rms_px=reproj_rms_px,
            )

        z: np.ndarray = np.array(
            [
                refined_pose_c[0],
                refined_pose_c[1],
                refined_pose_c[2],
                refined_pose_c[5],
            ],
            dtype=float,
        )

        z_hat: np.ndarray
        h_legacy: np.ndarray
        z_hat, h_legacy = self._apriltag_model.linearize_pose(x_lin)
        h_pose: np.ndarray = self._lift_world_base_jacobian(h_legacy)
        world_odom_jacobian: np.ndarray
        odom_base_jacobian: np.ndarray
        world_odom_jacobian, odom_base_jacobian = self._world_base_error_jacobians()
        h: np.ndarray = np.zeros(
            (h_pose.shape[0], self._state.index.total_dim), dtype=float
        )
        h[:, self._state.index.world_odom] = h_pose @ world_odom_jacobian
        h[:, self._state.index.pose] = h_pose @ odom_base_jacobian
        residual: np.ndarray = z - z_hat
        residual[3] = self._wrap_angle(residual[3])

        reproj_scale: float = max(1.0, reproj_rms_px / APRILTAG_REPROJ_NOISE_PX)
        r: np.ndarray = np.diag(
            [
                self._config.apriltag_pos_var * reproj_scale**2,
                self._config.apriltag_pos_var * reproj_scale**2,
                self._config.apriltag_pos_var * reproj_scale**2,
                self._config.apriltag_yaw_var * reproj_scale**2,
            ]
        )
        p_prior: np.ndarray = self._state.covariance
        s_hat: np.ndarray = h @ p_prior @ h.T
        s: np.ndarray = s_hat + r
        s = 0.5 * (s + s.T)
        base: float = 1e-12
        scale: float = max(1.0, float(np.max(np.abs(np.diag(s)))))
        jitter: float = base * scale
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
                reproj_rms_px=reproj_rms_px,
            )

        ph_t: np.ndarray = p_prior @ h.T
        tmp: np.ndarray = np.linalg.solve(l, ph_t.T)
        s_inv_ph_t: np.ndarray = np.linalg.solve(l.T, tmp)
        k_gain: np.ndarray = s_inv_ph_t.T

        delta: np.ndarray = k_gain @ residual
        odom_before: Pose3 = self._state.pose_ob.copy()
        vel_before: np.ndarray = self._state.vel_o_mps.copy()
        self._state.apply_delta(delta)
        identity: np.ndarray = np.eye(self._state.index.total_dim, dtype=float)
        temp: np.ndarray = identity - k_gain @ h
        self._state.covariance = temp @ p_prior @ temp.T + k_gain @ r @ k_gain.T
        self._state.covariance = 0.5 * (
            self._state.covariance + self._state.covariance.T
        )

        # AprilTag updates must not teleport odom so we reset deterministically
        # This applies a gauge transformation that keeps world-base fixed while
        # re-parameterizing the world-to-odom estimate
        self._apply_odom_continuity_reset(odom_before, vel_before)

        update: EkfUpdateData = EkfUpdateData(
            sensor="apriltags",
            frame_id=frame_id,
            t_meas=t_meas,
            accepted=True,
            reject_reason="",
            z_dim=4,
            z=z.tolist(),
            z_hat=z_hat.tolist(),
            nu=residual.tolist(),
            r=EkfMatrix(rows=4, cols=4, data=r.flatten().tolist()),
            s_hat=EkfMatrix(rows=4, cols=4, data=s_hat.flatten().tolist()),
            s=EkfMatrix(rows=4, cols=4, data=s.flatten().tolist()),
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

    def _lift_world_base_jacobian(self, legacy_h: np.ndarray) -> np.ndarray:
        if legacy_h.shape[1] != 9:
            raise ValueError("Expected legacy Jacobian with 9 columns")
        lifted: np.ndarray = np.zeros((legacy_h.shape[0], 6), dtype=float)
        lifted[:, 0:3] = legacy_h[:, 0:3]
        lifted[:, 3:6] = legacy_h[:, 6:9]
        return lifted

    def _world_base_error_jacobians(self) -> tuple[np.ndarray, np.ndarray]:
        world_odom: Pose3 = self._state.world_odom
        odom_base: Pose3 = self._state.pose_ob
        t_world_base: np.ndarray
        q_world_base: np.ndarray
        t_world_base, q_world_base = pose_compose(
            world_odom.translation_m,
            world_odom.rotation_wxyz,
            odom_base.translation_m,
            odom_base.rotation_wxyz,
        )
        world_odom_jacobian: np.ndarray = np.zeros((6, 6), dtype=float)
        odom_base_jacobian: np.ndarray = np.zeros((6, 6), dtype=float)
        axis: int
        for axis in range(6):
            step_world_odom: float = (
                _APRILTAG_JAC_EPS_M if axis < 3 else _APRILTAG_JAC_EPS_RAD
            )
            delta_world_odom: np.ndarray = np.zeros(6, dtype=float)
            delta_world_odom[axis] = step_world_odom
            t_world_odom_pert: np.ndarray
            q_world_odom_pert: np.ndarray
            t_world_odom_pert, q_world_odom_pert = pose_plus(
                world_odom.translation_m,
                world_odom.rotation_wxyz,
                delta_world_odom,
            )
            t_world_base_pert: np.ndarray
            q_world_base_pert: np.ndarray
            t_world_base_pert, q_world_base_pert = pose_compose(
                t_world_odom_pert,
                q_world_odom_pert,
                odom_base.translation_m,
                odom_base.rotation_wxyz,
            )
            delta_world_base: np.ndarray = pose_minus(
                t_world_base_pert,
                q_world_base_pert,
                t_world_base,
                q_world_base,
            )
            world_odom_jacobian[:, axis] = delta_world_base / step_world_odom

        for axis in range(6):
            step_odom: float = (
                _APRILTAG_JAC_EPS_M if axis < 3 else _APRILTAG_JAC_EPS_RAD
            )
            delta_odom: np.ndarray = np.zeros(6, dtype=float)
            delta_odom[axis] = step_odom
            t_odom_base_pert: np.ndarray
            q_odom_base_pert: np.ndarray
            t_odom_base_pert, q_odom_base_pert = pose_plus(
                odom_base.translation_m,
                odom_base.rotation_wxyz,
                delta_odom,
            )
            t_world_base_pert_odom: np.ndarray
            q_world_base_pert_odom: np.ndarray
            t_world_base_pert_odom, q_world_base_pert_odom = pose_compose(
                world_odom.translation_m,
                world_odom.rotation_wxyz,
                t_odom_base_pert,
                q_odom_base_pert,
            )
            delta_world_base_odom: np.ndarray = pose_minus(
                t_world_base_pert_odom,
                q_world_base_pert_odom,
                t_world_base,
                q_world_base,
            )
            odom_base_jacobian[:, axis] = delta_world_base_odom / step_odom

        return world_odom_jacobian, odom_base_jacobian

    def _apply_odom_continuity_reset(
        self, odom_before: Pose3, vel_before: np.ndarray
    ) -> None:
        """
        Restore odom pose and velocity while shifting world-odom to preserve
        world-base.

        This deterministic gauge reset is a coordinate transformation that
        re-parameterizes world->odom so the correction lives in the global
        alignment while odom remains continuous.
        """

        world_odom_after: Pose3 = self._state.world_odom.copy()
        odom_after: Pose3 = self._state.pose_ob.copy()
        world_odom_reset: Pose3 = self._reset_world_odom(
            world_odom_after, odom_after, odom_before
        )
        self._state.pose_ob = odom_before.copy()
        self._state.world_odom = world_odom_reset

        reset_jacobian: np.ndarray = self._odom_reset_jacobian(
            world_odom_after, odom_after, odom_before, world_odom_reset
        )
        if reset_jacobian.shape != (12, 12):
            raise ValueError("Expected reset Jacobian with shape (12, 12)")
        index: EkfStateIndex = self._state.index
        world_odom_indices: list[int] = list(
            range(index.world_odom.start, index.world_odom.stop)
        )
        pose_indices: list[int] = list(range(index.pose.start, index.pose.stop))
        reset_indices: list[int] = world_odom_indices + pose_indices
        if len(reset_indices) != 12:
            raise ValueError("Expected 12 reset indices for world_odom and pose")
        if len(set(reset_indices)) != len(reset_indices):
            raise ValueError("Reset indices must be unique")
        full_jacobian: np.ndarray = np.eye(index.total_dim, dtype=float)
        full_jacobian[np.ix_(reset_indices, reset_indices)] = reset_jacobian
        covariance_after: np.ndarray = self._state.covariance
        # Change-of-variables update keeps the shared covariance consistent
        self._state.covariance = full_jacobian @ covariance_after @ full_jacobian.T
        self._state.covariance = 0.5 * (
            self._state.covariance + self._state.covariance.T
        )
        diag_variances: np.ndarray = np.diag(self._state.covariance)
        if np.any(diag_variances < 0.0):
            diag_indices: tuple[np.ndarray, np.ndarray] = cast(
                tuple[np.ndarray, np.ndarray],
                np.diag_indices_from(self._state.covariance),
            )
            self._state.covariance[diag_indices] = np.maximum(diag_variances, 0.0)
            self._state.covariance = 0.5 * (
                self._state.covariance + self._state.covariance.T
            )
        # Symmetry tolerance for covariance numerical roundoff, unitless
        if not np.allclose(self._state.covariance, self._state.covariance.T, atol=1e-9):
            raise ValueError("Reset covariance lost symmetry")
        # Negative variance tolerance for numerical roundoff, variance units
        if np.any(np.diag(self._state.covariance) < -1e-12):
            raise ValueError("Reset covariance has negative variances")
        self._state.vel_o_mps = vel_before.copy()

    def _reset_world_odom(
        self, world_odom: Pose3, odom_after: Pose3, odom_before: Pose3
    ) -> Pose3:
        """
        Shift world->odom to keep world->base fixed while odom stays continuous
        """

        t_world_base: np.ndarray
        q_world_base: np.ndarray
        t_world_base, q_world_base = pose_compose(
            world_odom.translation_m,
            world_odom.rotation_wxyz,
            odom_after.translation_m,
            odom_after.rotation_wxyz,
        )
        t_odom_inv: np.ndarray
        q_odom_inv: np.ndarray
        t_odom_inv, q_odom_inv = pose_inverse(
            odom_before.translation_m,
            odom_before.rotation_wxyz,
        )
        t_world_odom: np.ndarray
        q_world_odom: np.ndarray
        t_world_odom, q_world_odom = pose_compose(
            t_world_base,
            q_world_base,
            t_odom_inv,
            q_odom_inv,
        )
        return Pose3(translation_m=t_world_odom, rotation_wxyz=q_world_odom)

    def _odom_reset_jacobian(
        self,
        world_odom_after: Pose3,
        odom_after: Pose3,
        odom_before: Pose3,
        world_odom_reset: Pose3,
    ) -> np.ndarray:
        """
        Return the gauge reset Jacobian over [world_odom, odom] perturbations
        """

        jacobian: np.ndarray = np.zeros((12, 12), dtype=float)
        axis: int
        for axis in range(12):
            step: float = _APRILTAG_JAC_EPS_M if axis % 6 < 3 else _APRILTAG_JAC_EPS_RAD
            delta: np.ndarray = np.zeros(6, dtype=float)
            delta[axis % 6] = step
            world_odom_pert: Pose3
            odom_base_pert: Pose3
            if axis < 6:
                t_world_odom_pert: np.ndarray
                q_world_odom_pert: np.ndarray
                t_world_odom_pert, q_world_odom_pert = pose_plus(
                    world_odom_after.translation_m,
                    world_odom_after.rotation_wxyz,
                    delta,
                )
                odom_base_pert = odom_after
                world_odom_pert = Pose3(
                    translation_m=t_world_odom_pert,
                    rotation_wxyz=q_world_odom_pert,
                )
            else:
                t_odom_base_pert: np.ndarray
                q_odom_base_pert: np.ndarray
                t_odom_base_pert, q_odom_base_pert = pose_plus(
                    odom_after.translation_m,
                    odom_after.rotation_wxyz,
                    delta,
                )
                odom_base_pert = Pose3(
                    translation_m=t_odom_base_pert,
                    rotation_wxyz=q_odom_base_pert,
                )
                world_odom_pert = world_odom_after
            world_odom_reset_pert: Pose3 = self._reset_world_odom(
                world_odom_pert, odom_base_pert, odom_before
            )
            delta_world_odom: np.ndarray = pose_minus(
                world_odom_reset_pert.translation_m,
                world_odom_reset_pert.rotation_wxyz,
                world_odom_reset.translation_m,
                world_odom_reset.rotation_wxyz,
            )
            jacobian[:6, axis] = delta_world_odom / step
        # Bottom-left block is zero because odom perturbations are reset away
        # Bottom-right block is identity because odom error coordinates remain
        # unchanged by the gauge transform
        jacobian[6:12, 6:12] = np.eye(6, dtype=float)
        return jacobian
