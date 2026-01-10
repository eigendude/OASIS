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

from typing import Optional

import numpy as np

from oasis_control.localization.ekf.core.ekf_core_state import EkfCoreStateMixin
from oasis_control.localization.ekf.core.ekf_core_utils import EkfCoreUtilsMixin
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_state import EkfState
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
from oasis_control.localization.ekf.se3 import pose_plus


class EkfCoreAprilTagMixin(EkfCoreStateMixin, EkfCoreUtilsMixin):
    _apriltag_model: AprilTagMeasurementModel
    _camera_info: Optional[CameraInfoData]
    _config: EkfConfig
    _state: EkfState
    _world_odom: Pose3
    _world_odom_cov: np.ndarray

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
        detections: list[EkfAprilTagDetectionUpdate] = []
        # Linearize once per message to keep per-detection updates deterministic
        x_lin: np.ndarray = self._world_base_legacy_state()
        for detection in detections_sorted:
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
            reproj_rms_px=0.0,
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
        z: Optional[np.ndarray] = self._apriltag_model.pose_measurement(detection)
        if z is None:
            return self.build_rejected_apriltag_detection(detection, frame_id, t_meas)

        z_hat: np.ndarray
        h: np.ndarray
        z_hat, h = self._apriltag_model.linearize_pose(x_lin)
        h = self._lift_world_odom_jacobian(h)
        residual: np.ndarray = z - z_hat
        residual[3] = self._wrap_angle(residual[3])

        r: np.ndarray = np.diag(
            [
                self._config.apriltag_pos_var,
                self._config.apriltag_pos_var,
                self._config.apriltag_pos_var,
                self._config.apriltag_yaw_var,
            ]
        )
        s_hat: np.ndarray = h @ self._world_odom_cov @ h.T
        s: np.ndarray = s_hat + r
        maha_d2: float = float(residual.T @ np.linalg.solve(s, residual))
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
            )

        hp: np.ndarray = h @ self._world_odom_cov
        k_gain: np.ndarray = np.linalg.solve(s.T, hp).T

        delta: np.ndarray = k_gain @ residual
        self._world_odom.translation_m, self._world_odom.rotation_wxyz = pose_plus(
            self._world_odom.translation_m,
            self._world_odom.rotation_wxyz,
            delta,
        )
        identity: np.ndarray = np.eye(6, dtype=float)
        temp: np.ndarray = identity - k_gain @ h
        self._world_odom_cov = (
            temp @ self._world_odom_cov @ temp.T + k_gain @ r @ k_gain.T
        )

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
            reproj_rms_px=0.0,
        )
        return EkfAprilTagDetectionUpdate(
            family=detection.family,
            tag_id=detection.tag_id,
            det_index_in_msg=detection.det_index_in_msg,
            update=update,
        )

    def _lift_world_odom_jacobian(self, legacy_h: np.ndarray) -> np.ndarray:
        if legacy_h.shape[1] != 9:
            raise ValueError("Expected legacy Jacobian with 9 columns")
        lifted: np.ndarray = np.zeros((legacy_h.shape[0], 6), dtype=float)
        lifted[:, 0:3] = legacy_h[:, 0:3]
        lifted[:, 3:6] = legacy_h[:, 6:9]
        return lifted
