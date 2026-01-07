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
Core EKF processing logic
"""

from __future__ import annotations

from typing import Optional
from typing import cast

from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import AprilTagDetectionArrayData
from oasis_control.localization.ekf.ekf_types import CameraInfoData
from oasis_control.localization.ekf.ekf_types import EkfAprilTagDetectionUpdate
from oasis_control.localization.ekf.ekf_types import EkfAprilTagUpdateData
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfMatrix
from oasis_control.localization.ekf.ekf_types import EkfOutputs
from oasis_control.localization.ekf.ekf_types import EkfUpdateData
from oasis_control.localization.ekf.ekf_types import ImuCalibrationData
from oasis_control.localization.ekf.ekf_types import MagSample
from oasis_control.localization.ekf.models.apriltag_measurement_model import (
    AprilTagMeasurementModel,
)
from oasis_control.localization.ekf.models.imu_process_model import ImuProcessModel
from oasis_control.localization.ekf.models.mag_measurement_model import (
    MagMeasurementModel,
)


class EkfCore:
    """
    Minimal EKF state manager with placeholder updates
    """

    def __init__(self, config: EkfConfig) -> None:
        self._config: EkfConfig = config
        self._process_model: ImuProcessModel = ImuProcessModel()
        self._mag_model: MagMeasurementModel = MagMeasurementModel()
        self._apriltag_model: AprilTagMeasurementModel = AprilTagMeasurementModel()
        self._initialized: bool = False
        self._calibration_initialized: bool = False
        self._last_t_meas: Optional[float] = None
        self._camera_info: Optional[CameraInfoData] = None

    def process_event(self, event: EkfEvent) -> EkfOutputs:
        odom_time_s: Optional[float] = None
        world_odom_time_s: Optional[float] = None
        mag_update: Optional[EkfUpdateData] = None
        apriltag_update: Optional[EkfAprilTagUpdateData] = None

        if event.event_type == EkfEventType.IMU:
            imu_packet: EkfImuPacket = cast(EkfImuPacket, event.payload)
            if imu_packet.calibration is not None:
                if imu_packet.calibration.valid and not self._calibration_initialized:
                    self.initialize_from_calibration(imu_packet.calibration)
                    self._calibration_initialized = True

            self.propagate_to(event.t_meas)
            odom_time_s = event.t_meas
            world_odom_time_s = event.t_meas
        elif event.event_type == EkfEventType.MAG:
            mag_sample: MagSample = cast(MagSample, event.payload)
            mag_update = self.update_with_mag(mag_sample, event.t_meas)
        elif event.event_type == EkfEventType.APRILTAG:
            apriltag_data: AprilTagDetectionArrayData = cast(
                AprilTagDetectionArrayData, event.payload
            )
            apriltag_update = self.update_with_apriltags(apriltag_data, event.t_meas)
        elif event.event_type == EkfEventType.CAMERA_INFO:
            self._camera_info = cast(CameraInfoData, event.payload)
            self._apriltag_model.set_camera_info(self._camera_info)

        return EkfOutputs(
            odom_time_s=odom_time_s,
            world_odom_time_s=world_odom_time_s,
            mag_update=mag_update,
            apriltag_update=apriltag_update,
        )

    def propagate_to(self, t_meas: float) -> None:
        """
        Propagate the filter to the measurement time
        """

        # TODO: Apply the continuous-time process model and integrate state
        self._last_t_meas = t_meas

    def update_with_mag(self, mag_sample: MagSample, t_meas: float) -> EkfUpdateData:
        """
        Apply the magnetometer update model
        """

        # TODO: Compute z, z_hat, and perform EKF update
        z_dim: int = 3
        z: list[float] = list(mag_sample.magnetic_field_t)
        zero_matrix: EkfMatrix = self._zero_matrix(z_dim)
        return EkfUpdateData(
            sensor="magnetic_field",
            frame_id=mag_sample.frame_id,
            t_meas=t_meas,
            accepted=False,
            reject_reason="TODO: magnetometer update not implemented",
            z_dim=z_dim,
            z=z,
            z_hat=[0.0] * z_dim,
            nu=[0.0] * z_dim,
            r=zero_matrix,
            s_hat=zero_matrix,
            s=zero_matrix,
            maha_d2=0.0,
            gate_d2_threshold=0.0,
            reproj_rms_px=0.0,
        )

    def update_with_apriltags(
        self, apriltag_data: AprilTagDetectionArrayData, t_meas: float
    ) -> EkfAprilTagUpdateData:
        """
        Apply the AprilTag update model
        """

        # TODO: Compute corner reprojection residuals and update state
        detections: list[EkfAprilTagDetectionUpdate] = []
        for detection in apriltag_data.detections:
            detections.append(
                self.build_rejected_apriltag_detection(
                    detection, apriltag_data.frame_id, t_meas
                )
            )

        return EkfAprilTagUpdateData(
            t_meas=t_meas,
            frame_id=apriltag_data.frame_id,
            detections=detections,
        )

    def initialize_from_calibration(self, calibration: ImuCalibrationData) -> None:
        """
        Initialize calibration parameters from a one-shot prior
        """

        # TODO: Initialize in-state calibration parameters with covariance
        self._initialized = True

    def build_rejected_apriltag_detection(
        self, detection: AprilTagDetection, frame_id: str, t_meas: float
    ) -> EkfAprilTagDetectionUpdate:
        z_dim: int = 8
        zero_matrix: EkfMatrix = self._zero_matrix(z_dim)
        update: EkfUpdateData = EkfUpdateData(
            sensor="apriltags",
            frame_id=frame_id,
            t_meas=t_meas,
            accepted=False,
            reject_reason="TODO: AprilTag update not implemented",
            z_dim=z_dim,
            z=[],
            z_hat=[],
            nu=[],
            r=zero_matrix,
            s_hat=zero_matrix,
            s=zero_matrix,
            maha_d2=0.0,
            gate_d2_threshold=0.0,
            reproj_rms_px=0.0,
        )
        return EkfAprilTagDetectionUpdate(
            family=detection.family,
            tag_id=detection.tag_id,
            det_index_in_msg=detection.det_index_in_msg,
            update=update,
        )

    def _zero_matrix(self, dim: int) -> EkfMatrix:
        return EkfMatrix(rows=dim, cols=dim, data=[0.0] * (dim * dim))
