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

from oasis_control.localization.ekf.core.ekf_core_apriltag import EkfCoreAprilTagMixin
from oasis_control.localization.ekf.core.ekf_core_constants import NS_PER_S
from oasis_control.localization.ekf.core.ekf_core_imu import EkfCoreImuMixin
from oasis_control.localization.ekf.core.ekf_core_mag import EkfCoreMagMixin
from oasis_control.localization.ekf.core.ekf_core_state import EkfCoreStateMixin
from oasis_control.localization.ekf.core.ekf_core_types import _Checkpoint
from oasis_control.localization.ekf.core.ekf_core_types import _ImuCoverageCheck
from oasis_control.localization.ekf.core.ekf_core_types import _ImuGap
from oasis_control.localization.ekf.core.ekf_core_types import _StateSnapshot
from oasis_control.localization.ekf.core.ekf_core_utils import EkfCoreUtilsMixin
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_state import EkfState
from oasis_control.localization.ekf.ekf_types import AprilTagDetectionArrayData
from oasis_control.localization.ekf.ekf_types import CameraInfoData
from oasis_control.localization.ekf.ekf_types import EkfAprilTagUpdateData
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfFrameOutputs
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfOutputs
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import EkfUpdateData
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import MagSample
from oasis_control.localization.ekf.ekf_types import to_ns
from oasis_control.localization.ekf.models.apriltag_measurement_model import (
    AprilTagMeasurementModel,
)
from oasis_control.localization.ekf.models.imu_process_model import ImuProcessModel
from oasis_control.localization.ekf.models.mag_measurement_model import (
    MagMeasurementModel,
)


class EkfCore(
    EkfCoreAprilTagMixin,
    EkfCoreImuMixin,
    EkfCoreMagMixin,
    EkfCoreStateMixin,
    EkfCoreUtilsMixin,
):
    """
    Minimal EKF state manager with placeholder updates
    """

    def __init__(self, config: EkfConfig) -> None:
        self._config: EkfConfig = config
        self._process_model: ImuProcessModel = ImuProcessModel(config)
        self._mag_model: MagMeasurementModel = MagMeasurementModel()
        self._apriltag_model: AprilTagMeasurementModel = AprilTagMeasurementModel()
        self._initialized: bool = False
        self._calibration_initialized: bool = False
        self._camera_info: Optional[CameraInfoData] = None
        self._camera_info_mismatch_count: int = 0
        self._t_frontier_ns: Optional[int] = None
        self._state: EkfState = EkfState(config)
        self._state_frontier: EkfState = self._state.copy()
        self._last_imu_time_ns: Optional[int] = None
        self._last_imu: Optional[ImuSample] = None
        self._imu_times_ns: list[int] = []
        self._imu_gaps: list[_ImuGap] = []
        self._imu_gap_reject_count: int = 0
        # Nanoseconds of IMU history to cover buffer + checkpoint interval
        # with 0.5 s slack for ordering jitter
        self._imu_retention_ns: int = (
            self._config.t_buffer_ns
            + self._config.checkpoint_interval_ns
            + int(0.5 * NS_PER_S)
        )
        self._checkpoints: list[_Checkpoint] = []
        self._last_checkpoint_time: Optional[int] = None

    def process_event(self, event: EkfEvent) -> EkfOutputs:
        t_meas: EkfTime = event.t_meas
        t_meas_ns: int = to_ns(t_meas)
        prev_frontier_ns: Optional[int] = self._t_frontier_ns
        odom_time: Optional[EkfTime] = None
        world_odom_time: Optional[EkfTime] = None
        frame_transforms: Optional[EkfFrameOutputs] = None
        mag_update: Optional[EkfUpdateData] = None
        apriltag_update: Optional[EkfAprilTagUpdateData] = None
        advance_frontier: bool = False
        apriltag_accepted: bool = False

        if event.event_type == EkfEventType.IMU:
            imu_packet: EkfImuPacket = cast(EkfImuPacket, event.payload)
            if imu_packet.calibration is not None:
                if imu_packet.calibration.valid and not self._calibration_initialized:
                    # Calibration messages are priors; applying them after init would
                    # cause nondeterministic jumps
                    self.initialize_from_calibration(imu_packet.calibration)
                    self._calibration_initialized = True

            self._ensure_initialized()
            self._process_imu_packet(imu_packet, t_meas_ns)
            advance_frontier = True
        elif event.event_type == EkfEventType.MAG:
            mag_sample: MagSample = cast(MagSample, event.payload)
            self._ensure_initialized()
            coverage: _ImuCoverageCheck = self._propagate_if_needed(t_meas_ns)
            if not coverage.covered:
                mag_update = self._build_rejected_mag_update(
                    mag_sample, t_meas, reject_reason="imu_gap"
                )
            else:
                mag_update = self.update_with_mag(mag_sample, t_meas)
                advance_frontier = mag_update.accepted
        elif event.event_type == EkfEventType.APRILTAG:
            apriltag_data: AprilTagDetectionArrayData = cast(
                AprilTagDetectionArrayData, event.payload
            )
            snapshot: _StateSnapshot = self._snapshot_state()
            self._ensure_initialized()
            coverage = self._propagate_if_needed(t_meas_ns)
            if not coverage.covered:
                apriltag_update = self.build_rejected_apriltag_update(
                    apriltag_data, t_meas, "imu_gap"
                )
                self._restore_snapshot(snapshot)
            else:
                apriltag_update = self.update_with_apriltags(apriltag_data, t_meas)
                apriltag_accepted = any(
                    detection.update.accepted
                    for detection in apriltag_update.detections
                )
                if apriltag_accepted:
                    advance_frontier = True
                else:
                    self._restore_snapshot(snapshot)
        elif event.event_type == EkfEventType.CAMERA_INFO:
            camera_info: CameraInfoData = cast(CameraInfoData, event.payload)
            self._cache_camera_info(camera_info)

        if advance_frontier:
            self._set_frontier(t_meas)
            self._maybe_checkpoint(t_meas, event.event_type)

        publish_transforms: bool = self._frontier_advanced(prev_frontier_ns)
        if not publish_transforms and apriltag_accepted:
            publish_transforms = True

        if publish_transforms:
            frontier_time: EkfTime = self._frontier_time()
            odom_time = frontier_time
            world_odom_time = frontier_time
            frame_transforms = self._frame_outputs()

        return EkfOutputs(
            odom_time=odom_time,
            world_odom_time=world_odom_time,
            frame_transforms=frame_transforms,
            mag_update=mag_update,
            apriltag_update=apriltag_update,
        )

    def _cache_camera_info(self, camera_info: CameraInfoData) -> None:
        if not self._camera_info_is_valid(camera_info):
            return

        if self._camera_info is None:
            self._camera_info = camera_info
            self._apriltag_model.set_camera_info(camera_info)
            return

        if self._camera_info_intrinsics_match(self._camera_info, camera_info):
            return

        if self._camera_info_mismatch_count == 0:
            self._camera_info_mismatch_count = 1

    def _camera_info_is_valid(self, camera_info: CameraInfoData) -> bool:
        return len(camera_info.k) == 9

    def _camera_info_intrinsics_match(
        self, cached: CameraInfoData, incoming: CameraInfoData
    ) -> bool:
        if cached.frame_id != incoming.frame_id:
            return False
        if cached.k != incoming.k:
            return False
        if cached.d != incoming.d:
            return False
        if cached.distortion_model.lower() != incoming.distortion_model.lower():
            return False
        if cached.p != incoming.p:
            return False
        return True
