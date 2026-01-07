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

import math
from typing import Optional
from typing import cast

import numpy as np

from oasis_control.localization.ekf.core.ekf_core_apriltag import _EkfCoreApriltagMixin
from oasis_control.localization.ekf.core.ekf_core_geometry import _EkfCoreGeometryMixin
from oasis_control.localization.ekf.core.ekf_core_helpers import _EkfCoreHelpersMixin
from oasis_control.localization.ekf.core.ekf_core_imu import _EkfCoreImuMixin
from oasis_control.localization.ekf.core.ekf_core_mag import _EkfCoreMagMixin
from oasis_control.localization.ekf.core.ekf_core_outputs import _EkfCoreOutputsMixin
from oasis_control.localization.ekf.core.ekf_core_state import _EkfCoreStateMixin
from oasis_control.localization.ekf.core.ekf_core_types import _Checkpoint
from oasis_control.localization.ekf.core.ekf_core_types import _ImuCoverageCheck
from oasis_control.localization.ekf.core.ekf_core_types import _ImuGap
from oasis_control.localization.ekf.core.ekf_core_types import _StateSnapshot
from oasis_control.localization.ekf.ekf_buffer import EkfBuffer
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_state import EkfState
from oasis_control.localization.ekf.ekf_state import EkfStateIndex
from oasis_control.localization.ekf.ekf_state import Pose3
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
from oasis_control.localization.ekf.ekf_types import ImuCalibrationData
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


_EVENT_ORDER: dict[EkfEventType, int] = {
    EkfEventType.IMU: 0,
    EkfEventType.CAMERA_INFO: 1,
    EkfEventType.MAG: 2,
    EkfEventType.APRILTAG: 3,
}

# Nanoseconds per second for time conversions
_NS_PER_S: int = 1_000_000_000


class EkfCore(
    _EkfCoreApriltagMixin,
    _EkfCoreGeometryMixin,
    _EkfCoreHelpersMixin,
    _EkfCoreImuMixin,
    _EkfCoreMagMixin,
    _EkfCoreOutputsMixin,
    _EkfCoreStateMixin,
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
        self._t_frontier_ns: Optional[int] = None
        self._state: EkfState = EkfState(config)
        self._state_frontier: EkfState = self._state.copy()
        self._world_odom: Pose3 = self._identity_pose()
        self._world_odom_cov: np.ndarray = self._build_world_odom_covariance()
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
            + int(0.5 * _NS_PER_S)
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
            self._camera_info = cast(CameraInfoData, event.payload)
            self._apriltag_model.set_camera_info(self._camera_info)

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

    def initialize_from_calibration(self, calibration: ImuCalibrationData) -> None:
        """
        Initialize calibration parameters from a one-shot prior
        """

        self._ensure_initialized()
        self._state.apply_imu_calibration(calibration)

    def is_out_of_order(self, t_meas: EkfTime) -> bool:
        if self._t_frontier_ns is None:
            return False
        t_meas_ns: int = to_ns(t_meas)
        return t_meas_ns < self._t_frontier_ns

    def replay(
        self, buffer: EkfBuffer, start_time: Optional[EkfTime] = None
    ) -> list[EkfOutputs]:
        if buffer.earliest_time() is None:
            return []

        earliest_time_ns: int = cast(int, buffer.earliest_time())
        replay_start_ns: int = (
            to_ns(start_time) if start_time is not None else earliest_time_ns
        )
        checkpoint: Optional[_Checkpoint] = self._find_checkpoint(replay_start_ns)
        if checkpoint is None:
            self._reset_state(replay_start_ns)
        else:
            self._restore_checkpoint(checkpoint)

        outputs: list[EkfOutputs] = []

        grouped_events: list[tuple[int, list[EkfEvent]]] = []
        current_time_ns: Optional[int] = None
        current_group: list[EkfEvent] = []
        for event in buffer.iter_events_from(replay_start_ns):
            event_time_ns: int = to_ns(event.t_meas)
            if current_time_ns is None or event_time_ns != current_time_ns:
                if current_group:
                    grouped_events.append((cast(int, current_time_ns), current_group))
                current_time_ns = event_time_ns
                current_group = [event]
            else:
                current_group.append(event)

        if current_group:
            grouped_events.append((cast(int, current_time_ns), current_group))

        for _time_ns, group in grouped_events:
            sorted_group: list[EkfEvent] = sorted(group, key=self._event_sort_key)
            for event in sorted_group:
                output: EkfOutputs = self.process_event(event)
                if (
                    output.odom_time is not None
                    or output.world_odom_time is not None
                    or output.mag_update is not None
                    or output.apriltag_update is not None
                ):
                    outputs.append(output)

        return outputs

    def state(self) -> np.ndarray:
        return self._state.legacy_state().copy()

    def covariance(self) -> np.ndarray:
        return self._state.covariance.copy()

    def world_odom_covariance(self) -> np.ndarray:
        """
        Return the world-to-odom covariance in tangent-space coordinates
        """

        return self._world_odom_cov.copy()

    def odom_pose(self) -> Pose3:
        """
        Return the odom-to-base pose from the canonical EKF state
        """

        return self._state.pose_ob.copy()

    def world_odom_pose(self) -> Pose3:
        """
        Return the world-to-odom pose used for global alignment
        """

        return self._world_odom.copy()

    def world_pose(self) -> Pose3:
        """
        Return the world-to-base pose composed from world and odom frames
        """

        return self._compose_world_base()

    def frame_transforms(self) -> EkfFrameOutputs:
        """
        Return the current frame transforms derived from EKF state
        """

        return self._frame_outputs()

    def state_index(self) -> EkfStateIndex:
        return self._state.index.copy()

    def process_noise(self, dt_s: float) -> np.ndarray:
        if not math.isfinite(dt_s):
            raise ValueError("dt_s must be finite")
        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        return self._process_model.discrete_process_noise(self._state, dt_s=dt_s)

    def frontier_time(self) -> Optional[EkfTime]:
        if self._t_frontier_ns is None:
            return None
        return self._ekf_time_from_ns(self._t_frontier_ns)

    def _frontier_time(self) -> EkfTime:
        if self._t_frontier_ns is None:
            raise RuntimeError("Frontier time requested before initialization")
        return self._ekf_time_from_ns(self._t_frontier_ns)

    def reset(self) -> None:
        self._initialized = False
        self._calibration_initialized = False
        self._t_frontier_ns = None
        self._state.reset()
        self._state_frontier = self._state.copy()
        self._world_odom = self._identity_pose()
        self._world_odom_cov = self._build_world_odom_covariance()
        self._last_imu_time_ns = None
        self._last_imu = None
        self._imu_times_ns = []
        self._imu_gaps = []
        self._imu_gap_reject_count = 0
        self._checkpoints = []
        self._last_checkpoint_time = None

    def _event_sort_key(self, event: EkfEvent) -> int:
        return _EVENT_ORDER[event.event_type]

    def _frontier_advanced(self, prev_frontier_ns: Optional[int]) -> bool:
        if self._t_frontier_ns is None:
            return False
        if prev_frontier_ns is None:
            return True
        return self._t_frontier_ns > prev_frontier_ns

    def _set_frontier(self, t_meas: EkfTime) -> None:
        self._t_frontier_ns = to_ns(t_meas)
        self._state_frontier = self._state.copy()

    def _ekf_time_from_ns(self, t_meas_ns: int) -> EkfTime:
        sec: int
        nanosec: int
        sec, nanosec = divmod(t_meas_ns, _NS_PER_S)
        return EkfTime(sec=sec, nanosec=nanosec)

    def _maybe_checkpoint(self, t_meas: EkfTime, event_type: EkfEventType) -> None:
        t_meas_ns: int = to_ns(t_meas)
        if self._last_checkpoint_time is None:
            self._save_checkpoint(t_meas_ns)
            return
        dt_since_ns: int = t_meas_ns - self._last_checkpoint_time
        if (
            dt_since_ns >= self._config.checkpoint_interval_ns
            or event_type != EkfEventType.IMU
        ):
            self._save_checkpoint(t_meas_ns)
        self._evict_checkpoints()

    def _save_checkpoint(self, t_meas_ns: int) -> None:
        self._checkpoints.append(
            _Checkpoint(
                t_meas_ns=t_meas_ns,
                state=self._state.copy(),
                world_odom=self._world_odom.copy(),
                world_odom_cov=self._world_odom_cov.copy(),
                last_imu_time_ns=self._last_imu_time_ns,
                last_imu=self._last_imu,
                imu_times_ns=list(self._imu_times_ns),
                imu_gaps=list(self._imu_gaps),
            )
        )
        self._last_checkpoint_time = t_meas_ns

    def _evict_checkpoints(self) -> None:
        if self._t_frontier_ns is None:
            return
        cutoff_ns: int = self._t_frontier_ns - self._config.t_buffer_ns
        while self._checkpoints and self._checkpoints[0].t_meas_ns < cutoff_ns:
            self._checkpoints.pop(0)

    def _find_checkpoint(self, t_meas_ns: int) -> Optional[_Checkpoint]:
        checkpoint: Optional[_Checkpoint] = None
        for candidate in self._checkpoints:
            if candidate.t_meas_ns <= t_meas_ns:
                checkpoint = candidate
            else:
                break
        return checkpoint

    def _restore_checkpoint(self, checkpoint: _Checkpoint) -> None:
        self._state = checkpoint.state.copy()
        self._last_imu_time_ns = checkpoint.last_imu_time_ns
        self._last_imu = checkpoint.last_imu
        self._imu_times_ns = list(checkpoint.imu_times_ns)
        self._imu_gaps = list(checkpoint.imu_gaps)
        self._t_frontier_ns = checkpoint.t_meas_ns
        self._state_frontier = self._state.copy()
        self._world_odom = checkpoint.world_odom.copy()
        self._world_odom_cov = checkpoint.world_odom_cov.copy()
        self._checkpoints = [checkpoint]
        self._last_checkpoint_time = checkpoint.t_meas_ns
        self._initialized = True

    def _reset_state(self, t_meas_ns: int) -> None:
        self._state.reset()
        self._t_frontier_ns = t_meas_ns
        self._last_imu_time_ns = None
        self._last_imu = None
        self._imu_times_ns = []
        self._imu_gaps = []
        self._imu_gap_reject_count = 0
        self._state_frontier = self._state.copy()
        self._world_odom = self._identity_pose()
        self._world_odom_cov = self._build_world_odom_covariance()
        self._checkpoints = []
        self._last_checkpoint_time = None
        self._initialized = True
