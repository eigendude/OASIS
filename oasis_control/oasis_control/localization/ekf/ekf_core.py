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

import logging
import math
from bisect import bisect_right
from dataclasses import dataclass
from typing import Optional
from typing import cast

import numpy as np

from oasis_control.localization.ekf.ekf_buffer import EkfBuffer
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
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfFrameOutputs
from oasis_control.localization.ekf.ekf_types import EkfFrameTransform
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfMatrix
from oasis_control.localization.ekf.ekf_types import EkfOutputs
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import EkfUpdateData
from oasis_control.localization.ekf.ekf_types import FrameId
from oasis_control.localization.ekf.ekf_types import ImuCalibrationData
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import MagSample
from oasis_control.localization.ekf.ekf_types import to_ns
from oasis_control.localization.ekf.models.apriltag_measurement_model import (
    SUPPORTED_DISTORTION_MODELS,
)
from oasis_control.localization.ekf.models.apriltag_measurement_model import (
    AprilTagMeasurementModel,
)
from oasis_control.localization.ekf.models.imu_process_model import ImuProcessModel
from oasis_control.localization.ekf.models.mag_measurement_model import (
    MagMeasurementModel,
)
from oasis_control.localization.ekf.se3 import pose_compose
from oasis_control.localization.ekf.se3 import pose_plus
from oasis_control.localization.ekf.se3 import quat_to_rpy


_EVENT_ORDER: dict[EkfEventType, int] = {
    EkfEventType.IMU: 0,
    EkfEventType.CAMERA_INFO: 1,
    EkfEventType.MAG: 2,
    EkfEventType.APRILTAG: 3,
}

# Nanoseconds per second for time conversions
_NS_PER_S: int = 1_000_000_000

_LOG: logging.Logger = logging.getLogger(__name__)


@dataclass
class _Checkpoint:
    t_meas_ns: int
    state: EkfState
    world_odom: Pose3
    world_odom_cov: np.ndarray
    last_imu_time_ns: Optional[int]
    last_imu: Optional[ImuSample]
    imu_times_ns: list[int]
    imu_gaps: list["_ImuGap"]


@dataclass
class _StateSnapshot:
    initialized: bool
    t_frontier_ns: Optional[int]
    state: EkfState
    state_frontier: EkfState
    world_odom: Pose3
    world_odom_cov: np.ndarray
    last_imu_time_ns: Optional[int]
    last_imu: Optional[ImuSample]
    imu_times_ns: list[int]
    imu_gaps: list["_ImuGap"]
    checkpoints: list[_Checkpoint]
    last_checkpoint_time: Optional[int]


@dataclass(frozen=True)
class _ImuGap:
    start_ns: int
    end_ns: int
    dt_ns: int


@dataclass(frozen=True)
class _ImuCoverageCheck:
    covered: bool
    reason: str
    max_gap_ns: int
    gap_start_ns: Optional[int]
    gap_end_ns: Optional[int]


class EkfCore:
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

    def update_with_mag(self, mag_sample: MagSample, t_meas: EkfTime) -> EkfUpdateData:
        """
        Apply the magnetometer update model
        """

        z_dim: int = 3
        z: np.ndarray = np.asarray(mag_sample.magnetic_field_t, dtype=float)
        r: np.ndarray = np.asarray(mag_sample.magnetic_field_cov, dtype=float).reshape(
            (z_dim, z_dim)
        )
        z_hat: np.ndarray
        h: np.ndarray
        state_legacy: np.ndarray = self._state.legacy_state()
        z_hat, h = self._mag_model.linearize(state_legacy)
        h = self._state.lift_legacy_jacobian(h)
        residual: np.ndarray = z - z_hat
        s_hat: np.ndarray = h @ self._state.covariance @ h.T
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
                mag_reject_update: EkfUpdateData = EkfUpdateData(
                    sensor="magnetic_field",
                    frame_id=mag_sample.frame_id,
                    t_meas=t_meas,
                    accepted=False,
                    reject_reason="singular S",
                    z_dim=z_dim,
                    z=z.tolist(),
                    z_hat=z_hat.tolist(),
                    nu=residual.tolist(),
                    r=EkfMatrix(rows=z_dim, cols=z_dim, data=r.flatten().tolist()),
                    s_hat=EkfMatrix(
                        rows=z_dim, cols=z_dim, data=s_hat.flatten().tolist()
                    ),
                    s=EkfMatrix(rows=z_dim, cols=z_dim, data=s.flatten().tolist()),
                    maha_d2=0.0,
                    gate_d2_threshold=0.0,
                    reproj_rms_px=0.0,
                )
                return mag_reject_update

        y: np.ndarray = np.linalg.solve(l, residual)
        x: np.ndarray = np.linalg.solve(l.T, y)
        maha_d2: float = float(residual.T @ x)
        gate_d2_threshold: float = 0.0

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

        mag_accept_update: EkfUpdateData = EkfUpdateData(
            sensor="magnetic_field",
            frame_id=mag_sample.frame_id,
            t_meas=t_meas,
            accepted=True,
            reject_reason="",
            z_dim=z_dim,
            z=z.tolist(),
            z_hat=z_hat.tolist(),
            nu=residual.tolist(),
            r=EkfMatrix(rows=z_dim, cols=z_dim, data=r.flatten().tolist()),
            s_hat=EkfMatrix(rows=z_dim, cols=z_dim, data=s_hat.flatten().tolist()),
            s=EkfMatrix(rows=z_dim, cols=z_dim, data=s.flatten().tolist()),
            maha_d2=maha_d2,
            gate_d2_threshold=gate_d2_threshold,
            reproj_rms_px=0.0,
        )

        return mag_accept_update

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

    def _checkpoint_snapshot(
        self, t_meas: EkfTime
    ) -> Optional[tuple[EkfTime, EkfState]]:
        t_meas_ns: int = to_ns(t_meas)
        checkpoint: Optional[_Checkpoint] = self._find_checkpoint(t_meas_ns)
        if checkpoint is None:
            return None
        checkpoint_time: EkfTime = self._ekf_time_from_ns(checkpoint.t_meas_ns)
        return (checkpoint_time, checkpoint.state.copy())

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
        z_hat_values: list[float] = [] if z_hat is None else z_hat.tolist()
        nu_values: list[float] = [] if residual is None else residual.tolist()
        z_dim: int = len(z_values) if z_values else 4
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

    def _build_rejected_mag_update(
        self, mag_sample: MagSample, t_meas: EkfTime, *, reject_reason: str
    ) -> EkfUpdateData:
        z_dim: int = 3
        zero_matrix: EkfMatrix = EkfMatrix(rows=z_dim, cols=z_dim, data=[0.0] * 9)
        r: EkfMatrix = EkfMatrix(
            rows=z_dim, cols=z_dim, data=list(mag_sample.magnetic_field_cov)
        )
        return EkfUpdateData(
            sensor="magnetic_field",
            frame_id=mag_sample.frame_id,
            t_meas=t_meas,
            accepted=False,
            reject_reason=reject_reason,
            z_dim=z_dim,
            z=list(mag_sample.magnetic_field_t),
            z_hat=[],
            nu=[],
            r=r,
            s_hat=zero_matrix,
            s=zero_matrix,
            maha_d2=0.0,
            gate_d2_threshold=0.0,
            reproj_rms_px=0.0,
        )

    def _zero_matrix(self, dim: int) -> EkfMatrix:
        return EkfMatrix(rows=dim, cols=dim, data=[0.0] * (dim * dim))

    def _snapshot_state(self) -> _StateSnapshot:
        return _StateSnapshot(
            initialized=self._initialized,
            t_frontier_ns=self._t_frontier_ns,
            state=self._state.copy(),
            state_frontier=self._state_frontier.copy(),
            world_odom=self._world_odom.copy(),
            world_odom_cov=self._world_odom_cov.copy(),
            last_imu_time_ns=self._last_imu_time_ns,
            last_imu=self._last_imu,
            imu_times_ns=list(self._imu_times_ns),
            imu_gaps=list(self._imu_gaps),
            checkpoints=list(self._checkpoints),
            last_checkpoint_time=self._last_checkpoint_time,
        )

    def _restore_snapshot(self, snapshot: _StateSnapshot) -> None:
        self._initialized = snapshot.initialized
        self._t_frontier_ns = snapshot.t_frontier_ns
        self._state = snapshot.state.copy()
        self._state_frontier = snapshot.state_frontier.copy()
        self._world_odom = snapshot.world_odom.copy()
        self._world_odom_cov = snapshot.world_odom_cov.copy()
        self._last_imu_time_ns = snapshot.last_imu_time_ns
        self._last_imu = snapshot.last_imu
        self._imu_times_ns = list(snapshot.imu_times_ns)
        self._imu_gaps = list(snapshot.imu_gaps)
        self._checkpoints = list(snapshot.checkpoints)
        self._last_checkpoint_time = snapshot.last_checkpoint_time

    def _ensure_initialized(self) -> None:
        if self._initialized:
            return
        self._state.reset()
        self._initialized = True
        self._state_frontier = self._state.copy()
        self._world_odom = self._identity_pose()
        self._world_odom_cov = self._build_world_odom_covariance()

    def _record_imu_time(self, t_meas_ns: int) -> None:
        insert_index: int = bisect_right(self._imu_times_ns, t_meas_ns)
        self._imu_times_ns.insert(insert_index, t_meas_ns)

    def _record_imu_gap(self, start_ns: int, end_ns: int, dt_ns: int) -> None:
        self._imu_gaps.append(_ImuGap(start_ns=start_ns, end_ns=end_ns, dt_ns=dt_ns))
        _LOG.warning(
            "IMU gap detected start_ns=%d end_ns=%d dt_ns=%d max_ns=%d",
            start_ns,
            end_ns,
            dt_ns,
            self._config.dt_imu_max_ns,
        )

    def _record_imu_coverage_failure(
        self, t_start_ns: int, t_end_ns: int, coverage: _ImuCoverageCheck
    ) -> None:
        self._imu_gap_reject_count += 1
        _LOG.warning(
            (
                "IMU coverage failed start_ns=%d end_ns=%d max_gap_ns=%d "
                "gap_start_ns=%s gap_end_ns=%s reason=%s count=%d"
            ),
            t_start_ns,
            t_end_ns,
            coverage.max_gap_ns,
            coverage.gap_start_ns,
            coverage.gap_end_ns,
            coverage.reason,
            self._imu_gap_reject_count,
        )

    def _ensure_imu_coverage(
        self, t_start_ns: int, t_end_ns: int
    ) -> _ImuCoverageCheck:
        if t_end_ns <= t_start_ns:
            return _ImuCoverageCheck(True, "", 0, None, None)
        if not self._imu_times_ns:
            return _ImuCoverageCheck(
                False, "missing_imu", 0, t_start_ns, t_end_ns
            )
        start_index: int = bisect_right(self._imu_times_ns, t_start_ns) - 1
        if start_index < 0:
            return _ImuCoverageCheck(
                False, "missing_imu_before_start", 0, t_start_ns, t_end_ns
            )
        prev_time_ns: int = self._imu_times_ns[start_index]
        lead_gap_ns: int = t_start_ns - prev_time_ns
        if lead_gap_ns > self._config.dt_imu_max_ns:
            return _ImuCoverageCheck(
                False, "imu_gap", lead_gap_ns, prev_time_ns, t_start_ns
            )
        max_gap_ns: int = lead_gap_ns
        gap_start_ns: Optional[int] = prev_time_ns
        gap_end_ns: Optional[int] = t_start_ns
        next_index: int = start_index + 1
        while (
            next_index < len(self._imu_times_ns)
            and self._imu_times_ns[next_index] <= t_end_ns
        ):
            next_time_ns: int = self._imu_times_ns[next_index]
            gap_ns: int = next_time_ns - prev_time_ns
            if gap_ns > max_gap_ns:
                max_gap_ns = gap_ns
                gap_start_ns = prev_time_ns
                gap_end_ns = next_time_ns
            if gap_ns > self._config.dt_imu_max_ns:
                return _ImuCoverageCheck(
                    False, "imu_gap", gap_ns, prev_time_ns, next_time_ns
                )
            prev_time_ns = next_time_ns
            next_index += 1
        tail_gap_ns: int = t_end_ns - prev_time_ns
        if tail_gap_ns > max_gap_ns:
            max_gap_ns = tail_gap_ns
            gap_start_ns = prev_time_ns
            gap_end_ns = t_end_ns
        if tail_gap_ns > self._config.dt_imu_max_ns:
            return _ImuCoverageCheck(
                False, "imu_gap", tail_gap_ns, prev_time_ns, t_end_ns
            )
        return _ImuCoverageCheck(
            True,
            "",
            max_gap_ns,
            gap_start_ns,
            gap_end_ns,
        )

    def _process_imu_packet(
        self,
        imu_packet: EkfImuPacket,
        t_meas_ns: int,
    ) -> None:
        imu_sample: ImuSample = imu_packet.imu
        self._record_imu_time(t_meas_ns)
        if self._last_imu is None or self._last_imu_time_ns is None:
            self._last_imu = imu_sample
            self._last_imu_time_ns = t_meas_ns
            return
        imu_dt_ns: int = t_meas_ns - self._last_imu_time_ns
        if imu_dt_ns <= 0:
            self._last_imu = imu_sample
            self._last_imu_time_ns = t_meas_ns
            return
        if imu_dt_ns > self._config.dt_imu_max_ns:
            self._record_imu_gap(self._last_imu_time_ns, t_meas_ns, imu_dt_ns)
            self._last_imu = imu_sample
            self._last_imu_time_ns = t_meas_ns
            return
        if self._t_frontier_ns is not None:
            self._propagate_with_imu(self._last_imu, self._t_frontier_ns, t_meas_ns)
        self._last_imu = imu_sample
        self._last_imu_time_ns = t_meas_ns

    def _propagate_if_needed(self, t_meas_ns: int) -> _ImuCoverageCheck:
        if self._t_frontier_ns is None:
            return _ImuCoverageCheck(True, "", 0, None, None)
        if t_meas_ns <= self._t_frontier_ns:
            return _ImuCoverageCheck(True, "", 0, None, None)
        coverage: _ImuCoverageCheck = self._ensure_imu_coverage(
            self._t_frontier_ns, t_meas_ns
        )
        if not coverage.covered:
            self._record_imu_coverage_failure(
                self._t_frontier_ns, t_meas_ns, coverage
            )
            return coverage
        if self._last_imu is None or self._last_imu_time_ns is None:
            coverage = _ImuCoverageCheck(
                False, "missing_imu", 0, self._t_frontier_ns, t_meas_ns
            )
            self._record_imu_coverage_failure(
                self._t_frontier_ns, t_meas_ns, coverage
            )
            return coverage
        self._propagate_with_imu(self._last_imu, self._t_frontier_ns, t_meas_ns)
        return coverage

    def _propagate_with_imu(
        self, imu_sample: ImuSample, t_start_ns: int, t_end_ns: int
    ) -> None:
        dt_total_ns: int = t_end_ns - t_start_ns
        if dt_total_ns <= 0:
            return
        if dt_total_ns > self._config.dt_imu_max_ns:
            return
        dt_total_s: float = float(dt_total_ns) * 1.0e-9
        omega_raw_rps: np.ndarray = np.asarray(
            imu_sample.angular_velocity_rps, dtype=float
        )
        accel_raw_mps2: np.ndarray = np.asarray(
            imu_sample.linear_acceleration_mps2, dtype=float
        )
        try:
            self._process_model.propagate_nominal_substepped(
                self._state,
                omega_raw_rps=omega_raw_rps,
                accel_raw_mps2=accel_raw_mps2,
                dt_s=dt_total_s,
                max_dt_s=self._config.max_dt_sec,
            )
            # Sub-step nominal propagation for numerical stability. The v0
            # process noise Q uses closed-form coefficients for dt_total_s and
            # does not accumulate per-substep values. Without Phi/G weighting,
            # summing per-step Q is not equivalent to the closed-form dt^4 / 4,
            # dt^3 / 2, dt^2 terms
            #
            # TODO(ekf-v1): add Phi/G and integrate Q via
            # Q = ∫ Phi(τ) G Qc Gᵀ Phi(τ)ᵀ dτ
            # across substeps
            q = self._process_model.discrete_process_noise(self._state, dt_s=dt_total_s)
        except ValueError as exc:
            _LOG.info("Skipping IMU propagation, %s", exc)
            return
        self._state.covariance = self._state.covariance + q

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

    def _wrap_angle(self, angle: float) -> float:
        wrapped: float = (angle + math.pi) % (2.0 * math.pi) - math.pi
        return wrapped

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

    def _identity_pose(self) -> Pose3:
        return Pose3(
            translation_m=np.zeros(3, dtype=float),
            rotation_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        )

    def _build_world_odom_covariance(self) -> np.ndarray:
        cov: np.ndarray = np.zeros((6, 6), dtype=float)
        cov[0:3, 0:3] = np.eye(3, dtype=float) * self._config.pos_var
        cov[3:6, 3:6] = np.eye(3, dtype=float) * self._config.ang_var
        return cov

    def _frame_outputs(self) -> EkfFrameOutputs:
        t_odom_base: EkfFrameTransform = self._pose_to_transform(
            self._state.pose_ob,
            parent_frame="odom",
            child_frame="base",
        )
        t_world_odom: EkfFrameTransform = self._pose_to_transform(
            self._world_odom,
            parent_frame="world",
            child_frame="odom",
        )
        t_world_base_pose: Pose3 = self._compose_world_base()
        t_world_base: EkfFrameTransform = self._pose_to_transform(
            t_world_base_pose,
            parent_frame="world",
            child_frame="base",
        )
        return EkfFrameOutputs(
            t_odom_base=t_odom_base,
            t_world_odom=t_world_odom,
            t_world_base=t_world_base,
        )

    def _pose_to_transform(
        self, pose: Pose3, *, parent_frame: FrameId, child_frame: FrameId
    ) -> EkfFrameTransform:
        return EkfFrameTransform(
            parent_frame=parent_frame,
            child_frame=child_frame,
            translation_m=pose.translation_m.tolist(),
            rotation_wxyz=pose.rotation_wxyz.tolist(),
        )

    def _world_base_legacy_state(self) -> np.ndarray:
        world_base: Pose3 = self._compose_world_base()
        rpy: np.ndarray = quat_to_rpy(world_base.rotation_wxyz)
        zero_velocity: np.ndarray = np.zeros(3, dtype=float)
        return np.concatenate(
            (
                world_base.translation_m,
                zero_velocity,
                rpy,
            ),
            axis=0,
        )

    def _compose_world_base(self) -> Pose3:
        t_world_base: np.ndarray
        q_world_base: np.ndarray
        t_world_base, q_world_base = pose_compose(
            self._world_odom.translation_m,
            self._world_odom.rotation_wxyz,
            self._state.pose_ob.translation_m,
            self._state.pose_ob.rotation_wxyz,
        )
        return Pose3(translation_m=t_world_base, rotation_wxyz=q_world_base)

    def _lift_world_odom_jacobian(self, legacy_h: np.ndarray) -> np.ndarray:
        if legacy_h.shape[1] != 9:
            raise ValueError("Expected legacy Jacobian with 9 columns")
        lifted: np.ndarray = np.zeros((legacy_h.shape[0], 6), dtype=float)
        lifted[:, 0:3] = legacy_h[:, 0:3]
        lifted[:, 3:6] = legacy_h[:, 6:9]
        return lifted
