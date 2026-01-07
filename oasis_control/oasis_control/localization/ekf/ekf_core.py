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
from dataclasses import dataclass
from typing import Optional
from typing import cast

import numpy as np

from oasis_control.localization.ekf.ekf_buffer import EkfBuffer
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
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import MagSample
from oasis_control.localization.ekf.models.apriltag_measurement_model import (
    AprilTagMeasurementModel,
)
from oasis_control.localization.ekf.models.imu_process_model import ImuProcessModel
from oasis_control.localization.ekf.models.mag_measurement_model import (
    MagMeasurementModel,
)


_STATE_DIM: int = 9
_EVENT_ORDER: dict[EkfEventType, int] = {
    EkfEventType.IMU: 0,
    EkfEventType.CAMERA_INFO: 1,
    EkfEventType.APRILTAG: 2,
    EkfEventType.MAG: 3,
}


@dataclass
class _Checkpoint:
    t_meas: float
    x: np.ndarray
    p: np.ndarray
    last_imu_time: Optional[float]
    last_imu: Optional[ImuSample]


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
        self._camera_info: Optional[CameraInfoData] = None
        self._t_frontier: Optional[float] = None
        self._x: np.ndarray = np.zeros(_STATE_DIM, dtype=float)
        self._p: np.ndarray = np.eye(_STATE_DIM, dtype=float)
        self._x_frontier: np.ndarray = self._x.copy()
        self._p_frontier: np.ndarray = self._p.copy()
        self._last_imu_time: Optional[float] = None
        self._last_imu: Optional[ImuSample] = None
        self._checkpoints: list[_Checkpoint] = []
        self._last_checkpoint_time: Optional[float] = None

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

            self._ensure_initialized(event.t_meas)
            self._process_imu_packet(imu_packet, event.t_meas)
            odom_time_s = event.t_meas
            world_odom_time_s = event.t_meas
        elif event.event_type == EkfEventType.MAG:
            mag_sample: MagSample = cast(MagSample, event.payload)
            self._ensure_initialized(event.t_meas)
            self._propagate_if_needed(event.t_meas)
            mag_update = self.update_with_mag(mag_sample, event.t_meas)
        elif event.event_type == EkfEventType.APRILTAG:
            apriltag_data: AprilTagDetectionArrayData = cast(
                AprilTagDetectionArrayData, event.payload
            )
            self._ensure_initialized(event.t_meas)
            self._propagate_if_needed(event.t_meas)
            apriltag_update = self.update_with_apriltags(apriltag_data, event.t_meas)
        elif event.event_type == EkfEventType.CAMERA_INFO:
            self._camera_info = cast(CameraInfoData, event.payload)
            self._apriltag_model.set_camera_info(self._camera_info)

        self._set_frontier(event.t_meas)
        self._maybe_checkpoint(event.t_meas, event.event_type)

        return EkfOutputs(
            odom_time_s=odom_time_s,
            world_odom_time_s=world_odom_time_s,
            mag_update=mag_update,
            apriltag_update=apriltag_update,
        )

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

        detections_sorted: list[AprilTagDetection] = sorted(
            apriltag_data.detections, key=lambda det: (det.family, det.tag_id)
        )
        detections: list[EkfAprilTagDetectionUpdate] = []
        for detection in detections_sorted:
            detection_update: EkfAprilTagDetectionUpdate = (
                self._update_with_apriltag_detection(
                    detection, apriltag_data.frame_id, t_meas
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

        # TODO: Initialize in-state calibration parameters with covariance
        self._initialized = True

    def is_out_of_order(self, t_meas: float) -> bool:
        if self._t_frontier is None:
            return False
        return t_meas < (self._t_frontier - 1.0e-9)

    def replay(
        self, buffer: EkfBuffer, start_time: Optional[float] = None
    ) -> EkfOutputs:
        if buffer.earliest_time() is None:
            return EkfOutputs(
                odom_time_s=None,
                world_odom_time_s=None,
                mag_update=None,
                apriltag_update=None,
            )

        earliest_time: float = cast(float, buffer.earliest_time())
        replay_start: float = start_time if start_time is not None else earliest_time
        checkpoint: Optional[_Checkpoint] = self._find_checkpoint(replay_start)
        if checkpoint is None:
            self._reset_state(replay_start)
        else:
            self._restore_checkpoint(checkpoint)

        outputs: EkfOutputs = EkfOutputs(
            odom_time_s=None,
            world_odom_time_s=None,
            mag_update=None,
            apriltag_update=None,
        )

        grouped_events: list[tuple[float, list[EkfEvent]]] = []
        current_time: Optional[float] = None
        current_group: list[EkfEvent] = []
        for event in buffer.iter_events_from(replay_start):
            if current_time is None or event.t_meas != current_time:
                if current_group:
                    grouped_events.append((cast(float, current_time), current_group))
                current_time = event.t_meas
                current_group = [event]
            else:
                current_group.append(event)

        if current_group:
            grouped_events.append((cast(float, current_time), current_group))

        for t_meas, group in grouped_events:
            sorted_group: list[EkfEvent] = sorted(group, key=self._event_sort_key)
            for event in sorted_group:
                outputs = self.process_event(event)
            self._set_frontier(t_meas)

        return outputs

    def state(self) -> np.ndarray:
        return self._x.copy()

    def covariance(self) -> np.ndarray:
        return self._p.copy()

    def frontier_time(self) -> Optional[float]:
        return self._t_frontier

    def _checkpoint_snapshot(
        self, t_meas: float
    ) -> Optional[tuple[float, np.ndarray, np.ndarray]]:
        checkpoint: Optional[_Checkpoint] = self._find_checkpoint(t_meas)
        if checkpoint is None:
            return None
        return (checkpoint.t_meas, checkpoint.x.copy(), checkpoint.p.copy())

    def build_rejected_apriltag_detection(
        self,
        detection: AprilTagDetection,
        frame_id: str,
        t_meas: float,
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

    def _zero_matrix(self, dim: int) -> EkfMatrix:
        return EkfMatrix(rows=dim, cols=dim, data=[0.0] * (dim * dim))

    def _ensure_initialized(self, t_meas: float) -> None:
        if self._initialized:
            return
        diag_values: list[float] = (
            [self._config.pos_var] * 3
            + [self._config.vel_var] * 3
            + [self._config.ang_var] * 3
        )
        self._x = np.zeros(_STATE_DIM, dtype=float)
        self._p = np.diag(diag_values)
        self._initialized = True
        self._t_frontier = t_meas
        self._x_frontier = self._x.copy()
        self._p_frontier = self._p.copy()

    def _process_imu_packet(self, imu_packet: EkfImuPacket, t_meas: float) -> None:
        imu_sample: ImuSample = imu_packet.imu
        if self._last_imu is None or self._last_imu_time is None:
            self._last_imu = imu_sample
            self._last_imu_time = t_meas
            self._set_frontier(t_meas)
            return
        imu_dt: float = t_meas - self._last_imu_time
        if imu_dt <= 0.0:
            self._last_imu = imu_sample
            self._last_imu_time = t_meas
            self._set_frontier(t_meas)
            return
        if imu_dt > self._config.dt_imu_max:
            self._last_imu = imu_sample
            self._last_imu_time = t_meas
            self._set_frontier(t_meas)
            return
        if self._t_frontier is not None:
            self._propagate_with_imu(self._last_imu, self._t_frontier, t_meas)
        self._last_imu = imu_sample
        self._last_imu_time = t_meas
        self._set_frontier(t_meas)

    def _propagate_if_needed(self, t_meas: float) -> None:
        if self._t_frontier is None:
            self._t_frontier = t_meas
            return
        if t_meas <= self._t_frontier:
            return
        if self._last_imu is None or self._last_imu_time is None:
            self._t_frontier = t_meas
            return
        dt_total: float = t_meas - self._t_frontier
        if dt_total > self._config.dt_imu_max:
            return
        self._propagate_with_imu(self._last_imu, self._t_frontier, t_meas)
        self._set_frontier(t_meas)

    def _propagate_with_imu(
        self, imu_sample: ImuSample, t_start: float, t_end: float
    ) -> None:
        dt_total: float = t_end - t_start
        if dt_total <= 0.0:
            return
        if dt_total > self._config.dt_imu_max:
            return
        max_dt: float = self._config.max_dt_sec
        steps: int = max(1, int(math.ceil(dt_total / max_dt)))
        dt: float = dt_total / float(steps)
        for _ in range(steps):
            self._integrate_state(imu_sample, dt)

    def _integrate_state(self, imu_sample: ImuSample, dt: float) -> None:
        omega_rps: np.ndarray = np.asarray(imu_sample.angular_velocity_rps, dtype=float)
        accel_body_mps2: np.ndarray = np.asarray(
            imu_sample.linear_acceleration_mps2, dtype=float
        )
        angles: np.ndarray = self._x[6:9]
        angles = angles + omega_rps * dt
        rot_world_from_body: np.ndarray = self._rotation_matrix(
            angles[0], angles[1], angles[2]
        )
        accel_world_mps2: np.ndarray = rot_world_from_body @ accel_body_mps2

        # Gravity magnitude in world frame m/s^2
        gravity_mps2: float = self._config.gravity_mps2
        accel_world_mps2 = accel_world_mps2 - np.array([0.0, 0.0, gravity_mps2])

        vel: np.ndarray = self._x[3:6] + accel_world_mps2 * dt
        pos: np.ndarray = self._x[0:3] + vel * dt
        self._x[0:3] = pos
        self._x[3:6] = vel
        self._x[6:9] = angles
        self._p = self._p + self._process_noise(dt)

    def _process_noise(self, dt: float) -> np.ndarray:
        q: np.ndarray = np.zeros((_STATE_DIM, _STATE_DIM), dtype=float)

        # Process accel variance in (m/s^2)^2
        accel_noise_var: float = self._config.accel_noise_var

        # Process gyro variance in (rad/s)^2
        gyro_noise_var: float = self._config.gyro_noise_var

        # Position variance from accel integration in m^2
        pos_noise: float = accel_noise_var * (dt**3)

        # Velocity variance from accel integration in (m/s)^2
        vel_noise: float = accel_noise_var * dt

        # Angle variance from gyro integration in rad^2
        ang_noise: float = gyro_noise_var * dt

        for index in range(3):
            q[index, index] = pos_noise
            q[index + 3, index + 3] = vel_noise
            q[index + 6, index + 6] = ang_noise
        return q

    def _rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr: float = math.cos(roll)
        sr: float = math.sin(roll)
        cp: float = math.cos(pitch)
        sp: float = math.sin(pitch)
        cy: float = math.cos(yaw)
        sy: float = math.sin(yaw)
        return np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=float,
        )

    def _update_with_apriltag_detection(
        self, detection: AprilTagDetection, frame_id: str, t_meas: float
    ) -> EkfAprilTagDetectionUpdate:
        z: Optional[np.ndarray] = self._apriltag_model.pose_measurement(detection)
        if z is None:
            return self.build_rejected_apriltag_detection(detection, frame_id, t_meas)

        z_hat: np.ndarray
        h: np.ndarray
        z_hat, h = self._apriltag_model.linearize_pose(self._x)
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
        s_hat: np.ndarray = h @ self._p @ h.T
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

        hp: np.ndarray = h @ self._p
        k_gain: np.ndarray = np.linalg.solve(s.T, hp).T

        self._x = self._x + k_gain @ residual
        identity: np.ndarray = np.eye(_STATE_DIM, dtype=float)
        temp: np.ndarray = identity - k_gain @ h
        self._p = temp @ self._p @ temp.T + k_gain @ r @ k_gain.T

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

    def _event_sort_key(self, event: EkfEvent) -> tuple[int, str]:
        return (_EVENT_ORDER[event.event_type], event.event_type.value)

    def _set_frontier(self, t_meas: float) -> None:
        self._t_frontier = t_meas
        self._x_frontier = self._x.copy()
        self._p_frontier = self._p.copy()

    def _maybe_checkpoint(self, t_meas: float, event_type: EkfEventType) -> None:
        interval: float = self._config.checkpoint_interval_sec
        if self._last_checkpoint_time is None:
            self._save_checkpoint(t_meas)
            return
        dt_since: float = t_meas - self._last_checkpoint_time
        if dt_since >= interval or event_type != EkfEventType.IMU:
            self._save_checkpoint(t_meas)
        self._evict_checkpoints()

    def _save_checkpoint(self, t_meas: float) -> None:
        self._checkpoints.append(
            _Checkpoint(
                t_meas=t_meas,
                x=self._x.copy(),
                p=self._p.copy(),
                last_imu_time=self._last_imu_time,
                last_imu=self._last_imu,
            )
        )
        self._last_checkpoint_time = t_meas

    def _evict_checkpoints(self) -> None:
        if self._t_frontier is None:
            return
        cutoff: float = self._t_frontier - self._config.t_buffer_sec
        while self._checkpoints and self._checkpoints[0].t_meas < cutoff:
            self._checkpoints.pop(0)

    def _find_checkpoint(self, t_meas: float) -> Optional[_Checkpoint]:
        checkpoint: Optional[_Checkpoint] = None
        for candidate in self._checkpoints:
            if candidate.t_meas <= t_meas:
                checkpoint = candidate
            else:
                break
        return checkpoint

    def _restore_checkpoint(self, checkpoint: _Checkpoint) -> None:
        self._x = checkpoint.x.copy()
        self._p = checkpoint.p.copy()
        self._last_imu_time = checkpoint.last_imu_time
        self._last_imu = checkpoint.last_imu
        self._t_frontier = checkpoint.t_meas
        self._x_frontier = self._x.copy()
        self._p_frontier = self._p.copy()
        self._checkpoints = [checkpoint]
        self._last_checkpoint_time = checkpoint.t_meas
        self._initialized = True

    def _reset_state(self, t_meas: float) -> None:
        diag_values: list[float] = (
            [self._config.pos_var] * 3
            + [self._config.vel_var] * 3
            + [self._config.ang_var] * 3
        )
        self._x = np.zeros(_STATE_DIM, dtype=float)
        self._p = np.diag(diag_values)
        self._t_frontier = t_meas
        self._last_imu_time = None
        self._last_imu = None
        self._x_frontier = self._x.copy()
        self._p_frontier = self._p.copy()
        self._checkpoints = []
        self._last_checkpoint_time = None
        self._initialized = True
