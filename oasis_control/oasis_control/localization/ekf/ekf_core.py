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
from oasis_control.localization.ekf.ekf_types import CameraInfoData
from oasis_control.localization.ekf.ekf_types import EkfAprilTagDetectionUpdate
from oasis_control.localization.ekf.ekf_types import EkfAprilTagUpdateData
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfMatrix
from oasis_control.localization.ekf.ekf_types import EkfOutputs
from oasis_control.localization.ekf.ekf_types import EkfPose
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import EkfUpdateData
from oasis_control.localization.ekf.ekf_types import EventAprilTagPose
from oasis_control.localization.ekf.ekf_types import ImuCalibrationData
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import MagSample
from oasis_control.localization.ekf.ekf_types import from_seconds
from oasis_control.localization.ekf.ekf_types import to_ns
from oasis_control.localization.ekf.ekf_types import to_seconds
from oasis_control.localization.ekf.models.imu_process_model import ImuProcessModel
from oasis_control.localization.ekf.models.mag_measurement_model import (
    MagMeasurementModel,
)
from oasis_control.localization.ekf.pose_math import normalize_quaternion
from oasis_control.localization.ekf.pose_math import quaternion_from_rpy
from oasis_control.localization.ekf.pose_math import quaternion_inverse
from oasis_control.localization.ekf.pose_math import quaternion_log
from oasis_control.localization.ekf.pose_math import quaternion_multiply
from oasis_control.localization.ekf.pose_math import rotate_vector
from oasis_control.localization.ekf.pose_math import rpy_from_quaternion


_STATE_DIM: int = 9
_EVENT_ORDER: dict[EkfEventType, int] = {
    EkfEventType.IMU: 0,
    EkfEventType.CAMERA_INFO: 1,
    EkfEventType.MAG: 2,
    EkfEventType.APRILTAG: 3,
}


@dataclass
class _Checkpoint:
    t_meas: float
    x: np.ndarray
    p: np.ndarray
    last_imu_time: Optional[float]
    last_imu: Optional[ImuSample]
    world_odom_p: np.ndarray
    world_odom_q: np.ndarray


@dataclass
class _StateSnapshot:
    initialized: bool
    t_frontier: Optional[float]
    x: np.ndarray
    p: np.ndarray
    x_frontier: np.ndarray
    p_frontier: np.ndarray
    last_imu_time: Optional[float]
    last_imu: Optional[ImuSample]
    checkpoints: list[_Checkpoint]
    last_checkpoint_time: Optional[float]
    world_odom_p: np.ndarray
    world_odom_q: np.ndarray


class EkfCore:
    """
    Minimal EKF state manager with placeholder updates
    """

    def __init__(self, config: EkfConfig) -> None:
        self._config: EkfConfig = config
        self._process_model: ImuProcessModel = ImuProcessModel()
        self._mag_model: MagMeasurementModel = MagMeasurementModel()
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
        self._world_odom_p: np.ndarray = np.zeros(3, dtype=float)
        self._world_odom_q: np.ndarray = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

    def process_event(self, event: EkfEvent) -> EkfOutputs:
        t_meas_s: float = to_seconds(event.t_meas)
        prev_frontier: Optional[float] = self._t_frontier
        odom_time_s: Optional[float] = None
        world_odom_time_s: Optional[float] = None
        mag_update: Optional[EkfUpdateData] = None
        apriltag_update: Optional[EkfAprilTagUpdateData] = None
        advance_frontier: bool = True
        world_odom_updated: bool = False
        warnings: list[str] = []

        if event.event_type == EkfEventType.IMU:
            imu_packet: EkfImuPacket = cast(EkfImuPacket, event.payload)
            if imu_packet.calibration is not None:
                if imu_packet.calibration.valid and not self._calibration_initialized:
                    self.initialize_from_calibration(imu_packet.calibration)
                    self._calibration_initialized = True

            self._ensure_initialized(t_meas_s)
            imu_warning: Optional[str] = self._process_imu_packet(imu_packet, t_meas_s)
            if imu_warning is not None:
                warnings.append(imu_warning)
        elif event.event_type == EkfEventType.MAG:
            mag_sample: MagSample = cast(MagSample, event.payload)
            self._ensure_initialized(t_meas_s)
            mag_warning: Optional[str] = self._propagate_if_needed(t_meas_s)
            if mag_warning is not None:
                warnings.append(mag_warning)
            mag_update = self.update_with_mag(mag_sample, t_meas_s)
        elif event.event_type == EkfEventType.APRILTAG:
            apriltag_pose: EventAprilTagPose = cast(EventAprilTagPose, event.payload)
            snapshot: _StateSnapshot = self._snapshot_state()
            self._ensure_initialized(t_meas_s)
            apriltag_warning: Optional[str] = self._propagate_if_needed(t_meas_s)
            if apriltag_warning is not None:
                warnings.append(apriltag_warning)
            apriltag_update = self.update_with_apriltag_pose(apriltag_pose, t_meas_s)
            apriltag_accepted: bool = apriltag_update.detections[0].update.accepted
            world_odom_updated = apriltag_accepted
            if not apriltag_accepted:
                self._restore_snapshot(snapshot)
                advance_frontier = False
        elif event.event_type == EkfEventType.CAMERA_INFO:
            self._camera_info = cast(CameraInfoData, event.payload)
            advance_frontier = False

        if advance_frontier:
            self._set_frontier(t_meas_s)
            self._maybe_checkpoint(t_meas_s, event.event_type)

        if self._frontier_advanced(prev_frontier):
            odom_time_s = self._t_frontier
        if world_odom_updated:
            world_odom_time_s = self._t_frontier

        return EkfOutputs(
            odom_time_s=odom_time_s,
            world_odom_time_s=world_odom_time_s,
            mag_update=mag_update,
            apriltag_update=apriltag_update,
            warnings=warnings,
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

    def update_with_apriltag_pose(
        self, apriltag_pose: EventAprilTagPose, t_meas: float
    ) -> EkfAprilTagUpdateData:
        """
        Apply the AprilTag pose update model
        """

        detection_update: EkfAprilTagDetectionUpdate = self._update_with_apriltag_pose(
            apriltag_pose, t_meas
        )
        return EkfAprilTagUpdateData(
            t_meas=t_meas,
            frame_id=apriltag_pose.frame_id,
            detections=[detection_update],
        )

    def initialize_from_calibration(self, calibration: ImuCalibrationData) -> None:
        """
        Initialize calibration parameters from a one-shot prior
        """

        # TODO: Initialize in-state calibration parameters with covariance
        self._initialized = True

    def is_out_of_order(self, t_meas: EkfTime) -> bool:
        if self._t_frontier is None:
            return False
        t_meas_ns: int = to_ns(t_meas)
        frontier_ns: int = to_ns(from_seconds(self._t_frontier))
        return t_meas_ns < frontier_ns

    def replay(
        self, buffer: EkfBuffer, start_time: Optional[EkfTime] = None
    ) -> list[EkfOutputs]:
        if buffer.earliest_time() is None:
            return []

        earliest_time_ns: int = cast(int, buffer.earliest_time())
        replay_start_ns: int = (
            to_ns(start_time) if start_time is not None else earliest_time_ns
        )
        replay_start_s: float = float(replay_start_ns) * 1.0e-9
        checkpoint: Optional[_Checkpoint] = self._find_checkpoint(replay_start_s)
        if checkpoint is None:
            self._reset_state(replay_start_s)
        else:
            self._restore_checkpoint(checkpoint)

        outputs_list: list[EkfOutputs] = []

        grouped_events: list[tuple[int, list[EkfEvent]]] = []
        current_time_ns: Optional[int] = None
        current_group: list[EkfEvent] = []
        for event in buffer.iter_events_from(replay_start_ns):
            event_time_ns: int = to_ns(event.t_meas)
            if current_time_ns is None or event_time_ns != current_time_ns:
                if current_group:
                    grouped_events.append(
                        (cast(int, current_time_ns), current_group)
                    )
                current_time_ns = event_time_ns
                current_group = [event]
            else:
                current_group.append(event)

        if current_group:
            grouped_events.append((cast(int, current_time_ns), current_group))

        for _, group in grouped_events:
            sorted_group: list[EkfEvent] = sorted(group, key=self._event_sort_key)
            for event in sorted_group:
                outputs: EkfOutputs = self.process_event(event)
                if (
                    outputs.odom_time_s is not None
                    or outputs.world_odom_time_s is not None
                    or outputs.mag_update is not None
                    or outputs.apriltag_update is not None
                    or outputs.warnings
                ):
                    outputs_list.append(outputs)

        return outputs_list

    def state(self) -> np.ndarray:
        return self._x.copy()

    def covariance(self) -> np.ndarray:
        return self._p.copy()

    def world_base_pose(self) -> EkfPose:
        position: list[float] = self._x[0:3].tolist()
        quat: tuple[float, float, float, float] = quaternion_from_rpy(
            float(self._x[6]),
            float(self._x[7]),
            float(self._x[8]),
        )
        quat_list: list[float] = list(normalize_quaternion(quat))
        return EkfPose(position_m=position, orientation_xyzw=quat_list)

    def odom_base_pose(self) -> EkfPose:
        world_base: EkfPose = self.world_base_pose()
        p_world_base: np.ndarray = np.asarray(world_base.position_m, dtype=float)
        q_world_base: np.ndarray = np.asarray(world_base.orientation_xyzw, dtype=float)
        p_odom_world: np.ndarray
        q_odom_world: np.ndarray
        p_odom_world, q_odom_world = self._transform_inverse(
            self._world_odom_p, self._world_odom_q
        )
        odom_base_p: np.ndarray
        odom_base_q: np.ndarray
        odom_base_p, odom_base_q = self._transform_multiply(
            p_odom_world, q_odom_world, p_world_base, q_world_base
        )
        return EkfPose(
            position_m=odom_base_p.tolist(),
            orientation_xyzw=odom_base_q.tolist(),
        )

    def world_odom_pose(self) -> EkfPose:
        return EkfPose(
            position_m=self._world_odom_p.tolist(),
            orientation_xyzw=self._world_odom_q.tolist(),
        )

    def frontier_time(self) -> Optional[float]:
        return self._t_frontier

    def set_state(
        self,
        *,
        position_m: list[float],
        velocity_mps: list[float],
        rpy_rad: list[float],
        covariance_diag: Optional[list[float]] = None,
        t_meas: Optional[float] = None,
    ) -> None:
        """
        Override the EKF state for testing or reset workflows
        """

        x: np.ndarray = np.zeros(_STATE_DIM, dtype=float)
        x[0:3] = np.asarray(position_m, dtype=float)
        x[3:6] = np.asarray(velocity_mps, dtype=float)
        x[6:9] = np.asarray(rpy_rad, dtype=float)
        self._x = x
        if covariance_diag is not None:
            self._p = np.diag(np.asarray(covariance_diag, dtype=float))
        if t_meas is not None:
            self._t_frontier = t_meas
            self._x_frontier = self._x.copy()
            self._p_frontier = self._p.copy()
        self._initialized = True

    def reset(self) -> None:
        self._initialized = False
        self._calibration_initialized = False
        self._t_frontier = None
        self._x = np.zeros(_STATE_DIM, dtype=float)
        self._p = np.eye(_STATE_DIM, dtype=float)
        self._x_frontier = self._x.copy()
        self._p_frontier = self._p.copy()
        self._last_imu_time = None
        self._last_imu = None
        self._checkpoints = []
        self._last_checkpoint_time = None
        self._world_odom_p = np.zeros(3, dtype=float)
        self._world_odom_q = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

    def _checkpoint_snapshot(
        self, t_meas: EkfTime
    ) -> Optional[tuple[float, np.ndarray, np.ndarray]]:
        t_meas_s: float = to_seconds(t_meas)
        checkpoint: Optional[_Checkpoint] = self._find_checkpoint(t_meas_s)
        if checkpoint is None:
            return None
        return (checkpoint.t_meas, checkpoint.x.copy(), checkpoint.p.copy())

    def build_rejected_apriltag_pose(
        self,
        apriltag_pose: EventAprilTagPose,
        t_meas: float,
        *,
        reject_reason: str,
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
        z_dim: int = len(z_values) if z_values else 6
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
            frame_id=apriltag_pose.frame_id,
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
            family=apriltag_pose.family,
            tag_id=apriltag_pose.tag_id,
            det_index_in_msg=apriltag_pose.det_index_in_msg,
            update=update,
        )

    def build_rejected_apriltag_update(
        self,
        apriltag_pose: EventAprilTagPose,
        t_meas: float,
        *,
        reject_reason: str,
    ) -> EkfAprilTagUpdateData:
        detection_update: EkfAprilTagDetectionUpdate = (
            self.build_rejected_apriltag_pose(
                apriltag_pose,
                t_meas,
                reject_reason=reject_reason,
            )
        )
        return EkfAprilTagUpdateData(
            t_meas=t_meas,
            frame_id=apriltag_pose.frame_id,
            detections=[detection_update],
        )

    def _zero_matrix(self, dim: int) -> EkfMatrix:
        return EkfMatrix(rows=dim, cols=dim, data=[0.0] * (dim * dim))

    def _snapshot_state(self) -> _StateSnapshot:
        return _StateSnapshot(
            initialized=self._initialized,
            t_frontier=self._t_frontier,
            x=self._x.copy(),
            p=self._p.copy(),
            x_frontier=self._x_frontier.copy(),
            p_frontier=self._p_frontier.copy(),
            last_imu_time=self._last_imu_time,
            last_imu=self._last_imu,
            checkpoints=list(self._checkpoints),
            last_checkpoint_time=self._last_checkpoint_time,
            world_odom_p=self._world_odom_p.copy(),
            world_odom_q=self._world_odom_q.copy(),
        )

    def _restore_snapshot(self, snapshot: _StateSnapshot) -> None:
        self._initialized = snapshot.initialized
        self._t_frontier = snapshot.t_frontier
        self._x = snapshot.x.copy()
        self._p = snapshot.p.copy()
        self._x_frontier = snapshot.x_frontier.copy()
        self._p_frontier = snapshot.p_frontier.copy()
        self._last_imu_time = snapshot.last_imu_time
        self._last_imu = snapshot.last_imu
        self._checkpoints = list(snapshot.checkpoints)
        self._last_checkpoint_time = snapshot.last_checkpoint_time
        self._world_odom_p = snapshot.world_odom_p.copy()
        self._world_odom_q = snapshot.world_odom_q.copy()

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
        self._world_odom_p = np.zeros(3, dtype=float)
        self._world_odom_q = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self._world_odom_p = np.zeros(3, dtype=float)
        self._world_odom_q = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

    def _process_imu_packet(
        self, imu_packet: EkfImuPacket, t_meas: float
    ) -> Optional[str]:
        imu_sample: ImuSample = imu_packet.imu
        if self._last_imu is None or self._last_imu_time is None:
            self._last_imu = imu_sample
            self._last_imu_time = t_meas
            self._set_frontier(t_meas)
            return None
        imu_dt: float = t_meas - self._last_imu_time
        if imu_dt <= 0.0:
            self._last_imu = imu_sample
            self._last_imu_time = t_meas
            self._set_frontier(t_meas)
            return None
        if imu_dt > self._config.dt_imu_max:
            # Advance time without propagation to keep the frontier consistent
            warning: str = "skipped_propagation_dt_too_large"
            self._last_imu = imu_sample
            self._last_imu_time = t_meas
            self._set_frontier(t_meas)
            return warning
        if self._t_frontier is not None:
            self._propagate_with_imu(self._last_imu, self._t_frontier, t_meas)
        self._last_imu = imu_sample
        self._last_imu_time = t_meas
        self._set_frontier(t_meas)
        return None

    def _propagate_if_needed(self, t_meas: float) -> Optional[str]:
        if self._t_frontier is None:
            self._set_frontier(t_meas)
            return None
        if t_meas <= self._t_frontier:
            return None
        if self._last_imu is None or self._last_imu_time is None:
            self._set_frontier(t_meas)
            return None
        dt_total: float = t_meas - self._t_frontier
        if dt_total > self._config.dt_imu_max:
            return "skipped_propagation_dt_too_large"
        self._propagate_with_imu(self._last_imu, self._t_frontier, t_meas)
        self._set_frontier(t_meas)
        return None

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

    def _state_world_base_pose(self) -> tuple[np.ndarray, np.ndarray]:
        position: np.ndarray = self._x[0:3].copy()
        quat: tuple[float, float, float, float] = quaternion_from_rpy(
            float(self._x[6]),
            float(self._x[7]),
            float(self._x[8]),
        )
        quat_array: np.ndarray = np.asarray(normalize_quaternion(quat), dtype=float)
        return position, quat_array

    def _odom_base_from_state(self) -> tuple[np.ndarray, np.ndarray]:
        p_world_base: np.ndarray
        q_world_base: np.ndarray
        p_world_base, q_world_base = self._state_world_base_pose()
        p_odom_world: np.ndarray
        q_odom_world: np.ndarray
        p_odom_world, q_odom_world = self._transform_inverse(
            self._world_odom_p, self._world_odom_q
        )
        return self._transform_multiply(
            p_odom_world, q_odom_world, p_world_base, q_world_base
        )

    def _update_world_odom_from_odom_base(
        self, p_odom_base: np.ndarray, q_odom_base: np.ndarray
    ) -> None:
        # Keep odom->base_link continuous by solving for world->odom so that
        # T_world_odom * T_odom_base_pred == T_world_base_updated
        p_world_base: np.ndarray
        q_world_base: np.ndarray
        p_world_base, q_world_base = self._state_world_base_pose()
        p_base_odom: np.ndarray
        q_base_odom: np.ndarray
        p_base_odom, q_base_odom = self._transform_inverse(p_odom_base, q_odom_base)
        p_world_odom: np.ndarray
        q_world_odom: np.ndarray
        p_world_odom, q_world_odom = self._transform_multiply(
            p_world_base, q_world_base, p_base_odom, q_base_odom
        )
        self._world_odom_p = p_world_odom
        self._world_odom_q = q_world_odom

    def _transform_multiply(
        self,
        p_ab: np.ndarray,
        q_ab: np.ndarray,
        p_bc: np.ndarray,
        q_bc: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        rotated: tuple[float, float, float] = rotate_vector(q_ab, p_bc)
        p_ac: np.ndarray = p_ab + np.asarray(rotated, dtype=float)
        q_ac: tuple[float, float, float, float] = quaternion_multiply(q_ab, q_bc)
        return p_ac, np.asarray(normalize_quaternion(q_ac), dtype=float)

    def _transform_inverse(
        self, p_ab: np.ndarray, q_ab: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        q_ba: tuple[float, float, float, float] = quaternion_inverse(q_ab)
        rotated: tuple[float, float, float] = rotate_vector(q_ba, -p_ab)
        p_ba: np.ndarray = np.asarray(rotated, dtype=float)
        return p_ba, np.asarray(normalize_quaternion(q_ba), dtype=float)

    def _update_with_apriltag_pose(
        self, apriltag_pose: EventAprilTagPose, t_meas: float
    ) -> EkfAprilTagDetectionUpdate:
        position_meas: np.ndarray = np.asarray(
            apriltag_pose.p_meas_world_base_m, dtype=float
        )
        quat_meas: tuple[float, float, float, float] = normalize_quaternion(
            apriltag_pose.q_meas_world_base_xyzw
        )
        position_pred: np.ndarray = self._x[0:3]
        quat_pred: tuple[float, float, float, float] = quaternion_from_rpy(
            float(self._x[6]),
            float(self._x[7]),
            float(self._x[8]),
        )
        quat_pred = normalize_quaternion(quat_pred)
        quat_error: tuple[float, float, float, float] = quaternion_multiply(
            quat_meas, quaternion_inverse(quat_pred)
        )
        r_theta: tuple[float, float, float] = quaternion_log(quat_error)

        residual: np.ndarray = np.concatenate(
            (
                position_meas - position_pred,
                np.asarray(r_theta, dtype=float),
            )
        )

        z_roll: float
        z_pitch: float
        z_yaw: float
        z_roll, z_pitch, z_yaw = rpy_from_quaternion(quat_meas)
        z_hat_roll: float
        z_hat_pitch: float
        z_hat_yaw: float
        z_hat_roll, z_hat_pitch, z_hat_yaw = rpy_from_quaternion(quat_pred)

        z: np.ndarray = np.array(
            [
                position_meas[0],
                position_meas[1],
                position_meas[2],
                z_roll,
                z_pitch,
                z_yaw,
            ],
            dtype=float,
        )
        z_hat: np.ndarray = np.array(
            [
                position_pred[0],
                position_pred[1],
                position_pred[2],
                z_hat_roll,
                z_hat_pitch,
                z_hat_yaw,
            ],
            dtype=float,
        )

        h: np.ndarray = np.zeros((6, _STATE_DIM), dtype=float)
        h[0:3, 0:3] = np.eye(3, dtype=float)
        h[3:6, 6:9] = np.eye(3, dtype=float)

        r: np.ndarray = np.asarray(apriltag_pose.covariance, dtype=float).reshape(6, 6)
        s_hat: np.ndarray = h @ self._p @ h.T
        s: np.ndarray = s_hat + r
        maha_d2: float = float(residual.T @ np.linalg.solve(s, residual))
        gate_d2_threshold: float = self._config.apriltag_gate_d2
        if gate_d2_threshold > 0.0 and maha_d2 > gate_d2_threshold:
            return self.build_rejected_apriltag_pose(
                apriltag_pose,
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

        odom_base_pred_p: np.ndarray
        odom_base_pred_q: np.ndarray
        odom_base_pred_p, odom_base_pred_q = self._odom_base_from_state()

        hp: np.ndarray = h @ self._p
        k_gain: np.ndarray = np.linalg.solve(s.T, hp).T

        self._x = self._x + k_gain @ residual
        identity: np.ndarray = np.eye(_STATE_DIM, dtype=float)
        temp: np.ndarray = identity - k_gain @ h
        self._p = temp @ self._p @ temp.T + k_gain @ r @ k_gain.T

        self._update_world_odom_from_odom_base(odom_base_pred_p, odom_base_pred_q)

        update: EkfUpdateData = EkfUpdateData(
            sensor="apriltags",
            frame_id=apriltag_pose.frame_id,
            t_meas=t_meas,
            accepted=True,
            reject_reason="",
            z_dim=6,
            z=z.tolist(),
            z_hat=z_hat.tolist(),
            nu=residual.tolist(),
            r=EkfMatrix(rows=6, cols=6, data=r.flatten().tolist()),
            s_hat=EkfMatrix(rows=6, cols=6, data=s_hat.flatten().tolist()),
            s=EkfMatrix(rows=6, cols=6, data=s.flatten().tolist()),
            maha_d2=maha_d2,
            gate_d2_threshold=gate_d2_threshold,
            reproj_rms_px=0.0,
        )
        return EkfAprilTagDetectionUpdate(
            family=apriltag_pose.family,
            tag_id=apriltag_pose.tag_id,
            det_index_in_msg=apriltag_pose.det_index_in_msg,
            update=update,
        )

    def _wrap_angle(self, angle: float) -> float:
        wrapped: float = (angle + math.pi) % (2.0 * math.pi) - math.pi
        return wrapped

    def _event_sort_key(self, event: EkfEvent) -> int:
        return _EVENT_ORDER[event.event_type]

    def _frontier_advanced(self, prev_frontier: Optional[float]) -> bool:
        if self._t_frontier is None:
            return False
        if prev_frontier is None:
            return True
        return self._t_frontier > prev_frontier

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
                world_odom_p=self._world_odom_p.copy(),
                world_odom_q=self._world_odom_q.copy(),
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
        self._world_odom_p = checkpoint.world_odom_p.copy()
        self._world_odom_q = checkpoint.world_odom_q.copy()
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
        self._world_odom_p = np.zeros(3, dtype=float)
        self._world_odom_q = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self._initialized = True
