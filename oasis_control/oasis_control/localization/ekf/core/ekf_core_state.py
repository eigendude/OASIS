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
State and replay helpers for the EKF core
"""

from __future__ import annotations

import math
from typing import Optional
from typing import cast

import numpy as np

from oasis_control.localization.ekf.core.ekf_core_constants import _EVENT_ORDER
from oasis_control.localization.ekf.core.ekf_core_constants import NS_PER_S
from oasis_control.localization.ekf.core.ekf_core_types import _Checkpoint
from oasis_control.localization.ekf.core.ekf_core_types import _ImuGap
from oasis_control.localization.ekf.core.ekf_core_types import _StateSnapshot
from oasis_control.localization.ekf.ekf_buffer import EkfBuffer
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_state import EkfState
from oasis_control.localization.ekf.ekf_state import EkfStateIndex
from oasis_control.localization.ekf.ekf_state import Pose3
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfFrameOutputs
from oasis_control.localization.ekf.ekf_types import EkfFrameTransform
from oasis_control.localization.ekf.ekf_types import EkfOutputs
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import FrameId
from oasis_control.localization.ekf.ekf_types import ImuCalibrationData
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import to_ns
from oasis_control.localization.ekf.models.imu_process_model import ImuProcessModel
from oasis_control.localization.ekf.se3 import pose_compose
from oasis_control.localization.ekf.se3 import quat_to_rpy


class EkfCoreStateMixin:
    _calibration_initialized: bool
    _checkpoints: list[_Checkpoint]
    _config: EkfConfig
    _imu_gap_reject_count: int
    _imu_gaps: list[_ImuGap]
    _imu_retention_ns: int
    _imu_times_ns: list[int]
    _initialized: bool
    _last_checkpoint_time: Optional[int]
    _last_imu: Optional[ImuSample]
    _last_imu_time_ns: Optional[int]
    _process_model: ImuProcessModel
    _state: EkfState
    _state_frontier: EkfState
    _t_frontier_ns: Optional[int]
    _world_odom: Pose3
    _world_odom_cov: np.ndarray

    def process_event(self, event: EkfEvent) -> EkfOutputs:
        raise NotImplementedError

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
        sec, nanosec = divmod(t_meas_ns, NS_PER_S)
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
