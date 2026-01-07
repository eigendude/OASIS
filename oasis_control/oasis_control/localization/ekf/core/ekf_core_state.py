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
State management helpers for the EKF core
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from oasis_control.localization.ekf.core.ekf_core_types import _Checkpoint
from oasis_control.localization.ekf.core.ekf_core_types import _StateSnapshot
from oasis_control.localization.ekf.ekf_state import EkfState
from oasis_control.localization.ekf.ekf_state import Pose3
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import to_ns


class _EkfCoreStateMixin:
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
