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
IMU propagation helpers for the EKF core
"""

from __future__ import annotations

import logging
from bisect import bisect_right
from typing import Optional

import numpy as np

from oasis_control.localization.ekf.core.ekf_core_types import _ImuCoverageCheck
from oasis_control.localization.ekf.core.ekf_core_types import _ImuGap
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_state import EkfState
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.models.imu_process_model import ImuProcessModel


_LOG: logging.Logger = logging.getLogger(__name__)


class EkfCoreImuMixin:
    _config: EkfConfig
    _imu_gap_reject_count: int
    _imu_gaps: list[_ImuGap]
    _imu_retention_ns: int
    _imu_times_ns: list[int]
    _last_checkpoint_time: Optional[int]
    _last_imu: Optional[ImuSample]
    _last_imu_time_ns: Optional[int]
    _process_model: ImuProcessModel
    _state: EkfState
    _state_frontier: EkfState
    _t_frontier_ns: Optional[int]

    def _record_imu_time(self, t_meas_ns: int) -> None:
        if not self._imu_times_ns or t_meas_ns >= self._imu_times_ns[-1]:
            self._imu_times_ns.append(t_meas_ns)
        else:
            insert_index: int = bisect_right(self._imu_times_ns, t_meas_ns)
            self._imu_times_ns.insert(insert_index, t_meas_ns)
        self._prune_imu_history(t_meas_ns)

    def _prune_imu_history(self, t_meas_ns: int) -> None:
        reference_ns: int = t_meas_ns
        if self._t_frontier_ns is not None:
            reference_ns = min(reference_ns, self._t_frontier_ns)
        if self._last_checkpoint_time is not None:
            reference_ns = min(reference_ns, self._last_checkpoint_time)
        cutoff_ns: int = reference_ns - self._imu_retention_ns
        if cutoff_ns <= 0:
            return
        prune_index: int = bisect_right(self._imu_times_ns, cutoff_ns)
        if prune_index > 0:
            self._imu_times_ns = self._imu_times_ns[prune_index:]
        while self._imu_gaps and self._imu_gaps[0].end_ns < cutoff_ns:
            self._imu_gaps.pop(0)

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

    def _ensure_imu_coverage(self, t_start_ns: int, t_end_ns: int) -> _ImuCoverageCheck:
        if t_end_ns <= t_start_ns:
            return _ImuCoverageCheck(True, "", 0, None, None)
        if not self._imu_times_ns:
            return _ImuCoverageCheck(False, "missing_imu", 0, t_start_ns, t_end_ns)
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
            self._reset_frontier_after_gap(t_meas_ns)
            self._last_imu = imu_sample
            self._last_imu_time_ns = t_meas_ns
            return
        if self._t_frontier_ns is not None:
            self._propagate_with_imu(self._last_imu, self._t_frontier_ns, t_meas_ns)
        self._last_imu = imu_sample
        self._last_imu_time_ns = t_meas_ns

    def _reset_frontier_after_gap(self, t_meas_ns: int) -> None:
        self._t_frontier_ns = t_meas_ns
        self._state_frontier = self._state.copy()

    def _propagate_if_needed(self, t_meas_ns: int) -> _ImuCoverageCheck:
        if self._t_frontier_ns is None:
            return _ImuCoverageCheck(True, "", 0, None, None)
        if t_meas_ns <= self._t_frontier_ns:
            return _ImuCoverageCheck(True, "", 0, None, None)
        coverage: _ImuCoverageCheck = self._ensure_imu_coverage(
            self._t_frontier_ns, t_meas_ns
        )
        if not coverage.covered:
            self._record_imu_coverage_failure(self._t_frontier_ns, t_meas_ns, coverage)
            return coverage
        if self._last_imu is None or self._last_imu_time_ns is None:
            coverage = _ImuCoverageCheck(
                False, "missing_imu", 0, self._t_frontier_ns, t_meas_ns
            )
            self._record_imu_coverage_failure(self._t_frontier_ns, t_meas_ns, coverage)
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
