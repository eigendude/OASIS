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
Internal EKF state containers
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from oasis_control.localization.ekf.ekf_state import EkfState
from oasis_control.localization.ekf.ekf_types import ImuSample


@dataclass
class _Checkpoint:
    t_meas_ns: int
    state: EkfState
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
