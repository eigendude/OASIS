################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""IMU pairing counters for diagnostics."""

from __future__ import annotations


# Warmup duration in seconds for unpaired IMU accounting
_IMU_PAIR_WARMUP_SEC: float = 2.0


class ImuPairTracker:
    """Track IMU pairing counts for diagnostics.

    Attributes:
        raw_msgs_seen: Count of raw IMU messages observed
        cal_msgs_seen: Count of calibration messages observed
        imu_pairs_received: Count of exact-time IMU pairs observed
        imu_pairs_rejected_invalid_cal: Rejections due to invalid calibration
        imu_pairs_rejected_frame_mismatch: Rejections due to frame mismatch
        imu_pairs_rejected_bad_cov: Rejections due to invalid covariance
        imu_pairs_rejected_other: Rejections due to other exceptions
    """

    def __init__(self, *, warmup_sec: float = _IMU_PAIR_WARMUP_SEC) -> None:
        if warmup_sec < 0.0:
            raise ValueError("warmup_sec must be non-negative")
        self._warmup_ns: int = int(warmup_sec * 1e9)
        self._warmup_start_ns: int | None = None
        self.raw_msgs_seen: int = 0
        self.cal_msgs_seen: int = 0
        self.imu_pairs_received: int = 0
        self.imu_pairs_rejected_invalid_cal: int = 0
        self.imu_pairs_rejected_frame_mismatch: int = 0
        self.imu_pairs_rejected_bad_cov: int = 0
        self.imu_pairs_rejected_other: int = 0

    def record_raw(self, t_ns: int) -> None:
        """Record a raw IMU message arrival."""
        self._update_warmup_start(t_ns)
        self.raw_msgs_seen += 1

    def record_calibration(self, t_ns: int) -> None:
        """Record an IMU calibration message arrival."""
        self._update_warmup_start(t_ns)
        self.cal_msgs_seen += 1

    def record_pair(self, t_ns: int) -> None:
        """Record an exact-time IMU pair arrival."""
        self._update_warmup_start(t_ns)
        self.imu_pairs_received += 1

    def reject_invalid_cal(self) -> None:
        """Record a rejection due to invalid calibration."""
        self.imu_pairs_rejected_invalid_cal += 1

    def reject_frame_mismatch(self) -> None:
        """Record a rejection due to a frame mismatch."""
        self.imu_pairs_rejected_frame_mismatch += 1

    def reject_bad_cov(self) -> None:
        """Record a rejection due to invalid covariance."""
        self.imu_pairs_rejected_bad_cov += 1

    def reject_other(self) -> None:
        """Record a rejection due to an unexpected exception."""
        self.imu_pairs_rejected_other += 1

    def unpaired_raw(self, t_ns: int) -> int:
        """Return the count of raw IMU messages without a calibration match."""
        if not self._warmup_elapsed(t_ns):
            return 0
        return max(0, self.raw_msgs_seen - self.imu_pairs_received)

    def _update_warmup_start(self, t_ns: int) -> None:
        if self._warmup_start_ns is None:
            self._warmup_start_ns = t_ns

    def _warmup_elapsed(self, t_ns: int) -> bool:
        if self._warmup_start_ns is None:
            return False
        return t_ns - self._warmup_start_ns >= self._warmup_ns
