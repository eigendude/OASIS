################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from typing import List
from typing import Mapping


ROOT: Path = Path(__file__).resolve().parents[4]
PACKAGE_ROOT: Path = ROOT / "oasis_control"
PACKAGE_SRC: Path = PACKAGE_ROOT / "oasis_control"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
if "oasis_control" in sys.modules:
    module_path_raw = getattr(sys.modules["oasis_control"], "__path__", None)
    module_paths: List[str] = []
    if module_path_raw is not None:
        module_paths = list(module_path_raw)
        if str(PACKAGE_SRC) not in module_paths:
            module_paths.append(str(PACKAGE_SRC))
            sys.modules["oasis_control"].__path__ = module_paths

from oasis_control.localization.ahrs.ahrs_types.imu_packet import ImuPacket
from oasis_control.localization.ahrs.ahrs_types.mag_packet import MagPacket
from oasis_control.localization.ahrs.ahrs_types.stationary_packet import (
    StationaryPacket,
)
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.covariance import AhrsCovariance
from oasis_control.localization.ahrs.state.state_mapping import StateMapping
from oasis_control.localization.ahrs.timing.timeline_node import TimelineNode


def _identity(size: int) -> List[List[float]]:
    matrix: List[List[float]] = []
    row: int
    for row in range(size):
        values: List[float] = [0.0 for _ in range(size)]
        values[row] = 1.0
        matrix.append(values)
    return matrix


def _make_imu_packet(t_meas_ns: int) -> ImuPacket:
    R: List[List[float]] = _identity(3)
    calibration_prior: Mapping[str, object] = {"bias": [0.0, 0.0, 0.0]}
    calibration_meta: Mapping[str, object] = {"source": "test"}
    return ImuPacket(
        t_meas_ns=t_meas_ns,
        frame_id="imu",
        z_omega=[0.1, 0.2, 0.3],
        R_omega=R,
        z_accel=[0.0, 0.0, 9.81],
        R_accel=R,
        calibration_prior=calibration_prior,
        calibration_meta=calibration_meta,
    )


def _make_mag_packet(t_meas_ns: int) -> MagPacket:
    R: List[List[float]] = _identity(3)
    return MagPacket(
        t_meas_ns=t_meas_ns,
        frame_id="mag",
        z_m=[0.1, 0.2, 0.3],
        R_m_raw=R,
    )


def _make_stationary_packet(t_meas_ns: int, is_stationary: bool) -> StationaryPacket:
    R: List[List[float]] = _identity(3)
    metadata: Mapping[str, object] = {"score": 0.1}
    return StationaryPacket(
        t_meas_ns=t_meas_ns,
        window_start_ns=t_meas_ns - 10,
        window_end_ns=t_meas_ns,
        is_stationary=is_stationary,
        R_v=R,
        R_omega=None,
        metadata=metadata,
    )


class TestTimelineNode(unittest.TestCase):
    """Tests for TimelineNode."""

    def test_duplicate_imu_rejected(self) -> None:
        """insert_imu rejects duplicate packets and updates diagnostics."""
        node: TimelineNode = TimelineNode(t_meas_ns=1_000)
        packet: ImuPacket = _make_imu_packet(1_000)
        self.assertTrue(node.insert_imu(packet))
        self.assertFalse(node.insert_imu(packet))
        self.assertEqual(node.diagnostics["duplicate_imu"], 1)

    def test_duplicate_stationary_rejected(self) -> None:
        """insert_stationary rejects duplicates and updates diagnostics."""
        node: TimelineNode = TimelineNode(t_meas_ns=2_000)
        packet: StationaryPacket = _make_stationary_packet(2_000, True)
        self.assertTrue(node.insert_stationary(packet))
        self.assertFalse(node.insert_stationary(packet))
        self.assertEqual(node.diagnostics["duplicate_stationary"], 1)

    def test_mismatched_timestamp_rejected(self) -> None:
        """Insert rejects mismatched timestamps."""
        node: TimelineNode = TimelineNode(t_meas_ns=1_000)
        packet: MagPacket = _make_mag_packet(2_000)
        with self.assertRaisesRegex(ValueError, "t_meas_ns mismatch"):
            node.insert_mag(packet)

    def test_imu_mag_share_node(self) -> None:
        """IMU and mag packets can share a node."""
        node: TimelineNode = TimelineNode(t_meas_ns=3_000)
        imu: ImuPacket = _make_imu_packet(3_000)
        mag: MagPacket = _make_mag_packet(3_000)
        self.assertTrue(node.insert_imu(imu))
        self.assertTrue(node.insert_mag(mag))

    def test_is_complete_requires_state_and_covariance(self) -> None:
        """is_complete only depends on state and covariance."""
        node: TimelineNode = TimelineNode(t_meas_ns=4_000)
        self.assertFalse(node.is_complete())
        node.state = AhrsState.reset()
        self.assertFalse(node.is_complete())
        size: int = StateMapping.dimension()
        covariance: AhrsCovariance = AhrsCovariance.from_matrix(_identity(size))
        node.covariance = covariance
        self.assertTrue(node.is_complete())
