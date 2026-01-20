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
from dataclasses import dataclass
from pathlib import Path
from typing import List
from typing import cast


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
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.covariance import AhrsCovariance
from oasis_control.localization.ahrs.timing.timeline_node import TimelineNode


@dataclass(frozen=True, slots=True)
class FakePacket:
    t_meas_ns: int


class TestTimelineNode(unittest.TestCase):
    """Tests for TimelineNode insertion behavior."""

    def test_insert_duplicate_imu(self) -> None:
        """Duplicate IMU packets are rejected and diagnosed."""
        node: TimelineNode = TimelineNode(100)
        imu: FakePacket = FakePacket(100)
        first: bool = node.insert_imu(cast(ImuPacket, imu))
        second: bool = node.insert_imu(cast(ImuPacket, imu))
        self.assertTrue(first)
        self.assertFalse(second)
        self.assertEqual(node.diagnostics["duplicate_imu"], 1)

    def test_insert_mismatch_timestamp(self) -> None:
        """Mismatched timestamps raise a validation error."""
        node: TimelineNode = TimelineNode(100)
        imu: FakePacket = FakePacket(200)
        with self.assertRaisesRegex(ValueError, "t_meas_ns mismatch"):
            node.insert_imu(cast(ImuPacket, imu))

    def test_insert_multiple_types(self) -> None:
        """IMU and mag packets may share a node."""
        node: TimelineNode = TimelineNode(250)
        imu: FakePacket = FakePacket(250)
        mag: FakePacket = FakePacket(250)
        imu_inserted: bool = node.insert_imu(cast(ImuPacket, imu))
        mag_inserted: bool = node.insert_mag(cast(MagPacket, mag))
        self.assertTrue(imu_inserted)
        self.assertTrue(mag_inserted)

    def test_is_complete_state_only(self) -> None:
        """is_complete depends only on state and covariance."""
        node: TimelineNode = TimelineNode(500)
        self.assertFalse(node.is_complete())
        state_obj: AhrsState = cast(AhrsState, object())
        node.state = state_obj
        self.assertFalse(node.is_complete())
        covariance_obj: AhrsCovariance = cast(AhrsCovariance, object())
        node.covariance = covariance_obj
        self.assertTrue(node.is_complete())


if __name__ == "__main__":
    unittest.main()
