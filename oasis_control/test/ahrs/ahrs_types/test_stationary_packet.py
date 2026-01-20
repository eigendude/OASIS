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

import json
import sys
import unittest
from pathlib import Path
from typing import List


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

from oasis_control.localization.ahrs.ahrs_types.stationary_packet import (
    StationaryPacket,
)


class TestStationaryPacket(unittest.TestCase):
    """Tests for the StationaryPacket container."""

    def _make_packet(self) -> StationaryPacket:
        R: List[List[float]] = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        return StationaryPacket(
            t_meas_ns=3_000,
            window_start_ns=2_000,
            window_end_ns=3_000,
            is_stationary=True,
            R_v=R,
            R_omega=R,
            metadata={"score": 0.1},
        )

    def test_validate_accepts_valid(self) -> None:
        """Validate accepts a properly shaped packet."""
        packet: StationaryPacket = self._make_packet()
        packet.validate()

    def test_validate_rejects_window_order(self) -> None:
        """Validate rejects window bounds with start after end."""
        packet: StationaryPacket = StationaryPacket(
            t_meas_ns=3,
            window_start_ns=4,
            window_end_ns=3,
            is_stationary=True,
            R_v=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            R_omega=None,
            metadata={},
        )
        with self.assertRaises(ValueError):
            packet.validate()

    def test_validate_rejects_t_meas_ns_mismatch(self) -> None:
        """Validate rejects t_meas_ns that is not window_end_ns."""
        packet: StationaryPacket = StationaryPacket(
            t_meas_ns=4,
            window_start_ns=2,
            window_end_ns=3,
            is_stationary=False,
            R_v=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            R_omega=None,
            metadata={},
        )
        with self.assertRaises(ValueError):
            packet.validate()

    def test_validate_rejects_bad_shapes(self) -> None:
        """Validate rejects incorrect matrix shapes."""
        packet: StationaryPacket = StationaryPacket(
            t_meas_ns=3,
            window_start_ns=1,
            window_end_ns=3,
            is_stationary=True,
            R_v=[[1.0, 0.0], [0.0, 1.0]],
            R_omega=None,
            metadata={},
        )
        with self.assertRaises(ValueError):
            packet.validate()

    def test_validate_rejects_non_spd(self) -> None:
        """Validate rejects non-SPD covariances."""
        packet: StationaryPacket = StationaryPacket(
            t_meas_ns=3,
            window_start_ns=1,
            window_end_ns=3,
            is_stationary=True,
            R_v=[[1.0, 2.0, 0.0], [2.0, 1.0, 0.0], [0.0, 0.0, -1.0]],
            R_omega=None,
            metadata={},
        )
        with self.assertRaises(ValueError):
            packet.validate()

    def test_as_dict_json_serializable(self) -> None:
        """as_dict returns JSON-serializable data."""
        packet: StationaryPacket = self._make_packet()
        payload: dict[str, object] = packet.as_dict()
        json.dumps(payload)


if __name__ == "__main__":
    unittest.main()
