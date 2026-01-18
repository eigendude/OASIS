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

from oasis_control.localization.ahrs.ahrs_types.mag_packet import MagPacket


class TestMagPacket(unittest.TestCase):
    """Tests for the MagPacket container."""

    def _make_packet(self) -> MagPacket:
        R: List[List[float]] = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        return MagPacket(
            t_meas_ns=2_000,
            frame_id="mag",
            z_m=[0.3, 0.1, -0.2],
            R_m_raw=R,
        )

    def test_validate_accepts_valid(self) -> None:
        """Validate accepts a properly shaped packet."""
        packet: MagPacket = self._make_packet()
        packet.validate()

    def test_validate_rejects_empty_frame(self) -> None:
        """Validate rejects missing frame_id."""
        packet: MagPacket = MagPacket(
            t_meas_ns=1,
            frame_id="",
            z_m=[0.0, 0.0, 0.0],
            R_m_raw=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        )
        with self.assertRaises(ValueError):
            packet.validate()

    def test_validate_rejects_bad_shapes(self) -> None:
        """Validate rejects incorrect vector and matrix shapes."""
        packet: MagPacket = self._make_packet()
        bad_packet: MagPacket = MagPacket(
            t_meas_ns=packet.t_meas_ns,
            frame_id=packet.frame_id,
            z_m=[0.0, 0.0],
            R_m_raw=packet.R_m_raw,
        )
        with self.assertRaises(ValueError):
            bad_packet.validate()
        bad_packet = MagPacket(
            t_meas_ns=packet.t_meas_ns,
            frame_id=packet.frame_id,
            z_m=packet.z_m,
            R_m_raw=[[1.0, 0.0], [0.0, 1.0]],
        )
        with self.assertRaises(ValueError):
            bad_packet.validate()

    def test_validate_rejects_non_spd(self) -> None:
        """Validate rejects non-SPD covariance."""
        packet: MagPacket = MagPacket(
            t_meas_ns=1,
            frame_id="mag",
            z_m=[0.0, 0.0, 0.0],
            R_m_raw=[[1.0, 2.0, 0.0], [2.0, 1.0, 0.0], [0.0, 0.0, -1.0]],
        )
        with self.assertRaises(ValueError):
            packet.validate()

    def test_as_dict_json_serializable(self) -> None:
        """as_dict returns JSON-serializable data."""
        packet: MagPacket = self._make_packet()
        payload: dict[str, object] = packet.as_dict()
        json.dumps(payload)


if __name__ == "__main__":
    unittest.main()
