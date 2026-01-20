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

from oasis_control.localization.ahrs.ahrs_types.imu_packet import ImuPacket


class TestImuPacket(unittest.TestCase):
    """Tests for the ImuPacket container."""

    def _make_packet(self) -> ImuPacket:
        R: List[List[float]] = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        return ImuPacket(
            t_meas_ns=1_000,
            frame_id="imu",
            z_omega=[0.1, 0.2, 0.3],
            R_omega=R,
            z_accel=[0.0, 0.0, 9.81],
            R_accel=R,
            calibration_prior={"bias": [0.0, 0.0, 0.0]},
            calibration_meta={"source": "static"},
        )

    def test_validate_accepts_valid(self) -> None:
        """Validate accepts a properly shaped packet."""
        packet: ImuPacket = self._make_packet()
        packet.validate()

    def test_validate_rejects_missing_calibration(self) -> None:
        """Validate rejects missing calibration prior for ExactTime sync."""
        with self.assertRaisesRegex(
            ValueError,
            "missing calibration_prior for ExactTime sync",
        ):
            ImuPacket(
                t_meas_ns=1,
                frame_id="imu",
                z_omega=[0.0, 0.0, 0.0],
                R_omega=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                z_accel=[0.0, 0.0, 0.0],
                R_accel=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                calibration_prior=None,
                calibration_meta={},
            )

    def test_validate_rejects_bad_shapes(self) -> None:
        """Validate rejects incorrect vector and matrix shapes."""
        packet: ImuPacket = self._make_packet()
        with self.assertRaisesRegex(ValueError, "z_omega must have length 3"):
            ImuPacket(
                t_meas_ns=packet.t_meas_ns,
                frame_id=packet.frame_id,
                z_omega=[0.0, 0.0],
                R_omega=packet.R_omega,
                z_accel=packet.z_accel,
                R_accel=packet.R_accel,
                calibration_prior=packet.calibration_prior,
                calibration_meta=packet.calibration_meta,
            )
        with self.assertRaisesRegex(ValueError, "R_omega must be 3x3"):
            ImuPacket(
                t_meas_ns=packet.t_meas_ns,
                frame_id=packet.frame_id,
                z_omega=packet.z_omega,
                R_omega=[[1.0, 0.0], [0.0, 1.0]],
                z_accel=packet.z_accel,
                R_accel=packet.R_accel,
                calibration_prior=packet.calibration_prior,
                calibration_meta=packet.calibration_meta,
            )

    def test_validate_rejects_non_spd(self) -> None:
        """Validate rejects non-SPD covariance."""
        with self.assertRaisesRegex(ValueError, "R_omega must be SPD"):
            ImuPacket(
                t_meas_ns=1,
                frame_id="imu",
                z_omega=[0.0, 0.0, 0.0],
                R_omega=[[1.0, 2.0, 0.0], [2.0, 1.0, 0.0], [0.0, 0.0, -1.0]],
                z_accel=[0.0, 0.0, 0.0],
                R_accel=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                calibration_prior={"bias": [0.0, 0.0, 0.0]},
                calibration_meta={},
            )

    def test_as_dict_json_serializable(self) -> None:
        """as_dict returns JSON-serializable data."""
        packet: ImuPacket = self._make_packet()
        payload: dict[str, object] = packet.as_dict()
        json.dumps(payload)

    def test_calibration_fingerprint_deterministic(self) -> None:
        """Fingerprint stays stable for identical calibration payloads."""
        packet_a: ImuPacket = self._make_packet()
        packet_b: ImuPacket = self._make_packet()
        packet_c: ImuPacket = ImuPacket(
            t_meas_ns=1_000,
            frame_id="imu",
            z_omega=[0.1, 0.2, 0.3],
            R_omega=packet_a.R_omega,
            z_accel=[0.0, 0.0, 9.81],
            R_accel=packet_a.R_accel,
            calibration_prior={"bias": [0.1, 0.0, 0.0]},
            calibration_meta={"source": "static"},
        )
        self.assertEqual(
            packet_a.calibration_fingerprint, packet_b.calibration_fingerprint
        )
        self.assertNotEqual(
            packet_a.calibration_fingerprint, packet_c.calibration_fingerprint
        )


if __name__ == "__main__":
    unittest.main()
