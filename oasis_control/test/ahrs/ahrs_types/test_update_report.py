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

from oasis_control.localization.ahrs.ahrs_types.update_report import UpdateReport


class TestUpdateReport(unittest.TestCase):
    """Tests for the UpdateReport container."""

    def _make_report(self) -> UpdateReport:
        R: List[List[float]] = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        return UpdateReport(
            t_meas_ns=4_000,
            measurement_type="mag",
            z=[1.0, 2.0, 3.0],
            z_hat=[0.9, 2.1, 3.1],
            nu=[0.1, -0.1, -0.1],
            R=R,
            S_hat=R,
            S=R,
            mahalanobis2=1.2,
            accepted=True,
            rejection_reason="",
        )

    def test_validate_accepts_valid(self) -> None:
        """Validate accepts a properly shaped report."""
        report: UpdateReport = self._make_report()
        report.validate()

    def test_validate_rejects_bad_shapes(self) -> None:
        """Validate rejects mismatched vector and matrix shapes."""
        report: UpdateReport = UpdateReport(
            t_meas_ns=1,
            measurement_type="mag",
            z=[1.0, 2.0],
            z_hat=[1.0, 2.0, 3.0],
            nu=[0.1, 0.2],
            R=[[1.0, 0.0], [0.0, 1.0]],
            S_hat=None,
            S=[[1.0, 0.0], [0.0, 1.0]],
            mahalanobis2=0.0,
            accepted=True,
            rejection_reason="",
        )
        with self.assertRaises(ValueError):
            report.validate()

    def test_validate_rejects_accept_reason_mismatch(self) -> None:
        """Validate rejects inconsistent accepted and rejection_reason."""
        report: UpdateReport = self._make_report()
        report = UpdateReport(
            t_meas_ns=report.t_meas_ns,
            measurement_type=report.measurement_type,
            z=report.z,
            z_hat=report.z_hat,
            nu=report.nu,
            R=report.R,
            S_hat=report.S_hat,
            S=report.S,
            mahalanobis2=report.mahalanobis2,
            accepted=True,
            rejection_reason="rejected",
        )
        with self.assertRaises(ValueError):
            report.validate()
        report = UpdateReport(
            t_meas_ns=report.t_meas_ns,
            measurement_type=report.measurement_type,
            z=report.z,
            z_hat=report.z_hat,
            nu=report.nu,
            R=report.R,
            S_hat=report.S_hat,
            S=report.S,
            mahalanobis2=report.mahalanobis2,
            accepted=False,
            rejection_reason="",
        )
        with self.assertRaises(ValueError):
            report.validate()

    def test_validate_rejects_non_finite_mahalanobis(self) -> None:
        """Validate rejects non-finite mahalanobis2 values."""
        report: UpdateReport = UpdateReport(
            t_meas_ns=1,
            measurement_type="mag",
            z=[1.0, 2.0, 3.0],
            z_hat=[1.0, 2.0, 3.0],
            nu=[0.0, 0.0, 0.0],
            R=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            S_hat=None,
            S=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            mahalanobis2=float("nan"),
            accepted=True,
            rejection_reason="",
        )
        with self.assertRaises(ValueError):
            report.validate()

    def test_as_dict_json_serializable(self) -> None:
        """as_dict returns JSON-serializable data."""
        report: UpdateReport = self._make_report()
        payload: dict[str, object] = report.as_dict()
        json.dumps(payload)


if __name__ == "__main__":
    unittest.main()
