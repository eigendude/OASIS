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

from oasis_control.localization.ahrs.filter.update_step import UpdateStep
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.covariance import AhrsCovariance
from oasis_control.localization.ahrs.state.state_mapping import StateMapping


def _identity(size: int) -> List[List[float]]:
    """Return an identity matrix."""
    result: List[List[float]] = [[0.0 for _ in range(size)] for _ in range(size)]
    idx: int
    for idx in range(size):
        result[idx][idx] = 1.0
    return result


def _make_h_scalar(size: int, value: float) -> List[List[float]]:
    """Return a 1xN Jacobian with the first element set."""
    row: List[float] = [0.0 for _ in range(size)]
    row[0] = value
    return [row]


class TestUpdateStep(unittest.TestCase):
    """Tests for UpdateStep."""

    def test_update_simple_case(self) -> None:
        """Update computes expected correction and covariance."""
        state: AhrsState = AhrsState.reset()
        size: int = StateMapping.dimension()
        covariance: AhrsCovariance = AhrsCovariance.from_matrix(_identity(size))
        H: List[List[float]] = _make_h_scalar(size, 1.0)
        R: List[List[float]] = [[0.5]]
        nu: List[float] = [2.0]

        state_out: AhrsState
        cov_out: AhrsCovariance
        state_out, cov_out, report = UpdateStep.update(
            state,
            covariance,
            H,
            R,
            nu,
        )

        self.assertTrue(report.accepted)
        self.assertIsNotNone(report.innovation_mahalanobis2)
        mahalanobis2: float = cast(float, report.innovation_mahalanobis2)
        self.assertAlmostEqual(mahalanobis2, 4.0 / 1.5, places=9)
        self.assertAlmostEqual(state_out.p_WB[0], 4.0 / 3.0, places=9)
        self.assertAlmostEqual(cov_out.P[0][0], 1.0 / 3.0, places=9)
        self.assertAlmostEqual(cov_out.P[1][1], 1.0, places=9)
        self.assertAlmostEqual(cov_out.P[0][1], cov_out.P[1][0], places=12)

    def test_rejects_non_spd(self) -> None:
        """Non-SPD S rejects the update without modifying inputs."""
        state: AhrsState = AhrsState.reset()
        size: int = StateMapping.dimension()
        covariance: AhrsCovariance = AhrsCovariance.from_matrix(_identity(size))
        H: List[List[float]] = _make_h_scalar(size, 1.0)
        R: List[List[float]] = [[-1.0]]
        nu: List[float] = [1.0]

        state_out: AhrsState
        cov_out: AhrsCovariance
        state_out, cov_out, report = UpdateStep.update(
            state,
            covariance,
            H,
            R,
            nu,
        )

        self.assertFalse(report.accepted)
        self.assertIs(state_out, state)
        self.assertIs(cov_out, covariance)


if __name__ == "__main__":
    unittest.main()
