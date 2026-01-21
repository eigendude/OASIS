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

from oasis_control.localization.ahrs.filter.predict_step import PredictStep
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


def _zero_matrix(size: int) -> List[List[float]]:
    """Return a square zero matrix."""
    return [[0.0 for _ in range(size)] for _ in range(size)]


class TestPredictStep(unittest.TestCase):
    """Tests for PredictStep."""

    def test_noop_dt_zero(self) -> None:
        """dt_ns == 0 returns the same state and covariance objects."""
        state: AhrsState = AhrsState.reset()
        covariance: AhrsCovariance = AhrsCovariance.from_matrix(
            _identity(StateMapping.dimension())
        )
        Q_c: List[List[float]] = _zero_matrix(39)

        state_out: AhrsState
        cov_out: AhrsCovariance
        state_out, cov_out = PredictStep.propagate(state, covariance, 0, Q_c)

        self.assertIs(state_out, state)
        self.assertIs(cov_out, covariance)

    def test_negative_dt_rejected(self) -> None:
        """Negative dt_ns is rejected."""
        state: AhrsState = AhrsState.reset()
        covariance: AhrsCovariance = AhrsCovariance.from_matrix(
            _identity(StateMapping.dimension())
        )
        Q_c: List[List[float]] = _zero_matrix(39)

        with self.assertRaises(ValueError):
            PredictStep.propagate(state, covariance, -1, Q_c)

    def test_invalid_qc_shape(self) -> None:
        """Invalid Q_c shape raises ValueError."""
        state: AhrsState = AhrsState.reset()
        covariance: AhrsCovariance = AhrsCovariance.from_matrix(
            _identity(StateMapping.dimension())
        )
        Q_c: List[List[float]] = [[0.0]]

        with self.assertRaises(ValueError):
            PredictStep.propagate(state, covariance, 1, Q_c)

    def test_covariance_symmetry(self) -> None:
        """Prediction output covariance remains symmetric."""
        state: AhrsState = AhrsState.reset()
        covariance: AhrsCovariance = AhrsCovariance.from_matrix(
            _identity(StateMapping.dimension())
        )
        Q_c: List[List[float]] = _zero_matrix(39)

        _, cov_out = PredictStep.propagate(state, covariance, 10, Q_c)
        P_out: List[List[float]] = cov_out.P
        size: int = len(P_out)
        i: int
        for i in range(size):
            j: int
            for j in range(size):
                self.assertAlmostEqual(P_out[i][j], P_out[j][i], places=12)


if __name__ == "__main__":
    unittest.main()
