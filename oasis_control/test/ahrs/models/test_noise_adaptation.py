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

from oasis_control.localization.ahrs.math_utils.linalg import LinearAlgebra
from oasis_control.localization.ahrs.models.noise_adaptation import NoiseAdaptation


def _diag(value: float) -> List[List[float]]:
    """Return a 3x3 diagonal matrix."""
    return [
        [value, 0.0, 0.0],
        [0.0, value, 0.0],
        [0.0, 0.0, value],
    ]


class TestNoiseAdaptation(unittest.TestCase):
    """Tests for noise adaptation."""

    def test_symmetry_preserved(self) -> None:
        """Update produces symmetric covariance."""
        R_m = _diag(1.0)
        nu: List[float] = [0.1, -0.2, 0.3]
        S_hat = _diag(0.1)
        min_eig: float = 0.01
        max_eig: float = 10.0
        R_new: List[List[float]] = NoiseAdaptation.update_mag_covariance(
            R_m, nu, S_hat, 0.5, min_eig, max_eig
        )
        self.assertEqual(R_new, LinearAlgebra.symmetrize(R_new))

    def test_eigen_bounds_enforced(self) -> None:
        """Clamping keeps diagonal entries within bounds."""
        R_m = _diag(1.0)
        nu: List[float] = [10.0, 0.0, 0.0]
        S_hat = _diag(0.0)
        min_eig: float = 0.1
        max_eig: float = 1.5
        R_new: List[List[float]] = NoiseAdaptation.update_mag_covariance(
            R_m, nu, S_hat, 1.0, min_eig, max_eig
        )
        self.assertTrue(LinearAlgebra.is_spd(R_new))
        for i in range(3):
            self.assertGreaterEqual(R_new[i][i], 0.1)
            self.assertLessEqual(R_new[i][i], 1.5)

    def test_alpha_validation(self) -> None:
        """Alpha bounds are enforced."""
        R_m = _diag(1.0)
        nu: List[float] = [0.1, -0.2, 0.3]
        S_hat = _diag(0.1)
        min_eig: float = 0.01
        max_eig: float = 10.0
        with self.assertRaisesRegex(ValueError, "alpha must be in \\[0, 1\\]"):
            NoiseAdaptation.update_mag_covariance(
                R_m, nu, S_hat, -0.1, min_eig, max_eig
            )
        with self.assertRaisesRegex(ValueError, "alpha must be in \\[0, 1\\]"):
            NoiseAdaptation.update_mag_covariance(R_m, nu, S_hat, 1.1, min_eig, max_eig)
        R_zero: List[List[float]] = NoiseAdaptation.update_mag_covariance(
            R_m, nu, S_hat, 0.0, min_eig, max_eig
        )
        R_one: List[List[float]] = NoiseAdaptation.update_mag_covariance(
            R_m, nu, S_hat, 1.0, min_eig, max_eig
        )
        self.assertTrue(LinearAlgebra.is_spd(R_zero))
        self.assertTrue(LinearAlgebra.is_spd(R_one))


if __name__ == "__main__":
    unittest.main()
