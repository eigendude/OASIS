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
from typing import Dict
from typing import Iterable
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

from oasis_control.localization.ahrs.math_utils.units import Units


def _assert_keys_present(
    testcase: unittest.TestCase,
    data: Dict[str, str],
    keys: Iterable[str],
) -> None:
    """Assert that all keys exist and values are non-empty strings."""
    for key in keys:
        testcase.assertIn(key, data)
        testcase.assertTrue(isinstance(data[key], str))
        testcase.assertNotEqual(data[key], "")


class TestUnits(unittest.TestCase):
    """Tests for unit maps."""

    def test_state_units_keys(self) -> None:
        """state_units includes required keys and is stable."""
        required: Iterable[str] = (
            "p_WB",
            "v_WB",
            "q_WB",
            "omega_WB",
            "b_g",
            "b_a",
            "A_a",
            "T_BI",
            "T_BM",
            "g_W",
            "m_W",
        )
        units_first: Dict[str, str] = Units.state_units()
        units_second: Dict[str, str] = Units.state_units()
        _assert_keys_present(self, units_first, required)
        self.assertEqual(units_first, units_second)

    def test_imu_units_keys(self) -> None:
        """imu_units includes required keys and is stable."""
        required: Iterable[str] = ("z_omega", "R_omega", "z_accel", "R_accel")
        units_first: Dict[str, str] = Units.imu_units()
        units_second: Dict[str, str] = Units.imu_units()
        _assert_keys_present(self, units_first, required)
        self.assertEqual(units_first, units_second)

    def test_mag_units_keys(self) -> None:
        """mag_units includes required keys and is stable."""
        required: Iterable[str] = ("z_m", "R_m_raw")
        units_first: Dict[str, str] = Units.mag_units()
        units_second: Dict[str, str] = Units.mag_units()
        _assert_keys_present(self, units_first, required)
        self.assertEqual(units_first, units_second)

    def test_noise_units_keys(self) -> None:
        """noise_units includes required keys and is stable."""
        required: Iterable[str] = (
            "sigma_w_v",
            "sigma_w_omega",
            "sigma_w_bg",
            "sigma_w_ba",
            "sigma_w_Aa",
            "sigma_w_BI",
            "sigma_w_BM",
            "sigma_w_BI_rho",
            "sigma_w_BI_theta",
            "sigma_w_BM_rho",
            "sigma_w_BM_theta",
            "sigma_w_g",
            "sigma_w_m",
        )
        units_first: Dict[str, str] = Units.noise_units()
        units_second: Dict[str, str] = Units.noise_units()
        _assert_keys_present(self, units_first, required)
        self.assertEqual(units_first, units_second)


if __name__ == "__main__":
    unittest.main()
