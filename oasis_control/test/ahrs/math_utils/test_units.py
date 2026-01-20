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

import unittest
from typing import Dict

from oasis_control.localization.ahrs.math_utils.units import Units


class TestUnits(unittest.TestCase):
    def test_state_units_keys(self) -> None:
        units: Dict[str, str] = Units.state_units()
        self.assertIn("p_WB", units)
        self.assertIn("v_WB", units)
        self.assertIn("q_WB", units)
        for value in units.values():
            self.assertTrue(value)

    def test_imu_units_keys(self) -> None:
        units: Dict[str, str] = Units.imu_units()
        self.assertIn("z_omega", units)
        self.assertIn("z_accel", units)
        self.assertIn("R_omega", units)
        self.assertIn("R_accel", units)
        for value in units.values():
            self.assertTrue(value)

    def test_mag_units_keys(self) -> None:
        units: Dict[str, str] = Units.mag_units()
        self.assertIn("z_m", units)
        self.assertIn("R_m_raw", units)
        for value in units.values():
            self.assertTrue(value)

    def test_noise_units_keys(self) -> None:
        units: Dict[str, str] = Units.noise_units()
        self.assertIn("sigma_w_bg", units)
        self.assertIn("sigma_w_ba", units)
        self.assertIn("sigma_w_m", units)
        for value in units.values():
            self.assertTrue(value)

    def test_units_stability(self) -> None:
        state1: Dict[str, str] = Units.state_units()
        state2: Dict[str, str] = Units.state_units()
        imu1: Dict[str, str] = Units.imu_units()
        imu2: Dict[str, str] = Units.imu_units()
        mag1: Dict[str, str] = Units.mag_units()
        mag2: Dict[str, str] = Units.mag_units()
        noise1: Dict[str, str] = Units.noise_units()
        noise2: Dict[str, str] = Units.noise_units()
        self.assertEqual(state1, state2)
        self.assertEqual(imu1, imu2)
        self.assertEqual(mag1, mag2)
        self.assertEqual(noise1, noise2)


if __name__ == "__main__":
    unittest.main()
