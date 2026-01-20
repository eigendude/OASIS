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
from typing import Sequence


ROOT: Path = Path(__file__).resolve().parents[4]
PACKAGE_ROOT: Path = ROOT / "oasis_control"
PACKAGE_SRC: Path = PACKAGE_ROOT / "oasis_control"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
if "oasis_control" in sys.modules:
    module_path_raw: Sequence[str] | None = getattr(
        sys.modules["oasis_control"],
        "__path__",
        None,
    )
    module_paths: List[str] = []
    if module_path_raw is not None:
        module_paths = list(module_path_raw)
        if str(PACKAGE_SRC) not in module_paths:
            module_paths.append(str(PACKAGE_SRC))
            sys.modules["oasis_control"].__path__ = module_paths

from oasis_control.localization.ahrs.config.ahrs_params import AhrsParams


class TestAhrsParams(unittest.TestCase):
    """Unit tests for AhrsParams"""

    def test_defaults_validate(self) -> None:
        """Defaults produce a valid parameter set"""
        params: AhrsParams = AhrsParams.defaults()
        params.validate()
        expected_keys: List[str] = [
            "t_buffer_sec",
            "ε_wall_future_ns",
            "Δt_clock_jump_max_ns",
            "Δt_imu_max_ns",
            "sigma_w_v",
            "sigma_w_omega",
            "sigma_w_bg",
            "sigma_w_ba",
            "sigma_w_Aa",
            "sigma_w_BI_rho",
            "sigma_w_BI_theta",
            "sigma_w_BM_rho",
            "sigma_w_BM_theta",
            "sigma_w_g",
            "sigma_w_m",
            "alpha",
            "R_min",
            "R_max",
            "R_m0_policy",
            "t_stationary_window_ns",
            "tau_omega",
            "tau_accel",
            "R_v0",
            "R_omega0",
            "world_frame",
            "odom_frame",
            "base_frame",
            "imu_frame",
            "mag_frame",
        ]
        self.assertEqual(list(params.as_dict().keys()), expected_keys)

    def test_from_dict_overrides(self) -> None:
        """from_dict applies overrides to defaults"""
        data: dict[str, object] = {
            "t_buffer_sec": 3.5,
            "alpha": 0.2,
            "world_frame": "map",
        }
        params: AhrsParams = AhrsParams.from_dict(data)
        self.assertEqual(params.t_buffer_sec, 3.5)
        self.assertEqual(params.alpha, 0.2)
        self.assertEqual(params.world_frame, "map")

    def test_from_dict_unknown(self) -> None:
        """from_dict rejects unknown keys"""
        data: dict[str, object] = {"zzz": 1, "aaa": 2}
        with self.assertRaisesRegex(ValueError, "unknown parameter: aaa"):
            AhrsParams.from_dict(data)

    def test_negative_sigma(self) -> None:
        """validate rejects negative process noise"""
        data: dict[str, object] = {"sigma_w_v": -0.01}
        with self.assertRaisesRegex(ValueError, "sigma_w_v must be >= 0"):
            AhrsParams.from_dict(data)

    def test_r_max_diagonal(self) -> None:
        """R_max must not be smaller than R_min on the diagonal"""
        data: dict[str, object] = {
            "R_min": [
                [2.0, 0.0, 0.0],
                [0.0, 2.0, 0.0],
                [0.0, 0.0, 2.0],
            ],
            "R_max": [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
        }
        with self.assertRaisesRegex(
            ValueError,
            "R_max diagonal must be >= R_min diagonal",
        ):
            AhrsParams.from_dict(data)


if __name__ == "__main__":
    unittest.main()
