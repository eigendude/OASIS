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

from oasis_control.localization.ahrs.config.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.config.ahrs_params import AhrsParams


class TestAhrsConfig(unittest.TestCase):
    """Unit tests for AhrsConfig"""

    def test_from_params(self) -> None:
        """from_params computes the nanosecond buffer length"""
        params: AhrsParams = AhrsParams.defaults()
        config: AhrsConfig = AhrsConfig.from_params(params)
        expected_ns: int = int(params.t_buffer_sec * 1_000_000_000)
        self.assertEqual(config.t_buffer_ns, expected_ns)
        expected_keys: List[str] = [
            "t_buffer_sec",
            "t_buffer_ns",
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
        self.assertEqual(list(config.as_dict().keys()), expected_keys)

    def test_validate_buffer_mismatch(self) -> None:
        """validate rejects mismatched t_buffer_ns"""
        params: AhrsParams = AhrsParams.defaults()
        config: AhrsConfig = AhrsConfig.from_params(params)
        config_data: dict[str, object] = config.as_dict()
        config_data["t_buffer_ns"] = config.t_buffer_ns + 1
        invalid: AhrsConfig = AhrsConfig(**config_data)
        with self.assertRaisesRegex(
            ValueError,
            "t_buffer_ns must match t_buffer_sec",
        ):
            invalid.validate()


if __name__ == "__main__":
    unittest.main()
