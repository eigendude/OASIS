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

from oasis_control.localization.ahrs.config.ahrs_params import AhrsParams


class TestAhrsParams(unittest.TestCase):
    """Tests for the AhrsParams container."""

    def test_defaults_validate(self) -> None:
        """Defaults validate and serialize deterministically."""
        params: AhrsParams = AhrsParams.defaults()
        params.validate()
        payload: dict[str, object] = params.as_dict()
        json.dumps(payload)

    def test_round_trip(self) -> None:
        """Defaults round-trip through as_dict and from_dict."""
        params: AhrsParams = AhrsParams.defaults()
        params2: AhrsParams = AhrsParams.from_dict(params.as_dict())
        self.assertEqual(params2, params)

    def test_from_dict_overrides(self) -> None:
        """from_dict merges overrides with defaults."""
        params: AhrsParams = AhrsParams.from_dict(
            {
                "alpha": 0.5,
                "world_frame": "world_ned",
            }
        )
        self.assertEqual(params.alpha, 0.5)
        self.assertEqual(params.world_frame, "world_ned")
        self.assertEqual(params.t_buffer_sec, AhrsParams.defaults().t_buffer_sec)

    def test_from_dict_unknown_key(self) -> None:
        """from_dict rejects unknown keys deterministically."""
        with self.assertRaises(ValueError) as context:
            AhrsParams.from_dict({"z_key": 1, "a_key": 2})
        self.assertEqual(str(context.exception), "unknown parameter: a_key")

    def test_validate_range_errors(self) -> None:
        """Validation rejects invalid ranges."""
        params: AhrsParams = AhrsParams.defaults()
        bad_params: AhrsParams = AhrsParams(
            t_buffer_sec=0.0,
            epsilon_wall_future_ns=params.epsilon_wall_future_ns,
            dt_clock_jump_max_ns=params.dt_clock_jump_max_ns,
            dt_imu_max_ns=params.dt_imu_max_ns,
            sigma_w_v=params.sigma_w_v,
            sigma_w_omega=params.sigma_w_omega,
            sigma_w_bg=params.sigma_w_bg,
            sigma_w_ba=params.sigma_w_ba,
            sigma_w_Aa=params.sigma_w_Aa,
            sigma_w_BI_rho=params.sigma_w_BI_rho,
            sigma_w_BI_theta=params.sigma_w_BI_theta,
            sigma_w_BM_rho=params.sigma_w_BM_rho,
            sigma_w_BM_theta=params.sigma_w_BM_theta,
            sigma_w_g=params.sigma_w_g,
            sigma_w_m=params.sigma_w_m,
            alpha=params.alpha,
            R_min=params.R_min,
            R_max=params.R_max,
            R_m0_policy=params.R_m0_policy,
            t_stationary_window_ns=params.t_stationary_window_ns,
            tau_omega=params.tau_omega,
            tau_accel=params.tau_accel,
            R_v0=params.R_v0,
            R_omega0=params.R_omega0,
            world_frame=params.world_frame,
            odom_frame=params.odom_frame,
            base_frame=params.base_frame,
            imu_frame=params.imu_frame,
            mag_frame=params.mag_frame,
        )
        with self.assertRaises(ValueError) as context:
            bad_params.validate()
        self.assertEqual(str(context.exception), "t_buffer_sec must be > 0")

    def test_validate_negative_variance(self) -> None:
        """Validation rejects negative variances deterministically."""
        params: AhrsParams = AhrsParams.defaults()
        bad_params: AhrsParams = AhrsParams(
            t_buffer_sec=params.t_buffer_sec,
            epsilon_wall_future_ns=params.epsilon_wall_future_ns,
            dt_clock_jump_max_ns=params.dt_clock_jump_max_ns,
            dt_imu_max_ns=params.dt_imu_max_ns,
            sigma_w_v=-1.0,
            sigma_w_omega=params.sigma_w_omega,
            sigma_w_bg=params.sigma_w_bg,
            sigma_w_ba=params.sigma_w_ba,
            sigma_w_Aa=params.sigma_w_Aa,
            sigma_w_BI_rho=params.sigma_w_BI_rho,
            sigma_w_BI_theta=params.sigma_w_BI_theta,
            sigma_w_BM_rho=params.sigma_w_BM_rho,
            sigma_w_BM_theta=params.sigma_w_BM_theta,
            sigma_w_g=params.sigma_w_g,
            sigma_w_m=params.sigma_w_m,
            alpha=params.alpha,
            R_min=params.R_min,
            R_max=params.R_max,
            R_m0_policy=params.R_m0_policy,
            t_stationary_window_ns=params.t_stationary_window_ns,
            tau_omega=params.tau_omega,
            tau_accel=params.tau_accel,
            R_v0=params.R_v0,
            R_omega0=params.R_omega0,
            world_frame=params.world_frame,
            odom_frame=params.odom_frame,
            base_frame=params.base_frame,
            imu_frame=params.imu_frame,
            mag_frame=params.mag_frame,
        )
        with self.assertRaises(ValueError) as context:
            bad_params.validate()
        self.assertEqual(str(context.exception), "sigma_w_v must be >= 0")

    def test_from_dict_rejects_bool_matrix_entries(self) -> None:
        """from_dict rejects bool matrix entries deterministically."""
        with self.assertRaises(ValueError) as context:
            AhrsParams.from_dict(
                {
                    "R_min": [
                        [True, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0],
                    ]
                }
            )
        self.assertEqual(str(context.exception), "matrix entries must be float")

    def test_validate_alpha_bounds(self) -> None:
        """Validation rejects alpha outside [0, 1]."""
        params: AhrsParams = AhrsParams.defaults()
        bad_params: AhrsParams = AhrsParams(
            t_buffer_sec=params.t_buffer_sec,
            epsilon_wall_future_ns=params.epsilon_wall_future_ns,
            dt_clock_jump_max_ns=params.dt_clock_jump_max_ns,
            dt_imu_max_ns=params.dt_imu_max_ns,
            sigma_w_v=params.sigma_w_v,
            sigma_w_omega=params.sigma_w_omega,
            sigma_w_bg=params.sigma_w_bg,
            sigma_w_ba=params.sigma_w_ba,
            sigma_w_Aa=params.sigma_w_Aa,
            sigma_w_BI_rho=params.sigma_w_BI_rho,
            sigma_w_BI_theta=params.sigma_w_BI_theta,
            sigma_w_BM_rho=params.sigma_w_BM_rho,
            sigma_w_BM_theta=params.sigma_w_BM_theta,
            sigma_w_g=params.sigma_w_g,
            sigma_w_m=params.sigma_w_m,
            alpha=1.5,
            R_min=params.R_min,
            R_max=params.R_max,
            R_m0_policy=params.R_m0_policy,
            t_stationary_window_ns=params.t_stationary_window_ns,
            tau_omega=params.tau_omega,
            tau_accel=params.tau_accel,
            R_v0=params.R_v0,
            R_omega0=params.R_omega0,
            world_frame=params.world_frame,
            odom_frame=params.odom_frame,
            base_frame=params.base_frame,
            imu_frame=params.imu_frame,
            mag_frame=params.mag_frame,
        )
        with self.assertRaises(ValueError) as context:
            bad_params.validate()
        self.assertEqual(str(context.exception), "alpha must be in [0, 1]")

    def test_validate_r_max_diagonal(self) -> None:
        """Validation rejects R_max diagonals smaller than R_min."""
        params: AhrsParams = AhrsParams.defaults()
        r_min: List[List[float]] = [
            [2.0, 0.0, 0.0],
            [0.0, 2.0, 0.0],
            [0.0, 0.0, 2.0],
        ]
        r_max: List[List[float]] = [
            [1.0, 0.0, 0.0],
            [0.0, 2.0, 0.0],
            [0.0, 0.0, 2.0],
        ]
        bad_params: AhrsParams = AhrsParams(
            t_buffer_sec=params.t_buffer_sec,
            epsilon_wall_future_ns=params.epsilon_wall_future_ns,
            dt_clock_jump_max_ns=params.dt_clock_jump_max_ns,
            dt_imu_max_ns=params.dt_imu_max_ns,
            sigma_w_v=params.sigma_w_v,
            sigma_w_omega=params.sigma_w_omega,
            sigma_w_bg=params.sigma_w_bg,
            sigma_w_ba=params.sigma_w_ba,
            sigma_w_Aa=params.sigma_w_Aa,
            sigma_w_BI_rho=params.sigma_w_BI_rho,
            sigma_w_BI_theta=params.sigma_w_BI_theta,
            sigma_w_BM_rho=params.sigma_w_BM_rho,
            sigma_w_BM_theta=params.sigma_w_BM_theta,
            sigma_w_g=params.sigma_w_g,
            sigma_w_m=params.sigma_w_m,
            alpha=params.alpha,
            R_min=r_min,
            R_max=r_max,
            R_m0_policy=params.R_m0_policy,
            t_stationary_window_ns=params.t_stationary_window_ns,
            tau_omega=params.tau_omega,
            tau_accel=params.tau_accel,
            R_v0=params.R_v0,
            R_omega0=params.R_omega0,
            world_frame=params.world_frame,
            odom_frame=params.odom_frame,
            base_frame=params.base_frame,
            imu_frame=params.imu_frame,
            mag_frame=params.mag_frame,
        )
        with self.assertRaises(ValueError) as context:
            bad_params.validate()
        self.assertEqual(
            str(context.exception),
            "R_max diagonal must be >= R_min diagonal",
        )


if __name__ == "__main__":
    unittest.main()
