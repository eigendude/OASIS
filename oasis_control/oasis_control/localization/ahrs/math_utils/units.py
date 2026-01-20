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

from typing import Dict


class Units:
    """Unit conventions for AHRS state and measurements.

    Responsibility:
        Declare the canonical units for every state element and measurement
        used by the AHRS core. These units are the contract for all modules.

    Purpose:
        Provide a single, stable reference for units so that all modules use
        consistent scaling and interpretation of the AHRS state.

    Inputs/outputs:
        No runtime inputs; this module is purely documentation and shared
        constants.

    Dependencies:
        - Referenced by state, models, filter, and ahrs_types modules.

    Public API (to be implemented):
        - state_units()
        - imu_units()
        - mag_units()
        - noise_units()

    Data contract:
        - Positions p_WB: meters.
        - Velocities v_WB: meters per second.
        - Orientation q_WB: unit quaternion [w, x, y, z].
        - Angular rate ω_WB: radians per second.
        - Gyro bias b_g: radians per second.
        - Accel bias b_a: meters per second squared.
        - Accel calibration A_a: unitless 3x3.
        - Extrinsics T_BI, T_BM: rotation unitless, translation meters.
        - Gravity g_W: meters per second squared.
        - Magnetic field m_W: tesla.

        Measurements:
        - Gyro z_ω: rad/s in {I}.
        - Accel z_a: m/s^2 in {I}, specific force (a - g).
        - Mag z_m: tesla in {M}.

    Frames and units:
        - World {W}, body {B}, IMU {I}, magnetometer {M}.
        - All vectors carry the frame indicated by their subscript.

    Determinism and edge cases:
        - Units are fixed and must not change across runs.
        - Unit definitions are constant and must be applied uniformly.

    Equations:
        Specific force at rest is approximately -g in the sensor frame.

    Numerical stability notes:
        - Units do not impact stability directly, but inconsistent units will
          destabilize the filter.

    Suggested unit tests:
        - Documentation-only module; tests should validate unit strings or
          enumerations if implemented.
    """

    @staticmethod
    def state_units() -> Dict[str, str]:
        return {
            "p_WB": "m",
            "v_WB": "m/s",
            "q_WB": "unitless",
            "omega_WB": "rad/s",
            "b_g": "rad/s",
            "b_a": "m/s^2",
            "A_a": "unitless",
            "T_BI": "rotation unitless, translation m",
            "T_BM": "rotation unitless, translation m",
            "g_W": "m/s^2",
            "m_W": "tesla",
        }

    @staticmethod
    def imu_units() -> Dict[str, str]:
        return {
            "z_omega": "rad/s",
            "R_omega": "(rad/s)^2",
            "z_accel": "m/s^2",
            "R_accel": "(m/s^2)^2",
        }

    @staticmethod
    def mag_units() -> Dict[str, str]:
        return {
            "z_m": "tesla",
            "R_m_raw": "tesla^2",
        }

    @staticmethod
    def noise_units() -> Dict[str, str]:
        return {
            "sigma_w_v": "m/s^2/sqrt(s)",
            "sigma_w_omega": "rad/s^2/sqrt(s)",
            "sigma_w_bg": "rad/s/sqrt(s)",
            "sigma_w_ba": "m/s^2/sqrt(s)",
            "sigma_w_Aa": "1/sqrt(s)",
            "sigma_w_BI": "m/sqrt(s), rad/sqrt(s)",
            "sigma_w_BM": "m/sqrt(s), rad/sqrt(s)",
            "sigma_w_BI_rho": "m/sqrt(s)",
            "sigma_w_BI_theta": "rad/sqrt(s)",
            "sigma_w_BM_rho": "m/sqrt(s)",
            "sigma_w_BM_theta": "rad/sqrt(s)",
            "sigma_w_g": "m/s^2/sqrt(s)",
            "sigma_w_m": "tesla/sqrt(s)",
        }
