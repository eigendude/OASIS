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

from dataclasses import dataclass

from oasis_control.localization.ahrs.config.ahrs_params import AhrsParams


@dataclass(frozen=True, slots=True)
class AhrsConfig:
    """High-level configuration container for the AHRS core.

    Responsibility:
        Provide a cohesive configuration object that aggregates AhrsParams
        and exposes settings to the filter, models, and timing components.

    Purpose:
        Aggregate all AHRS parameters into a single object passed throughout
        the core, ensuring validation and immutability.

    Inputs/outputs:
        - Inputs: AhrsParams or configuration dictionaries.
        - Outputs: validated, immutable configuration used by the core.

    Dependencies:
        - Uses AhrsParams for schema and validation.

    Public API (to be implemented):
        - from_params(params)
        - validate()
        - as_dict()

    Data contract:
        - Contains all fields defined in AhrsParams.
        - May precompute derived values (e.g., noise variances).
        - Converts t_buffer_sec once in configuration into t_buffer_ns
          deterministically.

    Frames and units:
        - Frame identifiers are stored as strings.
        - Numeric fields use Units-defined scales.

    Determinism and edge cases:
        - Configuration is immutable once constructed for deterministic
          behavior.
        - Construction must not read ROS params or clocks.
        - Invalid parameters should raise validation errors.

    Equations:
        - No equations; configuration only.

    Numerical stability notes:
        - Ensure derived covariances are SPD.

    Suggested unit tests:
        - as_dict round-trip preserves values.
        - validate rejects missing required fields.
    """

    t_buffer_sec: float
    t_buffer_ns: int
    ε_wall_future_ns: int
    Δt_clock_jump_max_ns: int
    Δt_imu_max_ns: int
    sigma_w_v: float
    sigma_w_omega: float
    sigma_w_bg: float
    sigma_w_ba: float
    sigma_w_Aa: float
    sigma_w_BI_rho: float
    sigma_w_BI_theta: float
    sigma_w_BM_rho: float
    sigma_w_BM_theta: float
    sigma_w_g: float
    sigma_w_m: float
    alpha: float
    R_min: list[list[float]]
    R_max: list[list[float]]
    R_m0_policy: str
    t_stationary_window_ns: int
    tau_omega: float
    tau_accel: float
    R_v0: list[list[float]]
    R_omega0: list[list[float]]
    world_frame: str
    odom_frame: str
    base_frame: str
    imu_frame: str
    mag_frame: str

    @classmethod
    def from_params(cls, params: AhrsParams) -> AhrsConfig:
        """Build a config from validated parameters."""
        params.validate()
        t_buffer_ns: int = int(params.t_buffer_sec * 1_000_000_000)
        config: AhrsConfig = cls(
            t_buffer_sec=params.t_buffer_sec,
            t_buffer_ns=t_buffer_ns,
            ε_wall_future_ns=params.ε_wall_future_ns,
            Δt_clock_jump_max_ns=params.Δt_clock_jump_max_ns,
            Δt_imu_max_ns=params.Δt_imu_max_ns,
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
            R_min=[list(row) for row in params.R_min],
            R_max=[list(row) for row in params.R_max],
            R_m0_policy=params.R_m0_policy,
            t_stationary_window_ns=params.t_stationary_window_ns,
            tau_omega=params.tau_omega,
            tau_accel=params.tau_accel,
            R_v0=[list(row) for row in params.R_v0],
            R_omega0=[list(row) for row in params.R_omega0],
            world_frame=params.world_frame,
            odom_frame=params.odom_frame,
            base_frame=params.base_frame,
            imu_frame=params.imu_frame,
            mag_frame=params.mag_frame,
        )
        config.validate()
        return config

    def validate(self) -> None:
        """Validate configuration and derived buffer duration."""
        params: AhrsParams = AhrsParams(
            t_buffer_sec=self.t_buffer_sec,
            ε_wall_future_ns=self.ε_wall_future_ns,
            Δt_clock_jump_max_ns=self.Δt_clock_jump_max_ns,
            Δt_imu_max_ns=self.Δt_imu_max_ns,
            sigma_w_v=self.sigma_w_v,
            sigma_w_omega=self.sigma_w_omega,
            sigma_w_bg=self.sigma_w_bg,
            sigma_w_ba=self.sigma_w_ba,
            sigma_w_Aa=self.sigma_w_Aa,
            sigma_w_BI_rho=self.sigma_w_BI_rho,
            sigma_w_BI_theta=self.sigma_w_BI_theta,
            sigma_w_BM_rho=self.sigma_w_BM_rho,
            sigma_w_BM_theta=self.sigma_w_BM_theta,
            sigma_w_g=self.sigma_w_g,
            sigma_w_m=self.sigma_w_m,
            alpha=self.alpha,
            R_min=[list(row) for row in self.R_min],
            R_max=[list(row) for row in self.R_max],
            R_m0_policy=self.R_m0_policy,
            t_stationary_window_ns=self.t_stationary_window_ns,
            tau_omega=self.tau_omega,
            tau_accel=self.tau_accel,
            R_v0=[list(row) for row in self.R_v0],
            R_omega0=[list(row) for row in self.R_omega0],
            world_frame=self.world_frame,
            odom_frame=self.odom_frame,
            base_frame=self.base_frame,
            imu_frame=self.imu_frame,
            mag_frame=self.mag_frame,
        )
        params.validate()
        expected_ns: int = int(self.t_buffer_sec * 1_000_000_000)
        if self.t_buffer_ns != expected_ns:
            raise ValueError("t_buffer_ns must match t_buffer_sec")

    def as_dict(self) -> dict[str, object]:
        """Return a JSON-serializable dict representation."""
        return {
            "t_buffer_sec": self.t_buffer_sec,
            "t_buffer_ns": self.t_buffer_ns,
            "ε_wall_future_ns": self.ε_wall_future_ns,
            "Δt_clock_jump_max_ns": self.Δt_clock_jump_max_ns,
            "Δt_imu_max_ns": self.Δt_imu_max_ns,
            "sigma_w_v": self.sigma_w_v,
            "sigma_w_omega": self.sigma_w_omega,
            "sigma_w_bg": self.sigma_w_bg,
            "sigma_w_ba": self.sigma_w_ba,
            "sigma_w_Aa": self.sigma_w_Aa,
            "sigma_w_BI_rho": self.sigma_w_BI_rho,
            "sigma_w_BI_theta": self.sigma_w_BI_theta,
            "sigma_w_BM_rho": self.sigma_w_BM_rho,
            "sigma_w_BM_theta": self.sigma_w_BM_theta,
            "sigma_w_g": self.sigma_w_g,
            "sigma_w_m": self.sigma_w_m,
            "alpha": self.alpha,
            "R_min": [list(row) for row in self.R_min],
            "R_max": [list(row) for row in self.R_max],
            "R_m0_policy": self.R_m0_policy,
            "t_stationary_window_ns": self.t_stationary_window_ns,
            "tau_omega": self.tau_omega,
            "tau_accel": self.tau_accel,
            "R_v0": [list(row) for row in self.R_v0],
            "R_omega0": [list(row) for row in self.R_omega0],
            "world_frame": self.world_frame,
            "odom_frame": self.odom_frame,
            "base_frame": self.base_frame,
            "imu_frame": self.imu_frame,
            "mag_frame": self.mag_frame,
        }
