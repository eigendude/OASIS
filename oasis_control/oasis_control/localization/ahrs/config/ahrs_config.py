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
from typing import Mapping

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
    def from_params(cls, params: AhrsParams | Mapping[str, object]) -> "AhrsConfig":
        """Create a configuration from parameters or a parameter mapping."""
        if isinstance(params, Mapping):
            ahrs_params: AhrsParams = AhrsParams.from_dict(params)
        else:
            ahrs_params = params
        t_buffer_ns: int = cls._to_buffer_ns(ahrs_params.t_buffer_sec)
        return cls(
            t_buffer_sec=ahrs_params.t_buffer_sec,
            t_buffer_ns=t_buffer_ns,
            ε_wall_future_ns=ahrs_params.ε_wall_future_ns,
            Δt_clock_jump_max_ns=ahrs_params.Δt_clock_jump_max_ns,
            Δt_imu_max_ns=ahrs_params.Δt_imu_max_ns,
            sigma_w_v=ahrs_params.sigma_w_v,
            sigma_w_omega=ahrs_params.sigma_w_omega,
            sigma_w_bg=ahrs_params.sigma_w_bg,
            sigma_w_ba=ahrs_params.sigma_w_ba,
            sigma_w_Aa=ahrs_params.sigma_w_Aa,
            sigma_w_BI_rho=ahrs_params.sigma_w_BI_rho,
            sigma_w_BI_theta=ahrs_params.sigma_w_BI_theta,
            sigma_w_BM_rho=ahrs_params.sigma_w_BM_rho,
            sigma_w_BM_theta=ahrs_params.sigma_w_BM_theta,
            sigma_w_g=ahrs_params.sigma_w_g,
            sigma_w_m=ahrs_params.sigma_w_m,
            alpha=ahrs_params.alpha,
            R_min=ahrs_params.R_min,
            R_max=ahrs_params.R_max,
            R_m0_policy=ahrs_params.R_m0_policy,
            t_stationary_window_ns=ahrs_params.t_stationary_window_ns,
            tau_omega=ahrs_params.tau_omega,
            tau_accel=ahrs_params.tau_accel,
            R_v0=ahrs_params.R_v0,
            R_omega0=ahrs_params.R_omega0,
            world_frame=ahrs_params.world_frame,
            odom_frame=ahrs_params.odom_frame,
            base_frame=ahrs_params.base_frame,
            imu_frame=ahrs_params.imu_frame,
            mag_frame=ahrs_params.mag_frame,
        )

    def validate(self) -> None:
        """Validate configuration values and derived conversions."""
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
            R_min=self.R_min,
            R_max=self.R_max,
            R_m0_policy=self.R_m0_policy,
            t_stationary_window_ns=self.t_stationary_window_ns,
            tau_omega=self.tau_omega,
            tau_accel=self.tau_accel,
            R_v0=self.R_v0,
            R_omega0=self.R_omega0,
            world_frame=self.world_frame,
            odom_frame=self.odom_frame,
            base_frame=self.base_frame,
            imu_frame=self.imu_frame,
            mag_frame=self.mag_frame,
        )
        params.validate()
        if self.t_buffer_ns != self._to_buffer_ns(self.t_buffer_sec):
            raise ValueError("t_buffer_ns must match t_buffer_sec")

    def as_dict(self) -> dict[str, object]:
        """Return a JSON-serializable dict representation."""
        return {
            "t_buffer_sec": float(self.t_buffer_sec),
            "t_buffer_ns": int(self.t_buffer_ns),
            "ε_wall_future_ns": int(self.ε_wall_future_ns),
            "Δt_clock_jump_max_ns": int(self.Δt_clock_jump_max_ns),
            "Δt_imu_max_ns": int(self.Δt_imu_max_ns),
            "sigma_w_v": float(self.sigma_w_v),
            "sigma_w_omega": float(self.sigma_w_omega),
            "sigma_w_bg": float(self.sigma_w_bg),
            "sigma_w_ba": float(self.sigma_w_ba),
            "sigma_w_Aa": float(self.sigma_w_Aa),
            "sigma_w_BI_rho": float(self.sigma_w_BI_rho),
            "sigma_w_BI_theta": float(self.sigma_w_BI_theta),
            "sigma_w_BM_rho": float(self.sigma_w_BM_rho),
            "sigma_w_BM_theta": float(self.sigma_w_BM_theta),
            "sigma_w_g": float(self.sigma_w_g),
            "sigma_w_m": float(self.sigma_w_m),
            "alpha": float(self.alpha),
            "R_min": [list(row) for row in self.R_min],
            "R_max": [list(row) for row in self.R_max],
            "R_m0_policy": self.R_m0_policy,
            "t_stationary_window_ns": int(self.t_stationary_window_ns),
            "tau_omega": float(self.tau_omega),
            "tau_accel": float(self.tau_accel),
            "R_v0": [list(row) for row in self.R_v0],
            "R_omega0": [list(row) for row in self.R_omega0],
            "world_frame": self.world_frame,
            "odom_frame": self.odom_frame,
            "base_frame": self.base_frame,
            "imu_frame": self.imu_frame,
            "mag_frame": self.mag_frame,
        }

    @staticmethod
    def _to_buffer_ns(t_buffer_sec: float) -> int:
        return int(t_buffer_sec * 1_000_000_000)
