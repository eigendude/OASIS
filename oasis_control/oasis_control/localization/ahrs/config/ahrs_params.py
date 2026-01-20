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
from typing import Sequence

from oasis_control.localization.ahrs.math_utils.linalg import LinearAlgebra


@dataclass(frozen=True, slots=True)
class AhrsParams:
    """Configuration parameter definitions for the AHRS core.

    Responsibility:
        Document the configuration parameters used by the AHRS core, grouped
        by functional area (buffering, process noise, adaptation, frames).

    Purpose:
        Provide a structured list of parameters with units and expected ranges
        for the AHRS core components.

    Inputs/outputs:
        - Inputs: parameter dictionaries or configuration objects.
        - Outputs: validated parameter values for AHRS modules.

    Dependencies:
        - Used by AhrsConfig, ProcessModel, and ReplayEngine.

    Public API (to be implemented):
        - defaults()
        - validate()
        - from_dict(params)

    Data contract:
        Buffer/replay parameters:
        - t_buffer_sec: buffer horizon in seconds.
        - ε_wall_future_ns: allowable future timestamp tolerance in
          nanoseconds.
        - Δt_clock_jump_max_ns: clock jump detection threshold in
          nanoseconds.
        - Δt_imu_max_ns: maximum IMU time gap allowed for replay coverage
          in nanoseconds.

        Process noise intensities:
        - sigma_w_v: accel smoothness prior (m/s^2 / sqrt(s)).
        - sigma_w_omega: angular accel prior (rad/s^2 / sqrt(s)).
        - sigma_w_bg: gyro bias random walk (rad/s / sqrt(s)).
        - sigma_w_ba: accel bias random walk (m/s^2 / sqrt(s)).
        - sigma_w_Aa: accel calibration random walk (1 / sqrt(s)).
        - sigma_w_BI: IMU extrinsics random walk placeholder.
          Temporary 6D noise uses diag([sigma_rho^2 I3,
          sigma_theta^2 I3]) with sigma_rho in m / sqrt(s) and
          sigma_theta in rad / sqrt(s).
        - sigma_w_BM: mag extrinsics random walk placeholder.
          Temporary 6D noise uses diag([sigma_rho^2 I3,
          sigma_theta^2 I3]) with sigma_rho in m / sqrt(s) and
          sigma_theta in rad / sqrt(s).
        - Preferred split parameters: sigma_w_BI_rho, sigma_w_BI_theta,
          sigma_w_BM_rho, sigma_w_BM_theta.
        - sigma_w_g: gravity random walk (m/s^2 / sqrt(s)).
        - sigma_w_m: magnetic field random walk (tesla / sqrt(s)).

        Magnetometer adaptation:
        - alpha: adaptation rate in [0, 1].
        - R_min: minimum SPD covariance (tesla^2).
        - R_max: maximum SPD covariance (tesla^2).
        - R_m0_policy: initial R_m policy for startup.

        Stationary detection and pseudo-measurements:
        - t_stationary_window_ns: stationary window length in nanoseconds.
        - tau_omega: gyro whitened threshold (dimensionless).
        - tau_accel: accel whitened threshold (dimensionless).
        - R_v0: baseline ZUPT covariance (3, 3), full SPD.
        - R_omega0: baseline no-turn covariance (3, 3), full SPD.

        Frame identifiers:
        - world_frame, odom_frame, base_frame, imu_frame, mag_frame.
        - Used only by higher-level integration; core is ROS-agnostic.

        Naming + conversion:
        - ns everywhere except buffer length.
        - The only seconds input is t_buffer_sec, which is converted once in
          configuration into t_buffer_ns deterministically.
        - No float seconds are used for keying, ordering, equality,
          attachment, or replay.

    Frames and units:
        - Units match Units definitions and process noise conventions.

    Determinism and edge cases:
        - Parameters are explicit inputs; no implicit ROS parameter access.
        - validate() must reject invalid ranges (negative variances).
        - alpha outside [0, 1] should be rejected.

    Equations:
        - No equations; parameters configure other modules.

    Numerical stability notes:
        - Noise values too small may cause S to be ill-conditioned.

    Suggested unit tests:
        - validate rejects negative variances.
        - defaults produce a consistent parameter set.
    """

    t_buffer_sec: float
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

    @staticmethod
    def defaults() -> "AhrsParams":
        """Return a conservative, stable default configuration."""
        R_min: list[list[float]] = [
            [1.0e-12, 0.0, 0.0],
            [0.0, 1.0e-12, 0.0],
            [0.0, 0.0, 1.0e-12],
        ]
        R_max: list[list[float]] = [
            [2.5e-9, 0.0, 0.0],
            [0.0, 2.5e-9, 0.0],
            [0.0, 0.0, 2.5e-9],
        ]
        R_v0: list[list[float]] = [
            [1.0e-3, 0.0, 0.0],
            [0.0, 1.0e-3, 0.0],
            [0.0, 0.0, 1.0e-3],
        ]
        R_omega0: list[list[float]] = [
            [1.0e-3, 0.0, 0.0],
            [0.0, 1.0e-3, 0.0],
            [0.0, 0.0, 1.0e-3],
        ]
        return AhrsParams(
            t_buffer_sec=2.0,
            ε_wall_future_ns=50_000_000,
            Δt_clock_jump_max_ns=500_000_000,
            Δt_imu_max_ns=100_000_000,
            sigma_w_v=1.0e-2,
            sigma_w_omega=1.0e-2,
            sigma_w_bg=1.0e-5,
            sigma_w_ba=1.0e-4,
            sigma_w_Aa=1.0e-6,
            sigma_w_BI_rho=1.0e-5,
            sigma_w_BI_theta=1.0e-5,
            sigma_w_BM_rho=1.0e-5,
            sigma_w_BM_theta=1.0e-5,
            sigma_w_g=1.0e-6,
            sigma_w_m=1.0e-6,
            alpha=0.01,
            R_min=R_min,
            R_max=R_max,
            R_m0_policy="from_first_cov_or_def",
            t_stationary_window_ns=250_000_000,
            tau_omega=2.5,
            tau_accel=2.5,
            R_v0=R_v0,
            R_omega0=R_omega0,
            world_frame="world",
            odom_frame="odom",
            base_frame="base_link",
            imu_frame="imu",
            mag_frame="mag",
        )

    @classmethod
    def from_dict(cls, params: Mapping[str, object]) -> "AhrsParams":
        """Construct params from a mapping, rejecting unknown keys."""
        known_fields: set[str] = set(cls.__dataclass_fields__.keys())
        for key in sorted(params.keys()):
            if key not in known_fields:
                raise ValueError(f"unknown parameter: {key}")
        defaults: AhrsParams = cls.defaults()
        return cls(
            t_buffer_sec=params.get("t_buffer_sec", defaults.t_buffer_sec),
            ε_wall_future_ns=params.get(
                "ε_wall_future_ns", defaults.ε_wall_future_ns
            ),
            Δt_clock_jump_max_ns=params.get(
                "Δt_clock_jump_max_ns", defaults.Δt_clock_jump_max_ns
            ),
            Δt_imu_max_ns=params.get("Δt_imu_max_ns", defaults.Δt_imu_max_ns),
            sigma_w_v=params.get("sigma_w_v", defaults.sigma_w_v),
            sigma_w_omega=params.get("sigma_w_omega", defaults.sigma_w_omega),
            sigma_w_bg=params.get("sigma_w_bg", defaults.sigma_w_bg),
            sigma_w_ba=params.get("sigma_w_ba", defaults.sigma_w_ba),
            sigma_w_Aa=params.get("sigma_w_Aa", defaults.sigma_w_Aa),
            sigma_w_BI_rho=params.get(
                "sigma_w_BI_rho", defaults.sigma_w_BI_rho
            ),
            sigma_w_BI_theta=params.get(
                "sigma_w_BI_theta", defaults.sigma_w_BI_theta
            ),
            sigma_w_BM_rho=params.get(
                "sigma_w_BM_rho", defaults.sigma_w_BM_rho
            ),
            sigma_w_BM_theta=params.get(
                "sigma_w_BM_theta", defaults.sigma_w_BM_theta
            ),
            sigma_w_g=params.get("sigma_w_g", defaults.sigma_w_g),
            sigma_w_m=params.get("sigma_w_m", defaults.sigma_w_m),
            alpha=params.get("alpha", defaults.alpha),
            R_min=params.get("R_min", defaults.R_min),
            R_max=params.get("R_max", defaults.R_max),
            R_m0_policy=params.get("R_m0_policy", defaults.R_m0_policy),
            t_stationary_window_ns=params.get(
                "t_stationary_window_ns", defaults.t_stationary_window_ns
            ),
            tau_omega=params.get("tau_omega", defaults.tau_omega),
            tau_accel=params.get("tau_accel", defaults.tau_accel),
            R_v0=params.get("R_v0", defaults.R_v0),
            R_omega0=params.get("R_omega0", defaults.R_omega0),
            world_frame=params.get("world_frame", defaults.world_frame),
            odom_frame=params.get("odom_frame", defaults.odom_frame),
            base_frame=params.get("base_frame", defaults.base_frame),
            imu_frame=params.get("imu_frame", defaults.imu_frame),
            mag_frame=params.get("mag_frame", defaults.mag_frame),
        )

    def validate(self) -> None:
        """Validate parameter values and raise ValueError on failure."""
        if self.t_buffer_sec <= 0.0:
            raise ValueError("t_buffer_sec must be > 0")
        if self.ε_wall_future_ns < 0:
            raise ValueError("ε_wall_future_ns must be >= 0")
        if self.Δt_clock_jump_max_ns < 0:
            raise ValueError("Δt_clock_jump_max_ns must be >= 0")
        if self.Δt_imu_max_ns < 0:
            raise ValueError("Δt_imu_max_ns must be >= 0")
        for name in (
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
        ):
            value: float = getattr(self, name)
            if value < 0.0:
                raise ValueError(f"{name} must be >= 0")
        if self.alpha < 0.0 or self.alpha > 1.0:
            raise ValueError("alpha must be in [0, 1]")
        self._validate_spd_matrix("R_min", self.R_min)
        self._validate_spd_matrix("R_max", self.R_max)
        if any(
            self.R_max[i][i] < self.R_min[i][i]
            for i in range(3)
        ):
            raise ValueError("R_max diagonal must be >= R_min diagonal")
        if not self.R_m0_policy:
            raise ValueError("R_m0_policy must be non-empty")
        if self.t_stationary_window_ns <= 0:
            raise ValueError("t_stationary_window_ns must be > 0")
        if self.tau_omega <= 0.0:
            raise ValueError("tau_omega must be > 0")
        if self.tau_accel <= 0.0:
            raise ValueError("tau_accel must be > 0")
        self._validate_spd_matrix("R_v0", self.R_v0)
        self._validate_spd_matrix("R_omega0", self.R_omega0)
        for name in (
            "world_frame",
            "odom_frame",
            "base_frame",
            "imu_frame",
            "mag_frame",
        ):
            if not getattr(self, name):
                raise ValueError(f"{name} must be non-empty")

    def as_dict(self) -> dict[str, object]:
        """Return a JSON-serializable dict representation."""
        return {
            "t_buffer_sec": float(self.t_buffer_sec),
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
    def _validate_spd_matrix(
        name: str, matrix: Sequence[Sequence[float]]
    ) -> None:
        if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
            raise ValueError(f"{name} must be 3x3 SPD")
        if not LinearAlgebra.is_spd(matrix):
            raise ValueError(f"{name} must be 3x3 SPD")
