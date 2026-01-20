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
from typing import ClassVar
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

    _FIELD_NAMES: ClassVar[tuple[str, ...]] = (
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
    )

    @staticmethod
    def defaults() -> AhrsParams:
        """Return conservative default parameters."""
        params: AhrsParams = AhrsParams(
            t_buffer_sec=2.0,
            ε_wall_future_ns=50_000_000,
            Δt_clock_jump_max_ns=500_000_000,
            Δt_imu_max_ns=100_000_000,
            sigma_w_v=1e-2,
            sigma_w_omega=1e-3,
            sigma_w_bg=1e-6,
            sigma_w_ba=1e-5,
            sigma_w_Aa=1e-6,
            sigma_w_BI_rho=1e-5,
            sigma_w_BI_theta=1e-6,
            sigma_w_BM_rho=1e-5,
            sigma_w_BM_theta=1e-6,
            sigma_w_g=1e-6,
            sigma_w_m=1e-6,
            alpha=0.01,
            R_min=_diag3(1e-12),
            R_max=_diag3(2.5e-9),
            R_m0_policy="from_first_cov_or_default",
            t_stationary_window_ns=250_000_000,
            tau_omega=3.0,
            tau_accel=3.0,
            R_v0=_diag3(1e-4),
            R_omega0=_diag3(1e-5),
            world_frame="world",
            odom_frame="odom",
            base_frame="base_link",
            imu_frame="imu",
            mag_frame="mag",
        )
        params.validate()
        return params

    @classmethod
    def from_dict(cls, params: Mapping[str, object]) -> AhrsParams:
        """Create params from a mapping with defaults and validation."""
        if not isinstance(params, Mapping):
            raise ValueError("params must be a mapping")
        unknown_keys: list[str] = sorted(
            key for key in params.keys() if key not in cls._FIELD_NAMES
        )
        if unknown_keys:
            raise ValueError(f"unknown parameter: {unknown_keys[0]}")
        defaults: AhrsParams = cls.defaults()
        values: dict[str, object] = {
            name: getattr(defaults, name) for name in cls._FIELD_NAMES
        }
        for key, value in params.items():
            values[key] = value
        created: AhrsParams = cls(**values)
        created.validate()
        return created

    def validate(self) -> None:
        """Validate parameter ranges and raise ValueError on failure."""
        if not _is_number(self.t_buffer_sec) or self.t_buffer_sec <= 0.0:
            raise ValueError("t_buffer_sec must be > 0")
        _validate_non_negative_int("ε_wall_future_ns", self.ε_wall_future_ns)
        _validate_non_negative_int(
            "Δt_clock_jump_max_ns",
            self.Δt_clock_jump_max_ns,
        )
        _validate_non_negative_int("Δt_imu_max_ns", self.Δt_imu_max_ns)
        _validate_non_negative_float("sigma_w_v", self.sigma_w_v)
        _validate_non_negative_float("sigma_w_omega", self.sigma_w_omega)
        _validate_non_negative_float("sigma_w_bg", self.sigma_w_bg)
        _validate_non_negative_float("sigma_w_ba", self.sigma_w_ba)
        _validate_non_negative_float("sigma_w_Aa", self.sigma_w_Aa)
        _validate_non_negative_float("sigma_w_BI_rho", self.sigma_w_BI_rho)
        _validate_non_negative_float("sigma_w_BI_theta", self.sigma_w_BI_theta)
        _validate_non_negative_float("sigma_w_BM_rho", self.sigma_w_BM_rho)
        _validate_non_negative_float("sigma_w_BM_theta", self.sigma_w_BM_theta)
        _validate_non_negative_float("sigma_w_g", self.sigma_w_g)
        _validate_non_negative_float("sigma_w_m", self.sigma_w_m)
        if not _is_number(self.alpha) or self.alpha < 0.0 or self.alpha > 1.0:
            raise ValueError("alpha must be in [0, 1]")
        _validate_spd_matrix("R_min", self.R_min)
        _validate_spd_matrix("R_max", self.R_max)
        for index in range(3):
            if self.R_max[index][index] < self.R_min[index][index]:
                raise ValueError("R_max diagonal must be >= R_min diagonal")
        if not isinstance(self.R_m0_policy, str) or not self.R_m0_policy:
            raise ValueError("R_m0_policy must be non-empty")
        _validate_positive_int(
            "t_stationary_window_ns",
            self.t_stationary_window_ns,
        )
        _validate_positive_float("tau_omega", self.tau_omega)
        _validate_positive_float("tau_accel", self.tau_accel)
        _validate_spd_matrix("R_v0", self.R_v0)
        _validate_spd_matrix("R_omega0", self.R_omega0)
        _validate_non_empty_string("world_frame", self.world_frame)
        _validate_non_empty_string("odom_frame", self.odom_frame)
        _validate_non_empty_string("base_frame", self.base_frame)
        _validate_non_empty_string("imu_frame", self.imu_frame)
        _validate_non_empty_string("mag_frame", self.mag_frame)

    def as_dict(self) -> dict[str, object]:
        """Return a JSON-serializable dict representation."""
        return {
            "t_buffer_sec": self.t_buffer_sec,
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
            "R_min": _clone_matrix(self.R_min),
            "R_max": _clone_matrix(self.R_max),
            "R_m0_policy": self.R_m0_policy,
            "t_stationary_window_ns": self.t_stationary_window_ns,
            "tau_omega": self.tau_omega,
            "tau_accel": self.tau_accel,
            "R_v0": _clone_matrix(self.R_v0),
            "R_omega0": _clone_matrix(self.R_omega0),
            "world_frame": self.world_frame,
            "odom_frame": self.odom_frame,
            "base_frame": self.base_frame,
            "imu_frame": self.imu_frame,
            "mag_frame": self.mag_frame,
        }


def _is_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _diag3(value: float) -> list[list[float]]:
    return [[value, 0.0, 0.0], [0.0, value, 0.0], [0.0, 0.0, value]]


def _clone_matrix(matrix: Sequence[Sequence[float]]) -> list[list[float]]:
    return [list(row) for row in matrix]


def _validate_non_negative_int(name: str, value: int) -> None:
    if not _is_int(value) or value < 0:
        raise ValueError(f"{name} must be >= 0")


def _validate_positive_int(name: str, value: int) -> None:
    if not _is_int(value) or value <= 0:
        raise ValueError(f"{name} must be > 0")


def _validate_non_negative_float(name: str, value: float) -> None:
    if not _is_number(value) or value < 0.0:
        raise ValueError(f"{name} must be >= 0")


def _validate_positive_float(name: str, value: float) -> None:
    if not _is_number(value) or value <= 0.0:
        raise ValueError(f"{name} must be > 0")


def _validate_non_empty_string(name: str, value: str) -> None:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be non-empty")


def _validate_spd_matrix(name: str, matrix: Sequence[Sequence[float]]) -> None:
    if not isinstance(matrix, Sequence) or isinstance(matrix, (str, bytes)):
        raise ValueError(f"{name} must be 3x3 SPD")
    if len(matrix) != 3:
        raise ValueError(f"{name} must be 3x3 SPD")
    for row in matrix:
        if not isinstance(row, Sequence) or isinstance(row, (str, bytes)):
            raise ValueError(f"{name} must be 3x3 SPD")
        if len(row) != 3:
            raise ValueError(f"{name} must be 3x3 SPD")
    if not LinearAlgebra.is_spd(matrix):
        raise ValueError(f"{name} must be 3x3 SPD")
