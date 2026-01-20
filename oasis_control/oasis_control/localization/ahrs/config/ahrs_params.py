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
        - epsilon_wall_future_ns: allowable future timestamp tolerance in
          nanoseconds.
        - dt_clock_jump_max_ns: clock jump detection threshold in
          nanoseconds.
        - dt_imu_max_ns: maximum IMU time gap allowed for replay coverage
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
    epsilon_wall_future_ns: int
    dt_clock_jump_max_ns: int
    dt_imu_max_ns: int
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
    def defaults() -> AhrsParams:
        """Return a stable default parameter set."""
        params: AhrsParams = AhrsParams(
            t_buffer_sec=2.0,
            epsilon_wall_future_ns=50_000_000,
            dt_clock_jump_max_ns=500_000_000,
            dt_imu_max_ns=100_000_000,
            sigma_w_v=0.05,
            sigma_w_omega=0.05,
            sigma_w_bg=1e-5,
            sigma_w_ba=1e-4,
            sigma_w_Aa=1e-4,
            sigma_w_BI_rho=1e-4,
            sigma_w_BI_theta=1e-4,
            sigma_w_BM_rho=1e-4,
            sigma_w_BM_theta=1e-4,
            sigma_w_g=1e-5,
            sigma_w_m=1e-5,
            alpha=0.01,
            R_min=AhrsParams._diag(1e-12),
            R_max=AhrsParams._diag(2.5e-9),
            R_m0_policy="from_first_cov_or_def",
            t_stationary_window_ns=500_000_000,
            tau_omega=0.5,
            tau_accel=0.5,
            R_v0=AhrsParams._diag(1e-4),
            R_omega0=AhrsParams._diag(1e-4),
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
        """Construct parameters from a mapping, rejecting unknown keys."""
        if not isinstance(params, Mapping):
            raise ValueError("params must be a mapping")
        unknown_keys: list[str] = sorted(set(params.keys()) - set(cls._field_order()))
        if unknown_keys:
            raise ValueError(f"unknown parameter: {unknown_keys[0]}")
        defaults: AhrsParams = cls.defaults()
        return cls(
            t_buffer_sec=cls._as_float(
                "t_buffer_sec",
                params.get("t_buffer_sec", defaults.t_buffer_sec),
            ),
            epsilon_wall_future_ns=cls._as_int(
                "epsilon_wall_future_ns",
                params.get("epsilon_wall_future_ns", defaults.epsilon_wall_future_ns),
            ),
            dt_clock_jump_max_ns=cls._as_int(
                "dt_clock_jump_max_ns",
                params.get("dt_clock_jump_max_ns", defaults.dt_clock_jump_max_ns),
            ),
            dt_imu_max_ns=cls._as_int(
                "dt_imu_max_ns",
                params.get("dt_imu_max_ns", defaults.dt_imu_max_ns),
            ),
            sigma_w_v=cls._as_float(
                "sigma_w_v", params.get("sigma_w_v", defaults.sigma_w_v)
            ),
            sigma_w_omega=cls._as_float(
                "sigma_w_omega",
                params.get("sigma_w_omega", defaults.sigma_w_omega),
            ),
            sigma_w_bg=cls._as_float(
                "sigma_w_bg", params.get("sigma_w_bg", defaults.sigma_w_bg)
            ),
            sigma_w_ba=cls._as_float(
                "sigma_w_ba", params.get("sigma_w_ba", defaults.sigma_w_ba)
            ),
            sigma_w_Aa=cls._as_float(
                "sigma_w_Aa", params.get("sigma_w_Aa", defaults.sigma_w_Aa)
            ),
            sigma_w_BI_rho=cls._as_float(
                "sigma_w_BI_rho",
                params.get("sigma_w_BI_rho", defaults.sigma_w_BI_rho),
            ),
            sigma_w_BI_theta=cls._as_float(
                "sigma_w_BI_theta",
                params.get("sigma_w_BI_theta", defaults.sigma_w_BI_theta),
            ),
            sigma_w_BM_rho=cls._as_float(
                "sigma_w_BM_rho",
                params.get("sigma_w_BM_rho", defaults.sigma_w_BM_rho),
            ),
            sigma_w_BM_theta=cls._as_float(
                "sigma_w_BM_theta",
                params.get("sigma_w_BM_theta", defaults.sigma_w_BM_theta),
            ),
            sigma_w_g=cls._as_float(
                "sigma_w_g", params.get("sigma_w_g", defaults.sigma_w_g)
            ),
            sigma_w_m=cls._as_float(
                "sigma_w_m", params.get("sigma_w_m", defaults.sigma_w_m)
            ),
            alpha=cls._as_float("alpha", params.get("alpha", defaults.alpha)),
            R_min=cls._as_matrix(
                params.get("R_min", defaults.R_min),
            ),
            R_max=cls._as_matrix(
                params.get("R_max", defaults.R_max),
            ),
            R_m0_policy=cls._as_str(
                "R_m0_policy",
                params.get("R_m0_policy", defaults.R_m0_policy),
            ),
            t_stationary_window_ns=cls._as_int(
                "t_stationary_window_ns",
                params.get(
                    "t_stationary_window_ns",
                    defaults.t_stationary_window_ns,
                ),
            ),
            tau_omega=cls._as_float(
                "tau_omega", params.get("tau_omega", defaults.tau_omega)
            ),
            tau_accel=cls._as_float(
                "tau_accel", params.get("tau_accel", defaults.tau_accel)
            ),
            R_v0=cls._as_matrix(
                params.get("R_v0", defaults.R_v0),
            ),
            R_omega0=cls._as_matrix(
                params.get("R_omega0", defaults.R_omega0),
            ),
            world_frame=cls._as_str(
                "world_frame", params.get("world_frame", defaults.world_frame)
            ),
            odom_frame=cls._as_str(
                "odom_frame", params.get("odom_frame", defaults.odom_frame)
            ),
            base_frame=cls._as_str(
                "base_frame", params.get("base_frame", defaults.base_frame)
            ),
            imu_frame=cls._as_str(
                "imu_frame", params.get("imu_frame", defaults.imu_frame)
            ),
            mag_frame=cls._as_str(
                "mag_frame", params.get("mag_frame", defaults.mag_frame)
            ),
        )

    def validate(self) -> None:
        """Validate parameters and raise ValueError on failure."""
        if self.t_buffer_sec <= 0.0:
            raise ValueError("t_buffer_sec must be > 0")
        if self.epsilon_wall_future_ns < 0:
            raise ValueError("epsilon_wall_future_ns must be >= 0")
        if self.dt_clock_jump_max_ns < 0:
            raise ValueError("dt_clock_jump_max_ns must be >= 0")
        if self.dt_imu_max_ns < 0:
            raise ValueError("dt_imu_max_ns must be >= 0")
        self._validate_non_negative("sigma_w_v", self.sigma_w_v)
        self._validate_non_negative("sigma_w_omega", self.sigma_w_omega)
        self._validate_non_negative("sigma_w_bg", self.sigma_w_bg)
        self._validate_non_negative("sigma_w_ba", self.sigma_w_ba)
        self._validate_non_negative("sigma_w_Aa", self.sigma_w_Aa)
        self._validate_non_negative("sigma_w_BI_rho", self.sigma_w_BI_rho)
        self._validate_non_negative("sigma_w_BI_theta", self.sigma_w_BI_theta)
        self._validate_non_negative("sigma_w_BM_rho", self.sigma_w_BM_rho)
        self._validate_non_negative("sigma_w_BM_theta", self.sigma_w_BM_theta)
        self._validate_non_negative("sigma_w_g", self.sigma_w_g)
        self._validate_non_negative("sigma_w_m", self.sigma_w_m)
        if not (0.0 <= self.alpha <= 1.0):
            raise ValueError("alpha must be in [0, 1]")
        self._validate_spd_3x3("R_min", self.R_min)
        self._validate_spd_3x3("R_max", self.R_max)
        for i in range(3):
            if self.R_max[i][i] < self.R_min[i][i]:
                raise ValueError("R_max diagonal must be >= R_min diagonal")
        if not self.R_m0_policy:
            raise ValueError("R_m0_policy must be non-empty")
        if self.t_stationary_window_ns <= 0:
            raise ValueError("t_stationary_window_ns must be > 0")
        if self.tau_omega <= 0.0:
            raise ValueError("tau_omega must be > 0")
        if self.tau_accel <= 0.0:
            raise ValueError("tau_accel must be > 0")
        self._validate_spd_3x3("R_v0", self.R_v0)
        self._validate_spd_3x3("R_omega0", self.R_omega0)
        self._validate_non_empty("world_frame", self.world_frame)
        self._validate_non_empty("odom_frame", self.odom_frame)
        self._validate_non_empty("base_frame", self.base_frame)
        self._validate_non_empty("imu_frame", self.imu_frame)
        self._validate_non_empty("mag_frame", self.mag_frame)

    def as_dict(self) -> dict[str, object]:
        """Return a JSON-serializable dict representation."""
        return {
            "t_buffer_sec": self.t_buffer_sec,
            "epsilon_wall_future_ns": self.epsilon_wall_future_ns,
            "dt_clock_jump_max_ns": self.dt_clock_jump_max_ns,
            "dt_imu_max_ns": self.dt_imu_max_ns,
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

    @staticmethod
    def _diag(value: float) -> list[list[float]]:
        return [[value, 0.0, 0.0], [0.0, value, 0.0], [0.0, 0.0, value]]

    @staticmethod
    def _as_float(name: str, value: object) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{name} must be a float")
        return float(value)

    @staticmethod
    def _as_int(name: str, value: object) -> int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{name} must be an int")
        return int(value)

    @staticmethod
    def _as_str(name: str, value: object) -> str:
        if not isinstance(value, str):
            raise ValueError(f"{name} must be a string")
        return value

    @staticmethod
    def _as_matrix(value: object) -> list[list[float]]:
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
            raise ValueError("matrix must be a sequence")
        matrix: list[list[float]] = []
        for row in value:
            if not isinstance(row, Sequence) or isinstance(row, (str, bytes)):
                raise ValueError("matrix must be a sequence")
            float_row: list[float] = []
            for item in row:
                if isinstance(item, bool) or not isinstance(item, (int, float)):
                    raise ValueError("matrix entries must be float")
                float_row.append(float(item))
            matrix.append(float_row)
        return matrix

    @staticmethod
    def _validate_non_negative(name: str, value: float) -> None:
        if value < 0.0:
            raise ValueError(f"{name} must be >= 0")

    @staticmethod
    def _validate_non_empty(name: str, value: str) -> None:
        if not value:
            raise ValueError(f"{name} must be non-empty")

    @staticmethod
    def _validate_spd_3x3(name: str, matrix: Sequence[Sequence[float]]) -> None:
        is_three: bool = len(matrix) == 3 and all(len(row) == 3 for row in matrix)
        if not is_three:
            raise ValueError(f"{name} must be 3x3 SPD")
        if not LinearAlgebra.is_spd(matrix):
            raise ValueError(f"{name} must be 3x3 SPD")

    @staticmethod
    def _field_order() -> list[str]:
        return [
            "t_buffer_sec",
            "epsilon_wall_future_ns",
            "dt_clock_jump_max_ns",
            "dt_imu_max_ns",
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
