################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Structured configuration schema for AHRS mounting calibration."""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
from dataclasses import fields
from dataclasses import replace
from typing import Any

import numpy as np


# IMU raw topic name
TOPIC_IMU_RAW: str = "imu_raw"
# IMU calibration topic name
TOPIC_IMU_CALIBRATION: str = "imu_calibration"
# Magnetometer topic name
TOPIC_MAGNETIC_FIELD: str = "magnetic_field"

# Base frame name
FRAME_BASE: str = "base_link"
# World frame name
FRAME_WORLD: str = "world"

# Bootstrap duration in seconds
BOOTSTRAP_SEC: float = 5.0
# Enforce flat and stationary assumption at boot
BOOTSTRAP_REQUIRE_FLAT_STATIONARY: bool = True
# Require a valid mag reference before anchoring
BOOTSTRAP_MAG_REFERENCE_REQUIRED: bool = False

# Steady detection window duration in seconds
STEADY_SEC: float = 2.0
# Windowing policy for steady detection
STEADY_WINDOW_TYPE: str = "sliding"

# Threshold on mean gyro magnitude in rad/s
STEADY_OMEGA_MEAN_THRESH: float | None = None
# Threshold on gyro covariance trace in (rad/s)^2
STEADY_OMEGA_COV_THRESH: float | None = None
# Threshold on accel covariance trace in (m/s^2)^2
STEADY_A_COV_THRESH: float | None = None
# Minimum mean accel magnitude in m/s^2
STEADY_A_NORM_MIN: float | None = None
# Maximum mean accel magnitude in m/s^2
STEADY_A_NORM_MAX: float | None = None
# Scale factor applied to bootstrap gyro covariance
STEADY_K_OMEGA: float | None = None

# Default gyro noise covariance diagonal in (rad/s)^2
# Observed driver gyro covariance diag
# ~[2.30e-06, 3.55e-06, 2.78e-06] (rad/s)^2
IMU_OMEGA_COV_DIAG: np.ndarray = np.array([1e-4, 1e-4, 1e-4], dtype=np.float64)

# Default accel noise covariance diagonal in (m/s^2)^2
# Observed driver accel covariance diag
# ~[2.67e-03, 1.23e-03, 2.20e-03] (m/s^2)^2
IMU_ACCEL_COV_DIAG: np.ndarray = np.array([1e-2, 1e-2, 1e-2], dtype=np.float64)

# Gravity direction cluster threshold in degrees
CLUSTER_G_DEG: float = 7.5
# Mag direction cluster threshold in degrees
CLUSTER_H_DEG: float = 12.5
# Maximum number of keyframes (0 means unlimited)
CLUSTER_K_MAX: int = 0
# Policy for dropping keyframes when exceeding K_max
CLUSTER_DROP_POLICY: str = "lowest_information"

# Minimum number of steady segments required
DIVERSITY_N_MIN: int | None = None
# Minimum tilt diversity in degrees
DIVERSITY_TILT_MIN_DEG: float | None = None
# Minimum yaw diversity in degrees
DIVERSITY_YAW_MIN_DEG: float | None = None
# Require mag-based yaw diversity when mag is available
DIVERSITY_USE_MAG_FOR_YAW: bool = True

# Minimum mag magnitude for covariance conversion in tesla
MAG_S_MIN_T: float = 1e-6
# Use driver covariance as prior for mag direction noise
MAG_USE_DRIVER_COV_AS_PRIOR: bool = True

# Initial mag direction noise diagonal in unitless^2
MAG_RM_INIT_DIAG: np.ndarray = np.array([1e-3, 1e-3, 1e-3], dtype=np.float64)
# Minimum mag direction noise diagonal in unitless^2
MAG_RM_MIN_DIAG: np.ndarray = np.array([1e-5, 1e-5, 1e-5], dtype=np.float64)
# Maximum mag direction noise diagonal in unitless^2
MAG_RM_MAX_DIAG: np.ndarray = np.array([1e-1, 1e-1, 1e-1], dtype=np.float64)
# Mag covariance matching update rate
MAG_RM_ALPHA: float = 0.01
# Enable mag disturbance gating
MAG_DISTURBANCE_GATE_ENABLED: bool = True
# Mag disturbance gating threshold
MAG_DISTURBANCE_GATE_SIGMA: float | None = None

# Publish dynamic TF while estimate changes
TF_PUBLISH_DYNAMIC: bool = True
# Publish static TF when stable
TF_PUBLISH_STATIC_WHEN_STABLE: bool = True
# Republish static TF when saving calibration
TF_REPUBLISH_STATIC_ON_SAVE: bool = True

# Stability window duration in seconds
STABILITY_STABLE_WINDOW_SEC: float | None = None
# Stability rotation change threshold in radians
STABILITY_STABLE_ROT_THRESH_RAD: float | None = None
# Require need_more_tilt to be false for stability
STABILITY_REQUIRE_NEED_MORE_TILT_FALSE: bool = True
# Require need_more_yaw to be false for stability
STABILITY_REQUIRE_NEED_MORE_YAW_FALSE: bool = True

# Save period in seconds
SAVE_PERIOD_SEC: float = 2.0
# Output path for saved calibration
SAVE_OUTPUT_PATH: str | None = None
# Output format for saved calibration
SAVE_FORMAT: str = "yaml"
# Use atomic write for persistence
SAVE_ATOMIC_WRITE: bool = True

# Solver backend identifier
SOLVER_BACKEND: str | None = None
# Maximum solver iterations per update
SOLVER_MAX_ITERS: int | None = None
# Maximum update rate in Hz
SOLVER_UPDATE_RATE_HZ: float | None = None
# Robust loss identifier
SOLVER_ROBUST_LOSS: str | None = None
# Robust scale for accel residuals
SOLVER_ROBUST_SCALE_A: float | None = None
# Robust scale for mag residuals
SOLVER_ROBUST_SCALE_M: float | None = None
# Regularization against mount absorbing bias
SOLVER_REG_MOUNT_VS_BIAS: float | None = None
# Diagnostics publish rate in Hz
DIAG_PUBLISH_RATE_HZ: float | None = None
# Track and report time sync slop
DIAG_TRACK_TIME_SYNC_SLOP: bool = True


class MountingParamsError(Exception):
    """Raised when mounting parameter validation fails."""


def _as_float_array(value: Any, name: str) -> np.ndarray:
    """Coerce a value to a float64 numpy array with shape (3,)."""
    array: np.ndarray = np.asarray(value, dtype=np.float64)
    if array.shape != (3,):
        raise MountingParamsError(f"{name} must have shape (3,)")
    if not np.all(np.isfinite(array)):
        raise MountingParamsError(f"{name} must contain finite values")
    return array


def _require_positive(value: float, name: str) -> None:
    """Require a positive value."""
    if value <= 0.0:
        raise MountingParamsError(f"{name} must be positive")


def _require_non_negative(value: float, name: str) -> None:
    """Require a non-negative value."""
    if value < 0.0:
        raise MountingParamsError(f"{name} must be non-negative")


def _validate_optional_positive(value: float | None, name: str) -> None:
    """Validate an optional positive parameter."""
    if value is None:
        return
    _require_positive(value, name)


def _validate_optional_non_negative(value: float | None, name: str) -> None:
    """Validate an optional non-negative parameter."""
    if value is None:
        return
    _require_non_negative(value, name)


def _validate_optional_int(value: int | None, name: str) -> None:
    """Validate an optional integer value."""
    if value is None:
        return
    if not isinstance(value, int):
        raise MountingParamsError(f"{name} must be an int")


def _validate_optional_positive_int(value: int | None, name: str) -> None:
    """Validate an optional positive integer value."""
    if value is None:
        return
    if not isinstance(value, int):
        raise MountingParamsError(f"{name} must be an int")
    if value <= 0:
        raise MountingParamsError(f"{name} must be positive")


def _validate_optional_non_negative_int(value: int | None, name: str) -> None:
    """Validate an optional non-negative integer value."""
    if value is None:
        return
    if not isinstance(value, int):
        raise MountingParamsError(f"{name} must be an int")
    if value < 0:
        raise MountingParamsError(f"{name} must be non-negative")


def _validate_positive_array(values: np.ndarray, name: str) -> None:
    """Validate that an array contains strictly positive values."""
    if np.any(values <= 0.0):
        raise MountingParamsError(f"{name} must be positive")


def _validate_non_negative_array(values: np.ndarray, name: str) -> None:
    """Validate that an array contains non-negative values."""
    if np.any(values < 0.0):
        raise MountingParamsError(f"{name} must be non-negative")


@dataclass(frozen=True)
class TopicsParams:
    """Topic names used by mounting calibration."""

    # IMU raw topic name
    imu_raw: str = TOPIC_IMU_RAW
    # IMU calibration topic name
    imu_calibration: str = TOPIC_IMU_CALIBRATION
    # Magnetometer topic name
    magnetic_field: str = TOPIC_MAGNETIC_FIELD


@dataclass(frozen=True)
class FramesParams:
    """Frame identifiers for mounting calibration."""

    # Base frame name
    base_frame: str = FRAME_BASE
    # World frame name
    world_frame: str = FRAME_WORLD


@dataclass(frozen=True)
class BootstrapParams:
    """Bootstrap configuration for startup anchoring."""

    # Bootstrap duration in seconds
    bootstrap_sec: float = BOOTSTRAP_SEC
    # Enforce flat and stationary boot assumption
    require_flat_stationary: bool = BOOTSTRAP_REQUIRE_FLAT_STATIONARY
    # Require mag reference for anchoring
    mag_reference_required: bool = BOOTSTRAP_MAG_REFERENCE_REQUIRED


@dataclass(frozen=True)
class SteadyParams:
    """Steady detection parameters for gating steady segments."""

    # Steady window duration in seconds
    steady_sec: float = STEADY_SEC
    # Windowing policy identifier
    window_type: str = STEADY_WINDOW_TYPE

    # Threshold on mean gyro magnitude in rad/s
    omega_mean_thresh: float | None = STEADY_OMEGA_MEAN_THRESH
    # Threshold on gyro covariance trace in (rad/s)^2
    omega_cov_thresh: float | None = STEADY_OMEGA_COV_THRESH
    # Threshold on accel covariance trace in (m/s^2)^2
    a_cov_thresh: float | None = STEADY_A_COV_THRESH
    # Minimum mean accel magnitude in m/s^2
    a_norm_min: float | None = STEADY_A_NORM_MIN
    # Maximum mean accel magnitude in m/s^2
    a_norm_max: float | None = STEADY_A_NORM_MAX
    # Scale factor applied to bootstrap gyro covariance
    k_omega: float | None = STEADY_K_OMEGA


@dataclass(frozen=True)
class ImuParams:
    """IMU covariance defaults used when drivers report unknown values."""

    # Default gyro noise covariance diagonal in (rad/s)^2
    omega_cov_diag: np.ndarray = field(
        default_factory=lambda: IMU_OMEGA_COV_DIAG.copy()
    )
    # Default accel noise covariance diagonal in (m/s^2)^2
    accel_cov_diag: np.ndarray = field(
        default_factory=lambda: IMU_ACCEL_COV_DIAG.copy()
    )

    def __post_init__(self) -> None:
        """Coerce covariance diagonals into float64 numpy arrays."""
        object.__setattr__(
            self,
            "omega_cov_diag",
            _as_float_array(self.omega_cov_diag, "imu.omega_cov_diag"),
        )
        object.__setattr__(
            self,
            "accel_cov_diag",
            _as_float_array(self.accel_cov_diag, "imu.accel_cov_diag"),
        )


@dataclass(frozen=True)
class ClusterParams:
    """Keyframe clustering thresholds and policies."""

    # Gravity direction clustering threshold in degrees
    cluster_g_deg: float = CLUSTER_G_DEG
    # Magnetometer direction clustering threshold in degrees
    cluster_h_deg: float = CLUSTER_H_DEG
    # Maximum number of keyframes (0 means unlimited)
    K_max: int = CLUSTER_K_MAX
    # Policy for dropping keyframes when full
    drop_policy: str = CLUSTER_DROP_POLICY


@dataclass(frozen=True)
class DiversityParams:
    """Motion diversity requirements."""

    # Minimum number of steady segments required
    N_min: int | None = DIVERSITY_N_MIN
    # Minimum tilt diversity in degrees
    tilt_min_deg: float | None = DIVERSITY_TILT_MIN_DEG
    # Minimum yaw diversity in degrees
    yaw_min_deg: float | None = DIVERSITY_YAW_MIN_DEG
    # Require mag-based yaw diversity when mag is available
    use_mag_for_yaw: bool = DIVERSITY_USE_MAG_FOR_YAW


@dataclass(frozen=True)
class MagParams:
    """Magnetometer direction-noise parameters."""

    # Minimum mag magnitude for covariance conversion in tesla
    s_min_T: float = MAG_S_MIN_T
    # Use driver covariance as prior for mag direction noise
    use_driver_cov_as_prior: bool = MAG_USE_DRIVER_COV_AS_PRIOR

    # Initial mag direction noise diagonal in unitless^2
    Rm_init_diag: np.ndarray = field(default_factory=lambda: MAG_RM_INIT_DIAG.copy())
    # Minimum mag direction noise diagonal in unitless^2
    Rm_min_diag: np.ndarray = field(default_factory=lambda: MAG_RM_MIN_DIAG.copy())
    # Maximum mag direction noise diagonal in unitless^2
    Rm_max_diag: np.ndarray = field(default_factory=lambda: MAG_RM_MAX_DIAG.copy())
    # Mag covariance matching update rate
    Rm_alpha: float = MAG_RM_ALPHA
    # Enable mag disturbance gating
    disturbance_gate_enabled: bool = MAG_DISTURBANCE_GATE_ENABLED
    # Mag disturbance gating threshold
    disturbance_gate_sigma: float | None = MAG_DISTURBANCE_GATE_SIGMA

    def __post_init__(self) -> None:
        """Coerce diagonal arrays into float64 numpy arrays."""
        object.__setattr__(
            self,
            "Rm_init_diag",
            _as_float_array(self.Rm_init_diag, "mag.Rm_init_diag"),
        )
        object.__setattr__(
            self,
            "Rm_min_diag",
            _as_float_array(self.Rm_min_diag, "mag.Rm_min_diag"),
        )
        object.__setattr__(
            self,
            "Rm_max_diag",
            _as_float_array(self.Rm_max_diag, "mag.Rm_max_diag"),
        )


@dataclass(frozen=True)
class TfParams:
    """TF publishing policy parameters."""

    # Publish dynamic TF while estimates change
    publish_dynamic: bool = TF_PUBLISH_DYNAMIC
    # Publish static TF once stable
    publish_static_when_stable: bool = TF_PUBLISH_STATIC_WHEN_STABLE
    # Republish static TF when saving
    republish_static_on_save: bool = TF_REPUBLISH_STATIC_ON_SAVE


@dataclass(frozen=True)
class StabilityParams:
    """Stability detection parameters for TF publication."""

    # Stability window duration in seconds
    stable_window_sec: float | None = STABILITY_STABLE_WINDOW_SEC
    # Max allowed rotation change in radians
    stable_rot_thresh_rad: float | None = STABILITY_STABLE_ROT_THRESH_RAD
    # Require need_more_tilt to be false for stability
    require_need_more_tilt_false: bool = STABILITY_REQUIRE_NEED_MORE_TILT_FALSE
    # Require need_more_yaw to be false for stability
    require_need_more_yaw_false: bool = STABILITY_REQUIRE_NEED_MORE_YAW_FALSE


@dataclass(frozen=True)
class SaveParams:
    """Persistence and output file parameters."""

    # Save period in seconds
    save_period_sec: float = SAVE_PERIOD_SEC
    # Output path for saved calibration
    output_path: str | None = SAVE_OUTPUT_PATH
    # Output format name
    format: str = SAVE_FORMAT
    # Use atomic write for persistence
    atomic_write: bool = SAVE_ATOMIC_WRITE


@dataclass(frozen=True)
class SolverParams:
    """Optimizer configuration parameters."""

    # Solver backend identifier
    backend: str | None = SOLVER_BACKEND
    # Maximum solver iterations per update
    max_iters: int | None = SOLVER_MAX_ITERS
    # Maximum update rate in Hz
    update_rate_hz: float | None = SOLVER_UPDATE_RATE_HZ
    # Robust loss identifier
    robust_loss: str | None = SOLVER_ROBUST_LOSS
    # Robust scale for accel residuals
    robust_scale_a: float | None = SOLVER_ROBUST_SCALE_A
    # Robust scale for mag residuals
    robust_scale_m: float | None = SOLVER_ROBUST_SCALE_M
    # Regularization against mount absorbing bias
    reg_mount_vs_bias: float | None = SOLVER_REG_MOUNT_VS_BIAS


@dataclass(frozen=True)
class DiagParams:
    """Diagnostics publishing parameters."""

    # Diagnostics publish rate in Hz
    publish_rate_hz: float | None = DIAG_PUBLISH_RATE_HZ
    # Track and report time-sync slop
    track_time_sync_slop: bool = DIAG_TRACK_TIME_SYNC_SLOP


@dataclass(frozen=True)
class MountingParams:
    """Complete configuration tree for mounting calibration."""

    topics: TopicsParams
    frames: FramesParams
    bootstrap: BootstrapParams
    steady: SteadyParams
    imu: ImuParams
    cluster: ClusterParams
    diversity: DiversityParams
    mag: MagParams
    tf: TfParams
    stability: StabilityParams
    save: SaveParams
    solver: SolverParams
    diag: DiagParams

    @classmethod
    def defaults(cls) -> MountingParams:
        """Return the default mounting parameter tree."""
        return cls(
            topics=TopicsParams(),
            frames=FramesParams(),
            bootstrap=BootstrapParams(),
            steady=SteadyParams(),
            imu=ImuParams(),
            cluster=ClusterParams(),
            diversity=DiversityParams(),
            mag=MagParams(),
            tf=TfParams(),
            stability=StabilityParams(),
            save=SaveParams(),
            solver=SolverParams(),
            diag=DiagParams(),
        )

    def validate(self) -> None:
        """Validate parameter invariants and constraints."""
        if not self.topics.imu_raw:
            raise MountingParamsError("topics.imu_raw must be set")
        if not self.topics.imu_calibration:
            raise MountingParamsError("topics.imu_calibration must be set")
        if not self.topics.magnetic_field:
            raise MountingParamsError("topics.magnetic_field must be set")
        if not self.frames.base_frame:
            raise MountingParamsError("frames.base_frame must be set")
        if not self.frames.world_frame:
            raise MountingParamsError("frames.world_frame must be set")

        _require_positive(self.bootstrap.bootstrap_sec, "bootstrap.bootstrap_sec")
        _require_positive(self.steady.steady_sec, "steady.steady_sec")
        _validate_optional_positive(
            self.steady.omega_mean_thresh, "steady.omega_mean_thresh"
        )
        _validate_optional_positive(
            self.steady.omega_cov_thresh, "steady.omega_cov_thresh"
        )
        _validate_optional_positive(self.steady.a_cov_thresh, "steady.a_cov_thresh")
        _validate_optional_positive(self.steady.a_norm_min, "steady.a_norm_min")
        _validate_optional_positive(self.steady.a_norm_max, "steady.a_norm_max")
        _validate_optional_positive(self.steady.k_omega, "steady.k_omega")

        _validate_non_negative_array(self.imu.omega_cov_diag, "imu.omega_cov_diag")
        _validate_non_negative_array(self.imu.accel_cov_diag, "imu.accel_cov_diag")

        _validate_optional_non_negative(
            self.cluster.cluster_g_deg, "cluster.cluster_g_deg"
        )
        _validate_optional_non_negative(
            self.cluster.cluster_h_deg, "cluster.cluster_h_deg"
        )
        _validate_optional_non_negative_int(self.cluster.K_max, "cluster.K_max")

        _validate_optional_non_negative_int(self.diversity.N_min, "diversity.N_min")
        _validate_optional_positive(
            self.diversity.tilt_min_deg, "diversity.tilt_min_deg"
        )
        _validate_optional_positive(self.diversity.yaw_min_deg, "diversity.yaw_min_deg")

        _require_positive(self.mag.s_min_T, "mag.s_min_T")
        _validate_optional_positive(self.mag.Rm_alpha, "mag.Rm_alpha")
        _validate_optional_positive(
            self.mag.disturbance_gate_sigma, "mag.disturbance_gate_sigma"
        )
        _validate_positive_array(self.mag.Rm_init_diag, "mag.Rm_init_diag")
        _validate_positive_array(self.mag.Rm_min_diag, "mag.Rm_min_diag")
        _validate_positive_array(self.mag.Rm_max_diag, "mag.Rm_max_diag")
        if np.any(self.mag.Rm_min_diag > self.mag.Rm_max_diag):
            raise MountingParamsError("mag.Rm_min_diag must not exceed mag.Rm_max_diag")

        _validate_optional_positive(
            self.stability.stable_window_sec, "stability.stable_window_sec"
        )
        _validate_optional_positive(
            self.stability.stable_rot_thresh_rad,
            "stability.stable_rot_thresh_rad",
        )

        _require_positive(self.save.save_period_sec, "save.save_period_sec")

        _validate_optional_int(self.solver.max_iters, "solver.max_iters")
        _validate_optional_positive(self.solver.update_rate_hz, "solver.update_rate_hz")
        _validate_optional_positive(self.solver.robust_scale_a, "solver.robust_scale_a")
        _validate_optional_positive(self.solver.robust_scale_m, "solver.robust_scale_m")
        _validate_optional_non_negative(
            self.solver.reg_mount_vs_bias, "solver.reg_mount_vs_bias"
        )

        _validate_optional_positive(self.diag.publish_rate_hz, "diag.publish_rate_hz")

    def replace(self, **namespace_overrides: Any) -> MountingParams:
        """Return a modified copy of the parameters."""
        return replace(self, **namespace_overrides)

    def as_nested_dict(self) -> dict[str, Any]:
        """Return a nested dict representation for debugging."""
        return _dataclass_to_dict(self)


def _dataclass_to_dict(value: Any) -> Any:
    """Convert dataclasses and numpy arrays into plain Python values."""
    if hasattr(value, "__dataclass_fields__"):
        return {
            field.name: _dataclass_to_dict(getattr(value, field.name))
            for field in fields(value)
        }
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value
