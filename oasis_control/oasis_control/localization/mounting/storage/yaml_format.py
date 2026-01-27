################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""YAML schema utilities for mounting calibration snapshots."""

from __future__ import annotations

import importlib.machinery
import importlib.util
import numbers
from dataclasses import dataclass
from typing import Any
from typing import cast

import numpy as np


_yaml_spec: importlib.machinery.ModuleSpec | None = importlib.util.find_spec("yaml")
_yaml: Any
if _yaml_spec is not None:
    import yaml as _yaml
else:
    _yaml = None

yaml: Any = _yaml


class MountingYamlError(Exception):
    """Raised when the mounting YAML schema is invalid."""


@dataclass(frozen=True)
class FramesYaml:
    """Frame identifiers used by the calibration snapshot.

    Attributes:
        base_frame: Body frame id for the robot
        imu_frame: IMU sensor frame id
        mag_frame: Magnetometer sensor frame id
    """

    base_frame: str
    imu_frame: str
    mag_frame: str

    def __post_init__(self) -> None:
        """Validate frame identifiers."""
        object.__setattr__(
            self, "base_frame", _require_str(self.base_frame, "base_frame")
        )
        object.__setattr__(self, "imu_frame", _require_str(self.imu_frame, "imu_frame"))
        object.__setattr__(self, "mag_frame", _require_str(self.mag_frame, "mag_frame"))


@dataclass(frozen=True)
class FlagsYaml:
    """Boolean state flags captured with the snapshot.

    Attributes:
        anchored: Whether the mount estimate is anchored to priors
        mag_reference_invalid: Whether the mag reference is invalid
        mag_disturbance_detected: Whether mag disturbances were detected
        mag_dir_prior_from_driver_cov: Whether mag direction prior uses driver
            covariance
    """

    anchored: bool
    mag_reference_invalid: bool
    mag_disturbance_detected: bool
    mag_dir_prior_from_driver_cov: bool

    def __post_init__(self) -> None:
        """Validate flag values."""
        object.__setattr__(self, "anchored", _require_bool(self.anchored, "anchored"))
        object.__setattr__(
            self,
            "mag_reference_invalid",
            _require_bool(self.mag_reference_invalid, "mag_reference_invalid"),
        )
        object.__setattr__(
            self,
            "mag_disturbance_detected",
            _require_bool(self.mag_disturbance_detected, "mag_disturbance_detected"),
        )
        object.__setattr__(
            self,
            "mag_dir_prior_from_driver_cov",
            _require_bool(
                self.mag_dir_prior_from_driver_cov, "mag_dir_prior_from_driver_cov"
            ),
        )


@dataclass(frozen=True)
class MountRotationYaml:
    """Rotation from a sensor frame into the body frame.

    Attributes:
        quaternion_wxyz: Unit quaternion in [w, x, y, z] order
        rot_cov_rad2: 3x3 rotation covariance in tangent space, rad^2
    """

    quaternion_wxyz: np.ndarray
    rot_cov_rad2: np.ndarray

    def __post_init__(self) -> None:
        """Coerce array types and validate shapes."""
        quaternion: np.ndarray = _coerce_array(
            self.quaternion_wxyz, "quaternion_wxyz", (4,)
        )
        rot_cov: np.ndarray = _coerce_array(self.rot_cov_rad2, "rot_cov_rad2", (3, 3))
        object.__setattr__(self, "quaternion_wxyz", quaternion)
        object.__setattr__(self, "rot_cov_rad2", rot_cov)


@dataclass(frozen=True)
class ImuNuisanceYaml:
    """IMU nuisance parameters and uncertainty.

    Attributes:
        accel_bias_mps2: Accelerometer bias in m/s^2
        accel_A_row_major: Accelerometer scale matrix A_a, row-major 3x3
        accel_param_cov_row_major_12x12: 12x12 accel parameter covariance
        gyro_bias_rads: Gyro bias in rad/s
        gyro_bias_cov_row_major_3x3: 3x3 gyro bias covariance, row-major
    """

    accel_bias_mps2: np.ndarray
    accel_A_row_major: np.ndarray
    accel_param_cov_row_major_12x12: np.ndarray
    gyro_bias_rads: np.ndarray
    gyro_bias_cov_row_major_3x3: np.ndarray

    def __post_init__(self) -> None:
        """Coerce array types and validate shapes."""
        accel_bias: np.ndarray = _coerce_array(
            self.accel_bias_mps2, "accel_bias_mps2", (3,)
        )
        accel_a: np.ndarray = _coerce_array(
            self.accel_A_row_major, "accel_A_row_major", (9,)
        )
        accel_cov: np.ndarray = _coerce_array(
            self.accel_param_cov_row_major_12x12,
            "accel_param_cov_row_major_12x12",
            (144,),
        )
        gyro_bias: np.ndarray = _coerce_array(
            self.gyro_bias_rads, "gyro_bias_rads", (3,)
        )
        gyro_cov: np.ndarray = _coerce_array(
            self.gyro_bias_cov_row_major_3x3, "gyro_bias_cov_row_major_3x3", (9,)
        )
        object.__setattr__(self, "accel_bias_mps2", accel_bias)
        object.__setattr__(self, "accel_A_row_major", accel_a)
        object.__setattr__(self, "accel_param_cov_row_major_12x12", accel_cov)
        object.__setattr__(self, "gyro_bias_rads", gyro_bias)
        object.__setattr__(self, "gyro_bias_cov_row_major_3x3", gyro_cov)


@dataclass(frozen=True)
class MagNuisanceYaml:
    """Magnetometer nuisance parameters and uncertainty.

    Attributes:
        offset_t: Magnetometer offset in tesla
        offset_cov_row_major_3x3: 3x3 offset covariance, row-major
        R_m_unitless2_row_major_3x3: Current mag direction noise, unitless^2
        R_m0_unitless2_row_major_3x3: Initial mag direction noise, unitless^2
    """

    offset_t: np.ndarray
    offset_cov_row_major_3x3: np.ndarray
    R_m_unitless2_row_major_3x3: np.ndarray
    R_m0_unitless2_row_major_3x3: np.ndarray

    def __post_init__(self) -> None:
        """Coerce array types and validate shapes."""
        offset: np.ndarray = _coerce_array(self.offset_t, "offset_t", (3,))
        offset_cov: np.ndarray = _coerce_array(
            self.offset_cov_row_major_3x3, "offset_cov_row_major_3x3", (9,)
        )
        r_m: np.ndarray = _coerce_array(
            self.R_m_unitless2_row_major_3x3,
            "R_m_unitless2_row_major_3x3",
            (9,),
        )
        r_m0: np.ndarray = _coerce_array(
            self.R_m0_unitless2_row_major_3x3,
            "R_m0_unitless2_row_major_3x3",
            (9,),
        )
        object.__setattr__(self, "offset_t", offset)
        object.__setattr__(self, "offset_cov_row_major_3x3", offset_cov)
        object.__setattr__(self, "R_m_unitless2_row_major_3x3", r_m)
        object.__setattr__(self, "R_m0_unitless2_row_major_3x3", r_m0)


@dataclass(frozen=True)
class DiversityYaml:
    """Diversity metrics for the calibration dataset.

    Attributes:
        gravity_max_angle_deg: Max gravity direction span, degrees
        mag_proj_max_angle_deg: Max mag projection span, degrees
    """

    gravity_max_angle_deg: float
    mag_proj_max_angle_deg: float

    def __post_init__(self) -> None:
        """Validate diversity values."""
        object.__setattr__(
            self,
            "gravity_max_angle_deg",
            _require_float(self.gravity_max_angle_deg, "gravity_max_angle_deg"),
        )
        object.__setattr__(
            self,
            "mag_proj_max_angle_deg",
            _require_float(self.mag_proj_max_angle_deg, "mag_proj_max_angle_deg"),
        )


@dataclass(frozen=True)
class QualityYaml:
    """Quality metrics for the calibration snapshot.

    Attributes:
        raw_samples: Raw samples ingested
        steady_segments: Steady segments accepted
        keyframes: Keyframes retained
        total_duration_sec: Total duration used, seconds
        accel_residual_rms: Accel direction residual RMS
        mag_residual_rms: Mag direction residual RMS
        diversity: Diversity metrics for conditioning
    """

    raw_samples: int
    steady_segments: int
    keyframes: int
    total_duration_sec: float
    accel_residual_rms: float
    mag_residual_rms: float
    diversity: DiversityYaml

    def __post_init__(self) -> None:
        """Validate quality metrics."""
        object.__setattr__(
            self, "raw_samples", _require_int(self.raw_samples, "raw_samples")
        )
        object.__setattr__(
            self,
            "steady_segments",
            _require_int(self.steady_segments, "steady_segments"),
        )
        object.__setattr__(self, "keyframes", _require_int(self.keyframes, "keyframes"))
        object.__setattr__(
            self,
            "total_duration_sec",
            _require_float(self.total_duration_sec, "total_duration_sec"),
        )
        object.__setattr__(
            self,
            "accel_residual_rms",
            _require_float(self.accel_residual_rms, "accel_residual_rms"),
        )
        object.__setattr__(
            self,
            "mag_residual_rms",
            _require_float(self.mag_residual_rms, "mag_residual_rms"),
        )
        object.__setattr__(self, "diversity", _require_diversity(self.diversity))


@dataclass(frozen=True)
class MountingSnapshotYaml:
    """Top-level mounting calibration snapshot as persisted to YAML.

    Attributes:
        format_version: Snapshot format version, must be 1
        frames: Frame identifiers for body, IMU, and mag
        flags: Boolean flags describing solver state
        R_BI: IMU-to-body rotation
        R_BM: Magnetometer-to-body rotation
        imu: IMU nuisance parameters
        mag: Magnetometer nuisance parameters
        quality: Quality metadata
    """

    format_version: int
    frames: FramesYaml
    flags: FlagsYaml
    R_BI: MountRotationYaml
    R_BM: MountRotationYaml
    imu: ImuNuisanceYaml
    mag: MagNuisanceYaml
    quality: QualityYaml

    def __post_init__(self) -> None:
        """Validate snapshot components and version."""
        object.__setattr__(
            self, "format_version", _require_int(self.format_version, "format_version")
        )
        if self.format_version != 1:
            raise MountingYamlError("format_version must be 1")
        object.__setattr__(self, "frames", _require_frames(self.frames))
        object.__setattr__(self, "flags", _require_flags(self.flags))
        object.__setattr__(self, "R_BI", _require_mount(self.R_BI, "R_BI"))
        object.__setattr__(self, "R_BM", _require_mount(self.R_BM, "R_BM"))
        object.__setattr__(self, "imu", _require_imu(self.imu))
        object.__setattr__(self, "mag", _require_mag(self.mag))
        object.__setattr__(self, "quality", _require_quality(self.quality))


def snapshot_to_dict(snapshot: MountingSnapshotYaml) -> dict[str, object]:
    """Convert a snapshot to a YAML-safe dictionary."""
    mounts: dict[str, object] = {
        "R_BI": _mount_to_dict(snapshot.R_BI),
        "R_BM": _mount_to_dict(snapshot.R_BM),
    }
    nuisance: dict[str, object] = {
        "imu": _imu_to_dict(snapshot.imu),
        "mag": _mag_to_dict(snapshot.mag),
    }
    data: dict[str, object] = {
        "format_version": snapshot.format_version,
        "frames": {
            "base_frame": snapshot.frames.base_frame,
            "imu_frame": snapshot.frames.imu_frame,
            "mag_frame": snapshot.frames.mag_frame,
        },
        "flags": {
            "anchored": snapshot.flags.anchored,
            "mag_reference_invalid": snapshot.flags.mag_reference_invalid,
            "mag_disturbance_detected": snapshot.flags.mag_disturbance_detected,
            "mag_dir_prior_from_driver_cov": snapshot.flags.mag_dir_prior_from_driver_cov,
        },
        "mounts": mounts,
        "nuisance": nuisance,
        "quality": {
            "raw_samples": snapshot.quality.raw_samples,
            "steady_segments": snapshot.quality.steady_segments,
            "keyframes": snapshot.quality.keyframes,
            "total_duration_sec": snapshot.quality.total_duration_sec,
            "accel_residual_rms": snapshot.quality.accel_residual_rms,
            "mag_residual_rms": snapshot.quality.mag_residual_rms,
            "diversity": {
                "gravity_max_angle_deg": snapshot.quality.diversity.gravity_max_angle_deg,
                "mag_proj_max_angle_deg": snapshot.quality.diversity.mag_proj_max_angle_deg,
            },
        },
    }
    return data


def snapshot_from_dict(data: dict[str, object]) -> MountingSnapshotYaml:
    """Parse a YAML dictionary into a snapshot."""
    if not isinstance(data, dict):
        raise MountingYamlError("YAML root must be a mapping")
    _require_keys(
        "root",
        data,
        {"format_version", "frames", "flags", "mounts", "nuisance", "quality"},
    )

    format_version: int = _require_int(data["format_version"], "format_version")
    frames: FramesYaml = _frames_from_dict(_require_mapping(data["frames"], "frames"))
    flags: FlagsYaml = _flags_from_dict(_require_mapping(data["flags"], "flags"))
    mounts_data: dict[str, object] = _require_mapping(data["mounts"], "mounts")
    _require_keys("mounts", mounts_data, {"R_BI", "R_BM"})
    r_bi: MountRotationYaml = _mount_from_dict(
        _require_mapping(mounts_data["R_BI"], "mounts.R_BI"),
        "mounts.R_BI",
    )
    r_bm: MountRotationYaml = _mount_from_dict(
        _require_mapping(mounts_data["R_BM"], "mounts.R_BM"),
        "mounts.R_BM",
    )
    nuisance_data: dict[str, object] = _require_mapping(data["nuisance"], "nuisance")
    _require_keys("nuisance", nuisance_data, {"imu", "mag"})
    imu: ImuNuisanceYaml = _imu_from_dict(
        _require_mapping(nuisance_data["imu"], "nuisance.imu")
    )
    mag: MagNuisanceYaml = _mag_from_dict(
        _require_mapping(nuisance_data["mag"], "nuisance.mag")
    )
    quality: QualityYaml = _quality_from_dict(
        _require_mapping(data["quality"], "quality")
    )

    return MountingSnapshotYaml(
        format_version=format_version,
        frames=frames,
        flags=flags,
        R_BI=r_bi,
        R_BM=r_bm,
        imu=imu,
        mag=mag,
        quality=quality,
    )


def dumps_yaml(snapshot: MountingSnapshotYaml) -> str:
    """Serialize a snapshot to deterministic YAML."""
    if yaml is None:
        raise MountingYamlError("PyYAML is required for serialization")

    data: dict[str, object] = snapshot_to_dict(snapshot)
    safe_dump: Any = cast(Any, yaml.safe_dump)
    return safe_dump(
        data,
        sort_keys=False,
        indent=2,
        default_flow_style=False,
    )


def loads_yaml(text: str) -> MountingSnapshotYaml:
    """Parse a snapshot from YAML text."""
    if yaml is None:
        raise MountingYamlError("PyYAML is required for parsing")
    loaded: Any = yaml.safe_load(text)
    if not isinstance(loaded, dict):
        raise MountingYamlError("YAML root must be a mapping")
    return snapshot_from_dict(loaded)


def _require_keys(scope: str, data: dict[str, object], required: set[str]) -> None:
    """Ensure a mapping has exactly the required keys."""
    unknown: set[str] = {key for key in data.keys() if key not in required}
    if unknown:
        raise MountingYamlError(
            f"Unexpected keys in {scope}: {', '.join(sorted(unknown))}"
        )
    missing: set[str] = {key for key in required if key not in data}
    if missing:
        raise MountingYamlError(
            f"Missing keys in {scope}: {', '.join(sorted(missing))}"
        )


def _require_mapping(value: object, name: str) -> dict[str, object]:
    """Ensure the value is a dictionary."""
    if not isinstance(value, dict):
        raise MountingYamlError(f"{name} must be a mapping")
    return value


def _require_str(value: object, name: str) -> str:
    """Ensure the value is a string."""
    if not isinstance(value, str):
        raise MountingYamlError(f"{name} must be a string")
    return value


def _require_bool(value: object, name: str) -> bool:
    """Ensure the value is a boolean."""
    if not isinstance(value, bool):
        raise MountingYamlError(f"{name} must be a boolean")
    return value


def _require_int(value: object, name: str) -> int:
    """Ensure the value is an integer."""
    if isinstance(value, bool) or not isinstance(value, numbers.Integral):
        raise MountingYamlError(f"{name} must be an integer")
    return int(value)


def _require_float(value: object, name: str) -> float:
    """Ensure the value is a float."""
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise MountingYamlError(f"{name} must be a float")
    return float(value)


def _coerce_array(value: object, name: str, shape: tuple[int, ...]) -> np.ndarray:
    """Convert an input to a numpy array with the required shape."""
    try:
        array: np.ndarray = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise MountingYamlError(f"{name} must be numeric") from exc
    if array.shape != shape:
        raise MountingYamlError(f"{name} must have shape {shape}")
    return array


def _require_frames(value: FramesYaml) -> FramesYaml:
    """Ensure the value is a FramesYaml instance."""
    if not isinstance(value, FramesYaml):
        raise MountingYamlError("frames must be FramesYaml")
    return value


def _require_flags(value: FlagsYaml) -> FlagsYaml:
    """Ensure the value is a FlagsYaml instance."""
    if not isinstance(value, FlagsYaml):
        raise MountingYamlError("flags must be FlagsYaml")
    return value


def _require_mount(value: MountRotationYaml, name: str) -> MountRotationYaml:
    """Ensure the value is a MountRotationYaml instance."""
    if not isinstance(value, MountRotationYaml):
        raise MountingYamlError(f"{name} must be MountRotationYaml")
    return value


def _require_imu(value: ImuNuisanceYaml) -> ImuNuisanceYaml:
    """Ensure the value is an ImuNuisanceYaml instance."""
    if not isinstance(value, ImuNuisanceYaml):
        raise MountingYamlError("imu must be ImuNuisanceYaml")
    return value


def _require_mag(value: MagNuisanceYaml) -> MagNuisanceYaml:
    """Ensure the value is a MagNuisanceYaml instance."""
    if not isinstance(value, MagNuisanceYaml):
        raise MountingYamlError("mag must be MagNuisanceYaml")
    return value


def _require_quality(value: QualityYaml) -> QualityYaml:
    """Ensure the value is a QualityYaml instance."""
    if not isinstance(value, QualityYaml):
        raise MountingYamlError("quality must be QualityYaml")
    return value


def _require_diversity(value: DiversityYaml) -> DiversityYaml:
    """Ensure the value is a DiversityYaml instance."""
    if not isinstance(value, DiversityYaml):
        raise MountingYamlError("diversity must be DiversityYaml")
    return value


def _frames_from_dict(data: dict[str, object]) -> FramesYaml:
    """Parse frames from a dictionary."""
    _require_keys("frames", data, {"base_frame", "imu_frame", "mag_frame"})
    return FramesYaml(
        base_frame=_require_str(data["base_frame"], "frames.base_frame"),
        imu_frame=_require_str(data["imu_frame"], "frames.imu_frame"),
        mag_frame=_require_str(data["mag_frame"], "frames.mag_frame"),
    )


def _flags_from_dict(data: dict[str, object]) -> FlagsYaml:
    """Parse flags from a dictionary."""
    _require_keys(
        "flags",
        data,
        {
            "anchored",
            "mag_reference_invalid",
            "mag_disturbance_detected",
            "mag_dir_prior_from_driver_cov",
        },
    )
    return FlagsYaml(
        anchored=_require_bool(data["anchored"], "flags.anchored"),
        mag_reference_invalid=_require_bool(
            data["mag_reference_invalid"], "flags.mag_reference_invalid"
        ),
        mag_disturbance_detected=_require_bool(
            data["mag_disturbance_detected"], "flags.mag_disturbance_detected"
        ),
        mag_dir_prior_from_driver_cov=_require_bool(
            data["mag_dir_prior_from_driver_cov"],
            "flags.mag_dir_prior_from_driver_cov",
        ),
    )


def _mount_from_dict(data: dict[str, object], scope: str) -> MountRotationYaml:
    """Parse a mount rotation from a dictionary."""
    _require_keys(scope, data, {"quaternion_wxyz", "rot_cov_rad2"})
    quaternion: np.ndarray = _coerce_array(
        data["quaternion_wxyz"], f"{scope}.quaternion_wxyz", (4,)
    )
    rot_cov: np.ndarray = _coerce_array(
        data["rot_cov_rad2"], f"{scope}.rot_cov_rad2", (3, 3)
    )
    return MountRotationYaml(
        quaternion_wxyz=quaternion,
        rot_cov_rad2=rot_cov,
    )


def _imu_from_dict(data: dict[str, object]) -> ImuNuisanceYaml:
    """Parse IMU nuisance data from a dictionary."""
    _require_keys(
        "nuisance.imu",
        data,
        {
            "accel_bias_mps2",
            "accel_A_row_major",
            "accel_param_cov_row_major_12x12",
            "gyro_bias_rads",
            "gyro_bias_cov_row_major_3x3",
        },
    )
    accel_bias: np.ndarray = _coerce_array(
        data["accel_bias_mps2"], "nuisance.imu.accel_bias_mps2", (3,)
    )
    accel_a: np.ndarray = _coerce_array(
        data["accel_A_row_major"], "nuisance.imu.accel_A_row_major", (9,)
    )
    accel_cov: np.ndarray = _coerce_array(
        data["accel_param_cov_row_major_12x12"],
        "nuisance.imu.accel_param_cov_row_major_12x12",
        (144,),
    )
    gyro_bias: np.ndarray = _coerce_array(
        data["gyro_bias_rads"], "nuisance.imu.gyro_bias_rads", (3,)
    )
    gyro_cov: np.ndarray = _coerce_array(
        data["gyro_bias_cov_row_major_3x3"],
        "nuisance.imu.gyro_bias_cov_row_major_3x3",
        (9,),
    )
    return ImuNuisanceYaml(
        accel_bias_mps2=accel_bias,
        accel_A_row_major=accel_a,
        accel_param_cov_row_major_12x12=accel_cov,
        gyro_bias_rads=gyro_bias,
        gyro_bias_cov_row_major_3x3=gyro_cov,
    )


def _mag_from_dict(data: dict[str, object]) -> MagNuisanceYaml:
    """Parse magnetometer nuisance data from a dictionary."""
    _require_keys(
        "nuisance.mag",
        data,
        {
            "offset_t",
            "offset_cov_row_major_3x3",
            "R_m_unitless2_row_major_3x3",
            "R_m0_unitless2_row_major_3x3",
        },
    )
    offset_t: np.ndarray = _coerce_array(
        data["offset_t"], "nuisance.mag.offset_t", (3,)
    )
    offset_cov: np.ndarray = _coerce_array(
        data["offset_cov_row_major_3x3"],
        "nuisance.mag.offset_cov_row_major_3x3",
        (9,),
    )
    r_m: np.ndarray = _coerce_array(
        data["R_m_unitless2_row_major_3x3"],
        "nuisance.mag.R_m_unitless2_row_major_3x3",
        (9,),
    )
    r_m0: np.ndarray = _coerce_array(
        data["R_m0_unitless2_row_major_3x3"],
        "nuisance.mag.R_m0_unitless2_row_major_3x3",
        (9,),
    )
    return MagNuisanceYaml(
        offset_t=offset_t,
        offset_cov_row_major_3x3=offset_cov,
        R_m_unitless2_row_major_3x3=r_m,
        R_m0_unitless2_row_major_3x3=r_m0,
    )


def _quality_from_dict(data: dict[str, object]) -> QualityYaml:
    """Parse quality data from a dictionary."""
    _require_keys(
        "quality",
        data,
        {
            "raw_samples",
            "steady_segments",
            "keyframes",
            "total_duration_sec",
            "accel_residual_rms",
            "mag_residual_rms",
            "diversity",
        },
    )
    diversity: DiversityYaml = _diversity_from_dict(
        _require_mapping(data["diversity"], "quality.diversity")
    )
    return QualityYaml(
        raw_samples=_require_int(data["raw_samples"], "quality.raw_samples"),
        steady_segments=_require_int(
            data["steady_segments"], "quality.steady_segments"
        ),
        keyframes=_require_int(data["keyframes"], "quality.keyframes"),
        total_duration_sec=_require_float(
            data["total_duration_sec"], "quality.total_duration_sec"
        ),
        accel_residual_rms=_require_float(
            data["accel_residual_rms"], "quality.accel_residual_rms"
        ),
        mag_residual_rms=_require_float(
            data["mag_residual_rms"], "quality.mag_residual_rms"
        ),
        diversity=diversity,
    )


def _diversity_from_dict(data: dict[str, object]) -> DiversityYaml:
    """Parse diversity metrics from a dictionary."""
    _require_keys(
        "quality.diversity",
        data,
        {"gravity_max_angle_deg", "mag_proj_max_angle_deg"},
    )
    return DiversityYaml(
        gravity_max_angle_deg=_require_float(
            data["gravity_max_angle_deg"], "quality.diversity.gravity_max_angle_deg"
        ),
        mag_proj_max_angle_deg=_require_float(
            data["mag_proj_max_angle_deg"], "quality.diversity.mag_proj_max_angle_deg"
        ),
    )


def _mount_to_dict(mount: MountRotationYaml) -> dict[str, object]:
    """Convert a mount rotation to a YAML-safe dictionary."""
    return {
        "quaternion_wxyz": mount.quaternion_wxyz.tolist(),
        "rot_cov_rad2": mount.rot_cov_rad2.tolist(),
    }


def _imu_to_dict(imu: ImuNuisanceYaml) -> dict[str, object]:
    """Convert IMU nuisance data to a YAML-safe dictionary."""
    return {
        "accel_bias_mps2": imu.accel_bias_mps2.tolist(),
        "accel_A_row_major": imu.accel_A_row_major.tolist(),
        "accel_param_cov_row_major_12x12": imu.accel_param_cov_row_major_12x12.tolist(),
        "gyro_bias_rads": imu.gyro_bias_rads.tolist(),
        "gyro_bias_cov_row_major_3x3": imu.gyro_bias_cov_row_major_3x3.tolist(),
    }


def _mag_to_dict(mag: MagNuisanceYaml) -> dict[str, object]:
    """Convert magnetometer nuisance data to a YAML-safe dictionary."""
    return {
        "offset_t": mag.offset_t.tolist(),
        "offset_cov_row_major_3x3": mag.offset_cov_row_major_3x3.tolist(),
        "R_m_unitless2_row_major_3x3": mag.R_m_unitless2_row_major_3x3.tolist(),
        "R_m0_unitless2_row_major_3x3": mag.R_m0_unitless2_row_major_3x3.tolist(),
    }
