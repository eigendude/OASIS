################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Semantic mounting calibration state containers."""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import replace
from typing import Any

import numpy as np


# Quaternion normalization tolerance used for validation
QUAT_NORM_TOL: float = 1e-8

# Unit vector normalization tolerance used for validation
UNIT_VEC_TOL: float = 1e-8

# Default diagonal entries for R_m adaptive noise covariance
DEFAULT_RM_DIAG: tuple[float, float, float] = (1e-3, 1e-3, 1e-3)


class MountingStateError(Exception):
    """Raised when mounting state data is invalid."""


def _as_f64(name: str, arr: Any, shape: tuple[int, ...]) -> np.ndarray:
    """Coerce input to a float64 array with a specific shape."""
    data: np.ndarray = np.asarray(arr, dtype=np.float64)
    if data.shape != shape:
        raise MountingStateError(f"{name} must have shape {shape}")
    if not np.all(np.isfinite(data)):
        raise MountingStateError(f"{name} must be finite")
    return data


def _unit3(name: str, vec: Any) -> np.ndarray:
    """Return a normalized 3D vector after validation."""
    data: np.ndarray = _as_f64(name, vec, (3,))
    norm: float = float(np.linalg.norm(data))
    if norm <= UNIT_VEC_TOL:
        raise MountingStateError(f"{name} must be non-zero")
    return data / norm


def _unit_quat_wxyz(name: str, quat: Any) -> np.ndarray:
    """Return a normalized quaternion in [w, x, y, z] order."""
    data: np.ndarray = _as_f64(name, quat, (4,))
    norm: float = float(np.linalg.norm(data))
    if norm <= QUAT_NORM_TOL:
        raise MountingStateError(f"{name} must be non-zero")
    return data / norm


def _is_rotation_matrix(matrix: np.ndarray) -> bool:
    """Return True when the input is a valid rotation matrix."""
    if matrix.shape != (3, 3):
        return False
    if not np.all(np.isfinite(matrix)):
        return False
    det: float = float(np.linalg.det(matrix))
    if abs(det - 1.0) > 1e-6:
        return False
    ident: np.ndarray = matrix @ matrix.T
    return bool(np.allclose(ident, np.eye(3), rtol=0.0, atol=1e-6))


@dataclass(frozen=True)
class ImuNuisance:
    """IMU nuisance parameters.

    Attributes:
        b_a_mps2: Accelerometer bias in meters per second squared
        A_a: Accelerometer scale/misalignment matrix (unitless)
        b_g_rads: Gyroscope bias in radians per second
    """

    b_a_mps2: np.ndarray
    A_a: np.ndarray
    b_g_rads: np.ndarray

    def __post_init__(self) -> None:
        """Validate IMU nuisance parameters."""
        b_a_mps2: np.ndarray = _as_f64("b_a_mps2", self.b_a_mps2, (3,))
        A_a: np.ndarray = _as_f64("A_a", self.A_a, (3, 3))
        b_g_rads: np.ndarray = _as_f64("b_g_rads", self.b_g_rads, (3,))
        object.__setattr__(self, "b_a_mps2", b_a_mps2)
        object.__setattr__(self, "A_a", A_a)
        object.__setattr__(self, "b_g_rads", b_g_rads)


@dataclass(frozen=True)
class ImuCalibrationPriorState:
    """Calibration prior for IMU nuisance parameters.

    Attributes:
        b_a_mps2: Accelerometer bias prior in meters per second squared
        A_a: Accelerometer scale/misalignment prior (unitless)
        b_g_rads: Gyroscope bias prior in radians per second
        cov_a_params: Covariance of [b_a, vec(A_a)] in (m/s^2)^2
        cov_b_g: Covariance of gyro bias in (rad/s)^2
    """

    b_a_mps2: np.ndarray
    A_a: np.ndarray
    b_g_rads: np.ndarray
    cov_a_params: np.ndarray | None
    cov_b_g: np.ndarray | None

    def __post_init__(self) -> None:
        """Validate prior mean and covariance shapes."""
        b_a_mps2: np.ndarray = _as_f64("b_a_mps2", self.b_a_mps2, (3,))
        A_a: np.ndarray = _as_f64("A_a", self.A_a, (3, 3))
        b_g_rads: np.ndarray = _as_f64("b_g_rads", self.b_g_rads, (3,))
        object.__setattr__(self, "b_a_mps2", b_a_mps2)
        object.__setattr__(self, "A_a", A_a)
        object.__setattr__(self, "b_g_rads", b_g_rads)

        if self.cov_a_params is None:
            cov_a_params: np.ndarray | None = None
        else:
            cov_a_params = _as_f64("cov_a_params", self.cov_a_params, (12, 12))
        if self.cov_b_g is None:
            cov_b_g: np.ndarray | None = None
        else:
            cov_b_g = _as_f64("cov_b_g", self.cov_b_g, (3, 3))
        object.__setattr__(self, "cov_a_params", cov_a_params)
        object.__setattr__(self, "cov_b_g", cov_b_g)


@dataclass(frozen=True)
class MagNuisance:
    """Magnetometer nuisance parameters.

    Attributes:
        b_m_T: Magnetometer bias in tesla
        R_m_unitless2: Adaptive magnetic direction residual covariance
    """

    b_m_T: np.ndarray | None
    R_m_unitless2: np.ndarray

    def __post_init__(self) -> None:
        """Validate magnetometer nuisance parameters."""
        if self.b_m_T is None:
            b_m_T: np.ndarray | None = None
        else:
            b_m_T = _as_f64("b_m_T", self.b_m_T, (3,))
        R_m_unitless2: np.ndarray = _as_f64(
            "R_m_unitless2",
            self.R_m_unitless2,
            (3, 3),
        )
        object.__setattr__(self, "b_m_T", b_m_T)
        object.__setattr__(self, "R_m_unitless2", R_m_unitless2)


@dataclass(frozen=True)
class MountEstimate:
    """Mounting transform estimate between sensors and the body frame.

    Attributes:
        q_BI_wxyz: IMU-to-body quaternion (w, x, y, z)
        q_BM_wxyz: Magnetometer-to-body quaternion (w, x, y, z)
    """

    q_BI_wxyz: np.ndarray
    q_BM_wxyz: np.ndarray

    def __post_init__(self) -> None:
        """Validate mounting estimate parameters."""
        q_BI_wxyz: np.ndarray = _unit_quat_wxyz("q_BI_wxyz", self.q_BI_wxyz)
        q_BM_wxyz: np.ndarray = _unit_quat_wxyz("q_BM_wxyz", self.q_BM_wxyz)
        object.__setattr__(self, "q_BI_wxyz", q_BI_wxyz)
        object.__setattr__(self, "q_BM_wxyz", q_BM_wxyz)


@dataclass(frozen=True)
class KeyframeAttitude:
    """Attitude state for a single keyframe.

    Attributes:
        keyframe_id: Unique keyframe identifier
        q_WB_wxyz: Body attitude in the world frame (w, x, y, z)
    """

    keyframe_id: int
    q_WB_wxyz: np.ndarray

    def __post_init__(self) -> None:
        """Validate keyframe attitude values."""
        self.validate()

    def validate(self) -> None:
        """Validate keyframe attitude fields."""
        if not isinstance(self.keyframe_id, int) or isinstance(self.keyframe_id, bool):
            raise MountingStateError("keyframe_id must be an int")
        q_WB_wxyz: np.ndarray = _unit_quat_wxyz("q_WB_wxyz", self.q_WB_wxyz)
        object.__setattr__(self, "q_WB_wxyz", q_WB_wxyz)


@dataclass(frozen=True)
class MountingState:
    """Semantic mounting calibration state.

    Attributes:
        mount: Estimated mounting transforms
        g_W_unit: Unit gravity direction in the world frame
        m_W_unit: Unit magnetic direction in the world frame
        imu: IMU nuisance parameters
        imu_prior: IMU calibration prior parameters
        mag: Magnetometer nuisance parameters
        keyframes: Keyframe attitude states
        anchored: True when the state is anchored
        mag_reference_invalid: True when mag reference is invalid
    """

    mount: MountEstimate
    g_W_unit: np.ndarray
    m_W_unit: np.ndarray | None
    imu: ImuNuisance
    imu_prior: ImuCalibrationPriorState | None
    mag: MagNuisance
    keyframes: tuple[KeyframeAttitude, ...]
    anchored: bool
    mag_reference_invalid: bool

    def __post_init__(self) -> None:
        """Normalize and validate the mounting state."""
        g_W_unit: np.ndarray = _unit3("g_W_unit", self.g_W_unit)
        if self.m_W_unit is None:
            m_W_unit: np.ndarray | None = None
        else:
            m_W_unit = _unit3("m_W_unit", self.m_W_unit)
        keyframes: tuple[KeyframeAttitude, ...] = tuple(self.keyframes)
        for attitude in keyframes:
            if not isinstance(attitude, KeyframeAttitude):
                raise MountingStateError("keyframes must be KeyframeAttitude values")
        sorted_keyframes: tuple[KeyframeAttitude, ...] = tuple(
            sorted(keyframes, key=lambda item: item.keyframe_id)
        )
        keyframe_ids: list[int] = [item.keyframe_id for item in sorted_keyframes]
        if len(set(keyframe_ids)) != len(keyframe_ids):
            raise MountingStateError("keyframe_id values must be unique")
        object.__setattr__(self, "g_W_unit", g_W_unit)
        object.__setattr__(self, "m_W_unit", m_W_unit)
        object.__setattr__(self, "keyframes", sorted_keyframes)
        self.validate()

    @classmethod
    def default(cls) -> MountingState:
        """Return a default mounting state."""
        mount: MountEstimate = MountEstimate(
            q_BI_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            q_BM_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        )
        imu: ImuNuisance = ImuNuisance(
            b_a_mps2=np.zeros(3, dtype=np.float64),
            A_a=np.eye(3, dtype=np.float64),
            b_g_rads=np.zeros(3, dtype=np.float64),
        )
        mag: MagNuisance = MagNuisance(
            b_m_T=None,
            R_m_unitless2=np.diag(np.array(DEFAULT_RM_DIAG, dtype=np.float64)),
        )
        return cls(
            mount=mount,
            g_W_unit=np.array([0.0, 0.0, -1.0], dtype=np.float64),
            m_W_unit=None,
            imu=imu,
            imu_prior=None,
            mag=mag,
            keyframes=(),
            anchored=False,
            mag_reference_invalid=False,
        )

    def validate(self) -> None:
        """Validate state shapes, values, and ordering."""
        if not isinstance(self.anchored, bool):
            raise MountingStateError("anchored must be a bool")
        if not isinstance(self.mag_reference_invalid, bool):
            raise MountingStateError("mag_reference_invalid must be a bool")
        if not isinstance(self.mount, MountEstimate):
            raise MountingStateError("mount must be MountEstimate")
        if not isinstance(self.imu, ImuNuisance):
            raise MountingStateError("imu must be ImuNuisance")
        if self.imu_prior is not None and not isinstance(
            self.imu_prior, ImuCalibrationPriorState
        ):
            raise MountingStateError("imu_prior must be ImuCalibrationPriorState")
        if not isinstance(self.mag, MagNuisance):
            raise MountingStateError("mag must be MagNuisance")
        _unit3("g_W_unit", self.g_W_unit)
        if self.m_W_unit is not None:
            _unit3("m_W_unit", self.m_W_unit)
        keyframe_ids: list[int] = [item.keyframe_id for item in self.keyframes]
        if keyframe_ids != sorted(keyframe_ids):
            raise MountingStateError("keyframes must be sorted by keyframe_id")
        if len(set(keyframe_ids)) != len(keyframe_ids):
            raise MountingStateError("keyframe_id values must be unique")
        for attitude in self.keyframes:
            attitude.validate()

    def replace(self, **kwargs: Any) -> MountingState:
        """Return a new MountingState with fields replaced."""
        return replace(self, **kwargs)

    def keyframe_ids(self) -> tuple[int, ...]:
        """Return the keyframe identifiers in deterministic order."""
        return tuple(item.keyframe_id for item in self.keyframes)

    def get_keyframe(self, keyframe_id: int) -> KeyframeAttitude:
        """Return the keyframe attitude for the given identifier."""
        for attitude in self.keyframes:
            if attitude.keyframe_id == keyframe_id:
                return attitude
        raise MountingStateError(f"Keyframe {keyframe_id} not found")

    def with_updated_keyframe(self, attitude: KeyframeAttitude) -> MountingState:
        """Return a new state with the keyframe inserted or replaced."""
        attitude.validate()
        keyframes: list[KeyframeAttitude] = list(self.keyframes)
        replaced: bool = False
        for index, existing in enumerate(keyframes):
            if existing.keyframe_id == attitude.keyframe_id:
                keyframes[index] = attitude
                replaced = True
                break
        if not replaced:
            keyframes.append(attitude)
        keyframes_sorted: tuple[KeyframeAttitude, ...] = tuple(
            sorted(keyframes, key=lambda item: item.keyframe_id)
        )
        return replace(self, keyframes=keyframes_sorted)
