################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
"""Pure-Python TF publication policy helpers."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from oasis_control.localization.mounting.math_utils.quat import Quaternion
from oasis_control.localization.mounting.math_utils.se3 import SE3


_ROT_DET_TOL: float = 1e-6
_ROT_ORTH_TOL: float = 1e-6


class TfPublisherError(Exception):
    """Raised when TF publication inputs are invalid."""


@dataclass(frozen=True)
class PublishedTransform:
    """Published transform output from the TF policy engine.

    Attributes:
        t_ns: Timestamp for the transform in nanoseconds
        parent_frame: Parent frame ID
        child_frame: Child frame ID
        translation_m: Translation vector in meters
        quaternion_wxyz: Unit quaternion in [w, x, y, z] order
        is_static: True when the transform should be latched as static
    """

    t_ns: int
    parent_frame: str
    child_frame: str
    translation_m: NDArray[np.float64]
    quaternion_wxyz: NDArray[np.float64]
    is_static: bool

    def __post_init__(self) -> None:
        """Validate published transform fields."""
        if not isinstance(self.t_ns, int) or isinstance(self.t_ns, bool):
            raise TfPublisherError("t_ns must be an int")
        if self.t_ns < 0:
            raise TfPublisherError("t_ns must be non-negative")
        _require_frame(self.parent_frame, "parent_frame")
        _require_frame(self.child_frame, "child_frame")
        translation_m: NDArray[np.float64] = np.asarray(self.translation_m, dtype=float)
        if translation_m.shape != (3,):
            raise TfPublisherError("translation_m must be shape (3,)")
        if not np.all(np.isfinite(translation_m)):
            raise TfPublisherError("translation_m must be finite")
        quaternion_wxyz: NDArray[np.float64] = np.asarray(
            self.quaternion_wxyz,
            dtype=float,
        )
        if quaternion_wxyz.shape != (4,):
            raise TfPublisherError("quaternion_wxyz must be shape (4,)")
        if not np.all(np.isfinite(quaternion_wxyz)):
            raise TfPublisherError("quaternion_wxyz must be finite")
        norm: float = float(np.linalg.norm(quaternion_wxyz))
        if norm <= 0.0:
            raise TfPublisherError("quaternion_wxyz must be non-zero")
        object.__setattr__(self, "translation_m", translation_m)
        object.__setattr__(self, "quaternion_wxyz", quaternion_wxyz / norm)


class TfPublisher:
    """Decide when to publish dynamic and static mounting transforms.

    Dynamic transforms are emitted on every update when enabled, even after
    stability is reached. Static transforms are emitted once on the transition
    to stable, and optionally re-emitted when the calibration file is saved.
    """

    def __init__(
        self,
        *,
        base_frame: str,
        imu_frame: str,
        mag_frame: str,
        publish_dynamic: bool = True,
        publish_static_when_stable: bool = True,
        republish_static_on_save: bool = True,
    ) -> None:
        """Initialize the TF publisher policy."""
        _require_frame(base_frame, "base_frame")
        _require_frame(imu_frame, "imu_frame")
        _require_frame(mag_frame, "mag_frame")
        if not isinstance(publish_dynamic, bool):
            raise TfPublisherError("publish_dynamic must be a bool")
        if not isinstance(publish_static_when_stable, bool):
            raise TfPublisherError("publish_static_when_stable must be a bool")
        if not isinstance(republish_static_on_save, bool):
            raise TfPublisherError("republish_static_on_save must be a bool")
        self._base_frame: str = base_frame
        self._imu_frame: str = imu_frame
        self._mag_frame: str = mag_frame
        self._publish_dynamic: bool = publish_dynamic
        self._publish_static_when_stable: bool = publish_static_when_stable
        self._republish_static_on_save: bool = republish_static_on_save
        self._published_static: bool = False
        self._was_stable: bool = False
        self._last_t_ns: int | None = None

    def reset(self) -> None:
        """Reset the static publication state."""
        self._published_static = False
        self._was_stable = False
        self._last_t_ns = None

    def update(
        self,
        *,
        t_ns: int,
        T_BI: object,
        T_BM: object,
        is_stable: bool,
        saved: bool,
    ) -> list[PublishedTransform]:
        """Compute transforms to publish for the current mounting estimate."""
        if not isinstance(t_ns, int) or isinstance(t_ns, bool):
            raise TfPublisherError("t_ns must be an int")
        if t_ns < 0:
            raise TfPublisherError("t_ns must be non-negative")
        if self._last_t_ns is not None and t_ns < self._last_t_ns:
            raise TfPublisherError("t_ns must be non-decreasing")
        if not isinstance(is_stable, bool):
            raise TfPublisherError("is_stable must be a bool")
        if not isinstance(saved, bool):
            raise TfPublisherError("saved must be a bool")

        R_BI: NDArray[np.float64]
        p_BI: NDArray[np.float64]
        R_BM: NDArray[np.float64]
        p_BM: NDArray[np.float64]
        R_BI, p_BI = _extract_transform(T_BI, "T_BI")
        R_BM, p_BM = _extract_transform(T_BM, "T_BM")

        outputs: list[PublishedTransform] = []

        if self._publish_dynamic:
            outputs.extend(
                _build_transforms(
                    t_ns=t_ns,
                    parent_frame=self._base_frame,
                    imu_frame=self._imu_frame,
                    mag_frame=self._mag_frame,
                    R_BI=R_BI,
                    p_BI=p_BI,
                    R_BM=R_BM,
                    p_BM=p_BM,
                    is_static=False,
                )
            )

        publish_static: bool = False
        if self._publish_static_when_stable:
            transitioned: bool = is_stable and not self._was_stable
            republish: bool = (
                self._republish_static_on_save
                and saved
                and is_stable
                and self._published_static
            )
            publish_static = (transitioned and not self._published_static) or republish

        if publish_static:
            outputs.extend(
                _build_transforms(
                    t_ns=t_ns,
                    parent_frame=self._base_frame,
                    imu_frame=self._imu_frame,
                    mag_frame=self._mag_frame,
                    R_BI=R_BI,
                    p_BI=p_BI,
                    R_BM=R_BM,
                    p_BM=p_BM,
                    is_static=True,
                )
            )
            self._published_static = True

        self._was_stable = is_stable
        self._last_t_ns = t_ns
        return outputs


def _build_transforms(
    *,
    t_ns: int,
    parent_frame: str,
    imu_frame: str,
    mag_frame: str,
    R_BI: NDArray[np.float64],
    p_BI: NDArray[np.float64],
    R_BM: NDArray[np.float64],
    p_BM: NDArray[np.float64],
    is_static: bool,
) -> list[PublishedTransform]:
    """Build published transforms for IMU and magnetometer frames."""
    transforms: list[PublishedTransform] = []
    transforms.append(
        _transform_to_published(
            t_ns=t_ns,
            parent_frame=parent_frame,
            child_frame=imu_frame,
            R=R_BI,
            p=p_BI,
            is_static=is_static,
        )
    )
    transforms.append(
        _transform_to_published(
            t_ns=t_ns,
            parent_frame=parent_frame,
            child_frame=mag_frame,
            R=R_BM,
            p=p_BM,
            is_static=is_static,
        )
    )
    return transforms


def _transform_to_published(
    *,
    t_ns: int,
    parent_frame: str,
    child_frame: str,
    R: NDArray[np.float64],
    p: NDArray[np.float64],
    is_static: bool,
) -> PublishedTransform:
    """Convert rotation and translation to a PublishedTransform."""
    quaternion: Quaternion = Quaternion.from_matrix(R).normalized()
    wxyz: NDArray[np.float64] = quaternion.to_wxyz()
    return PublishedTransform(
        t_ns=t_ns,
        parent_frame=parent_frame,
        child_frame=child_frame,
        translation_m=p,
        quaternion_wxyz=wxyz,
        is_static=is_static,
    )


def _extract_transform(
    transform: object,
    name: str,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Extract rotation and translation arrays from a transform-like object."""
    if isinstance(transform, SE3):
        R: NDArray[np.float64] = transform.R
        p: NDArray[np.float64] = transform.p
        return _validate_transform(R, p, name)
    if hasattr(transform, "R") and hasattr(transform, "p"):
        R_attr: NDArray[np.float64] = np.asarray(getattr(transform, "R"), dtype=float)
        p_attr: NDArray[np.float64] = np.asarray(getattr(transform, "p"), dtype=float)
        return _validate_transform(R_attr, p_attr, name)
    raise TfPublisherError(f"{name} must provide R and p attributes")


def _validate_transform(
    R: NDArray[np.float64],
    p: NDArray[np.float64],
    name: str,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Validate rotation and translation arrays for a transform."""
    R_mat: NDArray[np.float64] = np.asarray(R, dtype=float)
    p_vec: NDArray[np.float64] = np.asarray(p, dtype=float)
    if R_mat.shape != (3, 3):
        raise TfPublisherError(f"{name}.R must be shape (3, 3)")
    if p_vec.shape != (3,):
        raise TfPublisherError(f"{name}.p must be shape (3,)")
    if not np.all(np.isfinite(R_mat)):
        raise TfPublisherError(f"{name}.R must be finite")
    if not np.all(np.isfinite(p_vec)):
        raise TfPublisherError(f"{name}.p must be finite")
    det: float = float(np.linalg.det(R_mat))
    if abs(det - 1.0) > _ROT_DET_TOL:
        raise TfPublisherError(f"{name}.R must have determinant 1")
    ident: NDArray[np.float64] = R_mat @ R_mat.T
    if not np.allclose(ident, np.eye(3), rtol=0.0, atol=_ROT_ORTH_TOL):
        raise TfPublisherError(f"{name}.R must be orthonormal")
    return R_mat, p_vec


def _require_frame(frame: str, name: str) -> None:
    """Ensure a frame string is valid and non-empty."""
    if not isinstance(frame, str):
        raise TfPublisherError(f"{name} must be a str")
    if not frame:
        raise TfPublisherError(f"{name} must be non-empty")
