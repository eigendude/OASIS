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
        translation_m: Translation vector in meters, always zeros
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
        if np.any(translation_m != 0.0):
            raise TfPublisherError("translation_m must be zero")
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
        mag_frame: str | None,
        publish_dynamic: bool = True,
        publish_static_when_stable: bool = True,
        republish_static_on_save: bool = True,
    ) -> None:
        """Initialize the TF publisher policy."""
        _require_frame(base_frame, "base_frame")
        _require_frame(imu_frame, "imu_frame")
        if mag_frame is not None:
            _require_frame(mag_frame, "mag_frame")
        if not isinstance(publish_dynamic, bool):
            raise TfPublisherError("publish_dynamic must be a bool")
        if not isinstance(publish_static_when_stable, bool):
            raise TfPublisherError("publish_static_when_stable must be a bool")
        if not isinstance(republish_static_on_save, bool):
            raise TfPublisherError("republish_static_on_save must be a bool")
        self._base_frame: str = base_frame
        self._imu_frame: str = imu_frame
        self._mag_frame: str | None = mag_frame
        self._publish_dynamic: bool = publish_dynamic
        self._publish_static_when_stable: bool = publish_static_when_stable
        self._republish_static_on_save: bool = republish_static_on_save
        self._published_static_children: set[str] = set()
        self._last_t_ns: int | None = None

    def set_mag_frame(self, mag_frame: str | None) -> None:
        """Set the magnetometer frame identifier when it becomes available."""
        if mag_frame is None:
            return
        _require_frame(mag_frame, "mag_frame")
        self._mag_frame = mag_frame

    def reset(self) -> None:
        """Reset the static publication state."""
        self._published_static_children.clear()
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

        R_BI: NDArray[np.float64] = _extract_rotation(T_BI, "T_BI")
        mag_frame: str | None = self._mag_frame
        R_BM: NDArray[np.float64] | None = None
        if mag_frame is not None:
            R_BM = _extract_rotation(T_BM, "T_BM")

        outputs: list[PublishedTransform] = []

        if self._publish_dynamic:
            outputs.extend(
                _build_transforms(
                    t_ns=t_ns,
                    parent_frame=self._base_frame,
                    imu_frame=self._imu_frame,
                    mag_frame=mag_frame,
                    R_BI=R_BI,
                    R_BM=R_BM,
                    is_static=False,
                )
            )

        republish: bool = (
            self._publish_static_when_stable
            and self._republish_static_on_save
            and saved
            and is_stable
        )
        if self._publish_static_when_stable and is_stable:
            imu_frame: str = self._imu_frame
            if republish or imu_frame not in self._published_static_children:
                outputs.append(
                    _transform_to_published(
                        t_ns=t_ns,
                        parent_frame=self._base_frame,
                        child_frame=imu_frame,
                        R=R_BI,
                        is_static=True,
                    )
                )
                self._published_static_children.add(imu_frame)
            if mag_frame is not None and (
                republish or mag_frame not in self._published_static_children
            ):
                if R_BM is None:
                    raise TfPublisherError("R_BM is required with mag_frame")
                outputs.append(
                    _transform_to_published(
                        t_ns=t_ns,
                        parent_frame=self._base_frame,
                        child_frame=mag_frame,
                        R=R_BM,
                        is_static=True,
                    )
                )
                self._published_static_children.add(mag_frame)

        self._last_t_ns = t_ns
        return outputs


def _build_transforms(
    *,
    t_ns: int,
    parent_frame: str,
    imu_frame: str,
    mag_frame: str | None,
    R_BI: NDArray[np.float64],
    R_BM: NDArray[np.float64] | None,
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
            is_static=is_static,
        )
    )
    if mag_frame is not None:
        if R_BM is None:
            raise TfPublisherError("R_BM is required with mag_frame")
        transforms.append(
            _transform_to_published(
                t_ns=t_ns,
                parent_frame=parent_frame,
                child_frame=mag_frame,
                R=R_BM,
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
    is_static: bool,
) -> PublishedTransform:
    """Convert rotation to a PublishedTransform with zero translation."""
    quaternion: Quaternion = Quaternion.from_matrix(R).normalized()
    wxyz: NDArray[np.float64] = quaternion.to_wxyz()
    return PublishedTransform(
        t_ns=t_ns,
        parent_frame=parent_frame,
        child_frame=child_frame,
        translation_m=np.zeros(3, dtype=np.float64),
        quaternion_wxyz=wxyz,
        is_static=is_static,
    )


def _extract_rotation(
    transform: object,
    name: str,
) -> NDArray[np.float64]:
    """Extract a rotation matrix from a transform-like object."""
    if isinstance(transform, SE3):
        return _validate_rotation(transform.R, name)
    if hasattr(transform, "R"):
        R_attr: NDArray[np.float64] = np.asarray(getattr(transform, "R"), dtype=float)
        return _validate_rotation(R_attr, name)
    R_direct: NDArray[np.float64] = np.asarray(transform, dtype=float)
    if R_direct.shape != (3, 3):
        raise TfPublisherError(f"{name} must be a 3x3 rotation matrix")
    return _validate_rotation(R_direct, name)


def _validate_rotation(
    R: NDArray[np.float64],
    name: str,
) -> NDArray[np.float64]:
    """Validate rotation arrays for a transform."""
    R_mat: NDArray[np.float64] = np.asarray(R, dtype=float)
    if R_mat.shape != (3, 3):
        raise TfPublisherError(f"{name}.R must be shape (3, 3)")
    if not np.all(np.isfinite(R_mat)):
        raise TfPublisherError(f"{name}.R must be finite")
    det: float = float(np.linalg.det(R_mat))
    if abs(det - 1.0) > _ROT_DET_TOL:
        raise TfPublisherError(f"{name}.R must have determinant 1")
    ident: NDArray[np.float64] = R_mat @ R_mat.T
    if not np.allclose(ident, np.eye(3), rtol=0.0, atol=_ROT_ORTH_TOL):
        raise TfPublisherError(f"{name}.R must be orthonormal")
    return R_mat


def _require_frame(frame: str, name: str) -> None:
    """Ensure a frame string is valid and non-empty."""
    if not isinstance(frame, str):
        raise TfPublisherError(f"{name} must be a str")
    if not frame:
        raise TfPublisherError(f"{name} must be non-empty")
