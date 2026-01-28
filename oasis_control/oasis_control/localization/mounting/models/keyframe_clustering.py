################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Keyframe clustering for mounting calibration."""

from __future__ import annotations

import numpy as np

from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.mounting_types import Keyframe
from oasis_control.localization.mounting.mounting_types import SteadySegment


class KeyframeClusteringError(Exception):
    """Raised when keyframe clustering encounters invalid data."""


def _as_float_array(value: np.ndarray, name: str, shape: tuple[int, ...]) -> np.ndarray:
    """Coerce an array to float64 with a required shape."""
    array: np.ndarray = np.asarray(value, dtype=np.float64)
    if array.shape != shape:
        raise KeyframeClusteringError(f"{name} must have shape {shape}")
    if not np.all(np.isfinite(array)):
        raise KeyframeClusteringError(f"{name} must contain finite values")
    return array


def _unit_vector(vector: np.ndarray, name: str) -> np.ndarray:
    """Return a unit vector after validating the input."""
    vec: np.ndarray = _as_float_array(vector, name, (3,))
    norm: float = float(np.linalg.norm(vec))
    if not np.isfinite(norm) or norm <= 0.0:
        raise KeyframeClusteringError(f"{name} must be non-zero to normalize")
    return vec / norm


def _angle_deg(u: np.ndarray, v: np.ndarray) -> float:
    """Return the angle between two vectors in degrees."""
    u_unit: np.ndarray = _unit_vector(u, "u")
    v_unit: np.ndarray = _unit_vector(v, "v")
    dot: float = float(np.dot(u_unit, v_unit))
    dot = float(np.clip(dot, -1.0, 1.0))
    return float(np.degrees(np.arccos(dot)))


def _trace(matrix: np.ndarray) -> float:
    """Return the trace of a matrix after validation."""
    mat: np.ndarray = _as_float_array(matrix, "matrix", (3, 3))
    return float(np.trace(mat))


class KeyframeClustering:
    """Online clustering of steady segments into keyframes."""

    def __init__(self, params: MountingParams) -> None:
        """Create a keyframe clustering helper from parameters."""
        self._params: MountingParams = params
        self._keyframes: dict[int, Keyframe] = {}
        self._next_id: int = 0

    def reset(self) -> None:
        """Reset keyframe state."""
        self._keyframes = {}
        self._next_id = 0

    def keyframes(self) -> tuple[Keyframe, ...]:
        """Return a deterministic snapshot of keyframes."""
        return tuple(self._keyframes[key] for key in sorted(self._keyframes))

    def add_segment(self, segment: SteadySegment) -> tuple[int, Keyframe]:
        """Assign a steady segment to a keyframe and return the assignment."""
        if not isinstance(segment, SteadySegment):
            raise KeyframeClusteringError("segment must be a SteadySegment")
        gravity_dir_I: np.ndarray = segment.gravity_up_dir_I()
        mag_dir_M: np.ndarray | None
        if segment.m_mean_T is None:
            mag_dir_M = None
        else:
            mag_dir_M = _unit_vector(segment.m_mean_T, "segment.m_mean_T")

        assigned_id: int | None = None
        best_angle: float = float("inf")
        for keyframe_id, keyframe in self._keyframes.items():
            gravity_angle: float = _angle_deg(
                gravity_dir_I,
                keyframe.gravity_unit_mean_dir_I(),
            )
            if gravity_angle >= float(self._params.cluster.cluster_g_deg):
                continue
            if mag_dir_M is not None and keyframe.mag_weight > 0:
                mag_angle: float = _angle_deg(
                    mag_dir_M,
                    keyframe.mag_unit_mean_dir_M(),
                )
                if mag_angle >= float(self._params.cluster.cluster_h_deg):
                    continue
            if assigned_id is None or gravity_angle < best_angle:
                best_angle = gravity_angle
                assigned_id = keyframe_id
                continue
            if gravity_angle == best_angle and keyframe_id < assigned_id:
                best_angle = gravity_angle
                assigned_id = keyframe_id

        if assigned_id is not None:
            candidate: Keyframe = self._keyframes[assigned_id]
            updated: Keyframe = candidate.update_with_segment(segment)
            self._keyframes[assigned_id] = updated
            return assigned_id, updated

        new_keyframe: Keyframe = Keyframe(
            keyframe_id=self._next_id,
            gravity_mean_dir_I=gravity_dir_I,
            gravity_cov_dir_I=np.zeros((3, 3), dtype=np.float64),
            gravity_weight=1,
            omega_mean_rads_raw=segment.omega_mean_rads_raw,
            cov_omega_raw=segment.cov_omega_raw,
            omega_weight=1,
            accel_mean_mps2_raw=segment.accel_mean_mps2_raw,
            cov_accel_raw=segment.cov_accel_raw,
            accel_weight=1,
            mag_mean_dir_M=mag_dir_M,
            mag_cov_dir_M=(
                np.zeros((3, 3), dtype=np.float64) if mag_dir_M is not None else None
            ),
            mag_weight=1 if mag_dir_M is not None else 0,
            segment_count=1,
        )
        self._keyframes[self._next_id] = new_keyframe
        assigned_id = self._next_id
        self._next_id += 1

        self._enforce_kmax()
        return assigned_id, self._keyframes[assigned_id]

    def _enforce_kmax(self) -> None:
        """Apply keyframe budget policies when exceeding K_max."""
        k_max: int = int(self._params.cluster.K_max)
        if k_max == 0:
            return
        while len(self._keyframes) > k_max:
            drop_id: int = self._choose_drop_id()
            self._keyframes.pop(drop_id, None)

    def _choose_drop_id(self) -> int:
        """Select a keyframe to drop based on policy."""
        drop_policy: str = self._params.cluster.drop_policy
        if drop_policy == "oldest":
            return min(self._keyframes)
        if drop_policy == "lowest_information":
            return self._drop_lowest_information()
        if drop_policy == "redundant":
            return self._drop_redundant()
        raise KeyframeClusteringError(f"Unknown drop_policy: {drop_policy}")

    def _drop_lowest_information(self) -> int:
        """Return the keyframe id with the lowest information content."""
        candidates: list[tuple[int, int, float]] = []
        for keyframe_id, keyframe in self._keyframes.items():
            candidates.append(
                (
                    keyframe_id,
                    keyframe.segment_count,
                    _trace(keyframe.gravity_cov_dir_I),
                )
            )
        candidates_sorted: list[tuple[int, int, float]] = sorted(
            candidates,
            key=lambda item: (item[1], -item[2], item[0]),
        )
        return candidates_sorted[0][0]

    def _drop_redundant(self) -> int:
        """Return the keyframe id that is most redundant in gravity space."""
        keyframes: list[Keyframe] = list(self._keyframes.values())
        if len(keyframes) < 2:
            raise KeyframeClusteringError(
                "redundant policy requires at least 2 keyframes"
            )
        min_angles: dict[int, float] = {}
        for keyframe in keyframes:
            min_angles[keyframe.keyframe_id] = float("inf")
        for i, keyframe_i in enumerate(keyframes):
            for keyframe_j in keyframes[i + 1 :]:
                angle: float = _angle_deg(
                    keyframe_i.gravity_unit_mean_dir_I(),
                    keyframe_j.gravity_unit_mean_dir_I(),
                )
                min_angles[keyframe_i.keyframe_id] = min(
                    min_angles[keyframe_i.keyframe_id],
                    angle,
                )
                min_angles[keyframe_j.keyframe_id] = min(
                    min_angles[keyframe_j.keyframe_id],
                    angle,
                )
        min_items: list[tuple[int, float]] = sorted(
            min_angles.items(),
            key=lambda item: (item[1], item[0]),
        )
        return min_items[0][0]
