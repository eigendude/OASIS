################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Initialization helpers for mounting calibration solver."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from oasis_control.localization.mounting.math_utils.quat import Quaternion


# Units: unitless. Meaning: threshold for near-parallel direction alignment
_DIR_ALIGN_EPS: float = 1e-8

# Units: unitless. Meaning: dot-product limit for reference axis selection
_REF_DOT_LIMIT: float = 0.9


def _unit(vec: NDArray[np.float64]) -> NDArray[np.float64]:
    norm: float = float(np.linalg.norm(vec))
    if norm <= 0.0 or not np.isfinite(norm):
        raise ValueError("vector must be non-zero")
    return vec / norm


def seed_keyframe_attitude_from_measurements(
    gravity_dir_I: NDArray[np.float64],
    mag_dir_M: NDArray[np.float64] | None,
    q_BI_wxyz: NDArray[np.float64],
    q_BM_wxyz: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Seed the world-to-body attitude from gravity and optional mag."""
    g_I_unit: NDArray[np.float64] = _unit(np.asarray(gravity_dir_I, dtype=np.float64))
    R_BI: NDArray[np.float64] = Quaternion(q_BI_wxyz).normalized().as_matrix()
    g_B: NDArray[np.float64] = R_BI @ g_I_unit
    z_B: NDArray[np.float64] = _unit(-g_B)

    ref: NDArray[np.float64]
    x_B: NDArray[np.float64]
    if mag_dir_M is not None:
        m_M_unit: NDArray[np.float64] = _unit(np.asarray(mag_dir_M, dtype=np.float64))
        R_BM: NDArray[np.float64] = Quaternion(q_BM_wxyz).normalized().as_matrix()
        m_B: NDArray[np.float64] = R_BM @ m_M_unit
        x_B = m_B - float(m_B @ z_B) * z_B
        x_norm: float = float(np.linalg.norm(x_B))
        if x_norm <= _DIR_ALIGN_EPS:
            ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            if abs(float(ref @ z_B)) > _REF_DOT_LIMIT:
                ref = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            x_B = ref - float(ref @ z_B) * z_B
        x_B = _unit(x_B)
    else:
        ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(float(ref @ z_B)) > _REF_DOT_LIMIT:
            ref = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        x_B = _unit(ref - float(ref @ z_B) * z_B)

    y_B: NDArray[np.float64] = _unit(np.cross(z_B, x_B))
    x_B = _unit(np.cross(y_B, z_B))
    R_BW: NDArray[np.float64] = np.column_stack((x_B, y_B, z_B))
    R_WB: NDArray[np.float64] = R_BW.T
    return Quaternion.from_matrix(R_WB).to_wxyz()
