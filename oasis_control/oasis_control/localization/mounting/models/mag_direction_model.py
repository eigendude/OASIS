################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Direction-only magnetometer modeling utilities."""

from __future__ import annotations

import numpy as np


class MagDirectionModelError(Exception):
    """Raised when magnetometer direction modeling fails."""


def normalize(m_raw_T: np.ndarray) -> np.ndarray:
    """Return a unit magnetic-field direction vector."""
    m_vec: np.ndarray = np.asarray(m_raw_T, dtype=np.float64)
    if m_vec.shape != (3,):
        raise MagDirectionModelError("m_raw_T must have shape (3,)")
    if not np.all(np.isfinite(m_vec)):
        raise MagDirectionModelError("m_raw_T must be finite")
    magnitude: float = float(np.linalg.norm(m_vec))
    if not np.isfinite(magnitude) or magnitude <= 0.0:
        raise MagDirectionModelError("m_raw_T must be non-zero to normalize")
    return m_vec / magnitude


def direction_covariance_from_raw(
    *,
    m_raw_T: np.ndarray,
    cov_m_raw_T2: np.ndarray,
    s_min_T: float,
) -> np.ndarray:
    """Convert raw mag covariance to direction covariance."""
    m_vec: np.ndarray = np.asarray(m_raw_T, dtype=np.float64)
    cov_raw: np.ndarray = np.asarray(cov_m_raw_T2, dtype=np.float64)
    if m_vec.shape != (3,):
        raise MagDirectionModelError("m_raw_T must have shape (3,)")
    if cov_raw.shape != (3, 3):
        raise MagDirectionModelError("cov_m_raw_T2 must have shape (3, 3)")
    if not np.all(np.isfinite(m_vec)):
        raise MagDirectionModelError("m_raw_T must be finite")
    if not np.all(np.isfinite(cov_raw)):
        raise MagDirectionModelError("cov_m_raw_T2 must be finite")
    if not np.isfinite(s_min_T) or s_min_T <= 0.0:
        raise MagDirectionModelError("s_min_T must be positive")

    u: np.ndarray = normalize(m_vec)
    s: float = float(np.linalg.norm(m_vec))
    if s <= s_min_T:
        raise MagDirectionModelError("m_raw_T magnitude is below s_min_T")
    eye: np.ndarray = np.eye(3, dtype=np.float64)
    J: np.ndarray = (eye - np.outer(u, u)) / s
    Sigma_dir: np.ndarray = J @ cov_raw @ J.T
    return 0.5 * (Sigma_dir + Sigma_dir.T)


def direction_residual_cross(u_meas: np.ndarray, u_pred: np.ndarray) -> np.ndarray:
    """Return the direction residual defined by a cross product."""
    u_meas_vec: np.ndarray = np.asarray(u_meas, dtype=np.float64)
    u_pred_vec: np.ndarray = np.asarray(u_pred, dtype=np.float64)
    if u_meas_vec.shape != (3,) or u_pred_vec.shape != (3,):
        raise MagDirectionModelError("u_meas and u_pred must have shape (3,)")
    if not np.all(np.isfinite(u_meas_vec)) or not np.all(np.isfinite(u_pred_vec)):
        raise MagDirectionModelError("u_meas and u_pred must be finite")
    return np.cross(u_meas_vec, u_pred_vec)
