################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
Bootstrap helpers for AHRS gravity and magnetic field initialization
"""

from __future__ import annotations

import math

from oasis_control.localization.ahrs.ahrs_linalg import is_finite_seq
from oasis_control.localization.ahrs.ahrs_quat import quat_conj_wxyz
from oasis_control.localization.ahrs.ahrs_quat import quat_normalize_wxyz
from oasis_control.localization.ahrs.ahrs_quat import quat_rotate_wxyz
from oasis_control.localization.ahrs.ahrs_state import AhrsNominalState
from oasis_control.localization.ahrs.ahrs_types import ImuSample
from oasis_control.localization.ahrs.ahrs_types import MagSample


# Dimensionless threshold for treating vectors as degenerate
_EPS_NORM: float = 1.0e-6

# Dimensionless threshold for detecting nearly opposite unit vectors
_EPS_DOT: float = 1.0e-9

# Rad/s maximum angular rate for bootstrap stationary gate
_BOOTSTRAP_MAX_OMEGA_RPS: float = 0.25


def maybe_initialize_gravity(state: AhrsNominalState, imu: ImuSample) -> bool:
    """
    Initialize gravity vector and attitude from a stationary IMU sample
    """

    if _norm3(state.g_w_mps2) >= _EPS_NORM:
        return False

    accel: list[float] = list(imu.linear_acceleration_mps2)
    if len(accel) != 3 or not is_finite_seq(accel):
        return False

    omega: list[float] = list(imu.angular_velocity_rps)
    if len(omega) != 3 or not is_finite_seq(omega):
        return False

    omega_norm: float = _norm3(omega)
    if omega_norm > _BOOTSTRAP_MAX_OMEGA_RPS:
        return False

    accel_norm: float = _norm3(accel)
    if accel_norm < _EPS_NORM:
        return False

    q_bi: list[float] = list(state.t_bi.rotation_wxyz)
    if len(q_bi) != 4 or not is_finite_seq(q_bi):
        return False

    accel_body: list[float] = quat_rotate_wxyz(q_bi, accel)
    if len(accel_body) != 3 or not is_finite_seq(accel_body):
        return False

    g_mag: float = _norm3(accel_body)
    if g_mag < _EPS_NORM:
        return False

    state.g_w_mps2 = [0.0, 0.0, -g_mag]

    accel_unit: list[float]
    valid: bool
    accel_unit, valid = _unit3(accel_body)
    if not valid:
        return False

    gravity_unit: list[float] = [0.0, 0.0, -1.0]
    q_wb: list[float] = _quat_from_two_unit_vectors(gravity_unit, accel_unit)
    state.q_wb_wxyz = quat_normalize_wxyz(q_wb)
    return True


def maybe_initialize_magnetic_field(state: AhrsNominalState, mag: MagSample) -> bool:
    """
    Initialize magnetic field vector from a magnetometer sample
    """

    if _norm3(state.m_w_t) >= _EPS_NORM:
        return False

    if _norm3(state.g_w_mps2) < _EPS_NORM:
        return False

    magnetic: list[float] = list(mag.magnetic_field_t)
    if len(magnetic) != 3 or not is_finite_seq(magnetic):
        return False

    magnetic_norm: float = _norm3(magnetic)
    if magnetic_norm < _EPS_NORM:
        return False

    q_bm: list[float] = list(state.t_bm.rotation_wxyz)
    if len(q_bm) != 4 or not is_finite_seq(q_bm):
        return False

    magnetic_body: list[float] = quat_rotate_wxyz(q_bm, magnetic)
    if len(magnetic_body) != 3 or not is_finite_seq(magnetic_body):
        return False

    q_wb: list[float] = list(state.q_wb_wxyz)
    if len(q_wb) != 4 or not is_finite_seq(q_wb):
        return False

    q_bw: list[float] = quat_conj_wxyz(q_wb)
    magnetic_world: list[float] = quat_rotate_wxyz(q_bw, magnetic_body)
    if len(magnetic_world) != 3 or not is_finite_seq(magnetic_world):
        return False

    state.m_w_t = magnetic_world
    return True


def _norm3(v: list[float]) -> float:
    vx: float = v[0]
    vy: float = v[1]
    vz: float = v[2]
    return math.sqrt(vx * vx + vy * vy + vz * vz)


def _dot3(a: list[float], b: list[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross3(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _scale3(v: list[float], scale: float) -> list[float]:
    return [value * scale for value in v]


def _unit3(v: list[float]) -> tuple[list[float], bool]:
    norm: float = _norm3(v)
    if norm < _EPS_NORM:
        return [0.0, 0.0, 0.0], False
    return _scale3(v, 1.0 / norm), True


def _orthogonal_unit(v: list[float]) -> list[float]:
    axis: list[float]
    if abs(v[0]) < 0.9:
        axis = [1.0, 0.0, 0.0]
    else:
        axis = [0.0, 1.0, 0.0]

    orthogonal: list[float] = _cross3(v, axis)
    orthogonal_unit: list[float]
    valid: bool
    orthogonal_unit, valid = _unit3(orthogonal)
    if not valid:
        return [0.0, 0.0, 1.0]
    return orthogonal_unit


def _quat_from_two_unit_vectors(
    from_unit: list[float], to_unit: list[float]
) -> list[float]:
    dot: float = _dot3(from_unit, to_unit)
    if dot > 1.0 - _EPS_DOT:
        return [1.0, 0.0, 0.0, 0.0]
    if dot < -1.0 + _EPS_DOT:
        axis: list[float] = _orthogonal_unit(from_unit)
        quat_opposite: list[float] = [0.0, axis[0], axis[1], axis[2]]
        return quat_normalize_wxyz(quat_opposite)

    cross: list[float] = _cross3(from_unit, to_unit)
    quat_rot: list[float] = [1.0 + dot, cross[0], cross[1], cross[2]]
    return quat_normalize_wxyz(quat_rot)
