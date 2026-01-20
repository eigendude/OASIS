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
import math
from typing import List
from typing import Sequence
from typing import Tuple

from oasis_control.localization.ahrs.math_utils.quat import Quaternion
from oasis_control.localization.ahrs.math_utils.se3 import Se3
from oasis_control.localization.ahrs.state.state_mapping import StateMapping


Se3Transform = Tuple[List[List[float]], List[float]]


@dataclass(frozen=True, slots=True)
class AhrsState:
    """Mean-state container for the AHRS core.

    Purpose:
        Represent the nominal state that is propagated by the process model
        and corrected by measurement updates.

    Public API:
        - copy()
        - apply_error(delta_x)
        - as_vector()
        - from_vector(x)
        - reset()

    Data contract:
        Required fields and shapes:
        - p_WB: position of body in world, shape (3,).
        - v_WB: velocity of body in world, shape (3,).
        - q_WB: unit quaternion [w, x, y, z].
        - omega_WB: body angular rate in {B} (rad/s); angular
          velocity of {B} relative to {W} expressed in {B}.
        - b_g: gyro bias in {I}, shape (3,).
        - b_a: accel bias in {I}, shape (3,).
        - A_a: accel calibration matrix, shape (3, 3).
        - T_BI: IMU extrinsic transform (SE(3)).
        - T_BM: magnetometer extrinsic transform (SE(3)).
        - g_W: gravity vector in world, shape (3,).
        - m_W: magnetic field in world, shape (3,).

    Frames and units:
        - p_WB in meters, v_WB in meters per second.
        - q_WB rotates {W} -> {B}.
        - b_g in rad/s in {I}, b_a in m/s^2 in {I}.
        - A_a is unitless scale/misalignment.
        - T_BI, T_BM translations in meters, rotations unitless.
        - g_W in m/s^2, m_W in tesla.

    Responsibility:
        Define the nominal (mean) AHRS state elements and their ordering,
        units, and frames without any ROS types.

    Inputs/outputs:
        - Inputs are plain vectors/matrices representing state components.
        - Outputs are the same components, typically passed to process or
          measurement models.

    Dependencies:
        - Used by process_model, imu_model, mag_model, and filter steps.
        - Coupled with error_state and state_mapping for covariance
          operations.

    Determinism:
        State storage is deterministic; no hidden time or parameter lookups.

    Determinism and edge cases:
        - apply_error must use the canonical error-state ordering from
          StateMapping.
        - Quaternion normalization is required after applying delta_theta.

    Equations:
        - q_WB is updated with a small-angle quaternion: q <- dq ⊗ q.
        - Extrinsics use SE(3) perturbations: T <- Exp(delta_xi) * T.

    Numerical stability notes:
        - Normalize q_WB after any update.
        - Keep A_a invertible; enforce conditioning elsewhere.

    Serialization:
        The mean-state vector packs fields in the following order for
        debugging and tests only:

            p_WB(3), v_WB(3), q_WB(4), omega_WB(3), b_g(3), b_a(3),
            A_a(9 row-major), T_BI(R 9 row-major + p 3),
            T_BM(R 9 row-major + p 3), g_W(3), m_W(3)

        This serialization is not the error-state ordering.

    Suggested unit tests:
        - apply_error respects StateMapping index ranges.
        - Quaternion normalization maintains unit norm.
    """

    p_WB: List[float]
    v_WB: List[float]
    q_WB: List[float]
    omega_WB: List[float]
    b_g: List[float]
    b_a: List[float]
    A_a: List[List[float]]
    T_BI: Se3Transform
    T_BM: Se3Transform
    g_W: List[float]
    m_W: List[float]

    def __post_init__(self) -> None:
        """Validate and normalize state fields."""
        object.__setattr__(self, "p_WB", _copy_vec(self.p_WB))
        object.__setattr__(self, "v_WB", _copy_vec(self.v_WB))
        object.__setattr__(self, "q_WB", _copy_vec(self.q_WB))
        object.__setattr__(self, "omega_WB", _copy_vec(self.omega_WB))
        object.__setattr__(self, "b_g", _copy_vec(self.b_g))
        object.__setattr__(self, "b_a", _copy_vec(self.b_a))
        object.__setattr__(self, "A_a", _copy_mat(self.A_a))
        object.__setattr__(self, "T_BI", _copy_transform(self.T_BI))
        object.__setattr__(self, "T_BM", _copy_transform(self.T_BM))
        object.__setattr__(self, "g_W", _copy_vec(self.g_W))
        object.__setattr__(self, "m_W", _copy_vec(self.m_W))

        _assert_vector_length("p_WB", self.p_WB, 3)
        _assert_vector_length("v_WB", self.v_WB, 3)
        _assert_vector_length("q_WB", self.q_WB, 4)
        _assert_vector_length("omega_WB", self.omega_WB, 3)
        _assert_vector_length("b_g", self.b_g, 3)
        _assert_vector_length("b_a", self.b_a, 3)
        _assert_matrix_shape("A_a", self.A_a, 3, 3)
        _assert_transform("T_BI", self.T_BI)
        _assert_transform("T_BM", self.T_BM)
        _assert_vector_length("g_W", self.g_W, 3)
        _assert_vector_length("m_W", self.m_W, 3)

        _assert_finite_vector("p_WB", self.p_WB)
        _assert_finite_vector("v_WB", self.v_WB)
        _assert_finite_vector("q_WB", self.q_WB)
        _assert_finite_vector("omega_WB", self.omega_WB)
        _assert_finite_vector("b_g", self.b_g)
        _assert_finite_vector("b_a", self.b_a)
        _assert_finite_matrix("A_a", self.A_a)
        _assert_transform_finite("T_BI", self.T_BI)
        _assert_transform_finite("T_BM", self.T_BM)
        _assert_finite_vector("g_W", self.g_W)
        _assert_finite_vector("m_W", self.m_W)

        normalized_q: List[float] = Quaternion.normalize(self.q_WB)
        object.__setattr__(self, "q_WB", normalized_q)

    def copy(self) -> AhrsState:
        """Return a deep copy of the state."""
        return AhrsState(
            p_WB=_copy_vec(self.p_WB),
            v_WB=_copy_vec(self.v_WB),
            q_WB=_copy_vec(self.q_WB),
            omega_WB=_copy_vec(self.omega_WB),
            b_g=_copy_vec(self.b_g),
            b_a=_copy_vec(self.b_a),
            A_a=_copy_mat(self.A_a),
            T_BI=_copy_transform(self.T_BI),
            T_BM=_copy_transform(self.T_BM),
            g_W=_copy_vec(self.g_W),
            m_W=_copy_vec(self.m_W),
        )

    @staticmethod
    def reset() -> AhrsState:
        """Return the deterministic default mean state."""
        zero3: List[float] = [0.0, 0.0, 0.0]
        identity3: List[List[float]] = _identity3()
        identity_T: Se3Transform = (identity3, [0.0, 0.0, 0.0])
        return AhrsState(
            p_WB=list(zero3),
            v_WB=list(zero3),
            q_WB=[1.0, 0.0, 0.0, 0.0],
            omega_WB=list(zero3),
            b_g=list(zero3),
            b_a=list(zero3),
            A_a=_copy_mat(identity3),
            T_BI=_copy_transform(identity_T),
            T_BM=_copy_transform(identity_T),
            g_W=[0.0, 0.0, -9.81],
            m_W=[1.0, 0.0, 0.0],
        )

    def as_vector(self) -> List[float]:
        """Return a packed mean-state vector for debugging and tests."""
        values: List[float] = []
        values.extend(self.p_WB)
        values.extend(self.v_WB)
        values.extend(self.q_WB)
        values.extend(self.omega_WB)
        values.extend(self.b_g)
        values.extend(self.b_a)
        values.extend(_flatten_row_major(self.A_a))
        values.extend(_flatten_row_major(self.T_BI[0]))
        values.extend(self.T_BI[1])
        values.extend(_flatten_row_major(self.T_BM[0]))
        values.extend(self.T_BM[1])
        values.extend(self.g_W)
        values.extend(self.m_W)
        return list(values)

    @staticmethod
    def from_vector(x: Sequence[float]) -> AhrsState:
        """Create a mean state from a packed vector."""
        expected_len: int = 58
        if len(x) != expected_len:
            raise ValueError("x must have length 58")
        values: List[float] = [float(value) for value in x]
        value: float
        for value in values:
            if not math.isfinite(value):
                raise ValueError("x contains non-finite value")

        index: int = 0
        p_WB: List[float] = values[index : index + 3]
        index += 3
        v_WB: List[float] = values[index : index + 3]
        index += 3
        q_WB: List[float] = values[index : index + 4]
        index += 4
        omega_WB: List[float] = values[index : index + 3]
        index += 3
        b_g: List[float] = values[index : index + 3]
        index += 3
        b_a: List[float] = values[index : index + 3]
        index += 3
        A_a: List[List[float]] = _reshape_row_major(values[index : index + 9], 3)
        index += 9
        T_BI_R: List[List[float]] = _reshape_row_major(values[index : index + 9], 3)
        index += 9
        T_BI_p: List[float] = values[index : index + 3]
        index += 3
        T_BM_R: List[List[float]] = _reshape_row_major(values[index : index + 9], 3)
        index += 9
        T_BM_p: List[float] = values[index : index + 3]
        index += 3
        g_W: List[float] = values[index : index + 3]
        index += 3
        m_W: List[float] = values[index : index + 3]

        return AhrsState(
            p_WB=p_WB,
            v_WB=v_WB,
            q_WB=q_WB,
            omega_WB=omega_WB,
            b_g=b_g,
            b_a=b_a,
            A_a=A_a,
            T_BI=(T_BI_R, T_BI_p),
            T_BM=(T_BM_R, T_BM_p),
            g_W=g_W,
            m_W=m_W,
        )

    def apply_error(self, delta_x: Sequence[float]) -> AhrsState:
        """Return a new mean state with δx applied."""
        if len(delta_x) != StateMapping.dimension():
            raise ValueError("delta_x must have length 45")
        delta_list: List[float] = [float(value) for value in delta_x]
        value: float
        for value in delta_list:
            if not math.isfinite(value):
                raise ValueError("delta_x contains non-finite value")

        delta_p: List[float] = delta_list[StateMapping.slice_delta_p()]
        delta_v: List[float] = delta_list[StateMapping.slice_delta_v()]
        delta_theta: List[float] = delta_list[StateMapping.slice_delta_theta()]
        delta_omega: List[float] = delta_list[StateMapping.slice_delta_omega()]
        delta_b_g: List[float] = delta_list[StateMapping.slice_delta_b_g()]
        delta_b_a: List[float] = delta_list[StateMapping.slice_delta_b_a()]
        delta_A_a: List[float] = delta_list[StateMapping.slice_delta_A_a()]
        delta_xi_BI: List[float] = delta_list[StateMapping.slice_delta_xi_BI()]
        delta_xi_BM: List[float] = delta_list[StateMapping.slice_delta_xi_BM()]
        delta_g_W: List[float] = delta_list[StateMapping.slice_delta_g_W()]
        delta_m_W: List[float] = delta_list[StateMapping.slice_delta_m_W()]

        p_WB: List[float] = _add_vec(self.p_WB, delta_p)
        v_WB: List[float] = _add_vec(self.v_WB, delta_v)
        omega_WB: List[float] = _add_vec(self.omega_WB, delta_omega)
        b_g: List[float] = _add_vec(self.b_g, delta_b_g)
        b_a: List[float] = _add_vec(self.b_a, delta_b_a)

        delta_A_a_mat: List[List[float]] = _reshape_row_major(delta_A_a, 3)
        A_a: List[List[float]] = _add_mat(self.A_a, delta_A_a_mat)

        dq: List[float] = Quaternion.small_angle_quat(delta_theta)
        q_WB: List[float] = Quaternion.multiply(dq, self.q_WB)
        q_WB = Quaternion.normalize(q_WB)

        T_BI_delta: Se3Transform = Se3.exp(delta_xi_BI)
        T_BM_delta: Se3Transform = Se3.exp(delta_xi_BM)
        T_BI: Se3Transform = Se3.compose(T_BI_delta, self.T_BI)
        T_BM: Se3Transform = Se3.compose(T_BM_delta, self.T_BM)

        g_W: List[float] = _add_vec(self.g_W, delta_g_W)
        m_W: List[float] = _add_vec(self.m_W, delta_m_W)

        return AhrsState(
            p_WB=p_WB,
            v_WB=v_WB,
            q_WB=q_WB,
            omega_WB=omega_WB,
            b_g=b_g,
            b_a=b_a,
            A_a=A_a,
            T_BI=T_BI,
            T_BM=T_BM,
            g_W=g_W,
            m_W=m_W,
        )


def _copy_vec(vec: Sequence[float]) -> List[float]:
    """Return a copy of a vector."""
    return [float(value) for value in vec]


def _copy_mat(mat: Sequence[Sequence[float]]) -> List[List[float]]:
    """Return a deep copy of a matrix."""
    return [[float(value) for value in row] for row in mat]


def _copy_transform(T: Se3Transform) -> Se3Transform:
    """Return a deep copy of a transform."""
    R, p = T
    return (_copy_mat(R), _copy_vec(p))


def _identity3() -> List[List[float]]:
    """Return a 3x3 identity matrix."""
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _assert_vector_length(name: str, vec: Sequence[float], length: int) -> None:
    """Validate vector length."""
    if len(vec) != length:
        raise ValueError(f"{name} must have length {length}")


def _assert_matrix_shape(
    name: str,
    mat: Sequence[Sequence[float]],
    rows: int,
    cols: int,
) -> None:
    """Validate matrix shape."""
    if len(mat) != rows or any(len(row) != cols for row in mat):
        raise ValueError(f"{name} must be {rows}x{cols}")


def _assert_transform(name: str, T: Se3Transform) -> None:
    """Validate transform structure."""
    if not isinstance(T, tuple) or len(T) != 2:
        raise ValueError(f"{name} must be a (R, p) tuple")
    R, p = T
    _assert_matrix_shape(f"{name}.R", R, 3, 3)
    _assert_vector_length(f"{name}.p", p, 3)


def _assert_finite_vector(name: str, vec: Sequence[float]) -> None:
    """Validate vector entries are finite."""
    value: float
    for value in vec:
        if not math.isfinite(value):
            raise ValueError(f"{name} contains non-finite value")


def _assert_finite_matrix(name: str, mat: Sequence[Sequence[float]]) -> None:
    """Validate matrix entries are finite."""
    row: Sequence[float]
    for row in mat:
        _assert_finite_vector(name, row)


def _assert_transform_finite(name: str, T: Se3Transform) -> None:
    """Validate transform entries are finite."""
    R, p = T
    _assert_finite_matrix(f"{name}.R", R)
    _assert_finite_vector(f"{name}.p", p)


def _flatten_row_major(mat: Sequence[Sequence[float]]) -> List[float]:
    """Flatten a matrix in row-major order."""
    values: List[float] = []
    row: Sequence[float]
    for row in mat:
        value: float
        for value in row:
            values.append(float(value))
    return values


def _reshape_row_major(values: Sequence[float], rows: int) -> List[List[float]]:
    """Reshape a flat list into a square matrix."""
    if rows <= 0:
        raise ValueError("rows must be positive")
    if len(values) != rows * rows:
        raise ValueError("values must have length rows*rows")
    matrix: List[List[float]] = []
    index: int = 0
    row_index: int
    for row_index in range(rows):
        row: List[float] = []
        col_index: int
        for col_index in range(rows):
            row.append(float(values[index + col_index]))
        index += rows
        matrix.append(row)
    return matrix


def _add_vec(a: Sequence[float], b: Sequence[float]) -> List[float]:
    """Add two vectors."""
    if len(a) != len(b):
        raise ValueError("Vector lengths must match")
    out: List[float] = []
    idx: int
    for idx in range(len(a)):
        out.append(a[idx] + b[idx])
    return out


def _add_mat(
    A: Sequence[Sequence[float]],
    B: Sequence[Sequence[float]],
) -> List[List[float]]:
    """Add two matrices."""
    if len(A) != len(B) or any(len(row) != len(B[i]) for i, row in enumerate(A)):
        raise ValueError("Matrix shapes must match")
    result: List[List[float]] = []
    i: int
    for i in range(len(A)):
        row: List[float] = []
        j: int
        for j in range(len(A[i])):
            row.append(A[i][j] + B[i][j])
        result.append(row)
    return result
