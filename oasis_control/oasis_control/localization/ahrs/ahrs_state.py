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
Nominal-state container for the AHRS core math

The nominal state stores the best estimate of the physical system. The
error-state lives alongside it in the covariance, tracking small deviations
that are linearized around the nominal state. Keeping the nominal container
separate makes the core math deterministic and ROS-agnostic.

The nominal state includes an explicit angular rate term so that gyro
measurements have a direct landing spot even before the full prediction
model is implemented.
"""

from dataclasses import dataclass

from oasis_control.localization.ahrs.ahrs_types import AhrsSe3Transform


@dataclass(frozen=False)
class AhrsNominalState:
    """
    Nominal AHRS state variables used for prediction and updates

    Fields:
        p_wb_m: Position of the body in world in meters, XYZ order
        v_wb_mps: Velocity of the body in world in m/s, XYZ order
        q_wb_wxyz: Orientation of the body in world, quaternion wxyz order
        omega_wb_rps: Angular rate of the body in rad/s, XYZ order
        b_g_rps: Gyro bias in rad/s, XYZ order
        b_a_mps2: Accel bias in m/s^2, XYZ order
        a_a: Accel scale/misalignment matrix, 3x3 row-major
        t_bi: IMU -> body transform (T_BI)
        t_bm: Magnetometer -> body transform (T_BM)
        g_w_mps2: Gravity vector in world in m/s^2, XYZ order
        m_w_t: Magnetic field vector in world in tesla, XYZ order
    """

    p_wb_m: list[float]
    v_wb_mps: list[float]
    q_wb_wxyz: list[float]
    omega_wb_rps: list[float]
    b_g_rps: list[float]
    b_a_mps2: list[float]
    a_a: list[float]
    t_bi: AhrsSe3Transform
    t_bm: AhrsSe3Transform
    g_w_mps2: list[float]
    m_w_t: list[float]


def zero_vector3() -> list[float]:
    """
    Return a 3D zero vector
    """

    return [0.0, 0.0, 0.0]


def identity_quaternion_wxyz() -> list[float]:
    """
    Return the identity quaternion in wxyz order
    """

    return [1.0, 0.0, 0.0, 0.0]


def identity_mat3() -> list[float]:
    """
    Return a 3x3 identity matrix in row-major order
    """

    return [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]


def default_extrinsic(parent_frame: str, child_frame: str) -> AhrsSe3Transform:
    """
    Build a default identity extrinsic transform for a sensor frame
    """

    return AhrsSe3Transform(
        parent_frame=parent_frame,
        child_frame=child_frame,
        translation_m=zero_vector3(),
        rotation_wxyz=identity_quaternion_wxyz(),
    )


def default_nominal_state(
    *,
    body_frame_id: str,
    imu_frame_id: str,
    mag_frame_id: str,
) -> AhrsNominalState:
    """
    Create a default nominal state with zero motion and identity extrinsics

    The default values assume the body starts at the world origin with zero
    velocity, the identity orientation, zero biases, and sensor frames aligned
    to the body frame. Gravity and magnetic field vectors are set to zero for
    now and should be populated by initialization routines later on.
    """

    return AhrsNominalState(
        p_wb_m=zero_vector3(),
        v_wb_mps=zero_vector3(),
        q_wb_wxyz=identity_quaternion_wxyz(),
        omega_wb_rps=zero_vector3(),
        b_g_rps=zero_vector3(),
        b_a_mps2=zero_vector3(),
        a_a=identity_mat3(),
        t_bi=default_extrinsic(body_frame_id, imu_frame_id),
        t_bm=default_extrinsic(body_frame_id, mag_frame_id),
        g_w_mps2=zero_vector3(),
        m_w_t=zero_vector3(),
    )


def validate_vector_length(name: str, v: list[float], n: int) -> None:
    """
    Validate that a vector has the expected length
    """

    if len(v) != n:
        raise ValueError(f"{name} must be length {n}, got {len(v)}")


def validate_nominal_state(state: AhrsNominalState) -> None:
    """
    Validate nominal state vector sizes for basic consistency
    """

    validate_vector_length("p_wb_m", state.p_wb_m, 3)
    validate_vector_length("v_wb_mps", state.v_wb_mps, 3)
    validate_vector_length("q_wb_wxyz", state.q_wb_wxyz, 4)
    validate_vector_length("omega_wb_rps", state.omega_wb_rps, 3)
    validate_vector_length("b_g_rps", state.b_g_rps, 3)
    validate_vector_length("b_a_mps2", state.b_a_mps2, 3)
    validate_vector_length("a_a", state.a_a, 9)
    validate_vector_length("t_bi.translation_m", state.t_bi.translation_m, 3)
    validate_vector_length("t_bi.rotation_wxyz", state.t_bi.rotation_wxyz, 4)
    validate_vector_length("t_bm.translation_m", state.t_bm.translation_m, 3)
    validate_vector_length("t_bm.rotation_wxyz", state.t_bm.rotation_wxyz, 4)
    validate_vector_length("g_w_mps2", state.g_w_mps2, 3)
    validate_vector_length("m_w_t", state.m_w_t, 3)
