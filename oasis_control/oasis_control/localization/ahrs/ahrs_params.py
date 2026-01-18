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
ROS parameter helpers for the AHRS node
"""

from __future__ import annotations

import rclpy.node

from oasis_control.localization.ahrs.ahrs_config import AhrsConfig


PARAM_WORLD_FRAME_ID: str = "world_frame_id"
PARAM_ODOM_FRAME_ID: str = "odom_frame_id"
PARAM_BODY_FRAME_ID: str = "body_frame_id"

PARAM_T_BUFFER_SEC: str = "t_buffer_sec"
PARAM_EPS_WALL_FUTURE_SEC: str = "epsilon_wall_future_sec"
PARAM_DT_CLOCK_JUMP_MAX_SEC: str = "dt_clock_jump_max_sec"
PARAM_DT_IMU_MAX_SEC: str = "dt_imu_max_sec"

PARAM_MAG_ALPHA: str = "mag_alpha"
PARAM_MAG_R_MIN: str = "mag_r_min"
PARAM_MAG_R_MAX: str = "mag_r_max"
PARAM_MAG_R0: str = "mag_r0"

PARAM_Q_V: str = "q_v"
PARAM_Q_W: str = "q_w"
PARAM_Q_BG: str = "q_bg"
PARAM_Q_BA: str = "q_ba"
PARAM_Q_A: str = "q_a"
PARAM_Q_BI: str = "q_bi"
PARAM_Q_BM: str = "q_bm"
PARAM_Q_G: str = "q_g"
PARAM_Q_M: str = "q_m"

DEFAULT_WORLD_FRAME_ID: str = "world"
DEFAULT_ODOM_FRAME_ID: str = "odom"
DEFAULT_BODY_FRAME_ID: str = "base_link"

DEFAULT_T_BUFFER_SEC: float = 0.5
DEFAULT_EPS_WALL_FUTURE_SEC: float = 0.1
DEFAULT_DT_CLOCK_JUMP_MAX_SEC: float = 1.0
DEFAULT_DT_IMU_MAX_SEC: float = 0.5

DEFAULT_MAG_ALPHA: float = 1.0
DEFAULT_MAG_R_MIN: list[float] = [0.0] * 9
DEFAULT_MAG_R_MAX: list[float] = [0.0] * 9
DEFAULT_MAG_R0: list[float] = [0.0] * 9

DEFAULT_Q_V: float = 0.0
DEFAULT_Q_W: float = 0.0
DEFAULT_Q_BG: float = 0.0
DEFAULT_Q_BA: float = 0.0
DEFAULT_Q_A: float = 0.0
DEFAULT_Q_BI: float = 0.0
DEFAULT_Q_BM: float = 0.0
DEFAULT_Q_G: float = 0.0
DEFAULT_Q_M: float = 0.0


def declare_ahrs_params(node: rclpy.node.Node) -> None:
    """
    Declare AHRS ROS parameters with defaults
    """

    node.declare_parameter(PARAM_WORLD_FRAME_ID, DEFAULT_WORLD_FRAME_ID)
    node.declare_parameter(PARAM_ODOM_FRAME_ID, DEFAULT_ODOM_FRAME_ID)
    node.declare_parameter(PARAM_BODY_FRAME_ID, DEFAULT_BODY_FRAME_ID)

    node.declare_parameter(PARAM_T_BUFFER_SEC, DEFAULT_T_BUFFER_SEC)
    node.declare_parameter(PARAM_EPS_WALL_FUTURE_SEC, DEFAULT_EPS_WALL_FUTURE_SEC)
    node.declare_parameter(PARAM_DT_CLOCK_JUMP_MAX_SEC, DEFAULT_DT_CLOCK_JUMP_MAX_SEC)
    node.declare_parameter(PARAM_DT_IMU_MAX_SEC, DEFAULT_DT_IMU_MAX_SEC)

    node.declare_parameter(PARAM_MAG_ALPHA, DEFAULT_MAG_ALPHA)
    node.declare_parameter(PARAM_MAG_R_MIN, DEFAULT_MAG_R_MIN)
    node.declare_parameter(PARAM_MAG_R_MAX, DEFAULT_MAG_R_MAX)
    node.declare_parameter(PARAM_MAG_R0, DEFAULT_MAG_R0)

    node.declare_parameter(PARAM_Q_V, DEFAULT_Q_V)
    node.declare_parameter(PARAM_Q_W, DEFAULT_Q_W)
    node.declare_parameter(PARAM_Q_BG, DEFAULT_Q_BG)
    node.declare_parameter(PARAM_Q_BA, DEFAULT_Q_BA)
    node.declare_parameter(PARAM_Q_A, DEFAULT_Q_A)
    node.declare_parameter(PARAM_Q_BI, DEFAULT_Q_BI)
    node.declare_parameter(PARAM_Q_BM, DEFAULT_Q_BM)
    node.declare_parameter(PARAM_Q_G, DEFAULT_Q_G)
    node.declare_parameter(PARAM_Q_M, DEFAULT_Q_M)


def load_ahrs_config(node: rclpy.node.Node) -> AhrsConfig:
    """
    Load AHRS configuration from ROS parameters
    """

    world_frame_id: str = str(node.get_parameter(PARAM_WORLD_FRAME_ID).value)
    odom_frame_id: str = str(node.get_parameter(PARAM_ODOM_FRAME_ID).value)
    body_frame_id: str = str(node.get_parameter(PARAM_BODY_FRAME_ID).value)

    t_buffer_sec: float = float(node.get_parameter(PARAM_T_BUFFER_SEC).value)
    epsilon_wall_future_sec: float = float(
        node.get_parameter(PARAM_EPS_WALL_FUTURE_SEC).value
    )
    dt_clock_jump_max_sec: float = float(
        node.get_parameter(PARAM_DT_CLOCK_JUMP_MAX_SEC).value
    )
    dt_imu_max_sec: float = float(node.get_parameter(PARAM_DT_IMU_MAX_SEC).value)

    mag_alpha: float = float(node.get_parameter(PARAM_MAG_ALPHA).value)
    mag_r_min: list[float] = [
        float(value) for value in node.get_parameter(PARAM_MAG_R_MIN).value
    ]
    mag_r_max: list[float] = [
        float(value) for value in node.get_parameter(PARAM_MAG_R_MAX).value
    ]
    mag_r0: list[float] = [
        float(value) for value in node.get_parameter(PARAM_MAG_R0).value
    ]

    q_v: float = float(node.get_parameter(PARAM_Q_V).value)
    q_w: float = float(node.get_parameter(PARAM_Q_W).value)
    q_bg: float = float(node.get_parameter(PARAM_Q_BG).value)
    q_ba: float = float(node.get_parameter(PARAM_Q_BA).value)
    q_a: float = float(node.get_parameter(PARAM_Q_A).value)
    q_bi: float = float(node.get_parameter(PARAM_Q_BI).value)
    q_bm: float = float(node.get_parameter(PARAM_Q_BM).value)
    q_g: float = float(node.get_parameter(PARAM_Q_G).value)
    q_m: float = float(node.get_parameter(PARAM_Q_M).value)

    return AhrsConfig(
        world_frame_id=world_frame_id,
        odom_frame_id=odom_frame_id,
        body_frame_id=body_frame_id,
        t_buffer_sec=t_buffer_sec,
        epsilon_wall_future_sec=epsilon_wall_future_sec,
        dt_clock_jump_max_sec=dt_clock_jump_max_sec,
        dt_imu_max_sec=dt_imu_max_sec,
        mag_alpha=mag_alpha,
        mag_r_min=mag_r_min,
        mag_r_max=mag_r_max,
        mag_r0=mag_r0,
        q_v=q_v,
        q_w=q_w,
        q_bg=q_bg,
        q_ba=q_ba,
        q_a=q_a,
        q_bi=q_bi,
        q_bm=q_bm,
        q_g=q_g,
        q_m=q_m,
    )
