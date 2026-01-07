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

import math

from oasis_control.localization.ekf.core.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_buffer import EkfBuffer
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import EkfUpdateData
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import MagSample
from oasis_control.localization.ekf.ekf_types import from_ns


_NS_PER_S: int = 1_000_000_000
_NS_PER_MS: int = 1_000_000


def _ns_from_ms(milliseconds: int) -> int:
    return milliseconds * _NS_PER_MS


def _build_config(*, mag_alpha: float = 0.01) -> EkfConfig:
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_ns=_NS_PER_S,
        epsilon_wall_future_ns=_ns_from_ms(100),
        dt_clock_jump_max_ns=_NS_PER_S,
        dt_imu_max_ns=_ns_from_ms(50),
        pos_var=1.0,
        vel_var=1.0,
        ang_var=1.0,
        accel_noise_var=0.1,
        gyro_noise_var=0.1,
        gravity_mps2=9.81,
        max_dt_ns=_ns_from_ms(10),
        checkpoint_interval_ns=_ns_from_ms(250),
        apriltag_pos_var=1.0,
        apriltag_yaw_var=1.0,
        apriltag_gate_d2=0.0,
        apriltag_reproj_rms_gate_px=0.0,
        tag_size_m=0.16,
        tag_anchor_family="tag36h11",
        tag_anchor_id=0,
        tag_landmark_prior_sigma_t_m=0.1,
        tag_landmark_prior_sigma_rot_rad=0.1,
        extrinsic_prior_sigma_t_m=1.0,
        extrinsic_prior_sigma_rot_rad=math.pi,
        mag_alpha=mag_alpha,
        mag_r_min_t2=[1.0e-12, 1.0e-12, 1.0e-12],
        mag_r_max_t2=[2.5e-9, 2.5e-9, 2.5e-9],
        mag_r0_t2=[4.0e-10, 4.0e-10, 4.0e-10],
        mag_world_t=[1.0, 0.0, 0.0],
    )


def _build_mag_sample(*, covariance: list[float]) -> MagSample:
    return MagSample(
        frame_id="mag",
        magnetic_field_t=[0.0, 0.0, 0.0],
        magnetic_field_cov=covariance,
    )


def _build_imu_sample() -> ImuSample:
    accel_cov: list[float] = [0.0] * 9
    gyro_cov: list[float] = [0.0] * 9
    return ImuSample(
        frame_id="imu",
        angular_velocity_rps=[0.0, 0.0, 0.0],
        linear_acceleration_mps2=[0.0, 0.0, 9.81],
        angular_velocity_cov=gyro_cov,
        linear_acceleration_cov=accel_cov,
    )


def _diag_values(values: list[float]) -> list[float]:
    return [
        values[0],
        0.0,
        0.0,
        0.0,
        values[1],
        0.0,
        0.0,
        0.0,
        values[2],
    ]


def _assert_sequence_close(
    actual: list[float], expected: list[float], *, tol: float
) -> None:
    assert len(actual) == len(expected)
    index: int
    for index in range(len(actual)):
        assert math.isclose(actual[index], expected[index], abs_tol=tol)


def _assert_symmetric(matrix: list[list[float]], *, tol: float) -> None:
    for row_index in range(3):
        for col_index in range(3):
            assert math.isclose(
                matrix[row_index][col_index],
                matrix[col_index][row_index],
                abs_tol=tol,
            )


def _determinant_3x3(matrix: list[list[float]]) -> float:
    a11: float = matrix[0][0]
    a12: float = matrix[0][1]
    a13: float = matrix[0][2]
    a21: float = matrix[1][0]
    a22: float = matrix[1][1]
    a23: float = matrix[1][2]
    a31: float = matrix[2][0]
    a32: float = matrix[2][1]
    a33: float = matrix[2][2]
    return (
        a11 * (a22 * a33 - a23 * a32)
        - a12 * (a21 * a33 - a23 * a31)
        + a13 * (a21 * a32 - a22 * a31)
    )


def _assert_spd(matrix: list[list[float]]) -> None:
    _assert_symmetric(matrix, tol=1.0e-12)
    det1: float = matrix[0][0]
    det2: float = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]
    det3: float = _determinant_3x3(matrix)
    assert det1 > 0.0
    assert det2 > 0.0
    assert det3 > 0.0


def _capture_state(core: EkfCore) -> tuple[list[float], list[list[float]]]:
    state: list[float] = core.state().tolist()
    mag_covariance = core._state.mag_covariance
    assert mag_covariance is not None
    return state, mag_covariance.tolist()


def test_mag_covariance_initializes_from_first_valid_covariance() -> None:
    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)
    sample_covariance: list[float] = _diag_values([1.0e-9, 2.0e-9, 3.0e-9])
    update: EkfUpdateData = core.update_with_mag(
        _build_mag_sample(covariance=sample_covariance),
        EkfTime(sec=0, nanosec=0),
    )
    assert update.r.data == sample_covariance


def test_mag_covariance_initializes_from_default_when_invalid() -> None:
    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)
    sample_covariance: list[float] = [0.0] * 9
    update: EkfUpdateData = core.update_with_mag(
        _build_mag_sample(covariance=sample_covariance),
        EkfTime(sec=0, nanosec=0),
    )
    assert update.r.data == _diag_values(config.mag_r0_t2)


def test_mag_covariance_adapts_and_stays_bounded() -> None:
    config: EkfConfig = _build_config(mag_alpha=0.5)
    core: EkfCore = EkfCore(config)
    sample_covariance: list[float] = _diag_values([1.0e-9, 1.0e-9, 1.0e-9])
    core.update_with_mag(
        _build_mag_sample(covariance=sample_covariance),
        EkfTime(sec=0, nanosec=0),
    )
    mag_covariance = core._state.mag_covariance
    assert mag_covariance is not None
    matrix: list[list[float]] = mag_covariance.tolist()
    _assert_spd(matrix)
    diag_values: list[float] = [matrix[0][0], matrix[1][1], matrix[2][2]]
    for value in diag_values:
        assert config.mag_r_min_t2[0] <= value <= config.mag_r_max_t2[0]


def test_mag_replay_determinism_with_out_of_order_insert() -> None:
    config: EkfConfig = _build_config()
    buffer_ordered: EkfBuffer = EkfBuffer(config)
    buffer_out_of_order: EkfBuffer = EkfBuffer(config)
    imu_event_0: EkfEvent = EkfEvent(
        t_meas=from_ns(0),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=_build_imu_sample(), calibration=None),
    )
    imu_event_1: EkfEvent = EkfEvent(
        t_meas=from_ns(_ns_from_ms(10)),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=_build_imu_sample(), calibration=None),
    )
    imu_event_2: EkfEvent = EkfEvent(
        t_meas=from_ns(_ns_from_ms(20)),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=_build_imu_sample(), calibration=None),
    )
    mag_covariance: list[float] = _diag_values([1.0e-9, 1.0e-9, 1.0e-9])
    mag_event_1: EkfEvent = EkfEvent(
        t_meas=from_ns(_ns_from_ms(10)),
        event_type=EkfEventType.MAG,
        payload=_build_mag_sample(covariance=mag_covariance),
    )
    mag_event_2: EkfEvent = EkfEvent(
        t_meas=from_ns(_ns_from_ms(20)),
        event_type=EkfEventType.MAG,
        payload=_build_mag_sample(covariance=mag_covariance),
    )
    for event in [imu_event_0, imu_event_1, mag_event_1, imu_event_2, mag_event_2]:
        buffer_ordered.insert_event(event)
    for event in [imu_event_0, imu_event_2, mag_event_2, imu_event_1, mag_event_1]:
        buffer_out_of_order.insert_event(event)

    core_ordered: EkfCore = EkfCore(config)
    core_out_of_order: EkfCore = EkfCore(config)
    core_ordered.replay(buffer_ordered)
    core_out_of_order.replay(buffer_out_of_order)

    state_ordered, mag_ordered = _capture_state(core_ordered)
    state_out_of_order, mag_out_of_order = _capture_state(core_out_of_order)
    _assert_sequence_close(state_ordered, state_out_of_order, tol=1.0e-9)
    _assert_sequence_close(
        [value for row in mag_ordered for value in row],
        [value for row in mag_out_of_order for value in row],
        tol=1.0e-9,
    )
