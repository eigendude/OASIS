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

from oasis_control.localization.ahrs.ahrs_clock import AhrsClock
from oasis_control.localization.ahrs.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.ahrs_filter import AhrsFilter
from oasis_control.localization.ahrs.ahrs_types import AhrsEvent
from oasis_control.localization.ahrs.ahrs_types import AhrsEventType
from oasis_control.localization.ahrs.ahrs_types import AhrsImuPacket
from oasis_control.localization.ahrs.ahrs_types import AhrsOutputs
from oasis_control.localization.ahrs.ahrs_types import AhrsTime
from oasis_control.localization.ahrs.ahrs_types import AhrsUpdateData
from oasis_control.localization.ahrs.ahrs_types import ImuCalibrationData
from oasis_control.localization.ahrs.ahrs_types import ImuSample


class FixedClock(AhrsClock):
    def __init__(self, now_sec: int, now_nanosec: int) -> None:
        self._now_sec: int = now_sec
        self._now_nanosec: int = now_nanosec

    def now(self) -> AhrsTime:
        return AhrsTime(sec=self._now_sec, nanosec=self._now_nanosec)


def _config(*, gyro_gate_d2_threshold: float) -> AhrsConfig:
    return AhrsConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_sec=0.5,
        epsilon_wall_future_sec=1.0,
        dt_clock_jump_max_sec=0.0,
        dt_imu_max_sec=0.0,
        gyro_gate_d2_threshold=gyro_gate_d2_threshold,
        accel_gate_d2_threshold=9.0,
        accel_use_direction_only=False,
        mag_alpha=1.0,
        mag_r_min=[0.0] * 9,
        mag_r_max=[0.0] * 9,
        mag_r0=[0.0] * 9,
        q_v=0.0,
        q_w=0.0,
        q_bg=0.0,
        q_ba=0.0,
        q_a=0.0,
        q_bi=0.0,
        q_bm=0.0,
        q_g=0.0,
        q_m=0.0,
    )


def _imu_event_at(sec: int, nanosec: int) -> AhrsEvent:
    imu: ImuSample = ImuSample(
        frame_id="imu",
        angular_velocity_rps=[0.1, 0.0, 0.0],
        linear_acceleration_mps2=[0.0, 0.0, 0.0],
        angular_velocity_cov=[
            0.01,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.01,
        ],
        linear_acceleration_cov=[0.0] * 9,
    )
    calibration: ImuCalibrationData = ImuCalibrationData(
        valid=True,
        frame_id="imu",
        accel_bias_mps2=[0.0, 0.0, 0.0],
        accel_a=[0.0] * 9,
        accel_param_cov=[0.0] * 144,
        gyro_bias_rps=[0.0, 0.0, 0.0],
        gyro_bias_cov=[0.0] * 9,
        gravity_mps2=9.81,
        fit_sample_count=0,
        rms_residual_mps2=0.0,
        temperature_c=20.0,
        temperature_var_c2=0.0,
    )
    packet: AhrsImuPacket = AhrsImuPacket(imu=imu, calibration=calibration)

    return AhrsEvent(
        t_meas=AhrsTime(sec=sec, nanosec=nanosec),
        topic="imu",
        frame_id="imu",
        event_type=AhrsEventType.IMU,
        payload=packet,
    )


def test_filter_reports_gyro_update() -> None:
    clock: FixedClock = FixedClock(now_sec=10, now_nanosec=0)
    filt: AhrsFilter = AhrsFilter(
        config=_config(gyro_gate_d2_threshold=9.0),
        clock=clock,
    )

    outputs: AhrsOutputs = filt.handle_event(_imu_event_at(1, 0))

    gyro_update: AhrsUpdateData
    assert outputs.gyro_update is not None
    gyro_update = outputs.gyro_update
    assert gyro_update.accepted is True
    assert gyro_update.z == [0.1, 0.0, 0.0]
    assert gyro_update.r.rows == 3
    assert gyro_update.s.rows == 3
