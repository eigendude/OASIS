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
Stub AHRS filter implementation
"""

from __future__ import annotations

import math
from typing import Optional
from typing import cast

from oasis_control.localization.ahrs.ahrs_clock import AhrsClock
from oasis_control.localization.ahrs.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.ahrs_types import AhrsDiagnosticsData
from oasis_control.localization.ahrs.ahrs_types import AhrsEvent
from oasis_control.localization.ahrs.ahrs_types import AhrsEventType
from oasis_control.localization.ahrs.ahrs_types import AhrsFrameOutputs
from oasis_control.localization.ahrs.ahrs_types import AhrsFrameTransform
from oasis_control.localization.ahrs.ahrs_types import AhrsImuPacket
from oasis_control.localization.ahrs.ahrs_types import AhrsMatrix
from oasis_control.localization.ahrs.ahrs_types import AhrsOutputs
from oasis_control.localization.ahrs.ahrs_types import AhrsSe3Transform
from oasis_control.localization.ahrs.ahrs_types import AhrsStateData
from oasis_control.localization.ahrs.ahrs_types import AhrsTime
from oasis_control.localization.ahrs.ahrs_types import AhrsUpdateData
from oasis_control.localization.ahrs.ahrs_types import MagSample


class AhrsFilter:
    """
    Stubbed AHRS filter for deterministic event handling
    """

    def __init__(self, config: AhrsConfig, clock: AhrsClock) -> None:
        self._config: AhrsConfig = config
        self._clock: AhrsClock = clock

        self._initialized: bool = False
        self._t_filter: Optional[AhrsTime] = None
        self._state_seq: int = 0
        self._diag_seq: int = 0

        self._dropped_missing_stamp: int = 0
        self._dropped_future_stamp: int = 0
        self._dropped_too_old: int = 0
        self._dropped_nan_cov: int = 0
        self._dropped_imu_gap: int = 0
        self._dropped_clock_jump_reset: int = 0
        self._reset_count: int = 0
        self._last_reset_reason: str = ""

        self._last_imu_time: Optional[AhrsTime] = None

    def handle_event(self, event: AhrsEvent) -> AhrsOutputs:
        update_reports: dict[str, Optional[AhrsUpdateData]] = (
            self._build_update_reports(event)
        )

        if self._clock.is_future_stamp(
            event.t_meas, self._config.epsilon_wall_future_sec
        ):
            self._dropped_future_stamp += 1
            return self._outputs_for_drop(update_reports)

        if self._has_nan_covariance(event):
            self._dropped_nan_cov += 1
            return self._outputs_for_drop(update_reports)

        if self._is_clock_jump(event.t_meas):
            self._dropped_clock_jump_reset += 1
            self._reset_count += 1
            self._last_reset_reason = "clock_jump"
            self._initialized = False
            self._t_filter = None
            self._last_imu_time = None
            return self._outputs_for_drop(update_reports)

        if self._is_out_of_order(event.t_meas):
            self._dropped_too_old += 1
            return self._outputs_for_drop(update_reports)

        if self._is_imu_gap(event):
            self._dropped_imu_gap += 1
            return self._outputs_for_drop(update_reports)

        self._t_filter = event.t_meas
        self._initialized = True
        self._state_seq += 1
        self._diag_seq += 1

        if event.event_type == AhrsEventType.IMU:
            self._last_imu_time = event.t_meas

        state: AhrsStateData = self._build_state(event.t_meas)
        diagnostics: AhrsDiagnosticsData = self._build_diagnostics(event.t_meas)
        frame_transforms: AhrsFrameOutputs = self._build_frame_outputs()

        return AhrsOutputs(
            frontier_advanced=True,
            t_filter=event.t_meas,
            frame_transforms=frame_transforms,
            state=state,
            diagnostics=diagnostics,
            gyro_update=update_reports["gyro"],
            accel_update=update_reports["accel"],
            mag_update=update_reports["mag"],
        )

    def _is_out_of_order(self, t_meas: AhrsTime) -> bool:
        if self._t_filter is None:
            return False

        return (t_meas.sec, t_meas.nanosec) <= (
            self._t_filter.sec,
            self._t_filter.nanosec,
        )

    def _is_clock_jump(self, t_meas: AhrsTime) -> bool:
        if self._t_filter is None:
            return False

        if self._config.dt_clock_jump_max_sec <= 0.0:
            return False

        if (t_meas.sec, t_meas.nanosec) >= (
            self._t_filter.sec,
            self._t_filter.nanosec,
        ):
            return False

        dt_sec: float = self._seconds_between(self._t_filter, t_meas)
        return dt_sec > self._config.dt_clock_jump_max_sec

    def _is_imu_gap(self, event: AhrsEvent) -> bool:
        if event.event_type != AhrsEventType.IMU:
            return False

        if self._last_imu_time is None:
            return False

        if self._config.dt_imu_max_sec <= 0.0:
            return False

        dt_sec: float = self._seconds_between(self._last_imu_time, event.t_meas)
        return dt_sec > self._config.dt_imu_max_sec

    def _seconds_between(self, t_start: AhrsTime, t_end: AhrsTime) -> float:
        start_sec: float = float(t_start.sec) + float(t_start.nanosec) * 1e-9
        end_sec: float = float(t_end.sec) + float(t_end.nanosec) * 1e-9

        return abs(end_sec - start_sec)

    def _has_nan_covariance(self, event: AhrsEvent) -> bool:
        covariances: list[float] = []
        if event.event_type == AhrsEventType.IMU:
            payload_imu = cast(AhrsImuPacket, event.payload)
            covariances = (
                payload_imu.imu.angular_velocity_cov
                + payload_imu.imu.linear_acceleration_cov
            )
        elif event.event_type == AhrsEventType.MAG:
            payload_mag = cast(MagSample, event.payload)
            covariances = payload_mag.magnetic_field_cov

        return any(math.isnan(value) for value in covariances)

    def _build_update_reports(
        self, event: AhrsEvent
    ) -> dict[str, Optional[AhrsUpdateData]]:
        update_reports: dict[str, Optional[AhrsUpdateData]] = {
            "gyro": None,
            "accel": None,
            "mag": None,
        }

        if event.event_type == AhrsEventType.IMU:
            update_reports["gyro"] = self._build_update(
                sensor="gyro",
                frame_id=event.frame_id,
                t_meas=event.t_meas,
            )
            update_reports["accel"] = self._build_update(
                sensor="accel",
                frame_id=event.frame_id,
                t_meas=event.t_meas,
            )
        elif event.event_type == AhrsEventType.MAG:
            update_reports["mag"] = self._build_update(
                sensor="mag",
                frame_id=event.frame_id,
                t_meas=event.t_meas,
            )

        return update_reports

    def _build_update(
        self, sensor: str, frame_id: str, t_meas: AhrsTime
    ) -> AhrsUpdateData:
        empty_matrix: AhrsMatrix = AhrsMatrix(rows=0, cols=0, data=[])

        return AhrsUpdateData(
            sensor=sensor,
            frame_id=frame_id,
            t_meas=t_meas,
            accepted=False,
            reject_reason="not_implemented",
            z=[],
            z_hat=[],
            nu=[],
            r=empty_matrix,
            s_hat=empty_matrix,
            s=empty_matrix,
            maha_d2=0.0,
            gate_threshold=0.0,
        )

    def _build_state(self, t_filter: AhrsTime) -> AhrsStateData:
        empty_matrix: AhrsMatrix = AhrsMatrix(rows=0, cols=0, data=[])
        zero_vector: list[float] = [0.0, 0.0, 0.0]
        zero_quaternion: list[float] = [1.0, 0.0, 0.0, 0.0]

        t_bi: AhrsSe3Transform = AhrsSe3Transform(
            parent_frame=self._config.body_frame_id,
            child_frame="imu",
            translation_m=zero_vector,
            rotation_wxyz=zero_quaternion,
        )
        t_bm: AhrsSe3Transform = AhrsSe3Transform(
            parent_frame=self._config.body_frame_id,
            child_frame="mag",
            translation_m=zero_vector,
            rotation_wxyz=zero_quaternion,
        )

        return AhrsStateData(
            t_filter=t_filter,
            state_seq=self._state_seq,
            initialized=self._initialized,
            world_frame_id=self._config.world_frame_id,
            odom_frame_id=self._config.odom_frame_id,
            body_frame_id=self._config.body_frame_id,
            p_wb_m=zero_vector,
            v_wb_mps=zero_vector,
            q_wb_wxyz=zero_quaternion,
            b_g_rps=zero_vector,
            b_a_mps2=zero_vector,
            a_a=[0.0] * 9,
            t_bi=t_bi,
            t_bm=t_bm,
            g_w_mps2=zero_vector,
            m_w_t=zero_vector,
            error_state_names=[],
            p_cov=empty_matrix,
        )

    def _build_diagnostics(self, t_filter: AhrsTime) -> AhrsDiagnosticsData:
        return AhrsDiagnosticsData(
            t_filter=t_filter,
            diag_seq=self._diag_seq,
            buffer_node_count=0,
            buffer_span_sec=0.0,
            replay_happened=False,
            dropped_missing_stamp=self._dropped_missing_stamp,
            dropped_future_stamp=self._dropped_future_stamp,
            dropped_too_old=self._dropped_too_old,
            dropped_nan_cov=self._dropped_nan_cov,
            dropped_imu_gap=self._dropped_imu_gap,
            dropped_clock_jump_reset=self._dropped_clock_jump_reset,
            reset_count=self._reset_count,
            last_reset_reason=self._last_reset_reason,
        )

    def _build_frame_outputs(self) -> AhrsFrameOutputs:
        zero_vector: list[float] = [0.0, 0.0, 0.0]
        zero_quaternion: list[float] = [1.0, 0.0, 0.0, 0.0]

        t_world_odom: AhrsFrameTransform = AhrsFrameTransform(
            parent_frame="world",
            child_frame="odom",
            translation_m=zero_vector,
            rotation_wxyz=zero_quaternion,
        )
        t_odom_base: AhrsFrameTransform = AhrsFrameTransform(
            parent_frame="odom",
            child_frame="base_link",
            translation_m=zero_vector,
            rotation_wxyz=zero_quaternion,
        )
        t_world_base: AhrsFrameTransform = AhrsFrameTransform(
            parent_frame="world",
            child_frame="base_link",
            translation_m=zero_vector,
            rotation_wxyz=zero_quaternion,
        )

        return AhrsFrameOutputs(
            t_odom_base=t_odom_base,
            t_world_odom=t_world_odom,
            t_world_base=t_world_base,
        )

    def _outputs_for_drop(
        self, update_reports: dict[str, Optional[AhrsUpdateData]]
    ) -> AhrsOutputs:
        return AhrsOutputs(
            frontier_advanced=False,
            t_filter=self._t_filter,
            frame_transforms=None,
            state=None,
            diagnostics=None,
            gyro_update=update_reports["gyro"],
            accel_update=update_reports["accel"],
            mag_update=update_reports["mag"],
        )
