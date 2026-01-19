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
from typing import Iterable
from typing import Optional
from typing import cast

from oasis_control.localization.ahrs.ahrs_clock import AhrsClock
from oasis_control.localization.ahrs.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.ahrs_error_state import AhrsErrorStateLayout
from oasis_control.localization.ahrs.ahrs_error_state import error_state_names
from oasis_control.localization.ahrs.ahrs_predict import predict_step
from oasis_control.localization.ahrs.ahrs_state import AhrsNominalState
from oasis_control.localization.ahrs.ahrs_state import default_nominal_state
from oasis_control.localization.ahrs.ahrs_timeline import AhrsTimeBuffer
from oasis_control.localization.ahrs.ahrs_timeline import AhrsTimeNode
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
from oasis_control.localization.ahrs.ahrs_update_accel import update_accel
from oasis_control.localization.ahrs.ahrs_update_gyro import update_gyro
from oasis_control.localization.ahrs.ahrs_update_mag import update_mag


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
        self._buffer: AhrsTimeBuffer = AhrsTimeBuffer()
        self._replay_happened_since_publish: bool = False
        self._layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
        self._x: Optional[AhrsNominalState] = None
        self._p: Optional[list[float]] = None
        self._t_state: Optional[AhrsTime] = None
        self._mag_r_cov: Optional[list[float]] = None
        self._last_update_reports: dict[str, Optional[AhrsUpdateData]] = {
            "gyro": None,
            "accel": None,
            "mag": None,
        }

    def handle_event(self, event: AhrsEvent) -> AhrsOutputs:
        if self._clock.is_future_stamp(
            event.t_meas, self._config.epsilon_wall_future_sec
        ):
            self._dropped_future_stamp += 1
            return self._outputs_for_drop(self._build_update_reports(event))

        if self._has_nan_covariance(event):
            self._dropped_nan_cov += 1
            return self._outputs_for_drop(self._build_update_reports(event))

        if self._is_clock_jump(event.t_meas):
            self._reset_for_clock_jump()
            return self._outputs_for_drop(self._build_update_reports(event))

        if self._is_too_old(event.t_meas):
            self._dropped_too_old += 1
            return self._outputs_for_drop(self._build_update_reports(event))

        self._buffer.insert_event(event)

        if self._frontier_advances(event.t_meas):
            if self._replay_has_imu_gap(
                start_time=self._t_filter, end_time=event.t_meas
            ):
                self._dropped_imu_gap += 1
                return self._outputs_for_drop(self._build_update_reports(event))

            self._t_filter = event.t_meas
            self._initialized = True
            self._state_seq += 1
            self._diag_seq += 1
            self._evict_before(self._t_filter)
            self._replay_events(start_time=None, end_time=self._t_filter)

            state: AhrsStateData = self._build_state(event.t_meas)
            replay_happened: bool = self._replay_happened_since_publish
            diagnostics: AhrsDiagnosticsData = self._build_diagnostics(
                t_filter=event.t_meas,
                replay_happened=replay_happened,
            )
            self._replay_happened_since_publish = False
            frame_transforms: AhrsFrameOutputs = self._build_frame_outputs()

            update_reports: dict[str, Optional[AhrsUpdateData]] = (
                self._update_reports_for_event(event)
            )

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

        if self._replay_has_imu_gap(start_time=event.t_meas, end_time=self._t_filter):
            self._dropped_imu_gap += 1
        else:
            replayed: bool = self._replay_events(
                start_time=event.t_meas, end_time=self._t_filter
            )
            if replayed:
                self._replay_happened_since_publish = True

        return self._outputs_for_drop(self._build_update_reports(event))

    def _frontier_advances(self, t_meas: AhrsTime) -> bool:
        if self._t_filter is None:
            return True

        return self._time_key(t_meas) > self._time_key(self._t_filter)

    def _is_clock_jump(self, t_meas: AhrsTime) -> bool:
        if self._t_filter is None:
            return False

        if self._config.dt_clock_jump_max_sec <= 0.0:
            return False

        dt_sec: float = self._seconds_between(self._t_filter, t_meas)
        return dt_sec > self._config.dt_clock_jump_max_sec

    def _is_too_old(self, t_meas: AhrsTime) -> bool:
        if self._t_filter is None:
            return False

        if self._config.t_buffer_sec <= 0.0:
            return False

        # t_buffer_sec converted to nanoseconds for window comparison
        buffer_ns: int = int(self._config.t_buffer_sec * 1_000_000_000)
        cutoff_ns: int = self._time_to_ns(self._t_filter) - buffer_ns
        return self._time_to_ns(t_meas) < cutoff_ns

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

    def _reset_for_clock_jump(self) -> None:
        self._dropped_clock_jump_reset += 1
        self._reset_count += 1
        self._last_reset_reason = "clock_jump"
        self._initialized = False
        self._t_filter = None
        self._last_imu_time = None
        self._buffer = AhrsTimeBuffer()
        self._replay_happened_since_publish = False
        self._x = None
        self._p = None
        self._t_state = None
        self._mag_r_cov = None
        self._last_update_reports = {
            "gyro": None,
            "accel": None,
            "mag": None,
        }

    def _reset_for_invalid_dt(self) -> None:
        self._reset_count += 1
        self._last_reset_reason = "invalid_dt"
        self._initialized = False
        self._t_filter = None
        self._last_imu_time = None
        self._buffer = AhrsTimeBuffer()
        self._replay_happened_since_publish = False
        self._x = None
        self._p = None
        self._t_state = None
        self._mag_r_cov = None
        self._last_update_reports = {
            "gyro": None,
            "accel": None,
            "mag": None,
        }

    def _replay_has_imu_gap(
        self, start_time: Optional[AhrsTime], end_time: Optional[AhrsTime]
    ) -> bool:
        if end_time is None:
            return False

        if self._config.dt_imu_max_sec <= 0.0:
            return False

        # dt_imu_max_sec converted to nanoseconds for gap detection
        gap_threshold_ns: int = int(self._config.dt_imu_max_sec * 1_000_000_000)
        previous_imu_time: Optional[AhrsTime] = None
        for node in self._iter_nodes_for_replay(start_time, end_time):
            has_imu_event: bool = any(
                event.event_type == AhrsEventType.IMU for event in node.events
            )
            if not has_imu_event:
                continue

            if previous_imu_time is not None:
                dt_ns: int = self._time_to_ns(node.t) - self._time_to_ns(
                    previous_imu_time
                )
                if dt_ns > gap_threshold_ns:
                    return True
            previous_imu_time = node.t

        return False

    def _iter_nodes_for_replay(
        self, start_time: Optional[AhrsTime], end_time: AhrsTime
    ) -> Iterable[AhrsTimeNode]:
        nodes: Iterable[AhrsTimeNode]
        if start_time is None:
            nodes = self._buffer.iter_nodes()
        else:
            nodes = self._buffer.iter_nodes_from(start_time)

        end_key: tuple[int, int] = self._time_key(end_time)
        for node in nodes:
            if self._time_key(node.t) > end_key:
                break
            yield node

    def _replay_events(
        self, start_time: Optional[AhrsTime], end_time: Optional[AhrsTime]
    ) -> bool:
        if end_time is None:
            return False

        if start_time is None:
            self._reset_replay_state(None)
        elif self._t_state is None or self._time_key(start_time) < self._time_key(
            self._t_state
        ):
            self._reset_replay_state(start_time)

        replayed: bool = False
        for node in self._iter_nodes_for_replay(start_time, end_time):
            replayed = True
            try:
                self._propagate_to(node.t)
            except ValueError:
                self._reset_for_invalid_dt()
                return False
            for event in self._sorted_events(node):
                self._apply_event(event)
        return replayed

    def _sorted_events(self, node: AhrsTimeNode) -> list[AhrsEvent]:
        return sorted(
            node.events,
            key=lambda event: (
                event.event_type.value,
                event.topic,
                event.frame_id,
            ),
        )

    def _apply_event(self, event: AhrsEvent) -> None:
        if event.event_type == AhrsEventType.IMU:
            payload_imu: AhrsImuPacket = cast(AhrsImuPacket, event.payload)
            gyro_report: AhrsUpdateData
            if self._x is None or self._p is None:
                gyro_report = self._build_update(
                    sensor="gyro",
                    frame_id=event.frame_id,
                    t_meas=event.t_meas,
                    reject_reason="not_applied",
                )
            else:
                self._x, self._p, gyro_report = update_gyro(
                    self._layout,
                    self._x,
                    self._p,
                    payload_imu.imu,
                    self._config,
                    frame_id=event.frame_id,
                    t_meas=event.t_meas,
                )
            self._last_update_reports["gyro"] = gyro_report
            accel_report: AhrsUpdateData
            if self._x is None or self._p is None:
                accel_report = self._build_update(
                    sensor="accel",
                    frame_id=event.frame_id,
                    t_meas=event.t_meas,
                    reject_reason="not_applied",
                )
            else:
                self._x, self._p, accel_report = update_accel(
                    self._layout,
                    self._x,
                    self._p,
                    payload_imu.imu,
                    self._config,
                    frame_id=event.frame_id,
                    t_meas=event.t_meas,
                )
            self._last_update_reports["accel"] = accel_report
            self._last_imu_time = event.t_meas
            return

        if event.event_type == AhrsEventType.MAG:
            payload_mag: MagSample = cast(MagSample, event.payload)
            mag_report: AhrsUpdateData
            if self._x is None or self._p is None:
                mag_report = self._build_update(
                    sensor="mag",
                    frame_id=event.frame_id,
                    t_meas=event.t_meas,
                    reject_reason="not_applied",
                )
            else:
                self._x, self._p, mag_report, self._mag_r_cov = update_mag(
                    self._layout,
                    self._x,
                    self._p,
                    payload_mag,
                    self._config,
                    frame_id=event.frame_id,
                    t_meas=event.t_meas,
                    r_state=self._mag_r_cov,
                )
            self._last_update_reports["mag"] = mag_report

    def _evict_before(self, t_filter: AhrsTime) -> None:
        if self._config.t_buffer_sec <= 0.0:
            return

        # t_buffer_sec converted to nanoseconds for eviction cutoff
        buffer_ns: int = int(self._config.t_buffer_sec * 1_000_000_000)
        cutoff_ns: int = self._time_to_ns(t_filter) - buffer_ns
        cutoff_time: AhrsTime = self._time_from_ns(cutoff_ns)
        self._buffer.evict_older_than(cutoff_time)

    def _time_key(self, t: AhrsTime) -> tuple[int, int]:
        return (t.sec, t.nanosec)

    def _time_to_ns(self, t: AhrsTime) -> int:
        # 1e9 converts seconds to nanoseconds for integer time arithmetic
        return t.sec * 1_000_000_000 + t.nanosec

    def _time_from_ns(self, nanoseconds: int) -> AhrsTime:
        sec: int
        nanosec: int
        # 1e9 converts nanoseconds to seconds for integer time arithmetic
        sec, nanosec = divmod(nanoseconds, 1_000_000_000)
        return AhrsTime(sec=sec, nanosec=nanosec)

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
                reject_reason="not_applied",
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

    def _update_reports_for_event(
        self, event: AhrsEvent
    ) -> dict[str, Optional[AhrsUpdateData]]:
        update_reports: dict[str, Optional[AhrsUpdateData]] = {
            "gyro": None,
            "accel": None,
            "mag": None,
        }

        if event.event_type == AhrsEventType.IMU:
            update_reports["gyro"] = self._last_update_reports["gyro"]
            if update_reports["gyro"] is None:
                update_reports["gyro"] = self._build_update(
                    sensor="gyro",
                    frame_id=event.frame_id,
                    t_meas=event.t_meas,
                    reject_reason="not_applied",
                )
            update_reports["accel"] = self._last_update_reports["accel"]
            if update_reports["accel"] is None:
                update_reports["accel"] = self._build_update(
                    sensor="accel",
                    frame_id=event.frame_id,
                    t_meas=event.t_meas,
                )
        elif event.event_type == AhrsEventType.MAG:
            update_reports["mag"] = self._last_update_reports["mag"]
            if update_reports["mag"] is None:
                update_reports["mag"] = self._build_update(
                    sensor="mag",
                    frame_id=event.frame_id,
                    t_meas=event.t_meas,
                )

        return update_reports

    def _build_update(
        self,
        sensor: str,
        frame_id: str,
        t_meas: AhrsTime,
        *,
        reject_reason: str = "not_implemented",
    ) -> AhrsUpdateData:
        empty_matrix: AhrsMatrix = AhrsMatrix(rows=0, cols=0, data=[])

        return AhrsUpdateData(
            sensor=sensor,
            frame_id=frame_id,
            t_meas=t_meas,
            accepted=False,
            reject_reason=reject_reason,
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
        error_names: list[str] = error_state_names(self._layout)
        error_dim: int = self._layout.dim
        p_data: list[float]
        if self._p is None:
            p_data = [0.0] * (error_dim * error_dim)
        else:
            p_data = list(self._p)
        p_cov: AhrsMatrix = AhrsMatrix(
            rows=error_dim,
            cols=error_dim,
            data=p_data,
        )

        if self._x is None:
            x: AhrsNominalState = default_nominal_state(
                body_frame_id=self._config.body_frame_id,
                imu_frame_id="imu",
                mag_frame_id="mag",
            )
        else:
            x = self._x

        t_bi: AhrsSe3Transform = x.t_bi
        t_bm: AhrsSe3Transform = x.t_bm

        return AhrsStateData(
            t_filter=t_filter,
            state_seq=self._state_seq,
            initialized=self._initialized,
            world_frame_id=self._config.world_frame_id,
            odom_frame_id=self._config.odom_frame_id,
            body_frame_id=self._config.body_frame_id,
            p_wb_m=list(x.p_wb_m),
            v_wb_mps=list(x.v_wb_mps),
            q_wb_wxyz=list(x.q_wb_wxyz),
            b_g_rps=list(x.b_g_rps),
            b_a_mps2=list(x.b_a_mps2),
            a_a=list(x.a_a),
            t_bi=t_bi,
            t_bm=t_bm,
            g_w_mps2=list(x.g_w_mps2),
            m_w_t=list(x.m_w_t),
            error_state_names=error_names,
            p_cov=p_cov,
        )

    def _build_diagnostics(
        self, t_filter: AhrsTime, replay_happened: bool
    ) -> AhrsDiagnosticsData:
        return AhrsDiagnosticsData(
            t_filter=t_filter,
            diag_seq=self._diag_seq,
            buffer_node_count=self._buffer.node_count(),
            buffer_span_sec=self._buffer.span_sec(),
            replay_happened=replay_happened,
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

    def _reset_replay_state(self, t_state: Optional[AhrsTime]) -> None:
        if t_state is None:
            self._x = None
            self._p = None
            self._t_state = None
            self._mag_r_cov = None
            self._last_update_reports = {
                "gyro": None,
                "accel": None,
                "mag": None,
            }
            return

        self._x = default_nominal_state(
            body_frame_id=self._config.body_frame_id,
            imu_frame_id="imu",
            mag_frame_id="mag",
        )
        self._p = self._initial_covariance()
        self._t_state = t_state
        self._mag_r_cov = None
        self._last_update_reports = {
            "gyro": None,
            "accel": None,
            "mag": None,
        }

    def _initial_covariance(self) -> list[float]:
        dim: int = self._layout.dim
        p: list[float] = [0.0] * (dim * dim)
        # Initial covariance diagonal in nominal state units
        prior_var: float = 1.0e-3
        for i in range(dim):
            p[i * dim + i] = prior_var
        return p

    def _propagate_to(self, t_target: AhrsTime) -> None:
        if self._x is None or self._p is None or self._t_state is None:
            self._reset_replay_state(t_target)
            return

        dt_ns: int = self._time_to_ns(t_target) - self._time_to_ns(self._t_state)
        # 1e-9 converts nanoseconds to seconds for propagation
        dt_sec: float = float(dt_ns) * 1.0e-9
        if dt_sec < 0.0:
            # Negative dt implies a replay ordering error
            raise ValueError("dt_sec must be non-negative")

        if dt_sec > 0.0:
            self._x, self._p = predict_step(
                self._layout, self._x, self._p, dt_sec, self._config
            )

        self._t_state = t_target
