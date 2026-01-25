################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import math
from typing import Sequence

import message_filters
import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Quaternion as QuaternionMsg
from geometry_msgs.msg import Vector3 as Vector3Msg
from sensor_msgs.msg import Imu as ImuMsg
from sensor_msgs.msg import MagneticField as MagneticFieldMsg

from oasis_control.localization.ahrs.ahrs_types.imu_packet import ImuPacket
from oasis_control.localization.ahrs.ahrs_types.mag_packet import MagPacket
from oasis_control.localization.ahrs.config.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.config.ahrs_params import AhrsParams
from oasis_control.localization.ahrs.filter.ekf import AhrsEkf
from oasis_control.localization.ahrs.filter.update_step import UpdateReport
from oasis_control.localization.ahrs.math_utils.quat import Quaternion as MathQuaternion
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.covariance import AhrsCovariance
from oasis_control.localization.ahrs.state.state_mapping import StateMapping
from oasis_control.localization.ahrs.timing.replay_engine import ReplayEngine
from oasis_control.localization.ahrs.timing.time_base import TimeBase
from oasis_control.localization.ahrs.timing.timeline_node import TimelineNode
from oasis_msgs.msg import AhrsDiagnostics as AhrsDiagnosticsMsg
from oasis_msgs.msg import AhrsState as AhrsStateMsg
from oasis_msgs.msg import EkfUpdateReport as EkfUpdateReportMsg
from oasis_msgs.msg import ImuCalibration as ImuCalibrationMsg
from oasis_msgs.msg import Matrix as MatrixMsg


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "ahrs"

# ROS topics
IMU_RAW_TOPIC: str = "imu_raw"
IMU_CAL_TOPIC: str = "imu_calibration"
MAG_TOPIC: str = "magnetic_field"

ACCEL_UPDATE_TOPIC: str = "ahrs/updates/accel"
GYRO_UPDATE_TOPIC: str = "ahrs/updates/gyro"
MAG_UPDATE_TOPIC: str = "ahrs/updates/mag"
STATE_TOPIC: str = "ahrs/state"
DIAGNOSTICS_TOPIC: str = "ahrs/diagnostics"

EXTRINSICS_T_BI_TOPIC: str = "ahrs/extrinsics/t_bi"
EXTRINSICS_T_BM_TOPIC: str = "ahrs/extrinsics/t_bm"


################################################################################
# ROS node
################################################################################


class AhrsNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources
        """

        super().__init__(NODE_NAME)

        # QoS profile
        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        # ROS Publishers
        self._state_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=AhrsStateMsg,
            topic=STATE_TOPIC,
            qos_profile=qos_profile,
        )
        self._diagnostics_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=AhrsDiagnosticsMsg,
            topic=DIAGNOSTICS_TOPIC,
            qos_profile=qos_profile,
        )
        self._gyro_update_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=EkfUpdateReportMsg,
            topic=GYRO_UPDATE_TOPIC,
            qos_profile=qos_profile,
        )
        self._accel_update_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=EkfUpdateReportMsg,
            topic=ACCEL_UPDATE_TOPIC,
            qos_profile=qos_profile,
        )
        self._mag_update_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=EkfUpdateReportMsg,
            topic=MAG_UPDATE_TOPIC,
            qos_profile=qos_profile,
        )

        # AHRS filter
        self._params: AhrsParams = AhrsParams.defaults()
        self._config: AhrsConfig = AhrsConfig.from_params(self._params)
        self._ekf: AhrsEkf = AhrsEkf(config=self._config)
        self._replay_engine: ReplayEngine = ReplayEngine(
            t_buffer_ns=self._config.t_buffer_ns,
            ekf=self._ekf,
            publish_callback=self._publish_frontier,
        )

        self._state_seq: int = 0
        self._diag_seq: int = 0
        self._update_seq: int = 0
        self._dropped_missing_stamp: int = 0
        self._dropped_future_stamp: int = 0
        self._dropped_nan_cov: int = 0
        self._dropped_imu_coverage_gap: int = 0
        self._dropped_clock_jump_reset: int = 0
        self._last_reset_reason: str = ""
        self._last_replay_happened: bool = False
        self._calibration_prior_applied: bool = False

        # ROS Subscribers
        self._imu_raw_filter_sub: message_filters.Subscriber = (
            message_filters.Subscriber(
                self,
                ImuMsg,
                IMU_RAW_TOPIC,
                qos_profile=qos_profile,
            )
        )
        self._imu_cal_filter_sub: message_filters.Subscriber = (
            message_filters.Subscriber(
                self,
                ImuCalibrationMsg,
                IMU_CAL_TOPIC,
                qos_profile=qos_profile,
            )
        )
        self._mag_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=MagneticFieldMsg,
            topic=MAG_TOPIC,
            callback=self._handle_mag,
            qos_profile=qos_profile,
        )

        # ROS message synchronizers
        self._imu_sync: message_filters.TimeSynchronizer = (
            message_filters.TimeSynchronizer(
                [self._imu_raw_filter_sub, self._imu_cal_filter_sub],
                queue_size=20,
            )
        )
        self._imu_sync.registerCallback(self._handle_imu_raw_with_calibration)

        self.get_logger().info("AHRS node initialized")

    def stop(self) -> None:
        self.get_logger().info("AHRS node deinitialized")

        self.destroy_node()

    def _handle_imu_raw_with_calibration(
        self, imu_msg: ImuMsg, cal_msg: ImuCalibrationMsg
    ) -> None:
        if (
            imu_msg.header.stamp.sec != cal_msg.header.stamp.sec
            or imu_msg.header.stamp.nanosec != cal_msg.header.stamp.nanosec
        ):
            self.get_logger().warning(
                "IMU sample and calibration have mismatched timestamps"
            )
            return

        t_meas_ns: int
        try:
            t_meas_ns = TimeBase.stamp_to_ns(
                imu_msg.header.stamp.sec,
                imu_msg.header.stamp.nanosec,
            )
        except ValueError:
            self._dropped_missing_stamp += 1
            self.get_logger().warning("IMU sample has invalid timestamp")
            return
        if self._is_future_stamp(t_meas_ns):
            self._dropped_future_stamp += 1
            self.get_logger().warning("IMU sample too far in the future")
            return

        frame_id: str = imu_msg.header.frame_id
        z_omega: list[float] = [
            float(imu_msg.angular_velocity.x),
            float(imu_msg.angular_velocity.y),
            float(imu_msg.angular_velocity.z),
        ]
        z_accel: list[float] = [
            float(imu_msg.linear_acceleration.x),
            float(imu_msg.linear_acceleration.y),
            float(imu_msg.linear_acceleration.z),
        ]
        R_omega: list[list[float]] = _reshape_row_major(
            imu_msg.angular_velocity_covariance,
            3,
        )
        R_accel: list[list[float]] = _reshape_row_major(
            imu_msg.linear_acceleration_covariance,
            3,
        )
        if not _is_matrix_finite(R_omega) or not _is_matrix_finite(R_accel):
            self._dropped_nan_cov += 1
            self.get_logger().warning(
                "IMU packet rejected: covariance matrix has NaN or Inf"
            )
            return
        calibration_prior: dict[str, object] = {
            "valid": bool(cal_msg.valid),
            "gyro_bias_rads": [
                float(cal_msg.gyro_bias.x),
                float(cal_msg.gyro_bias.y),
                float(cal_msg.gyro_bias.z),
            ],
            "gyro_bias_cov_row_major": list(cal_msg.gyro_bias_cov),
            "accel_bias_mps2": [
                float(cal_msg.accel_bias.x),
                float(cal_msg.accel_bias.y),
                float(cal_msg.accel_bias.z),
            ],
            "accel_A_row_major": list(cal_msg.accel_a),
            "accel_param_cov_row_major_12x12": list(cal_msg.accel_param_cov),
        }
        calibration_meta: dict[str, object] = {
            "source": "imu_calibration",
            "frame_id": cal_msg.header.frame_id,
            "t_meas_ns": t_meas_ns,
            "gravity_mps2": float(cal_msg.gravity_mps2),
            "fit_sample_count": int(cal_msg.fit_sample_count),
            "rms_residual_mps2": float(cal_msg.rms_residual_mps2),
            "temperature_c": float(cal_msg.temperature_c),
            "temperature_var_c2": float(cal_msg.temperature_var_c2),
        }
        packet: ImuPacket
        try:
            packet = ImuPacket(
                t_meas_ns=t_meas_ns,
                frame_id=frame_id,
                z_omega=z_omega,
                R_omega=R_omega,
                z_accel=z_accel,
                R_accel=R_accel,
                calibration_prior=calibration_prior,
                calibration_meta=calibration_meta,
            )
        except ValueError as exc:
            self._dropped_nan_cov += 1
            self.get_logger().warning(f"IMU packet rejected: {exc}")
            return

        previous_frontier: int = self._replay_engine.frontier_time()
        inserted: bool = self._replay_engine.insert_imu(packet)
        self._last_replay_happened = t_meas_ns < previous_frontier
        if not inserted:
            return
        if calibration_prior["valid"]:
            self._calibration_prior_applied = True

        report_gyro: UpdateReport | None = self._ekf.last_reports.get("gyro")
        if report_gyro is not None:
            self._publish_update_report(
                report=report_gyro,
                t_meas_ns=t_meas_ns,
                frame_id=frame_id,
                sensor=f"{IMU_RAW_TOPIC}/gyro",
                z=z_omega,
                R=R_omega,
                update_index=0,
                publisher=self._gyro_update_pub,
            )
        report_accel: UpdateReport | None = self._ekf.last_reports.get("accel")
        if report_accel is not None:
            self._publish_update_report(
                report=report_accel,
                t_meas_ns=t_meas_ns,
                frame_id=frame_id,
                sensor=f"{IMU_RAW_TOPIC}/accel",
                z=z_accel,
                R=R_accel,
                update_index=1,
                publisher=self._accel_update_pub,
            )

    def _handle_mag(self, message: MagneticFieldMsg) -> None:
        t_meas_ns: int
        try:
            t_meas_ns = TimeBase.stamp_to_ns(
                message.header.stamp.sec,
                message.header.stamp.nanosec,
            )
        except ValueError:
            self._dropped_missing_stamp += 1
            self.get_logger().warning("Mag sample has invalid timestamp")
            return
        if self._is_future_stamp(t_meas_ns):
            self._dropped_future_stamp += 1
            self.get_logger().warning("Mag sample too far in the future")
            return
        frame_id: str = message.header.frame_id
        z_m: list[float] = [
            float(message.magnetic_field.x),
            float(message.magnetic_field.y),
            float(message.magnetic_field.z),
        ]
        R_m_raw: list[list[float]] = _reshape_row_major(
            message.magnetic_field_covariance,
            3,
        )
        if not _is_matrix_finite(R_m_raw):
            self._dropped_nan_cov += 1
            self.get_logger().warning(
                "Mag packet rejected: covariance matrix has NaN or Inf"
            )
            return
        packet: MagPacket
        try:
            packet = MagPacket(
                t_meas_ns=t_meas_ns,
                frame_id=frame_id,
                z_m=z_m,
                R_m_raw=R_m_raw,
            )
            packet.validate()
        except ValueError as exc:
            self._dropped_nan_cov += 1
            self.get_logger().warning(f"Mag packet rejected: {exc}")
            return
        previous_frontier: int = self._replay_engine.frontier_time()
        inserted: bool = self._replay_engine.insert_mag(packet)
        self._last_replay_happened = t_meas_ns < previous_frontier
        if not inserted:
            return
        report_mag: UpdateReport | None = self._ekf.last_reports.get("mag")
        if report_mag is not None:
            self._publish_update_report(
                report=report_mag,
                t_meas_ns=t_meas_ns,
                frame_id=frame_id,
                sensor=MAG_TOPIC,
                z=z_m,
                R=R_m_raw,
                update_index=0,
                publisher=self._mag_update_pub,
            )

    def _publish_frontier(self, t_filter_ns: int) -> None:
        stamp: TimeMsg = _stamp_from_ns(t_filter_ns)
        state: AhrsState = self._ekf.get_state()
        covariance: AhrsCovariance = self._ekf.get_covariance()
        state_msg: AhrsStateMsg = self._make_state_msg(
            stamp=stamp,
            state=state,
            covariance=covariance,
        )
        diag_msg: AhrsDiagnosticsMsg = self._make_diag_msg(
            stamp=stamp,
            t_filter_ns=t_filter_ns,
        )
        self._state_pub.publish(state_msg)
        self._diagnostics_pub.publish(diag_msg)

    def _make_state_msg(
        self,
        *,
        stamp: TimeMsg,
        state: AhrsState,
        covariance: AhrsCovariance,
    ) -> AhrsStateMsg:
        self._state_seq += 1
        msg: AhrsStateMsg = AhrsStateMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = self._params.world_frame
        msg.world_frame_id = self._params.world_frame
        msg.odom_frame_id = self._params.odom_frame
        msg.body_frame_id = self._params.base_frame
        msg.state_seq = self._state_seq
        msg.initialized = bool(
            getattr(
                self._ekf,
                "_calibration_prior_applied",
                self._calibration_prior_applied,
            )
        )
        msg.position_wb_m = _vector3_to_point(state.p_WB)
        msg.velocity_wb_mps = _vector3_to_msg(state.v_WB)
        msg.orientation_wb = _quat_to_msg(state.q_WB)
        msg.gyro_bias = _vector3_to_msg(state.b_g)
        msg.accel_bias = _vector3_to_msg(state.b_a)
        msg.accel_a = _flatten_row_major(state.A_a)
        msg.t_bi = _se3_to_pose(state.T_BI)
        msg.t_bm = _se3_to_pose(state.T_BM)
        msg.gravity_w_mps2 = _vector3_to_msg(state.g_W)
        msg.magnetic_field_w_t = _vector3_to_msg(state.m_W)
        msg.error_dim = StateMapping.dimension()
        msg.error_state_names = _error_state_names()
        P: list[list[float]] = covariance.as_matrix()
        msg.p = _matrix_msg(P)
        return msg

    def _make_diag_msg(
        self,
        *,
        stamp: TimeMsg,
        t_filter_ns: int,
    ) -> AhrsDiagnosticsMsg:
        self._diag_seq += 1
        msg: AhrsDiagnosticsMsg = AhrsDiagnosticsMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = self._params.world_frame
        msg.diag_seq = self._diag_seq
        msg.t_filter_sec = float(t_filter_ns) / 1_000_000_000.0
        buffer_nodes: Sequence[TimelineNode] = (
            self._replay_engine.ring_buffer.nodes_from(0)
        )
        msg.buffer_node_count = self._replay_engine.ring_buffer.size()
        msg.buffer_span_sec = _buffer_span_sec(buffer_nodes)
        msg.replay_happened = self._last_replay_happened
        msg.dropped_missing_stamp = self._dropped_missing_stamp
        msg.dropped_future_stamp = self._dropped_future_stamp
        msg.dropped_too_old = int(self._replay_engine.diagnostics["reject_too_old"])
        msg.dropped_nan_cov = self._dropped_nan_cov
        msg.dropped_imu_coverage_gap = self._dropped_imu_coverage_gap
        msg.dropped_clock_jump_reset = self._dropped_clock_jump_reset
        msg.last_reset_reason = self._last_reset_reason
        return msg

    def _publish_update_report(
        self,
        *,
        report: UpdateReport,
        t_meas_ns: int,
        frame_id: str,
        sensor: str,
        z: Sequence[float],
        R: Sequence[Sequence[float]],
        update_index: int,
        publisher: rclpy.publisher.Publisher,
    ) -> None:
        self._update_seq += 1
        msg: EkfUpdateReportMsg = EkfUpdateReportMsg()
        msg.header.stamp = _stamp_from_ns(t_meas_ns)
        msg.header.frame_id = frame_id
        msg.sensor = sensor
        msg.frame_id = frame_id
        msg.update_seq = self._update_seq
        msg.update_index_in_stamp = update_index
        msg.accepted = report.accepted
        msg.reject_reason = report.reason
        msg.z_dim = len(z)
        msg.z = [float(value) for value in z]
        msg.z_hat = [0.0 for _ in z]
        msg.nu = [0.0 for _ in z]
        msg.r = _matrix_msg(R)
        msg.s_hat = _empty_matrix_msg()
        msg.s = _empty_matrix_msg()
        if report.innovation_mahalanobis2 is None:
            msg.maha_d2 = 0.0
        else:
            msg.maha_d2 = float(report.innovation_mahalanobis2)
        msg.gate_d2_threshold = 0.0
        msg.reproj_rms_px = 0.0
        publisher.publish(msg)

    def _is_future_stamp(self, t_meas_ns: int) -> bool:
        now_ns: int = int(self.get_clock().now().nanoseconds)
        epsilon_ns: int = int(self._config.params.epsilon_wall_future_ns)
        return t_meas_ns - now_ns > epsilon_ns


def _reshape_row_major(values: Sequence[float], size: int) -> list[list[float]]:
    if len(values) != size * size:
        raise ValueError(f"expected {size}x{size} row-major data")
    matrix: list[list[float]] = []
    row: int
    for row in range(size):
        start: int = row * size
        end: int = start + size
        matrix.append([float(value) for value in values[start:end]])
    return matrix


def _flatten_row_major(matrix: Sequence[Sequence[float]]) -> list[float]:
    flattened: list[float] = []
    row: Sequence[float]
    for row in matrix:
        flattened.extend(float(value) for value in row)
    return flattened


def _stamp_from_ns(t_ns: int) -> TimeMsg:
    sec: int = int(t_ns // 1_000_000_000)
    nanosec: int = int(t_ns % 1_000_000_000)
    return TimeMsg(sec=sec, nanosec=nanosec)


def _vector3_to_point(values: Sequence[float]) -> PointMsg:
    return PointMsg(x=float(values[0]), y=float(values[1]), z=float(values[2]))


def _vector3_to_msg(values: Sequence[float]) -> Vector3Msg:
    return Vector3Msg(x=float(values[0]), y=float(values[1]), z=float(values[2]))


def _quat_to_msg(values: Sequence[float]) -> QuaternionMsg:
    return QuaternionMsg(
        x=float(values[1]),
        y=float(values[2]),
        z=float(values[3]),
        w=float(values[0]),
    )


def _se3_to_pose(
    transform: tuple[Sequence[Sequence[float]], Sequence[float]],
) -> PoseMsg:
    rotation: Sequence[Sequence[float]]
    translation: Sequence[float]
    rotation, translation = transform
    quat: list[float] = MathQuaternion.from_matrix(rotation)
    pose: PoseMsg = PoseMsg()
    pose.position = _vector3_to_point(translation)
    pose.orientation = _quat_to_msg(quat)
    return pose


def _matrix_msg(values: Sequence[Sequence[float]]) -> MatrixMsg:
    rows: int = len(values)
    cols: int = 0 if rows == 0 else len(values[0])
    msg: MatrixMsg = MatrixMsg()
    msg.rows = rows
    msg.cols = cols
    msg.data = _flatten_row_major(values)
    return msg


def _empty_matrix_msg() -> MatrixMsg:
    msg: MatrixMsg = MatrixMsg()
    msg.rows = 0
    msg.cols = 0
    msg.data = []
    return msg


def _error_state_names() -> list[str]:
    names: list[str] = [
        "dp.x",
        "dp.y",
        "dp.z",
        "dv.x",
        "dv.y",
        "dv.z",
        "dtheta.x",
        "dtheta.y",
        "dtheta.z",
        "domega.x",
        "domega.y",
        "domega.z",
        "dbg.x",
        "dbg.y",
        "dbg.z",
        "dba.x",
        "dba.y",
        "dba.z",
    ]
    idx: int
    for idx in range(9):
        names.append(f"dAa.{idx}")
    names.extend(
        [
            "dT_BI.rho.x",
            "dT_BI.rho.y",
            "dT_BI.rho.z",
            "dT_BI.theta.x",
            "dT_BI.theta.y",
            "dT_BI.theta.z",
            "dT_BM.rho.x",
            "dT_BM.rho.y",
            "dT_BM.rho.z",
            "dT_BM.theta.x",
            "dT_BM.theta.y",
            "dT_BM.theta.z",
            "dg.x",
            "dg.y",
            "dg.z",
            "dm.x",
            "dm.y",
            "dm.z",
        ]
    )
    if len(names) != StateMapping.dimension():
        raise ValueError("error_state_names length mismatch")
    return names


def _is_matrix_finite(values: Sequence[Sequence[float]]) -> bool:
    row: Sequence[float]
    for row in values:
        value: float
        for value in row:
            if not math.isfinite(float(value)):
                return False
    return True


def _buffer_span_sec(buffer_nodes: Sequence[TimelineNode]) -> float:
    if not buffer_nodes:
        return 0.0
    first_node: object = buffer_nodes[0]
    last_node: object = buffer_nodes[-1]
    first_ns: int = int(getattr(first_node, "t_meas_ns"))
    last_ns: int = int(getattr(last_node, "t_meas_ns"))
    if last_ns <= first_ns:
        return 0.0
    return float(last_ns - first_ns) / 1_000_000_000.0
