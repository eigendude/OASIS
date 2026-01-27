################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from collections import deque
from dataclasses import replace
from typing import Deque
from typing import Dict
from typing import Sequence

import message_filters
import numpy as np
import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
import tf2_ros
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Quaternion as QuaternionMsg
from geometry_msgs.msg import TransformStamped as TransformStampedMsg
from sensor_msgs.msg import Imu as ImuMsg
from sensor_msgs.msg import MagneticField as MagneticFieldMsg

from oasis_control.localization.mounting.config.mounting_config import MountingConfig
from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.math_utils.quat import Quaternion
from oasis_control.localization.mounting.math_utils.validation import (
    normalize_quaternion_wxyz,
)
from oasis_control.localization.mounting.math_utils.validation import (
    reshape_covariance as _reshape_covariance,
)
from oasis_control.localization.mounting.math_utils.validation import (
    reshape_matrix as _reshape_matrix,
)
from oasis_control.localization.mounting.mounting_types import ImuCalibrationPrior
from oasis_control.localization.mounting.mounting_types import ImuPacket
from oasis_control.localization.mounting.mounting_types import MagPacket
from oasis_control.localization.mounting.mounting_types import ResultSnapshot
from oasis_control.localization.mounting.pipeline.mounting_pipeline import (
    MountingPipeline,
)
from oasis_control.localization.mounting.pipeline.mounting_pipeline import (
    MountingPipelineError,
)
from oasis_control.localization.mounting.pipeline.mounting_pipeline import (
    PipelineOutputs,
)
from oasis_control.localization.mounting.storage.yaml_format import FlagsYaml
from oasis_control.localization.mounting.tf.tf_publisher import PublishedTransform
from oasis_msgs.msg import AhrsMount as AhrsMountMsg
from oasis_msgs.msg import AhrsMountDiagnostics as AhrsMountDiagnosticsMsg
from oasis_msgs.msg import ImuCalibration as ImuCalibrationMsg


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "ahrs_mounting"

# ROS topics
AHRS_MOUNT_TOPIC: str = "ahrs_mount"
AHRS_MOUNT_DIAGNOSTICS_TOPIC: str = "ahrs_mount_diagnostics"
AHRS_MOUNT_TRANSFORM_TOPIC: str = "ahrs_mount_transform"
IMU_CAL_TOPIC: str = "imu_calibration"
IMU_RAW_TOPIC: str = "imu_raw"
MAG_TOPIC: str = "magnetic_field"

# Default base frame identifier
DEFAULT_BASE_FRAME: str = "base_link"

# Default world frame identifier
DEFAULT_WORLD_FRAME: str = "world"

# Default bootstrap duration in seconds
DEFAULT_BOOTSTRAP_SEC: float = 5.0

# Default persistence save period in seconds
DEFAULT_SAVE_PERIOD_SEC: float = 2.0

# Pending IMU queue horizon in seconds
PENDING_DROP_HORIZON_SEC: float = 2.0

################################################################################
# ROS node
################################################################################


class AhrsMountingNode(rclpy.node.Node):
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
        self._ahrs_mount_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=AhrsMountMsg,
            topic=AHRS_MOUNT_TOPIC,
            qos_profile=qos_profile,
        )
        self._ahrs_mount_diag_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=AhrsMountDiagnosticsMsg,
            topic=AHRS_MOUNT_DIAGNOSTICS_TOPIC,
            qos_profile=qos_profile,
        )
        self._ahrs_mount_tf_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=TransformStampedMsg,
            topic=AHRS_MOUNT_TRANSFORM_TOPIC,
            qos_profile=qos_profile,
        )
        self._tf_broadcaster: tf2_ros.TransformBroadcaster = (
            tf2_ros.TransformBroadcaster(self)
        )
        self._tf_static_broadcaster: tf2_ros.StaticTransformBroadcaster = (
            tf2_ros.StaticTransformBroadcaster(self)
        )

        # AHRS filter
        params: MountingParams = MountingParams.defaults()
        params = params.replace(
            frames=replace(
                params.frames,
                base_frame=DEFAULT_BASE_FRAME,
                world_frame=DEFAULT_WORLD_FRAME,
            ),
            bootstrap=replace(
                params.bootstrap,
                bootstrap_sec=DEFAULT_BOOTSTRAP_SEC,
            ),
            save=replace(
                params.save,
                save_period_sec=DEFAULT_SAVE_PERIOD_SEC,
            ),
        )
        self._params: MountingParams = params
        self._config: MountingConfig = MountingConfig(params)
        self._pipeline: MountingPipeline = MountingPipeline(config=self._config)

        if not self._params.save.output_path:
            self.get_logger().info("Persistence disabled for AHRS mounting calibration")

        # ROS Subscribers
        self._imu_cal_filter_sub: message_filters.Subscriber = (
            message_filters.Subscriber(
                self,
                ImuCalibrationMsg,
                IMU_CAL_TOPIC,
                qos_profile=qos_profile,
            )
        )
        self._imu_raw_filter_sub: message_filters.Subscriber = (
            message_filters.Subscriber(
                self,
                ImuMsg,
                IMU_RAW_TOPIC,
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
        self._imu_raw_filter_sub.registerCallback(self._record_imu_raw)
        self._imu_cal_filter_sub.registerCallback(self._record_imu_calibration)

        # Pending IMU messages
        self._pending_raw_stamps: Deque[int] = deque()
        self._pending_raw_counts: Dict[int, int] = {}
        self._pending_cal_stamps: Deque[int] = deque()
        self._pending_cal_counts: Dict[int, int] = {}
        self._last_seen_time_ns: int | None = None

        # Statistics
        self._imu_raw_dropped_no_calibration: int = 0
        self._mag_dropped_bad_cov: int = 0
        self._time_sync_slop_max_ns: int = 0

        self.get_logger().info("AHRS mounting node initialized")

    def stop(self) -> None:
        self.get_logger().info("AHRS mounting node deinitialized")

        self.destroy_node()

    def _record_imu_raw(self, message: ImuMsg) -> None:
        t_ns: int = _time_to_ns(message.header.stamp)

        self._add_pending_stamp(
            self._pending_raw_stamps,
            self._pending_raw_counts,
            t_ns,
        )
        self._update_last_seen_time(t_ns)
        self._prune_pending_by_time()

    def _record_imu_calibration(self, message: ImuCalibrationMsg) -> None:
        t_ns: int = _time_to_ns(message.header.stamp)

        self._add_pending_stamp(
            self._pending_cal_stamps,
            self._pending_cal_counts,
            t_ns,
        )
        self._update_last_seen_time(t_ns)
        self._prune_pending_by_time()

    def _add_pending_stamp(
        self,
        stamps: Deque[int],
        counts: Dict[int, int],
        t_ns: int,
    ) -> None:
        counts[t_ns] = counts.get(t_ns, 0) + 1
        stamps.append(t_ns)

    def _update_last_seen_time(self, t_ns: int) -> None:
        if self._last_seen_time_ns is None or t_ns > self._last_seen_time_ns:
            self._last_seen_time_ns = t_ns

    def _prune_pending_by_time(self) -> None:
        if self._last_seen_time_ns is None:
            return

        horizon_ns: int = int(PENDING_DROP_HORIZON_SEC * 1e9)
        cutoff_ns: int = self._last_seen_time_ns - horizon_ns
        if cutoff_ns <= 0:
            return

        self._drop_pending_before(
            self._pending_raw_stamps,
            self._pending_raw_counts,
            cutoff_ns,
            count_raw_drops=True,
        )
        self._drop_pending_before(
            self._pending_cal_stamps,
            self._pending_cal_counts,
            cutoff_ns,
            count_raw_drops=False,
        )

    def _drop_pending_before(
        self,
        stamps: Deque[int],
        counts: Dict[int, int],
        cutoff_ns: int,
        *,
        count_raw_drops: bool,
    ) -> None:
        while stamps and stamps[0] < cutoff_ns:
            t_ns: int = stamps.popleft()
            if t_ns in counts:
                counts[t_ns] -= 1
                if counts[t_ns] <= 0:
                    counts.pop(t_ns, None)
                if count_raw_drops:
                    self._imu_raw_dropped_no_calibration += 1

        self._prune_pending(stamps, counts)

    def _prune_pending(self, stamps: Deque[int], counts: Dict[int, int]) -> None:
        while stamps and stamps[0] not in counts:
            stamps.popleft()

    def _consume_pending_stamp(
        self,
        stamps: Deque[int],
        counts: Dict[int, int],
        t_ns: int,
    ) -> None:
        if t_ns in counts:
            counts[t_ns] -= 1
            if counts[t_ns] <= 0:
                counts.pop(t_ns, None)

        self._prune_pending(stamps, counts)

    def _handle_imu_raw_with_calibration(
        self, imu_msg: ImuMsg, cal_msg: ImuCalibrationMsg
    ) -> None:
        t_imu_ns: int = _time_to_ns(imu_msg.header.stamp)
        t_cal_ns: int = _time_to_ns(cal_msg.header.stamp)
        self._update_last_seen_time(max(t_imu_ns, t_cal_ns))

        slop_ns: int = abs(t_imu_ns - t_cal_ns)
        if slop_ns > self._time_sync_slop_max_ns:
            self._time_sync_slop_max_ns = slop_ns

        self._consume_pending_stamp(
            self._pending_raw_stamps,
            self._pending_raw_counts,
            t_imu_ns,
        )
        self._consume_pending_stamp(
            self._pending_cal_stamps,
            self._pending_cal_counts,
            t_cal_ns,
        )
        self._prune_pending_by_time()

        imu_frame: str = imu_msg.header.frame_id
        cal_frame: str = cal_msg.header.frame_id

        if not cal_msg.valid:
            self._imu_raw_dropped_no_calibration += 1
            return
        if not imu_frame or not cal_frame or cal_frame != imu_frame:
            self._imu_raw_dropped_no_calibration += 1
            return

        try:
            imu_packet: ImuPacket = _build_imu_packet(imu_msg, cal_msg)
        except ValueError as exc:
            self.get_logger().warn(f"Dropping IMU pair: {exc}")
            self._imu_raw_dropped_no_calibration += 1
            return

        try:
            self._pipeline.ingest_imu_pair(imu_packet=imu_packet)
            outputs: PipelineOutputs = self._pipeline.step(t_now_ns=t_imu_ns)
        except MountingPipelineError as exc:
            self.get_logger().warn(f"Mounting pipeline error: {exc}")
            return

        self._publish_outputs(outputs)

    def _handle_mag(self, message: MagneticFieldMsg) -> None:
        if not message.header.frame_id:
            self._mag_dropped_bad_cov += 1
            return

        try:
            mag_packet: MagPacket = _build_mag_packet(message)
        except ValueError as exc:
            self.get_logger().warn(f"Dropping magnetometer sample: {exc}")
            self._mag_dropped_bad_cov += 1
            return

        if mag_packet.magnitude_T() < self._params.mag.s_min_T:
            self._mag_dropped_bad_cov += 1
            return

        try:
            self._pipeline.ingest_mag(mag_packet=mag_packet)
        except MountingPipelineError as exc:
            self.get_logger().warn(f"Mounting pipeline error: {exc}")

    def _publish_outputs(self, outputs: PipelineOutputs) -> None:
        snapshot: ResultSnapshot | None = outputs.snapshot
        t_ns: int = outputs.t_ns
        flags: FlagsYaml | None = self._pipeline.current_flags()
        anchored: bool = bool(flags.anchored) if flags is not None else False
        mag_reference_invalid: bool = (
            bool(flags.mag_reference_invalid) if flags is not None else True
        )
        mag_disturbance_detected: bool = (
            bool(flags.mag_disturbance_detected) if flags is not None else False
        )
        mag_dir_prior_from_driver_cov: bool = (
            bool(flags.mag_dir_prior_from_driver_cov) if flags is not None else False
        )

        # RMS is shared across accel and mag factors when reported
        shared_residual_rms: float = float(outputs.report.residual_rms or 0.0)
        if snapshot is not None:
            mount_msg: AhrsMountMsg = AhrsMountMsg()
            mount_msg.header.stamp = _ns_to_time_msg(t_ns)
            mount_msg.base_frame = snapshot.frame_base
            mount_msg.imu_frame = snapshot.frame_imu
            mount_msg.mag_frame = snapshot.frame_mag or ""
            mount_msg.anchored = anchored
            mount_msg.mag_reference_invalid = mag_reference_invalid
            mount_msg.mag_disturbance_detected = mag_disturbance_detected
            mount_msg.mag_dir_prior_from_driver_cov = mag_dir_prior_from_driver_cov

            q_bi_wxyz: Sequence[float] = (
                Quaternion.from_matrix(snapshot.R_BI).to_wxyz().tolist()
            )

            if snapshot.R_BM is not None:
                q_bm_wxyz: Sequence[float] = (
                    Quaternion.from_matrix(snapshot.R_BM).to_wxyz().tolist()
                )
            else:
                q_bm_wxyz = [1.0, 0.0, 0.0, 0.0]

            mount_msg.q_bi_wxyz = list(q_bi_wxyz)
            mount_msg.q_bm_wxyz = list(q_bm_wxyz)

            cov_r_bi: Sequence[float] = snapshot.cov_rot_BI.reshape(-1).tolist()
            cov_r_bm: Sequence[float] = (
                snapshot.cov_rot_BM.reshape(-1).tolist()
                if snapshot.cov_rot_BM is not None
                else [0.0] * 9
            )
            mount_msg.cov_r_bi_row_major_3x3 = list(cov_r_bi)
            mount_msg.cov_r_bm_row_major_3x3 = list(cov_r_bm)

            mount_msg.accel_bias_mps2 = snapshot.b_a_mps2.tolist()
            mount_msg.accel_a_row_major_3x3 = snapshot.A_a.reshape(-1).tolist()
            mount_msg.accel_param_cov_row_major_12x12 = [0.0] * 144
            mount_msg.gyro_bias_rads = snapshot.b_g_rads.tolist()
            mount_msg.gyro_bias_cov_row_major_3x3 = [0.0] * 9

            if snapshot.b_m_T is not None:
                mount_msg.mag_offset_t = snapshot.b_m_T.tolist()
            else:
                mount_msg.mag_offset_t = [0.0, 0.0, 0.0]

            mount_msg.mag_offset_cov_row_major_3x3 = [0.0] * 9

            if snapshot.R_m is not None:
                mount_msg.mag_r_m_unitless2_row_major_3x3 = snapshot.R_m.reshape(
                    -1
                ).tolist()
            else:
                mount_msg.mag_r_m_unitless2_row_major_3x3 = [0.0] * 9

            mount_msg.mag_r_m0_unitless2_row_major_3x3 = (
                np.diag(self._params.mag.Rm_init_diag).reshape(-1).tolist()
            )

            mount_msg.raw_samples = int(self._pipeline.raw_sample_count())
            mount_msg.steady_segments = int(snapshot.segment_count)
            mount_msg.keyframes = int(snapshot.keyframe_count)
            mount_msg.accel_residual_rms = shared_residual_rms
            mount_msg.mag_residual_rms = (
                shared_residual_rms if snapshot.R_BM is not None else 0.0
            )
            mount_msg.gravity_max_angle_deg = float(snapshot.diversity_tilt_deg or 0.0)
            mount_msg.mag_proj_max_angle_deg = float(snapshot.diversity_yaw_deg or 0.0)

            self._ahrs_mount_pub.publish(mount_msg)

            transform_msg: TransformStampedMsg = TransformStampedMsg()
            transform_msg.header.stamp = _ns_to_time_msg(t_ns)
            transform_msg.header.frame_id = snapshot.frame_base
            transform_msg.child_frame_id = snapshot.frame_imu
            transform_msg.transform.translation.x = 0.0
            transform_msg.transform.translation.y = 0.0
            transform_msg.transform.translation.z = 0.0
            transform_msg.transform.rotation = _quat_wxyz_to_msg(q_bi_wxyz)

            self._ahrs_mount_tf_pub.publish(transform_msg)

        published_transforms: Sequence[PublishedTransform] = (
            outputs.published_transforms
        )

        for published in published_transforms:
            tf_msg: TransformStampedMsg = _published_transform_to_msg(published)

            if published.is_static:
                self._tf_static_broadcaster.sendTransform(tf_msg)
            else:
                self._tf_broadcaster.sendTransform(tf_msg)

        diag_msg: AhrsMountDiagnosticsMsg = AhrsMountDiagnosticsMsg()
        diag_msg.header.stamp = _ns_to_time_msg(t_ns)
        diag_msg.imu_raw_dropped_no_calibration = int(
            self._imu_raw_dropped_no_calibration
        )
        diag_msg.mag_dropped_bad_cov = int(self._mag_dropped_bad_cov)
        diag_msg.time_sync_slop_max_ns = int(self._time_sync_slop_max_ns)
        diag_msg.bootstrap_active = bool(self._pipeline.is_bootstrapping())
        diag_msg.bootstrap_remaining_sec = self._pipeline.bootstrap_remaining_sec(t_ns)
        diag_msg.anchored = anchored
        diag_msg.steady_detected = bool(self._pipeline.steady_window_is_steady())
        diag_msg.steady_segments = int(self._pipeline.segment_count())
        diag_msg.keyframes = int(self._pipeline.keyframe_count())
        diag_msg.need_more_tilt = bool(outputs.report.need_more_tilt)
        diag_msg.need_more_yaw = bool(outputs.report.need_more_yaw)
        diag_msg.mag_reference_invalid = mag_reference_invalid
        diag_msg.mag_disturbance_detected = mag_disturbance_detected
        if snapshot is not None and snapshot.R_m is not None:
            diag_msg.mag_r_m_trace = float(np.trace(snapshot.R_m))
        else:
            diag_msg.mag_r_m_trace = 0.0
        diag_msg.mag_factors_dropped = 0
        diag_msg.optimizer_iterations_last = int(outputs.report.iterations)
        diag_msg.optimizer_step_norm_last = float(outputs.report.step_norm or 0.0)
        diag_msg.accel_residual_rms = shared_residual_rms
        diag_msg.mag_residual_rms = (
            shared_residual_rms if snapshot and snapshot.R_BM is not None else 0.0
        )
        last_save_ns: int = int(self._pipeline.last_save_time_ns() or 0)
        diag_msg.last_save_unix_ns = last_save_ns
        diag_msg.save_period_sec = float(self._params.save.save_period_sec)
        diag_msg.save_fail_count = int(self._pipeline.save_fail_count())

        self._ahrs_mount_diag_pub.publish(diag_msg)


def _time_to_ns(stamp: TimeMsg) -> int:
    sec_ns: int = int(stamp.sec) * int(1e9)

    return sec_ns + int(stamp.nanosec)


def _ns_to_time_msg(t_ns: int) -> TimeMsg:
    stamp: TimeMsg = TimeMsg()

    stamp.sec = int(t_ns // int(1e9))
    stamp.nanosec = int(t_ns % int(1e9))

    return stamp


def _quat_wxyz_to_msg(wxyz: Sequence[float]) -> QuaternionMsg:
    array: np.ndarray = normalize_quaternion_wxyz(wxyz)

    quat: QuaternionMsg = QuaternionMsg()
    quat.w = float(array[0])
    quat.x = float(array[1])
    quat.y = float(array[2])
    quat.z = float(array[3])

    return quat


def _build_imu_packet(imu_msg: ImuMsg, cal_msg: ImuCalibrationMsg) -> ImuPacket:
    imu_frame: str = imu_msg.header.frame_id
    t_ns: int = _time_to_ns(imu_msg.header.stamp)

    omega_raw: np.ndarray = np.array(
        [
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z,
        ],
        dtype=np.float64,
    )
    accel_raw: np.ndarray = np.array(
        [
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z,
        ],
        dtype=np.float64,
    )

    cov_omega_raw: np.ndarray = _reshape_covariance(
        imu_msg.angular_velocity_covariance, (3, 3), "angular_velocity_covariance"
    )
    cov_accel_raw: np.ndarray = _reshape_covariance(
        imu_msg.linear_acceleration_covariance,
        (3, 3),
        "linear_acceleration_covariance",
    )

    b_a_mps2: np.ndarray = np.array(
        [
            cal_msg.accel_bias.x,
            cal_msg.accel_bias.y,
            cal_msg.accel_bias.z,
        ],
        dtype=np.float64,
    )
    A_a: np.ndarray = _reshape_matrix(cal_msg.accel_a, (3, 3), "accel_a")
    b_g_rads: np.ndarray = np.array(
        [
            cal_msg.gyro_bias.x,
            cal_msg.gyro_bias.y,
            cal_msg.gyro_bias.z,
        ],
        dtype=np.float64,
    )

    cov_a_params: np.ndarray = _reshape_covariance(
        cal_msg.accel_param_cov,
        (12, 12),
        "accel_param_cov",
    )
    cov_b_g: np.ndarray = _reshape_covariance(
        cal_msg.gyro_bias_cov,
        (3, 3),
        "gyro_bias_cov",
    )

    calibration: ImuCalibrationPrior = ImuCalibrationPrior(
        valid=bool(cal_msg.valid),
        frame_id=imu_frame,
        b_a_mps2=b_a_mps2,
        A_a=A_a,
        b_g_rads=b_g_rads,
        cov_a_params=cov_a_params,
        cov_b_g=cov_b_g,
    )

    return ImuPacket(
        t_meas_ns=t_ns,
        frame_id=imu_frame,
        omega_raw_rads=omega_raw,
        cov_omega_raw=cov_omega_raw,
        a_raw_mps2=accel_raw,
        cov_a_raw=cov_accel_raw,
        calibration=calibration,
    )


def _build_mag_packet(message: MagneticFieldMsg) -> MagPacket:
    t_ns: int = _time_to_ns(message.header.stamp)
    m_raw: np.ndarray = np.array(
        [
            message.magnetic_field.x,
            message.magnetic_field.y,
            message.magnetic_field.z,
        ],
        dtype=np.float64,
    )
    cov_m_raw: np.ndarray = _reshape_covariance(
        message.magnetic_field_covariance,
        (3, 3),
        "magnetic_field_covariance",
    )

    return MagPacket(
        t_meas_ns=t_ns,
        frame_id=message.header.frame_id,
        m_raw_T=m_raw,
        cov_m_raw_T2=cov_m_raw,
    )


def _published_transform_to_msg(transform: PublishedTransform) -> TransformStampedMsg:
    tf_msg: TransformStampedMsg = TransformStampedMsg()

    tf_msg.header.stamp = _ns_to_time_msg(transform.t_ns)
    tf_msg.header.frame_id = transform.parent_frame
    tf_msg.child_frame_id = transform.child_frame
    tf_msg.transform.translation.x = 0.0
    tf_msg.transform.translation.y = 0.0
    tf_msg.transform.translation.z = 0.0
    tf_msg.transform.rotation = _quat_wxyz_to_msg(transform.quaternion_wxyz.tolist())

    return tf_msg
