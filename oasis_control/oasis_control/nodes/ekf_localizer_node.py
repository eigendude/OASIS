################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import importlib
import importlib.machinery
import importlib.util
import time
from collections import deque
from dataclasses import replace
from types import ModuleType
from typing import TYPE_CHECKING
from typing import Deque
from typing import Optional
from typing import cast

import message_filters
import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from nav_msgs.msg import Odometry as OdometryMsg
from rclpy.clock import ClockType
from sensor_msgs.msg import CameraInfo as CameraInfoMsg
from sensor_msgs.msg import Imu as ImuMsg
from sensor_msgs.msg import MagneticField as MagneticFieldMsg

from oasis_control.localization.ekf.apriltag_pose_selector import (
    AprilTagPoseDetectionCandidate,
)
from oasis_control.localization.ekf.apriltag_pose_selector import (
    select_best_apriltag_candidate,
)
from oasis_control.localization.ekf.ekf_buffer import EkfBuffer
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_types import CameraInfoData
from oasis_control.localization.ekf.ekf_types import EkfAprilTagDetectionUpdate
from oasis_control.localization.ekf.ekf_types import EkfAprilTagUpdateData
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfOutputs
from oasis_control.localization.ekf.ekf_types import EkfPose
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import EkfUpdateData
from oasis_control.localization.ekf.ekf_types import EventAprilTagPose
from oasis_control.localization.ekf.ekf_types import ImuCalibrationData
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import MagSample
from oasis_control.localization.ekf.ekf_types import from_seconds
from oasis_control.localization.ekf.ekf_types import to_ns
from oasis_control.localization.ekf.ekf_types import to_seconds
from oasis_control.localization.ekf.pose_math import is_finite_vector
from oasis_control.localization.ekf.reporting.update_reporter import UpdateReporter
from oasis_control.localization.ekf.ros_time_adapter import ekf_time_to_ros_time
from oasis_control.localization.ekf.ros_time_adapter import ros_time_to_ekf_time
from oasis_control.nodes import ekf_localizer_params as ekf_params
from oasis_msgs.msg import EkfAprilTagUpdateReport as EkfAprilTagUpdateReportMsg
from oasis_msgs.msg import EkfUpdateReport as EkfUpdateReportMsg


if TYPE_CHECKING:
    from apriltag_msgs.msg import AprilTagDetectionArray as AprilTagDetectionArrayMsg
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster

    from oasis_msgs.msg import ImuCalibration as ImuCalibrationMsg
else:
    AprilTagDetectionArrayMsg = object
    ImuCalibrationMsg = object

_HAS_TF2: bool = (
    importlib.util.find_spec("tf2_ros") is not None
    and importlib.util.find_spec("geometry_msgs.msg") is not None
)

if not TYPE_CHECKING and _HAS_TF2:
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
elif not TYPE_CHECKING:
    TransformBroadcaster = object
    TransformStamped = object


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "ekf_localizer"

# ROS topics
IMU_RAW_TOPIC: str = "imu_raw"
IMU_CAL_TOPIC: str = "imu_calibration"
MAG_TOPIC: str = "magnetic_field"
APRILTAG_TOPIC: str = "apriltags"
CAMERA_INFO_TOPIC: str = "camera_info"

ODOM_TOPIC: str = "odom"
WORLD_ODOM_TOPIC: str = "world_odom"
MAG_UPDATE_TOPIC: str = "ekf/updates/mag"
APRILTAG_UPDATE_TOPIC: str = "ekf/updates/apriltags"

# ROS parameters
PARAM_WORLD_FRAME_ID: str = "world_frame_id"
PARAM_ODOM_FRAME_ID: str = "odom_frame_id"
PARAM_BODY_FRAME_ID: str = "body_frame_id"

PARAM_TAG_SIZE_M: str = "tag_size_m"
PARAM_TAG_ANCHOR_FAMILY: str = "tag_anchor_family"
PARAM_TAG_ANCHOR_ID: str = "tag_anchor_id"

DEFAULT_WORLD_FRAME_ID: str = "world"
DEFAULT_ODOM_FRAME_ID: str = "odom"
DEFAULT_BODY_FRAME_ID: str = "base_link"

DEFAULT_TAG_SIZE_M: float = 0.162
DEFAULT_TAG_ANCHOR_FAMILY: str = "tag36h11"
DEFAULT_TAG_ANCHOR_ID: int = 0

SYNCED_IMU_STAMP_CACHE_SIZE: int = 50


################################################################################
# ROS node
################################################################################


class EkfLocalizerNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources
        """

        super().__init__(NODE_NAME)

        self.declare_parameter(PARAM_WORLD_FRAME_ID, DEFAULT_WORLD_FRAME_ID)
        self.declare_parameter(PARAM_ODOM_FRAME_ID, DEFAULT_ODOM_FRAME_ID)
        self.declare_parameter(PARAM_BODY_FRAME_ID, DEFAULT_BODY_FRAME_ID)
        self.declare_parameter(
            ekf_params.PARAM_T_BUFFER_SEC, ekf_params.DEFAULT_T_BUFFER_SEC
        )
        self.declare_parameter(
            ekf_params.PARAM_T_BUFFER_SEC_LEGACY, ekf_params.DEFAULT_T_BUFFER_SEC
        )
        self.declare_parameter(
            ekf_params.PARAM_EPS_WALL_FUTURE, ekf_params.DEFAULT_EPS_WALL_FUTURE
        )
        self.declare_parameter(
            ekf_params.PARAM_EPS_WALL_FUTURE_LEGACY,
            ekf_params.DEFAULT_EPS_WALL_FUTURE,
        )
        self.declare_parameter(
            ekf_params.PARAM_DT_CLOCK_JUMP_MAX, ekf_params.DEFAULT_DT_CLOCK_JUMP_MAX
        )
        self.declare_parameter(
            ekf_params.PARAM_DT_CLOCK_JUMP_MAX_LEGACY,
            ekf_params.DEFAULT_DT_CLOCK_JUMP_MAX,
        )
        self.declare_parameter(
            ekf_params.PARAM_DT_IMU_MAX, ekf_params.DEFAULT_DT_IMU_MAX
        )
        self.declare_parameter(
            ekf_params.PARAM_DT_IMU_MAX_LEGACY, ekf_params.DEFAULT_DT_IMU_MAX
        )
        self.declare_parameter(ekf_params.PARAM_POS_VAR, ekf_params.DEFAULT_POS_VAR)
        self.declare_parameter(ekf_params.PARAM_VEL_VAR, ekf_params.DEFAULT_VEL_VAR)
        self.declare_parameter(ekf_params.PARAM_ANG_VAR, ekf_params.DEFAULT_ANG_VAR)
        self.declare_parameter(
            ekf_params.PARAM_ACCEL_NOISE_VAR, ekf_params.DEFAULT_ACCEL_NOISE_VAR
        )
        self.declare_parameter(
            ekf_params.PARAM_GYRO_NOISE_VAR, ekf_params.DEFAULT_GYRO_NOISE_VAR
        )
        self.declare_parameter(
            ekf_params.PARAM_GRAVITY_MPS2, ekf_params.DEFAULT_GRAVITY_MPS2
        )
        self.declare_parameter(
            ekf_params.PARAM_MAX_DT_SEC, ekf_params.DEFAULT_MAX_DT_SEC
        )
        self.declare_parameter(
            ekf_params.PARAM_CHECKPOINT_INTERVAL_SEC,
            ekf_params.DEFAULT_CHECKPOINT_INTERVAL_SEC,
        )
        self.declare_parameter(
            ekf_params.PARAM_APRILTAG_POS_VAR, ekf_params.DEFAULT_APRILTAG_POS_VAR
        )
        self.declare_parameter(
            ekf_params.PARAM_APRILTAG_YAW_VAR, ekf_params.DEFAULT_APRILTAG_YAW_VAR
        )
        self.declare_parameter(
            ekf_params.PARAM_APRILTAG_POS_STD_M, ekf_params.DEFAULT_APRILTAG_POS_STD_M
        )
        self.declare_parameter(
            ekf_params.PARAM_APRILTAG_ROT_STD_RAD,
            ekf_params.DEFAULT_APRILTAG_ROT_STD_RAD,
        )
        self.declare_parameter(
            ekf_params.PARAM_APRILTAG_GATE_D2, ekf_params.DEFAULT_APRILTAG_GATE_D2
        )
        self.declare_parameter(PARAM_TAG_SIZE_M, DEFAULT_TAG_SIZE_M)
        self.declare_parameter(PARAM_TAG_ANCHOR_FAMILY, DEFAULT_TAG_ANCHOR_FAMILY)
        self.declare_parameter(PARAM_TAG_ANCHOR_ID, DEFAULT_TAG_ANCHOR_ID)
        self.declare_parameter(
            ekf_params.PARAM_TAG_LANDMARK_PRIOR_SIGMA_T_M,
            ekf_params.DEFAULT_TAG_LANDMARK_PRIOR_SIGMA_T_M,
        )
        self.declare_parameter(
            ekf_params.PARAM_TAG_LANDMARK_PRIOR_SIGMA_ROT_RAD,
            ekf_params.DEFAULT_TAG_LANDMARK_PRIOR_SIGMA_ROT_RAD,
        )
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        self._world_frame_id: str = str(self.get_parameter(PARAM_WORLD_FRAME_ID).value)
        self._odom_frame_id: str = str(self.get_parameter(PARAM_ODOM_FRAME_ID).value)
        self._body_frame_id: str = str(self.get_parameter(PARAM_BODY_FRAME_ID).value)
        self._t_buffer_sec: float = float(
            self._get_param_with_legacy(
                ekf_params.PARAM_T_BUFFER_SEC,
                ekf_params.PARAM_T_BUFFER_SEC_LEGACY,
                ekf_params.DEFAULT_T_BUFFER_SEC,
            )
        )
        self._eps_wall_future: float = float(
            self._get_param_with_legacy(
                ekf_params.PARAM_EPS_WALL_FUTURE,
                ekf_params.PARAM_EPS_WALL_FUTURE_LEGACY,
                ekf_params.DEFAULT_EPS_WALL_FUTURE,
            )
        )
        self._dt_clock_jump_max: float = float(
            self._get_param_with_legacy(
                ekf_params.PARAM_DT_CLOCK_JUMP_MAX,
                ekf_params.PARAM_DT_CLOCK_JUMP_MAX_LEGACY,
                ekf_params.DEFAULT_DT_CLOCK_JUMP_MAX,
            )
        )
        self._dt_imu_max: float = float(
            self._get_param_with_legacy(
                ekf_params.PARAM_DT_IMU_MAX,
                ekf_params.PARAM_DT_IMU_MAX_LEGACY,
                ekf_params.DEFAULT_DT_IMU_MAX,
            )
        )
        self._pos_var: float = float(self.get_parameter(ekf_params.PARAM_POS_VAR).value)
        self._vel_var: float = float(self.get_parameter(ekf_params.PARAM_VEL_VAR).value)
        self._ang_var: float = float(self.get_parameter(ekf_params.PARAM_ANG_VAR).value)
        self._accel_noise_var: float = float(
            self.get_parameter(ekf_params.PARAM_ACCEL_NOISE_VAR).value
        )
        self._gyro_noise_var: float = float(
            self.get_parameter(ekf_params.PARAM_GYRO_NOISE_VAR).value
        )
        self._gravity_mps2: float = float(
            self.get_parameter(ekf_params.PARAM_GRAVITY_MPS2).value
        )
        self._max_dt_sec: float = float(
            self.get_parameter(ekf_params.PARAM_MAX_DT_SEC).value
        )
        self._checkpoint_interval_sec: float = float(
            self.get_parameter(ekf_params.PARAM_CHECKPOINT_INTERVAL_SEC).value
        )
        self._apriltag_pos_var: float = float(
            self.get_parameter(ekf_params.PARAM_APRILTAG_POS_VAR).value
        )
        self._apriltag_yaw_var: float = float(
            self.get_parameter(ekf_params.PARAM_APRILTAG_YAW_VAR).value
        )
        self._apriltag_pos_std_m: float = float(
            self.get_parameter(ekf_params.PARAM_APRILTAG_POS_STD_M).value
        )
        self._apriltag_rot_std_rad: float = float(
            self.get_parameter(ekf_params.PARAM_APRILTAG_ROT_STD_RAD).value
        )
        self._apriltag_gate_d2: float = float(
            self.get_parameter(ekf_params.PARAM_APRILTAG_GATE_D2).value
        )
        self._tag_size_m: float = float(self.get_parameter(PARAM_TAG_SIZE_M).value)
        self._tag_anchor_family: str = str(
            self.get_parameter(PARAM_TAG_ANCHOR_FAMILY).value
        )
        self._tag_anchor_id: int = int(self.get_parameter(PARAM_TAG_ANCHOR_ID).value)
        self._tag_landmark_prior_sigma_t_m: float = float(
            self.get_parameter(ekf_params.PARAM_TAG_LANDMARK_PRIOR_SIGMA_T_M).value
        )
        self._tag_landmark_prior_sigma_rot_rad: float = float(
            self.get_parameter(ekf_params.PARAM_TAG_LANDMARK_PRIOR_SIGMA_ROT_RAD).value
        )

        config: EkfConfig = EkfConfig(
            world_frame_id=self._world_frame_id,
            odom_frame_id=self._odom_frame_id,
            body_frame_id=self._body_frame_id,
            t_buffer_sec=self._t_buffer_sec,
            epsilon_wall_future=self._eps_wall_future,
            dt_clock_jump_max=self._dt_clock_jump_max,
            dt_imu_max=self._dt_imu_max,
            pos_var=self._pos_var,
            vel_var=self._vel_var,
            ang_var=self._ang_var,
            accel_noise_var=self._accel_noise_var,
            gyro_noise_var=self._gyro_noise_var,
            gravity_mps2=self._gravity_mps2,
            max_dt_sec=self._max_dt_sec,
            checkpoint_interval_sec=self._checkpoint_interval_sec,
            apriltag_pos_var=self._apriltag_pos_var,
            apriltag_yaw_var=self._apriltag_yaw_var,
            apriltag_gate_d2=self._apriltag_gate_d2,
            tag_size_m=self._tag_size_m,
            tag_anchor_family=self._tag_anchor_family,
            tag_anchor_id=self._tag_anchor_id,
            tag_landmark_prior_sigma_t_m=self._tag_landmark_prior_sigma_t_m,
            tag_landmark_prior_sigma_rot_rad=self._tag_landmark_prior_sigma_rot_rad,
        )

        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        self._odom_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=OdometryMsg,
            topic=ODOM_TOPIC,
            qos_profile=qos_profile,
        )
        self._world_odom_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=OdometryMsg,
            topic=WORLD_ODOM_TOPIC,
            qos_profile=qos_profile,
        )
        self._mag_update_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=EkfUpdateReportMsg,
            topic=MAG_UPDATE_TOPIC,
            qos_profile=qos_profile,
        )
        self._apriltag_update_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=EkfAprilTagUpdateReportMsg,
            topic=APRILTAG_UPDATE_TOPIC,
            qos_profile=qos_profile,
        )
        self._tf_broadcaster: Optional[TransformBroadcaster] = None
        if _HAS_TF2:
            self._tf_broadcaster = TransformBroadcaster(self)

        self._imu_raw_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=ImuMsg,
            topic=IMU_RAW_TOPIC,
            callback=self._handle_imu_raw,
            qos_profile=qos_profile,
        )
        self._mag_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=MagneticFieldMsg,
            topic=MAG_TOPIC,
            callback=self._handle_mag,
            qos_profile=qos_profile,
        )
        self._camera_info_sub: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=CameraInfoMsg,
                topic=CAMERA_INFO_TOPIC,
                callback=self._handle_camera_info,
                qos_profile=qos_profile,
            )
        )

        self._imu_cal_sub: Optional[rclpy.subscription.Subscription] = None
        self._imu_raw_filter_sub: Optional[message_filters.Subscriber] = None
        self._imu_cal_filter_sub: Optional[message_filters.Subscriber] = None
        self._imu_sync: Optional[message_filters.TimeSynchronizer] = None
        self._apriltag_sub: Optional[rclpy.subscription.Subscription] = None

        self._has_imu_calibration: bool = False
        imu_cal_spec: Optional[importlib.machinery.ModuleSpec] = (
            importlib.util.find_spec("oasis_msgs.msg")
        )
        if imu_cal_spec is not None:
            oasis_msgs_module: ModuleType = importlib.import_module("oasis_msgs.msg")
            imu_cal_msg: Optional[type] = getattr(
                oasis_msgs_module, "ImuCalibration", None
            )
            if imu_cal_msg is not None:
                self._has_imu_calibration = True
                self._imu_raw_filter_sub = message_filters.Subscriber(
                    self,
                    ImuMsg,
                    IMU_RAW_TOPIC,
                    qos_profile=qos_profile,
                )
                self._imu_cal_filter_sub = message_filters.Subscriber(
                    self,
                    imu_cal_msg,
                    IMU_CAL_TOPIC,
                    qos_profile=qos_profile,
                )
                self._imu_sync = message_filters.TimeSynchronizer(
                    [self._imu_raw_filter_sub, self._imu_cal_filter_sub],
                    queue_size=20,
                )
                self._imu_sync.registerCallback(self._handle_imu_raw_with_calibration)

                self.get_logger().info(
                    "Using ExactTime sync for imu_raw + imu_calibration"
                )

        if not self._has_imu_calibration:
            self.get_logger().info(
                "ImuCalibration message unavailable, running imu_raw unsynced"
            )

        self._has_apriltag_msgs: bool = False
        apriltag_spec: Optional[importlib.machinery.ModuleSpec] = (
            importlib.util.find_spec("apriltag_msgs.msg")
        )
        if apriltag_spec is not None:
            apriltag_module: ModuleType = importlib.import_module("apriltag_msgs.msg")
            apriltag_msg: Optional[type] = getattr(
                apriltag_module, "AprilTagDetectionArray", None
            )
            if apriltag_msg is not None:
                self._has_apriltag_msgs = True
                self._apriltag_sub = self.create_subscription(
                    msg_type=apriltag_msg,
                    topic=APRILTAG_TOPIC,
                    callback=self._handle_apriltags,
                    qos_profile=qos_profile,
                )

        if not self._has_apriltag_msgs:
            self.get_logger().error(
                "AprilTagDetectionArray message unavailable, skipping apriltags"
            )

        self._buffer: EkfBuffer = EkfBuffer(config)
        self._core: EkfCore = EkfCore(config)
        self._reporter: UpdateReporter = UpdateReporter()

        self._camera_info: Optional[CameraInfoData] = None
        self._update_seq: int = 0
        self._synced_imu_timestamps_ns: Deque[int] = deque()
        self._synced_imu_timestamps_set: set[int] = set()
        self._future_reject_log_times: dict[str, float] = {}

        self.get_logger().info("EKF localizer initialized")

    def stop(self) -> None:
        self.get_logger().info("EKF localizer deinitialized")

        self.destroy_node()

    def _handle_camera_info(self, message: CameraInfoMsg) -> None:
        if message.width == 0 or message.height == 0:
            self.get_logger().warn("CameraInfo message is missing image dimensions")
            return

        data: CameraInfoData = self._camera_info_to_data(message)

        if self._camera_info is None:
            self._camera_info = data
            self.get_logger().info("Caching camera_info for AprilTag updates")
            event: EkfEvent = EkfEvent(
                t_meas=ros_time_to_ekf_time(message.header.stamp),
                event_type=EkfEventType.CAMERA_INFO,
                payload=data,
            )
            self._process_event(event, CAMERA_INFO_TOPIC)
            return

        if not self._camera_info_matches(self._camera_info, data):
            self.get_logger().warn(
                "CameraInfo mismatch detected, ignoring new intrinsics"
            )

    def _handle_imu_raw_with_calibration(
        self, imu_msg: ImuMsg, cal_msg: ImuCalibrationMsg
    ) -> None:
        timestamp: EkfTime = ros_time_to_ekf_time(imu_msg.header.stamp)
        self._record_synced_imu_timestamp(timestamp)

        calibration: Optional[ImuCalibrationData] = self._build_calibration_data(
            cal_msg
        )
        if (
            calibration is not None
            and calibration.frame_id
            and calibration.frame_id != imu_msg.header.frame_id
        ):
            self.get_logger().warn(
                "IMU calibration frame mismatch, ignoring calibration"
            )
            calibration = None

        self._process_imu_message(imu_msg, calibration, timestamp)

    def _handle_imu_raw(self, message: ImuMsg) -> None:
        timestamp: EkfTime = ros_time_to_ekf_time(message.header.stamp)
        if self._is_recent_synced_imu_timestamp(timestamp):
            return

        self._process_imu_message(message, None, timestamp)

    def _process_imu_message(
        self,
        message: ImuMsg,
        calibration: Optional[ImuCalibrationData],
        timestamp: EkfTime,
    ) -> None:
        imu_sample: ImuSample = ImuSample(
            frame_id=message.header.frame_id,
            angular_velocity_rps=[
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ],
            linear_acceleration_mps2=[
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ],
            angular_velocity_cov=list(message.angular_velocity_covariance),
            linear_acceleration_cov=list(message.linear_acceleration_covariance),
        )
        self._handle_imu_packet(imu_sample, calibration, timestamp)

    def _record_synced_imu_timestamp(self, timestamp: EkfTime) -> None:
        if len(self._synced_imu_timestamps_ns) >= SYNCED_IMU_STAMP_CACHE_SIZE:
            oldest: int = self._synced_imu_timestamps_ns.popleft()
            self._synced_imu_timestamps_set.discard(oldest)

        key: int = to_ns(timestamp)
        self._synced_imu_timestamps_ns.append(key)
        self._synced_imu_timestamps_set.add(key)

    def _is_recent_synced_imu_timestamp(self, timestamp: EkfTime) -> bool:
        return to_ns(timestamp) in self._synced_imu_timestamps_set

    def _handle_imu_packet(
        self,
        imu: ImuSample,
        calibration: Optional[ImuCalibrationData],
        timestamp: EkfTime,
    ) -> None:
        event: EkfEvent = EkfEvent(
            t_meas=timestamp,
            event_type=EkfEventType.IMU,
            payload=EkfImuPacket(imu=imu, calibration=calibration),
        )
        self._process_event(event, IMU_RAW_TOPIC)

    def _handle_mag(self, message: MagneticFieldMsg) -> None:
        timestamp: EkfTime = ros_time_to_ekf_time(message.header.stamp)
        mag_sample: MagSample = MagSample(
            frame_id=message.header.frame_id,
            magnetic_field_t=[
                message.magnetic_field.x,
                message.magnetic_field.y,
                message.magnetic_field.z,
            ],
            magnetic_field_cov=list(message.magnetic_field_covariance),
        )
        event: EkfEvent = EkfEvent(
            t_meas=timestamp,
            event_type=EkfEventType.MAG,
            payload=mag_sample,
        )
        self._process_event(event, MAG_TOPIC)

    def _handle_apriltags(self, message: AprilTagDetectionArrayMsg) -> None:
        timestamp: EkfTime = ros_time_to_ekf_time(message.header.stamp)
        candidates: list[AprilTagPoseDetectionCandidate] = (
            self._apriltag_candidates_from_msg(message)
        )
        selected: Optional[AprilTagPoseDetectionCandidate] = (
            select_best_apriltag_candidate(candidates)
        )
        if selected is None:
            self.get_logger().warn("No valid AprilTag poses in message, dropping")
            return

        event_payload: EventAprilTagPose = EventAprilTagPose(
            timestamp_s=to_seconds(timestamp),
            p_meas_world_base_m=selected.p_world_base_m,
            q_meas_world_base_xyzw=selected.q_world_base_xyzw,
            covariance=selected.covariance,
            tag_id=selected.tag_id,
            frame_id=selected.frame_id,
            source_topic=selected.source_topic,
            family=selected.family,
            det_index_in_msg=selected.det_index_in_msg,
        )
        event: EkfEvent = EkfEvent(
            t_meas=timestamp,
            event_type=EkfEventType.APRILTAG,
            payload=event_payload,
        )
        self._process_event(event, APRILTAG_TOPIC)

    def _process_event(self, event: EkfEvent, topic: str) -> None:
        if self._should_reject_future_event(event, topic):
            return

        t_meas_s: float = to_seconds(event.t_meas)
        latest_time: Optional[float] = self._buffer.latest_time()
        if latest_time is not None:
            dt_clock: float = t_meas_s - latest_time
            if abs(dt_clock) > self._dt_clock_jump_max:
                self.get_logger().warn(
                    "EKF clock jump detected: "
                    f"latest={latest_time:.3f}s t_meas={t_meas_s:.3f}s "
                    f"dt={dt_clock:.3f}s"
                )
                self._buffer.reset()
                self._core.reset()

        if self._buffer.too_old(event.t_meas):
            self.get_logger().warn("EKF event too old for buffer, dropping")
            self._publish_rejection(event, "Event too old for buffer")
            return

        self._buffer.insert_event(event)
        self._buffer.evict(event.t_meas)

        if self._core.is_out_of_order(event.t_meas):
            outputs: EkfOutputs = self._core.replay(self._buffer, event.t_meas)
        else:
            outputs = self._core.process_event(event)

        if outputs.odom_time_s is not None:
            self._publish_odom(outputs.odom_time_s)
            self._publish_tf_odom(outputs.odom_time_s)
        if outputs.world_odom_time_s is not None:
            self._publish_world_odom(outputs.world_odom_time_s)
            self._publish_tf_world_odom(outputs.world_odom_time_s)
        if outputs.mag_update is not None:
            self._publish_mag_update(outputs.mag_update)
        if outputs.apriltag_update is not None:
            apriltag_update: EkfAprilTagUpdateData = outputs.apriltag_update
            self._publish_apriltag_update(apriltag_update)
        self._record_warnings(outputs.warnings)

    def _publish_odom(self, timestamp: float) -> None:
        pose: EkfPose = self._core.odom_base_pose()
        message: OdometryMsg = self._build_odom(
            timestamp, frame_id=self._odom_frame_id, pose=pose
        )
        self._odom_pub.publish(message)

    def _publish_world_odom(self, timestamp: float) -> None:
        pose: EkfPose = self._core.world_base_pose()
        message: OdometryMsg = self._build_odom(
            timestamp, frame_id=self._world_frame_id, pose=pose
        )
        self._world_odom_pub.publish(message)

    def _publish_tf_odom(self, timestamp: float) -> None:
        if self._tf_broadcaster is None:
            return
        pose: EkfPose = self._core.odom_base_pose()
        self._publish_tf_transform(
            timestamp,
            parent_frame=self._odom_frame_id,
            child_frame=self._body_frame_id,
            pose=pose,
        )

    def _publish_tf_world_odom(self, timestamp: float) -> None:
        if self._tf_broadcaster is None:
            return
        pose: EkfPose = self._core.world_odom_pose()
        self._publish_tf_transform(
            timestamp,
            parent_frame=self._world_frame_id,
            child_frame=self._odom_frame_id,
            pose=pose,
        )

    def _publish_tf_transform(
        self,
        timestamp: float,
        *,
        parent_frame: str,
        child_frame: str,
        pose: EkfPose,
    ) -> None:
        if self._tf_broadcaster is None:
            return
        message: TransformStamped = TransformStamped()
        message.header.stamp = ekf_time_to_ros_time(from_seconds(timestamp))
        message.header.frame_id = parent_frame
        message.child_frame_id = child_frame
        message.transform.translation.x = pose.position_m[0]
        message.transform.translation.y = pose.position_m[1]
        message.transform.translation.z = pose.position_m[2]
        message.transform.rotation.x = pose.orientation_xyzw[0]
        message.transform.rotation.y = pose.orientation_xyzw[1]
        message.transform.rotation.z = pose.orientation_xyzw[2]
        message.transform.rotation.w = pose.orientation_xyzw[3]
        self._tf_broadcaster.sendTransform(message)

    def _publish_mag_update(self, update: EkfUpdateData) -> None:
        update_seq: int = self._next_update_seq()
        report: EkfUpdateReportMsg = self._reporter.build_update_report(
            update, update_seq, 0
        )
        self._mag_update_pub.publish(report)

    def _publish_apriltag_update(self, update: EkfAprilTagUpdateData) -> None:
        update_seq: int = self._next_update_seq()
        report: EkfAprilTagUpdateReportMsg = self._reporter.build_apriltag_report(
            update, update_seq
        )
        self._apriltag_update_pub.publish(report)

    def _publish_rejection(self, event: EkfEvent, reason: str) -> None:
        if event.event_type == EkfEventType.MAG:
            mag_sample: MagSample = cast(MagSample, event.payload)
            update: EkfUpdateData = self._core.update_with_mag(
                mag_sample, to_seconds(event.t_meas)
            )
            update = replace(update, accepted=False, reject_reason=reason)
            self._publish_mag_update(update)
        elif event.event_type == EkfEventType.APRILTAG:
            apriltag_pose: EventAprilTagPose = cast(EventAprilTagPose, event.payload)
            detection_update: EkfAprilTagDetectionUpdate = (
                self._core.build_rejected_apriltag_pose(
                    apriltag_pose,
                    to_seconds(event.t_meas),
                    reject_reason=reason,
                )
            )
            update_data: EkfAprilTagUpdateData = EkfAprilTagUpdateData(
                t_meas=to_seconds(event.t_meas),
                frame_id=apriltag_pose.frame_id,
                detections=[detection_update],
            )
            self._publish_apriltag_update(update_data)

    def _should_reject_future_event(self, event: EkfEvent, topic: str) -> bool:
        if self._using_sim_time():
            return False

        t_meas_s: float = to_seconds(event.t_meas)
        now_s: float = float(self.get_clock().now().nanoseconds) * 1.0e-9
        if t_meas_s <= now_s + self._eps_wall_future:
            return False

        delta: float = t_meas_s - now_s
        self._throttled_future_warning(topic, t_meas_s, now_s, delta)
        self._publish_rejection(event, "Future timestamp")
        return True

    def _throttled_future_warning(
        self, topic: str, t_meas_s: float, now_s: float, delta: float
    ) -> None:
        last_time: float = self._future_reject_log_times.get(topic, 0.0)
        monotonic_now: float = time.monotonic()
        if monotonic_now - last_time < 1.0:
            return
        self._future_reject_log_times[topic] = monotonic_now
        self.get_logger().warn(
            "Rejecting future-dated event on "
            f"{topic}: t_meas={t_meas_s:.3f}s now={now_s:.3f}s "
            f"delta={delta:.3f}s"
        )

    def _using_sim_time(self) -> bool:
        use_sim_time: bool = bool(self.get_parameter("use_sim_time").value)
        return self.get_clock().clock_type == ClockType.ROS_TIME and use_sim_time

    def _get_param_with_legacy(
        self, param_name: str, legacy_name: str, default: float
    ) -> float:
        current_value: float = float(self.get_parameter(param_name).value)
        legacy_value: float = float(self.get_parameter(legacy_name).value)
        if legacy_value != default and current_value == default:
            self.get_logger().warn(
                f"Parameter '{legacy_name}' is deprecated, " f"use '{param_name}'"
            )
            return legacy_value
        if legacy_value != default and current_value != default:
            self.get_logger().warn(
                f"Parameter '{legacy_name}' is deprecated and ignored in "
                f"favor of '{param_name}'"
            )
        return current_value

    def _record_warnings(self, warnings: list[str]) -> None:
        for warning in warnings:
            self._reporter.record_warning(warning)

    def _build_odom(
        self, timestamp: float, *, frame_id: str, pose: EkfPose
    ) -> OdometryMsg:
        message: OdometryMsg = OdometryMsg()
        message.header.stamp = ekf_time_to_ros_time(from_seconds(timestamp))
        message.header.frame_id = frame_id
        message.child_frame_id = self._body_frame_id
        message.pose.pose.position.x = pose.position_m[0]
        message.pose.pose.position.y = pose.position_m[1]
        message.pose.pose.position.z = pose.position_m[2]
        message.pose.pose.orientation.x = pose.orientation_xyzw[0]
        message.pose.pose.orientation.y = pose.orientation_xyzw[1]
        message.pose.pose.orientation.z = pose.orientation_xyzw[2]
        message.pose.pose.orientation.w = pose.orientation_xyzw[3]
        message.pose.covariance = [0.0] * 36
        message.twist.covariance = [0.0] * 36
        return message

    def _camera_info_to_data(self, message: CameraInfoMsg) -> CameraInfoData:
        return CameraInfoData(
            frame_id=message.header.frame_id,
            width=int(message.width),
            height=int(message.height),
            distortion_model=str(message.distortion_model),
            d=list(message.d),
            k=list(message.k),
            r=list(message.r),
            p=list(message.p),
        )

    def _camera_info_matches(
        self, cached: CameraInfoData, incoming: CameraInfoData
    ) -> bool:
        if cached.frame_id != incoming.frame_id:
            return False
        if cached.width != incoming.width or cached.height != incoming.height:
            return False
        if cached.distortion_model != incoming.distortion_model:
            return False
        if cached.d != incoming.d:
            return False
        if cached.k != incoming.k:
            return False
        if cached.r != incoming.r:
            return False
        if cached.p != incoming.p:
            return False
        return True

    def _apriltag_candidates_from_msg(
        self, message: AprilTagDetectionArrayMsg
    ) -> list[AprilTagPoseDetectionCandidate]:
        candidates: list[AprilTagPoseDetectionCandidate] = []
        for index, detection in enumerate(message.detections):
            pose_data: Optional[tuple[list[float], list[float]]] = (
                self._pose_from_apriltag_detection(detection)
            )
            if pose_data is None:
                continue
            position: list[float]
            quaternion: list[float]
            position, quaternion = pose_data
            if not self._is_valid_pose(position, quaternion):
                self.get_logger().warn(
                    f"Invalid AprilTag pose for tag_id={int(detection.id)}"
                )
                continue
            covariance: Optional[list[float]] = self._covariance_from_apriltag_detection(
                detection
            )
            if covariance is None:
                covariance = self._default_apriltag_covariance()

            decision_margin_field: Optional[object] = getattr(
                detection, "decision_margin", None
            )
            decision_margin: Optional[float] = (
                None
                if decision_margin_field is None
                else float(decision_margin_field)
            )
            pose_error_field: Optional[object] = getattr(detection, "pose_error", None)
            pose_error: Optional[float] = (
                None if pose_error_field is None else float(pose_error_field)
            )

            candidates.append(
                AprilTagPoseDetectionCandidate(
                    family=str(detection.family),
                    tag_id=int(detection.id),
                    det_index_in_msg=int(index),
                    frame_id=str(message.header.frame_id),
                    source_topic=APRILTAG_TOPIC,
                    p_world_base_m=position,
                    q_world_base_xyzw=quaternion,
                    covariance=covariance,
                    decision_margin=decision_margin,
                    pose_error=pose_error,
                )
            )
        return candidates

    def _build_calibration_data(self, message: ImuCalibrationMsg) -> ImuCalibrationData:
        return ImuCalibrationData(
            valid=bool(message.valid),
            frame_id=str(message.header.frame_id),
            accel_bias_mps2=[
                message.accel_bias.x,
                message.accel_bias.y,
                message.accel_bias.z,
            ],
            accel_a=list(message.accel_a),
            accel_param_cov=list(message.accel_param_cov),
            gyro_bias_rps=[
                message.gyro_bias.x,
                message.gyro_bias.y,
                message.gyro_bias.z,
            ],
            gyro_bias_cov=list(message.gyro_bias_cov),
            gravity_mps2=float(message.gravity_mps2),
            fit_sample_count=int(message.fit_sample_count),
            rms_residual_mps2=float(message.rms_residual_mps2),
            temperature_c=float(message.temperature_c),
            temperature_var_c2=float(message.temperature_var_c2),
        )

    def _next_update_seq(self) -> int:
        self._update_seq += 1
        return self._update_seq

    def _pose_from_apriltag_detection(
        self, detection: object
    ) -> Optional[tuple[list[float], list[float]]]:
        pose_field: Optional[object] = getattr(detection, "pose", None)
        if pose_field is None:
            return None
        pose: object = getattr(pose_field, "pose", pose_field)
        position: Optional[object] = getattr(pose, "position", None)
        orientation: Optional[object] = getattr(pose, "orientation", None)
        if position is None or orientation is None:
            return None
        return (
            [
                float(getattr(position, "x", 0.0)),
                float(getattr(position, "y", 0.0)),
                float(getattr(position, "z", 0.0)),
            ],
            [
                float(getattr(orientation, "x", 0.0)),
                float(getattr(orientation, "y", 0.0)),
                float(getattr(orientation, "z", 0.0)),
                float(getattr(orientation, "w", 1.0)),
            ],
        )

    def _covariance_from_apriltag_detection(
        self, detection: object
    ) -> Optional[list[float]]:
        pose_field: Optional[object] = getattr(detection, "pose", None)
        if pose_field is None:
            return None
        covariance_field: Optional[object] = getattr(pose_field, "covariance", None)
        if covariance_field is None:
            pose_container: Optional[object] = getattr(pose_field, "pose", None)
            if pose_container is not None:
                covariance_field = getattr(pose_container, "covariance", None)
        if covariance_field is None:
            return None
        covariance: list[float] = [float(value) for value in covariance_field]
        if len(covariance) != 36:
            return None
        if not is_finite_vector(covariance):
            return None
        return covariance

    def _default_apriltag_covariance(self) -> list[float]:
        # Units: m^2. Meaning: position variance from the configured stddev
        pos_var_m2: float = self._apriltag_pos_std_m**2

        # Units: rad^2. Meaning: rotation variance from the configured stddev
        rot_var_rad2: float = self._apriltag_rot_std_rad**2
        covariance: list[float] = [0.0] * 36
        for index in range(3):
            covariance[index * 6 + index] = pos_var_m2
        for index in range(3):
            covariance[(index + 3) * 6 + (index + 3)] = rot_var_rad2
        return covariance

    def _is_valid_pose(self, position: list[float], quaternion: list[float]) -> bool:
        if not is_finite_vector(position) or not is_finite_vector(quaternion):
            return False
        norm_sq: float = (
            quaternion[0] * quaternion[0]
            + quaternion[1] * quaternion[1]
            + quaternion[2] * quaternion[2]
            + quaternion[3] * quaternion[3]
        )
        return norm_sq > 0.0
