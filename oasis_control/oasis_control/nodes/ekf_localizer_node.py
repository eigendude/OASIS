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
import math
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
from sensor_msgs.msg import CameraInfo as CameraInfoMsg
from sensor_msgs.msg import Imu as ImuMsg
from sensor_msgs.msg import MagneticField as MagneticFieldMsg

from oasis_control.localization.ekf.ekf_buffer import EkfBuffer
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import AprilTagDetectionArrayData
from oasis_control.localization.ekf.ekf_types import CameraInfoData
from oasis_control.localization.ekf.ekf_types import EkfAprilTagDetectionUpdate
from oasis_control.localization.ekf.ekf_types import EkfAprilTagUpdateData
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfOutputs
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import EkfUpdateData
from oasis_control.localization.ekf.ekf_types import ImuCalibrationData
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import MagSample
from oasis_control.localization.ekf.ekf_types import from_seconds
from oasis_control.localization.ekf.ekf_types import to_ns
from oasis_control.localization.ekf.ekf_types import to_seconds
from oasis_control.localization.ekf.reporting.update_reporter import UpdateReporter
from oasis_control.localization.ekf.ros_time_adapter import ekf_time_to_ros_time
from oasis_control.localization.ekf.ros_time_adapter import ros_time_to_ekf_time
from oasis_control.nodes.ekf_localizer_params import DEFAULT_ACCEL_NOISE_VAR
from oasis_control.nodes.ekf_localizer_params import DEFAULT_ANG_VAR
from oasis_control.nodes.ekf_localizer_params import DEFAULT_APRILTAG_GATE_D2
from oasis_control.nodes.ekf_localizer_params import DEFAULT_APRILTAG_POS_VAR
from oasis_control.nodes.ekf_localizer_params import DEFAULT_APRILTAG_YAW_VAR
from oasis_control.nodes.ekf_localizer_params import DEFAULT_CHECKPOINT_INTERVAL_SEC
from oasis_control.nodes.ekf_localizer_params import DEFAULT_DT_CLOCK_JUMP_MAX
from oasis_control.nodes.ekf_localizer_params import DEFAULT_DT_IMU_MAX
from oasis_control.nodes.ekf_localizer_params import DEFAULT_EPS_WALL_FUTURE
from oasis_control.nodes.ekf_localizer_params import DEFAULT_GRAVITY_MPS2
from oasis_control.nodes.ekf_localizer_params import DEFAULT_GYRO_NOISE_VAR
from oasis_control.nodes.ekf_localizer_params import DEFAULT_MAX_DT_SEC
from oasis_control.nodes.ekf_localizer_params import DEFAULT_POS_VAR
from oasis_control.nodes.ekf_localizer_params import DEFAULT_TAG_LANDMARK_PRIOR_SIGMA_ROT_RAD
from oasis_control.nodes.ekf_localizer_params import DEFAULT_TAG_LANDMARK_PRIOR_SIGMA_T_M
from oasis_control.nodes.ekf_localizer_params import DEFAULT_T_BUFFER_SEC
from oasis_control.nodes.ekf_localizer_params import DEFAULT_VEL_VAR
from oasis_control.nodes.ekf_localizer_params import PARAM_ACCEL_NOISE_VAR
from oasis_control.nodes.ekf_localizer_params import PARAM_ANG_VAR
from oasis_control.nodes.ekf_localizer_params import PARAM_APRILTAG_GATE_D2
from oasis_control.nodes.ekf_localizer_params import PARAM_APRILTAG_POS_VAR
from oasis_control.nodes.ekf_localizer_params import PARAM_APRILTAG_YAW_VAR
from oasis_control.nodes.ekf_localizer_params import PARAM_CHECKPOINT_INTERVAL_SEC
from oasis_control.nodes.ekf_localizer_params import PARAM_DT_CLOCK_JUMP_MAX
from oasis_control.nodes.ekf_localizer_params import PARAM_DT_IMU_MAX
from oasis_control.nodes.ekf_localizer_params import PARAM_EPS_WALL_FUTURE
from oasis_control.nodes.ekf_localizer_params import PARAM_GRAVITY_MPS2
from oasis_control.nodes.ekf_localizer_params import PARAM_GYRO_NOISE_VAR
from oasis_control.nodes.ekf_localizer_params import PARAM_MAX_DT_SEC
from oasis_control.nodes.ekf_localizer_params import PARAM_POS_VAR
from oasis_control.nodes.ekf_localizer_params import PARAM_TAG_LANDMARK_PRIOR_SIGMA_ROT_RAD
from oasis_control.nodes.ekf_localizer_params import PARAM_TAG_LANDMARK_PRIOR_SIGMA_T_M
from oasis_control.nodes.ekf_localizer_params import PARAM_T_BUFFER_SEC
from oasis_control.nodes.ekf_localizer_params import PARAM_VEL_VAR
from oasis_msgs.msg import EkfAprilTagUpdateReport as EkfAprilTagUpdateReportMsg
from oasis_msgs.msg import EkfUpdateReport as EkfUpdateReportMsg


if TYPE_CHECKING:
    from apriltag_msgs.msg import AprilTagDetectionArray as AprilTagDetectionArrayMsg

    from oasis_msgs.msg import ImuCalibration as ImuCalibrationMsg
else:
    AprilTagDetectionArrayMsg = object
    ImuCalibrationMsg = object


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
        self.declare_parameter(PARAM_T_BUFFER_SEC, DEFAULT_T_BUFFER_SEC)
        self.declare_parameter(PARAM_EPS_WALL_FUTURE, DEFAULT_EPS_WALL_FUTURE)
        self.declare_parameter(PARAM_DT_CLOCK_JUMP_MAX, DEFAULT_DT_CLOCK_JUMP_MAX)
        self.declare_parameter(PARAM_DT_IMU_MAX, DEFAULT_DT_IMU_MAX)
        self.declare_parameter(PARAM_POS_VAR, DEFAULT_POS_VAR)
        self.declare_parameter(PARAM_VEL_VAR, DEFAULT_VEL_VAR)
        self.declare_parameter(PARAM_ANG_VAR, DEFAULT_ANG_VAR)
        self.declare_parameter(PARAM_ACCEL_NOISE_VAR, DEFAULT_ACCEL_NOISE_VAR)
        self.declare_parameter(PARAM_GYRO_NOISE_VAR, DEFAULT_GYRO_NOISE_VAR)
        self.declare_parameter(PARAM_GRAVITY_MPS2, DEFAULT_GRAVITY_MPS2)
        self.declare_parameter(PARAM_MAX_DT_SEC, DEFAULT_MAX_DT_SEC)
        self.declare_parameter(
            PARAM_CHECKPOINT_INTERVAL_SEC, DEFAULT_CHECKPOINT_INTERVAL_SEC
        )
        self.declare_parameter(PARAM_APRILTAG_POS_VAR, DEFAULT_APRILTAG_POS_VAR)
        self.declare_parameter(PARAM_APRILTAG_YAW_VAR, DEFAULT_APRILTAG_YAW_VAR)
        self.declare_parameter(PARAM_APRILTAG_GATE_D2, DEFAULT_APRILTAG_GATE_D2)
        self.declare_parameter(PARAM_TAG_SIZE_M, DEFAULT_TAG_SIZE_M)
        self.declare_parameter(PARAM_TAG_ANCHOR_FAMILY, DEFAULT_TAG_ANCHOR_FAMILY)
        self.declare_parameter(PARAM_TAG_ANCHOR_ID, DEFAULT_TAG_ANCHOR_ID)
        self.declare_parameter(
            PARAM_TAG_LANDMARK_PRIOR_SIGMA_T_M,
            DEFAULT_TAG_LANDMARK_PRIOR_SIGMA_T_M,
        )
        self.declare_parameter(
            PARAM_TAG_LANDMARK_PRIOR_SIGMA_ROT_RAD,
            DEFAULT_TAG_LANDMARK_PRIOR_SIGMA_ROT_RAD,
        )

        self._world_frame_id: str = str(self.get_parameter(PARAM_WORLD_FRAME_ID).value)
        self._odom_frame_id: str = str(self.get_parameter(PARAM_ODOM_FRAME_ID).value)
        self._body_frame_id: str = str(self.get_parameter(PARAM_BODY_FRAME_ID).value)
        self._t_buffer_sec: float = float(self.get_parameter(PARAM_T_BUFFER_SEC).value)
        self._eps_wall_future: float = float(
            self.get_parameter(PARAM_EPS_WALL_FUTURE).value
        )
        self._dt_clock_jump_max: float = float(
            self.get_parameter(PARAM_DT_CLOCK_JUMP_MAX).value
        )
        self._dt_imu_max: float = float(self.get_parameter(PARAM_DT_IMU_MAX).value)
        self._pos_var: float = float(self.get_parameter(PARAM_POS_VAR).value)
        self._vel_var: float = float(self.get_parameter(PARAM_VEL_VAR).value)
        self._ang_var: float = float(self.get_parameter(PARAM_ANG_VAR).value)
        self._accel_noise_var: float = float(
            self.get_parameter(PARAM_ACCEL_NOISE_VAR).value
        )
        self._gyro_noise_var: float = float(
            self.get_parameter(PARAM_GYRO_NOISE_VAR).value
        )
        self._gravity_mps2: float = float(self.get_parameter(PARAM_GRAVITY_MPS2).value)
        self._max_dt_sec: float = float(self.get_parameter(PARAM_MAX_DT_SEC).value)
        self._checkpoint_interval_sec: float = float(
            self.get_parameter(PARAM_CHECKPOINT_INTERVAL_SEC).value
        )
        self._apriltag_pos_var: float = float(
            self.get_parameter(PARAM_APRILTAG_POS_VAR).value
        )
        self._apriltag_yaw_var: float = float(
            self.get_parameter(PARAM_APRILTAG_YAW_VAR).value
        )
        self._apriltag_gate_d2: float = float(
            self.get_parameter(PARAM_APRILTAG_GATE_D2).value
        )
        self._tag_size_m: float = float(self.get_parameter(PARAM_TAG_SIZE_M).value)
        self._tag_anchor_family: str = str(
            self.get_parameter(PARAM_TAG_ANCHOR_FAMILY).value
        )
        self._tag_anchor_id: int = int(self.get_parameter(PARAM_TAG_ANCHOR_ID).value)
        self._tag_landmark_prior_sigma_t_m: float = float(
            self.get_parameter(PARAM_TAG_LANDMARK_PRIOR_SIGMA_T_M).value
        )
        self._tag_landmark_prior_sigma_rot_rad: float = float(
            self.get_parameter(PARAM_TAG_LANDMARK_PRIOR_SIGMA_ROT_RAD).value
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
        self._last_imu_time: Optional[float] = None
        self._update_seq: int = 0
        self._synced_imu_timestamps_ns: Deque[int] = deque()
        self._synced_imu_timestamps_set: set[int] = set()

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
            self._process_event(event)
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
        timestamp_s: float = to_seconds(timestamp)

        if self._last_imu_time is not None:
            dt_imu: float = timestamp_s - self._last_imu_time
            if dt_imu > self._dt_imu_max:
                self.get_logger().warn(
                    f"IMU dt exceeded limit ({dt_imu:.3f}s), skipping"
                )
                self._last_imu_time = timestamp_s
                return

        self._last_imu_time = timestamp_s

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
        self._process_event(event)

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
        self._process_event(event)

    def _handle_apriltags(self, message: AprilTagDetectionArrayMsg) -> None:
        timestamp: EkfTime = ros_time_to_ekf_time(message.header.stamp)

        detection_data: AprilTagDetectionArrayData = self._apriltag_data_from_msg(
            message
        )
        reject_reason: Optional[str] = None

        if self._camera_info is None:
            self.get_logger().warn("No camera_info cached, rejecting apriltags")
            reject_reason = "No camera_info cached"
        elif message.header.frame_id != self._camera_info.frame_id:
            self.get_logger().warn("AprilTag frame does not match cached camera_info")
            reject_reason = "AprilTag frame mismatch"

        event: EkfEvent = EkfEvent(
            t_meas=timestamp,
            event_type=EkfEventType.APRILTAG,
            payload=detection_data,
        )
        if reject_reason is not None:
            self._publish_rejection(event, reject_reason)
            return

        self._process_event(event)

    def _process_event(self, event: EkfEvent) -> None:
        if self._buffer.too_old(event.t_meas):
            self.get_logger().warn("EKF event too old for buffer, dropping")
            self._publish_rejection(event, "Event too old for buffer")
            return

        if self._buffer.detect_clock_jump(event.t_meas):
            self.get_logger().warn("EKF clock jump detected")

        self._buffer.insert_event(event)
        self._buffer.evict(event.t_meas)

        if self._core.is_out_of_order(event.t_meas):
            outputs: EkfOutputs = self._core.replay(self._buffer, event.t_meas)
        else:
            outputs = self._core.process_event(event)

        if outputs.odom_time_s is not None:
            self._publish_odom(outputs.odom_time_s)
        if outputs.world_odom_time_s is not None:
            self._publish_world_odom(outputs.world_odom_time_s)
        if outputs.mag_update is not None:
            self._publish_mag_update(outputs.mag_update)
        if outputs.apriltag_update is not None:
            apriltag_update: EkfAprilTagUpdateData = outputs.apriltag_update
            self._publish_apriltag_update(apriltag_update)

    def _publish_odom(self, timestamp: float) -> None:
        message: OdometryMsg = self._build_odom(timestamp)
        self._odom_pub.publish(message)

    def _publish_world_odom(self, timestamp: float) -> None:
        message: OdometryMsg = self._build_odom(timestamp)
        message.header.frame_id = self._world_frame_id
        self._world_odom_pub.publish(message)

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
            apriltag_data: AprilTagDetectionArrayData = cast(
                AprilTagDetectionArrayData, event.payload
            )
            update_data: EkfAprilTagUpdateData = self._core.update_with_apriltags(
                apriltag_data, to_seconds(event.t_meas)
            )
            update_data = self._override_apriltag_rejection(update_data, reason)
            self._publish_apriltag_update(update_data)

    def _override_apriltag_rejection(
        self, update_data: EkfAprilTagUpdateData, reason: str
    ) -> EkfAprilTagUpdateData:
        rejected: list[EkfAprilTagDetectionUpdate] = []
        for detection in update_data.detections:
            update: EkfUpdateData = replace(
                detection.update, accepted=False, reject_reason=reason
            )
            rejected.append(
                replace(
                    detection,
                    update=update,
                )
            )
        return EkfAprilTagUpdateData(
            t_meas=update_data.t_meas,
            frame_id=update_data.frame_id,
            detections=rejected,
        )

    def _build_odom(self, timestamp: float) -> OdometryMsg:
        message: OdometryMsg = OdometryMsg()
        message.header.stamp = ekf_time_to_ros_time(from_seconds(timestamp))
        message.header.frame_id = self._odom_frame_id
        message.child_frame_id = self._body_frame_id
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance = [0.0] * 36
        message.twist.covariance = [0.0] * 36

        # TODO: Publish TF when EKF state output is implemented
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

    def _apriltag_data_from_msg(
        self, message: AprilTagDetectionArrayMsg
    ) -> AprilTagDetectionArrayData:
        detections: list[AprilTagDetection] = []
        for index, detection in enumerate(message.detections):
            corners_px: list[float] = []
            for corner in detection.corners:
                corners_px.extend([float(corner.x), float(corner.y)])
            detections.append(
                AprilTagDetection(
                    family=str(detection.family),
                    tag_id=int(detection.id),
                    det_index_in_msg=int(index),
                    corners_px=corners_px,
                    pose_world_xyz_yaw=self._pose_from_apriltag_detection(detection),
                    decision_margin=float(detection.decision_margin),
                    homography=list(detection.homography),
                )
            )
        return AprilTagDetectionArrayData(
            frame_id=message.header.frame_id,
            detections=detections,
        )

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

    def _pose_from_apriltag_detection(self, detection: object) -> Optional[list[float]]:
        pose_field: Optional[object] = getattr(detection, "pose", None)
        if pose_field is None:
            return None
        pose: object = getattr(pose_field, "pose", pose_field)
        position: Optional[object] = getattr(pose, "position", None)
        orientation: Optional[object] = getattr(pose, "orientation", None)
        if position is None or orientation is None:
            return None
        yaw: float = self._yaw_from_quaternion(orientation)
        return [
            float(getattr(position, "x", 0.0)),
            float(getattr(position, "y", 0.0)),
            float(getattr(position, "z", 0.0)),
            yaw,
        ]

    def _yaw_from_quaternion(self, orientation: object) -> float:
        qx: float = float(getattr(orientation, "x", 0.0))
        qy: float = float(getattr(orientation, "y", 0.0))
        qz: float = float(getattr(orientation, "z", 0.0))
        qw: float = float(getattr(orientation, "w", 1.0))
        siny_cosp: float = 2.0 * (qw * qz + qx * qy)
        cosy_cosp: float = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
