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
from typing import Optional

import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
import rclpy.timer
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import (
    AccelWithCovarianceStamped as AccelWithCovarianceStampedMsg,
)
from geometry_msgs.msg import TransformStamped as TransformStampedMsg
from nav_msgs.msg import Odometry as OdometryMsg
from rclpy.time import Time
from sensor_msgs.msg import Imu as ImuMsg
from tf2_ros import Buffer
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros import TransformListener

from oasis_control.localization.ahrs.data.ahrs_output import AhrsOutput
from oasis_control.localization.ahrs.data.diagnostics import AhrsDiagnosticsSnapshot
from oasis_control.localization.ahrs.data.diagnostics import AhrsDiagnosticsState
from oasis_control.localization.ahrs.data.diagnostics import snapshot_diagnostics
from oasis_control.localization.ahrs.processing.attitude_mapper import map_imu_to_base
from oasis_control.localization.ahrs.processing.boot_mounting_calibrator import (
    DEFAULT_CALIBRATION_DURATION_SEC as DEFAULT_BOOT_MOUNTING_CALIBRATION_DURATION_SEC,
)
from oasis_control.localization.ahrs.processing.boot_mounting_calibrator import (
    DEFAULT_MIN_SAMPLE_COUNT as DEFAULT_BOOT_MOUNTING_MIN_SAMPLE_COUNT,
)
from oasis_control.localization.ahrs.processing.boot_mounting_calibrator import (
    DEFAULT_STATIONARY_ANGULAR_SPEED_THRESHOLD_RADS as DEFAULT_BOOT_MOUNTING_STATIONARY_ANGULAR_SPEED_THRESHOLD_RADS,
)
from oasis_control.localization.ahrs.processing.boot_mounting_calibrator import (
    BootMountingCalibrator,
)
from oasis_control.localization.ahrs.processing.boot_mounting_calibrator import (
    BootMountingSolution,
)
from oasis_control.localization.ahrs.processing.gravity_consistency import (
    GravityConsistencyDecision,
)
from oasis_control.localization.ahrs.processing.gravity_consistency import (
    GravityConsistencyPolicy,
)
from oasis_control.localization.ahrs.processing.gravity_consistency import (
    evaluate_gravity_consistency,
)
from oasis_control.localization.ahrs.processing.output_adapter import make_ahrs_output
from oasis_control.localization.common.algebra.covariance import (
    UNKNOWN_ORIENTATION_COVARIANCE,
)
from oasis_control.localization.common.algebra.covariance import (
    embed_linear_covariance_3x3,
)
from oasis_control.localization.common.algebra.covariance import (
    flatten_matrix3_row_major,
)
from oasis_control.localization.common.algebra.quat import Quaternion
from oasis_control.localization.common.algebra.quat import Vector3
from oasis_control.localization.common.algebra.quat import normalize_quaternion_xyzw
from oasis_control.localization.common.algebra.quat import quaternion_multiply_xyzw
from oasis_control.localization.common.data.gravity_sample import GravitySample
from oasis_control.localization.common.frames.mounting import MountedImuSample
from oasis_control.localization.common.frames.mounting import MountingTransform
from oasis_control.localization.common.frames.mounting import make_mounting_transform
from oasis_control.localization.common.measurements.gravity_direction import (
    GravityDirectionResidual,
)
from oasis_control.localization.common.validation.gravity_validation import (
    validate_gravity_sample,
)
from oasis_control.localization.common.validation.imu_validation import (
    validate_imu_sample,
)
from oasis_msgs.msg import AhrsStatus as AhrsStatusMsg


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "ahrs_node"

# ROS topics
GRAVITY_TOPIC: str = "gravity"
IMU_TOPIC: str = "imu"
OUTPUT_DIAG_TOPIC: str = "ahrs/diag"
OUTPUT_IMU_TOPIC: str = "ahrs/imu"
OUTPUT_ODOM_TOPIC: str = "ahrs/odom"

# ROS parameters
PARAM_BASE_FRAME_ID: str = "base_frame_id"
PARAM_IMU_FRAME_ID: str = "imu_frame_id"
PARAM_ODOM_FRAME_ID: str = "odom_frame_id"
PARAM_WORLD_FRAME_ID: str = "world_frame_id"
PARAM_GRAVITY_RESIDUAL_REJECT_THRESHOLD: str = "gravity_residual_reject_threshold"
PARAM_GRAVITY_MAHALANOBIS_REJECT_THRESHOLD: str = "gravity_mahalanobis_reject_threshold"
PARAM_MOUNTING_CALIBRATION_DURATION_SEC: str = "mounting_calibration_duration_sec"
PARAM_MOUNTING_STATIONARY_ANGULAR_SPEED_THRESHOLD_RADS: str = (
    "mounting_stationary_angular_speed_threshold_rads"
)
PARAM_MOUNTING_MIN_SAMPLE_COUNT: str = "mounting_min_sample_count"

DEFAULT_BASE_FRAME_ID: str = "base_link"
DEFAULT_IMU_FRAME_ID: str = "imu_link"
DEFAULT_ODOM_FRAME_ID: str = "odom"
DEFAULT_WORLD_FRAME_ID: str = "world"
DEFAULT_GRAVITY_RESIDUAL_REJECT_THRESHOLD: float = 0.35
DEFAULT_GRAVITY_MAHALANOBIS_REJECT_THRESHOLD: float = 5.0
DEFAULT_MOUNTING_CALIBRATION_DURATION_SEC: float = (
    DEFAULT_BOOT_MOUNTING_CALIBRATION_DURATION_SEC
)
DEFAULT_MOUNTING_STATIONARY_ANGULAR_SPEED_THRESHOLD_RADS: float = (
    DEFAULT_BOOT_MOUNTING_STATIONARY_ANGULAR_SPEED_THRESHOLD_RADS
)
DEFAULT_MOUNTING_MIN_SAMPLE_COUNT: int = DEFAULT_BOOT_MOUNTING_MIN_SAMPLE_COUNT

# Timer period for diagnostics and TF heartbeat
#
# Units: s
#
STATUS_TIMER_PERIOD_SEC: float = 0.1


################################################################################
# ROS node
################################################################################


class AhrsNode(rclpy.node.Node):
    """
    Lightweight event-driven AHRS runtime for mounted attitude publication.
    """

    def __init__(
        self,
        *,
        tf_buffer: Optional[Buffer] = None,
        tf_broadcaster: Optional[TransformBroadcaster] = None,
        enable_status_timer: bool = True,
    ) -> None:
        super().__init__(NODE_NAME)

        # ROS parameters
        self.declare_parameter(PARAM_BASE_FRAME_ID, DEFAULT_BASE_FRAME_ID)
        self.declare_parameter(PARAM_IMU_FRAME_ID, DEFAULT_IMU_FRAME_ID)
        self.declare_parameter(PARAM_ODOM_FRAME_ID, DEFAULT_ODOM_FRAME_ID)
        self.declare_parameter(PARAM_WORLD_FRAME_ID, DEFAULT_WORLD_FRAME_ID)
        self.declare_parameter(
            PARAM_GRAVITY_RESIDUAL_REJECT_THRESHOLD,
            DEFAULT_GRAVITY_RESIDUAL_REJECT_THRESHOLD,
        )
        self.declare_parameter(
            PARAM_GRAVITY_MAHALANOBIS_REJECT_THRESHOLD,
            DEFAULT_GRAVITY_MAHALANOBIS_REJECT_THRESHOLD,
        )
        self.declare_parameter(
            PARAM_MOUNTING_CALIBRATION_DURATION_SEC,
            DEFAULT_MOUNTING_CALIBRATION_DURATION_SEC,
        )
        self.declare_parameter(
            PARAM_MOUNTING_STATIONARY_ANGULAR_SPEED_THRESHOLD_RADS,
            DEFAULT_MOUNTING_STATIONARY_ANGULAR_SPEED_THRESHOLD_RADS,
        )
        self.declare_parameter(
            PARAM_MOUNTING_MIN_SAMPLE_COUNT,
            DEFAULT_MOUNTING_MIN_SAMPLE_COUNT,
        )

        self._base_frame_id: str = str(self.get_parameter(PARAM_BASE_FRAME_ID).value)
        self._imu_frame_id: str = str(self.get_parameter(PARAM_IMU_FRAME_ID).value)
        self._odom_frame_id: str = str(self.get_parameter(PARAM_ODOM_FRAME_ID).value)
        self._world_frame_id: str = str(self.get_parameter(PARAM_WORLD_FRAME_ID).value)
        self._gravity_consistency_policy: GravityConsistencyPolicy = (
            GravityConsistencyPolicy(
                residual_norm_threshold=float(
                    self.get_parameter(PARAM_GRAVITY_RESIDUAL_REJECT_THRESHOLD).value
                ),
                mahalanobis_distance_threshold=float(
                    self.get_parameter(PARAM_GRAVITY_MAHALANOBIS_REJECT_THRESHOLD).value
                ),
            )
        )
        self._mounting_calibrator: BootMountingCalibrator = BootMountingCalibrator(
            parent_frame_id=self._base_frame_id,
            child_frame_id=self._imu_frame_id,
            calibration_duration_sec=float(
                self.get_parameter(PARAM_MOUNTING_CALIBRATION_DURATION_SEC).value
            ),
            stationary_angular_speed_threshold_rads=float(
                self.get_parameter(
                    PARAM_MOUNTING_STATIONARY_ANGULAR_SPEED_THRESHOLD_RADS
                ).value
            ),
            min_sample_count=int(
                self.get_parameter(PARAM_MOUNTING_MIN_SAMPLE_COUNT).value
            ),
        )

        # AHRS state
        self._diagnostics: AhrsDiagnosticsState = AhrsDiagnosticsState()
        self._latest_gravity_sample: Optional[GravitySample] = None
        self._latest_output: Optional[AhrsOutput] = None
        self._latest_imu_angular_velocity_rads: Optional[Vector3] = None
        self._mounting_transform: Optional[MountingTransform] = None
        self._session_yaw_zero_initialized: bool = False
        self._session_yaw_offset_xyzw: Quaternion = (0.0, 0.0, 0.0, 1.0)

        # ROS QoS profiles
        sensor_qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        # ROS publishers
        self._diag_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=AhrsStatusMsg,
            topic=OUTPUT_DIAG_TOPIC,
            qos_profile=sensor_qos_profile,
        )
        self._imu_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=ImuMsg,
            topic=OUTPUT_IMU_TOPIC,
            qos_profile=sensor_qos_profile,
        )
        self._odom_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=OdometryMsg,
            topic=OUTPUT_ODOM_TOPIC,
            qos_profile=sensor_qos_profile,
        )

        # ROS subscribers
        self._gravity_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=AccelWithCovarianceStampedMsg,
            topic=GRAVITY_TOPIC,
            callback=self._handle_gravity,
            qos_profile=sensor_qos_profile,
        )
        self._imu_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=ImuMsg,
            topic=IMU_TOPIC,
            callback=self._handle_imu,
            qos_profile=sensor_qos_profile,
        )

        # TF buffer and listener
        self._tf_buffer: Buffer = tf_buffer if tf_buffer is not None else Buffer()
        self._transform_listener: Optional[TransformListener] = None
        if tf_buffer is None:
            self._transform_listener = TransformListener(self._tf_buffer, self)

        # TF broadcaster
        self._tf_broadcaster: TransformBroadcaster = (
            tf_broadcaster if tf_broadcaster is not None else TransformBroadcaster(self)
        )

        # ROS timers
        self._status_timer: Optional[rclpy.timer.Timer] = None
        if enable_status_timer:
            self._status_timer = self.create_timer(
                STATUS_TIMER_PERIOD_SEC,
                self._publish_runtime_outputs,
            )

        # Publish initial message
        self._publish_runtime_outputs()

        self.get_logger().info("AHRS node initialized")

    def stop(self) -> None:
        self.get_logger().info("AHRS node deinitialized")
        self.destroy_node()

    def _handle_gravity(self, message: AccelWithCovarianceStampedMsg) -> None:
        validation_result = validate_gravity_sample(
            timestamp_ns=_time_msg_to_ns(message.header.stamp),
            frame_id=message.header.frame_id,
            expected_frame_id=self._imu_frame_id,
            gravity_mps2=(
                float(message.accel.accel.linear.x),
                float(message.accel.accel.linear.y),
                float(message.accel.accel.linear.z),
            ),
            gravity_covariance_row_major=tuple(
                float(value) for value in message.accel.covariance
            ),
        )
        if not validation_result.accepted or validation_result.sample is None:
            self._diagnostics.rejected_gravity_count += 1
            if validation_result.rejection_reason == "bad_frame":
                self._diagnostics.last_bad_gravity_frame_id = message.header.frame_id
            self._publish_runtime_outputs()
            return

        if self._is_stale_gravity(validation_result.sample.timestamp_ns):
            self._diagnostics.dropped_stale_gravity_count += 1
            self._diagnostics.last_bad_gravity_frame_id = ""
            self._publish_runtime_outputs()
            return

        self._latest_gravity_sample = validation_result.sample
        self._diagnostics.accepted_gravity_count += 1
        self._diagnostics.has_gravity = True
        self._diagnostics.last_bad_gravity_frame_id = ""
        self._diagnostics.last_accepted_gravity_timestamp_ns = (
            validation_result.sample.timestamp_ns
        )
        self._update_mounting_calibration(validation_result.sample)
        self._publish_runtime_outputs()

    def _handle_imu(self, message: ImuMsg) -> None:
        validation_result = validate_imu_sample(
            timestamp_ns=_time_msg_to_ns(message.header.stamp),
            frame_id=message.header.frame_id,
            expected_frame_id=self._imu_frame_id,
            orientation_xyzw=(
                float(message.orientation.x),
                float(message.orientation.y),
                float(message.orientation.z),
                float(message.orientation.w),
            ),
            orientation_covariance_row_major=tuple(
                float(value) for value in message.orientation_covariance
            ),
            angular_velocity_rads=(
                float(message.angular_velocity.x),
                float(message.angular_velocity.y),
                float(message.angular_velocity.z),
            ),
            angular_velocity_covariance_row_major=tuple(
                float(value) for value in message.angular_velocity_covariance
            ),
            linear_acceleration_mps2=(
                float(message.linear_acceleration.x),
                float(message.linear_acceleration.y),
                float(message.linear_acceleration.z),
            ),
            linear_acceleration_covariance_row_major=tuple(
                float(value) for value in message.linear_acceleration_covariance
            ),
        )
        if not validation_result.accepted or validation_result.sample is None:
            self._diagnostics.rejected_imu_count += 1
            if validation_result.rejection_reason == "bad_frame":
                self._diagnostics.last_bad_imu_frame_id = message.header.frame_id
            self._publish_runtime_outputs()
            return

        if self._is_stale_imu(validation_result.sample.timestamp_ns):
            self._diagnostics.dropped_stale_imu_count += 1
            self._diagnostics.last_bad_imu_frame_id = ""
            self._publish_runtime_outputs()
            return

        self._diagnostics.accepted_imu_count += 1
        self._diagnostics.last_bad_imu_frame_id = ""
        self._diagnostics.last_accepted_imu_timestamp_ns = (
            validation_result.sample.timestamp_ns
        )
        self._latest_imu_angular_velocity_rads = (
            validation_result.sample.angular_velocity_rads
        )

        mounting_transform: Optional[MountingTransform] = self._resolve_mounting()
        if mounting_transform is None:
            self._latest_output = None
            self._publish_runtime_outputs()
            return

        mounted_imu_sample: MountedImuSample = map_imu_to_base(
            validation_result.sample,
            mounting_transform,
        )

        gravity_residual = None
        if self._latest_gravity_sample is not None:
            gravity_consistency_decision: GravityConsistencyDecision = (
                evaluate_gravity_consistency(
                    gravity_sample=self._latest_gravity_sample,
                    mounted_imu_sample=mounted_imu_sample,
                    mounting_transform=mounting_transform,
                    policy=self._gravity_consistency_policy,
                )
            )
            gravity_residual = gravity_consistency_decision.residual
            self._diagnostics.gravity_gated_in = gravity_consistency_decision.accepted
            self._diagnostics.gravity_rejected = (
                not gravity_consistency_decision.accepted
            )
            self._diagnostics.last_gravity_rejection_reason = (
                gravity_consistency_decision.rejection_reason
            )
            if not gravity_consistency_decision.accepted:
                self._diagnostics.gravity_rejection_count += 1
        else:
            self._diagnostics.gravity_gated_in = False
            self._diagnostics.gravity_rejected = False
            self._diagnostics.last_gravity_rejection_reason = ""

        self._diagnostics.last_gravity_residual = gravity_residual

        self._latest_output = self._make_session_yaw_zeroed_output(
            mounted_imu_sample=mounted_imu_sample,
            gravity_residual=gravity_residual,
        )
        self._imu_pub.publish(self._build_imu_message(self._latest_output))
        self._odom_pub.publish(self._build_odom_message(self._latest_output))
        self._publish_runtime_outputs()

    def _is_stale_imu(self, timestamp_ns: int) -> bool:
        last_timestamp_ns: Optional[int] = (
            self._diagnostics.last_accepted_imu_timestamp_ns
        )
        return last_timestamp_ns is not None and timestamp_ns < last_timestamp_ns

    def _is_stale_gravity(self, timestamp_ns: int) -> bool:
        last_timestamp_ns: Optional[int] = (
            self._diagnostics.last_accepted_gravity_timestamp_ns
        )
        return last_timestamp_ns is not None and timestamp_ns < last_timestamp_ns

    def _publish_runtime_outputs(self) -> None:
        self._publish_tf()
        self._publish_status()

    def _publish_tf(self) -> None:
        transforms: list[TransformStampedMsg] = [
            self._make_identity_transform(
                parent_frame_id=self._world_frame_id,
                child_frame_id=self._odom_frame_id,
                stamp=self._current_stamp(),
            )
        ]

        if self._mounting_transform is not None:
            transforms.append(
                self._build_mounting_transform(
                    mounting_transform=self._mounting_transform,
                    stamp=self._current_stamp(),
                )
            )

        if self._latest_output is not None:
            transforms.append(self._build_odom_to_base_transform(self._latest_output))

        self._tf_broadcaster.sendTransform(transforms)

    def _publish_status(self) -> None:
        status_snapshot: AhrsDiagnosticsSnapshot = snapshot_diagnostics(
            self._diagnostics
        )

        # The status enum is intentionally coarse. Consumers should use the
        # structured diagnostics fields below for finer-grained interpretation.
        status_message: AhrsStatusMsg = AhrsStatusMsg()
        status_message.header.stamp = self._current_stamp()
        status_message.header.frame_id = self._world_frame_id
        status_message.status = self._compute_status_code()
        status_message.accepted_imu_count = status_snapshot.accepted_imu_count
        status_message.accepted_gravity_count = status_snapshot.accepted_gravity_count
        status_message.rejected_imu_count = status_snapshot.rejected_imu_count
        status_message.rejected_gravity_count = status_snapshot.rejected_gravity_count
        status_message.dropped_stale_imu_count = status_snapshot.dropped_stale_imu_count
        status_message.dropped_stale_gravity_count = (
            status_snapshot.dropped_stale_gravity_count
        )
        status_message.gravity_rejection_count = status_snapshot.gravity_rejection_count
        status_message.gravity_residual_norm = status_snapshot.gravity_residual_norm
        status_message.gravity_mahalanobis_distance = (
            status_snapshot.gravity_mahalanobis_distance
        )
        status_message.has_gravity = status_snapshot.has_gravity
        status_message.has_mounting = status_snapshot.has_mounting
        status_message.gravity_gated_in = status_snapshot.gravity_gated_in
        status_message.gravity_rejected = status_snapshot.gravity_rejected
        status_message.transform_lookup_failure_count = (
            status_snapshot.transform_lookup_failure_count
        )
        status_message.invalid_mounting_transform_count = (
            status_snapshot.invalid_mounting_transform_count
        )
        status_message.last_mounting_lookup_error = (
            status_snapshot.last_mounting_lookup_error
        )
        status_message.last_gravity_rejection_reason = (
            status_snapshot.last_gravity_rejection_reason
        )
        status_message.status_text = self._compute_status_text()

        self._diag_pub.publish(status_message)

    def _compute_status_code(self) -> int:
        # Keep the public status code coarse and stable. Detailed runtime
        # meaning belongs in the structured AhrsStatus diagnostics fields.
        if self._diagnostics.last_bad_imu_frame_id:
            return AhrsStatusMsg.STATUS_BAD_IMU_FRAME

        if self._diagnostics.last_bad_gravity_frame_id:
            return AhrsStatusMsg.STATUS_BAD_GRAVITY_FRAME

        if self._diagnostics.accepted_imu_count == 0:
            return AhrsStatusMsg.STATUS_WAITING_FOR_IMU

        if self._diagnostics.accepted_gravity_count == 0:
            return AhrsStatusMsg.STATUS_WAITING_FOR_GRAVITY

        if not self._diagnostics.has_mounting:
            return AhrsStatusMsg.STATUS_MOUNTING_UNAVAILABLE

        return AhrsStatusMsg.STATUS_OK

    def _compute_status_text(self) -> str:
        if self._diagnostics.last_bad_imu_frame_id:
            return "Bad IMU frame"

        if self._diagnostics.last_bad_gravity_frame_id:
            return "Bad gravity frame"

        if self._diagnostics.accepted_imu_count == 0:
            return "Waiting for IMU samples"

        if self._diagnostics.accepted_gravity_count == 0:
            return "Waiting for gravity samples"

        if not self._diagnostics.has_mounting:
            return "Mounting transform unavailable"

        if self._diagnostics.gravity_rejected:
            return "Gravity consistency rejected"

        return "Mounted attitude output available"

    def _resolve_mounting(self) -> Optional[MountingTransform]:
        if self._mounting_transform is not None:
            self._diagnostics.has_mounting = True
            self._diagnostics.last_mounting_lookup_error = ""
            return self._mounting_transform

        try:
            transform_message = self._tf_buffer.lookup_transform(
                self._base_frame_id,
                self._imu_frame_id,
                Time(),
            )
        except TransformException as error:
            self._diagnostics.transform_lookup_failure_count += 1
            self._diagnostics.has_mounting = False
            self._diagnostics.last_mounting_lookup_error = str(error)
            return None

        quaternion_xyzw = normalize_quaternion_xyzw(
            (
                float(transform_message.transform.rotation.x),
                float(transform_message.transform.rotation.y),
                float(transform_message.transform.rotation.z),
                float(transform_message.transform.rotation.w),
            )
        )
        if quaternion_xyzw is None:
            self._diagnostics.invalid_mounting_transform_count += 1
            self._diagnostics.has_mounting = False
            self._diagnostics.last_mounting_lookup_error = (
                "mounting quaternion is non-finite or zero-norm"
            )
            return None

        self._mounting_transform = make_mounting_transform(
            parent_frame_id=self._base_frame_id,
            child_frame_id=self._imu_frame_id,
            quaternion_xyzw=quaternion_xyzw,
        )
        self._diagnostics.has_mounting = True
        self._diagnostics.last_mounting_lookup_error = ""
        return self._mounting_transform

    def _update_mounting_calibration(self, gravity_sample: GravitySample) -> None:
        if self._mounting_transform is not None:
            return

        solution: Optional[BootMountingSolution] = (
            self._mounting_calibrator.add_gravity_sample(
                gravity_sample=gravity_sample,
                angular_velocity_rads=self._latest_imu_angular_velocity_rads,
            )
        )
        if solution is None:
            self._diagnostics.has_mounting = False
            self._diagnostics.last_mounting_lookup_error = (
                "boot mounting calibration in progress"
            )
            return

        self._mounting_transform = solution.mounting_transform
        self._diagnostics.has_mounting = True
        self._diagnostics.last_mounting_lookup_error = ""
        self.get_logger().info(
            "Solved boot AHRS mounting: "
            f"{solution.roll_rad:.4f} rad, pitch {solution.pitch_rad:.4f} rad, "
            f"yaw fixed to {solution.yaw_rad:.4f} rad"
        )

    def _build_imu_message(self, ahrs_output: AhrsOutput) -> ImuMsg:
        imu_message: ImuMsg = ImuMsg()
        imu_message.header.stamp = _ns_to_time_msg(ahrs_output.mounted_imu.timestamp_ns)
        imu_message.header.frame_id = self._base_frame_id

        imu_message.orientation.x = ahrs_output.mounted_imu.orientation_xyzw[0]
        imu_message.orientation.y = ahrs_output.mounted_imu.orientation_xyzw[1]
        imu_message.orientation.z = ahrs_output.mounted_imu.orientation_xyzw[2]
        imu_message.orientation.w = ahrs_output.mounted_imu.orientation_xyzw[3]

        if ahrs_output.mounted_imu.orientation_covariance_unknown:
            imu_message.orientation_covariance = list(UNKNOWN_ORIENTATION_COVARIANCE)
        elif ahrs_output.mounted_imu.orientation_covariance_rad2 is not None:
            # Publish the mounted covariance exactly as produced by the frame
            # mapping stage.
            imu_message.orientation_covariance = flatten_matrix3_row_major(
                ahrs_output.mounted_imu.orientation_covariance_rad2
            )

        imu_message.angular_velocity.x = ahrs_output.mounted_imu.angular_velocity_rads[
            0
        ]
        imu_message.angular_velocity.y = ahrs_output.mounted_imu.angular_velocity_rads[
            1
        ]
        imu_message.angular_velocity.z = ahrs_output.mounted_imu.angular_velocity_rads[
            2
        ]
        if ahrs_output.mounted_imu.angular_velocity_covariance_rads2 is not None:
            imu_message.angular_velocity_covariance = flatten_matrix3_row_major(
                ahrs_output.mounted_imu.angular_velocity_covariance_rads2
            )

        imu_message.linear_acceleration.x = (
            ahrs_output.mounted_imu.linear_acceleration_mps2[0]
        )
        imu_message.linear_acceleration.y = (
            ahrs_output.mounted_imu.linear_acceleration_mps2[1]
        )
        imu_message.linear_acceleration.z = (
            ahrs_output.mounted_imu.linear_acceleration_mps2[2]
        )
        if ahrs_output.mounted_imu.linear_acceleration_covariance_mps2_2 is not None:
            imu_message.linear_acceleration_covariance = flatten_matrix3_row_major(
                ahrs_output.mounted_imu.linear_acceleration_covariance_mps2_2
            )

        return imu_message

    def _build_odom_message(self, ahrs_output: AhrsOutput) -> OdometryMsg:
        odom_message: OdometryMsg = OdometryMsg()
        odom_message.header.stamp = _ns_to_time_msg(
            ahrs_output.mounted_imu.timestamp_ns
        )
        odom_message.header.frame_id = self._odom_frame_id
        odom_message.child_frame_id = self._base_frame_id

        odom_message.pose.pose.position.x = 0.0
        odom_message.pose.pose.position.y = 0.0
        odom_message.pose.pose.position.z = 0.0
        odom_message.pose.pose.orientation.x = ahrs_output.mounted_imu.orientation_xyzw[
            0
        ]
        odom_message.pose.pose.orientation.y = ahrs_output.mounted_imu.orientation_xyzw[
            1
        ]
        odom_message.pose.pose.orientation.z = ahrs_output.mounted_imu.orientation_xyzw[
            2
        ]
        odom_message.pose.pose.orientation.w = ahrs_output.mounted_imu.orientation_xyzw[
            3
        ]

        if ahrs_output.mounted_imu.orientation_covariance_rad2 is not None:
            # The odom wrapper reuses the same rotated base-frame orientation
            # covariance block rather than inventing a separate model.
            odom_message.pose.covariance[21] = (
                ahrs_output.mounted_imu.orientation_covariance_rad2[0][0]
            )
            odom_message.pose.covariance[22] = (
                ahrs_output.mounted_imu.orientation_covariance_rad2[0][1]
            )
            odom_message.pose.covariance[23] = (
                ahrs_output.mounted_imu.orientation_covariance_rad2[0][2]
            )
            odom_message.pose.covariance[27] = (
                ahrs_output.mounted_imu.orientation_covariance_rad2[1][0]
            )
            odom_message.pose.covariance[28] = (
                ahrs_output.mounted_imu.orientation_covariance_rad2[1][1]
            )
            odom_message.pose.covariance[29] = (
                ahrs_output.mounted_imu.orientation_covariance_rad2[1][2]
            )
            odom_message.pose.covariance[33] = (
                ahrs_output.mounted_imu.orientation_covariance_rad2[2][0]
            )
            odom_message.pose.covariance[34] = (
                ahrs_output.mounted_imu.orientation_covariance_rad2[2][1]
            )
            odom_message.pose.covariance[35] = (
                ahrs_output.mounted_imu.orientation_covariance_rad2[2][2]
            )

        odom_message.twist.twist.linear.x = 0.0
        odom_message.twist.twist.linear.y = 0.0
        odom_message.twist.twist.linear.z = 0.0
        odom_message.twist.twist.angular.x = (
            ahrs_output.mounted_imu.angular_velocity_rads[0]
        )
        odom_message.twist.twist.angular.y = (
            ahrs_output.mounted_imu.angular_velocity_rads[1]
        )
        odom_message.twist.twist.angular.z = (
            ahrs_output.mounted_imu.angular_velocity_rads[2]
        )

        if ahrs_output.mounted_imu.angular_velocity_covariance_rads2 is not None:
            angular_twist_covariance = embed_linear_covariance_3x3(
                ahrs_output.mounted_imu.angular_velocity_covariance_rads2
            )
            odom_message.twist.covariance[21] = angular_twist_covariance[0]
            odom_message.twist.covariance[22] = angular_twist_covariance[1]
            odom_message.twist.covariance[23] = angular_twist_covariance[2]
            odom_message.twist.covariance[27] = angular_twist_covariance[6]
            odom_message.twist.covariance[28] = angular_twist_covariance[7]
            odom_message.twist.covariance[29] = angular_twist_covariance[8]
            odom_message.twist.covariance[33] = angular_twist_covariance[12]
            odom_message.twist.covariance[34] = angular_twist_covariance[13]
            odom_message.twist.covariance[35] = angular_twist_covariance[14]

        return odom_message

    def _build_odom_to_base_transform(
        self, ahrs_output: AhrsOutput
    ) -> TransformStampedMsg:
        transform_message: TransformStampedMsg = TransformStampedMsg()
        transform_message.header.stamp = _ns_to_time_msg(
            ahrs_output.mounted_imu.timestamp_ns
        )
        transform_message.header.frame_id = self._odom_frame_id
        transform_message.child_frame_id = self._base_frame_id
        transform_message.transform.translation.x = 0.0
        transform_message.transform.translation.y = 0.0
        transform_message.transform.translation.z = 0.0
        transform_message.transform.rotation.x = (
            ahrs_output.mounted_imu.orientation_xyzw[0]
        )
        transform_message.transform.rotation.y = (
            ahrs_output.mounted_imu.orientation_xyzw[1]
        )
        transform_message.transform.rotation.z = (
            ahrs_output.mounted_imu.orientation_xyzw[2]
        )
        transform_message.transform.rotation.w = (
            ahrs_output.mounted_imu.orientation_xyzw[3]
        )
        return transform_message

    def _build_mounting_transform(
        self,
        *,
        mounting_transform: MountingTransform,
        stamp: TimeMsg,
    ) -> TransformStampedMsg:
        transform_message: TransformStampedMsg = TransformStampedMsg()
        transform_message.header.stamp = stamp
        transform_message.header.frame_id = mounting_transform.parent_frame_id
        transform_message.child_frame_id = mounting_transform.child_frame_id
        # Publish the stored `q_BI` directly on the `base_link -> imu_link` TF.
        # TF lookup `lookup_transform(base_link, imu_link, ...)` then returns the
        # same IMU-to-base rotation used by runtime mounting.
        transform_message.transform.translation.x = 0.0
        transform_message.transform.translation.y = 0.0
        transform_message.transform.translation.z = 0.0
        transform_message.transform.rotation.x = mounting_transform.quaternion_xyzw[0]
        transform_message.transform.rotation.y = mounting_transform.quaternion_xyzw[1]
        transform_message.transform.rotation.z = mounting_transform.quaternion_xyzw[2]
        transform_message.transform.rotation.w = mounting_transform.quaternion_xyzw[3]
        return transform_message

    def _make_session_yaw_zeroed_output(
        self,
        *,
        mounted_imu_sample: MountedImuSample,
        gravity_residual: Optional[GravityDirectionResidual],
    ) -> AhrsOutput:
        mounted_yaw_rad: float = _yaw_from_quaternion_xyzw(
            mounted_imu_sample.orientation_xyzw
        )
        self.get_logger().debug(
            "AHRS session yaw pre-offset "
            f"timestamp_ns={mounted_imu_sample.timestamp_ns} "
            f"mounted_yaw_rad={mounted_yaw_rad:.4f} "
            f"session_yaw_initialized={self._session_yaw_zero_initialized}"
        )

        if not self._session_yaw_zero_initialized:
            self._session_yaw_offset_xyzw = _quaternion_from_yaw_rad(-mounted_yaw_rad)
            self._session_yaw_zero_initialized = True
            self.get_logger().info(
                "AHRS session yaw initialized: "
                f"mounted_yaw_rad={mounted_yaw_rad:.4f} "
                f"session_yaw_offset_rad="
                f"{_yaw_from_quaternion_xyzw(self._session_yaw_offset_xyzw):.4f}"
            )

        session_zeroed_orientation_xyzw: Quaternion = quaternion_multiply_xyzw(
            self._session_yaw_offset_xyzw,
            mounted_imu_sample.orientation_xyzw,
        )
        normalized_orientation_xyzw: Optional[Quaternion] = normalize_quaternion_xyzw(
            session_zeroed_orientation_xyzw
        )
        if normalized_orientation_xyzw is None:
            normalized_orientation_xyzw = mounted_imu_sample.orientation_xyzw

        session_zeroed_mounted_imu_sample: MountedImuSample = MountedImuSample(
            timestamp_ns=mounted_imu_sample.timestamp_ns,
            frame_id=mounted_imu_sample.frame_id,
            orientation_xyzw=normalized_orientation_xyzw,
            orientation_covariance_rad2=(
                mounted_imu_sample.orientation_covariance_rad2
            ),
            orientation_covariance_unknown=(
                mounted_imu_sample.orientation_covariance_unknown
            ),
            angular_velocity_rads=mounted_imu_sample.angular_velocity_rads,
            angular_velocity_covariance_rads2=(
                mounted_imu_sample.angular_velocity_covariance_rads2
            ),
            linear_acceleration_mps2=mounted_imu_sample.linear_acceleration_mps2,
            linear_acceleration_covariance_mps2_2=(
                mounted_imu_sample.linear_acceleration_covariance_mps2_2
            ),
        )

        return make_ahrs_output(session_zeroed_mounted_imu_sample, gravity_residual)

    def _make_identity_transform(
        self,
        parent_frame_id: str,
        child_frame_id: str,
        stamp: TimeMsg,
    ) -> TransformStampedMsg:
        transform_message: TransformStampedMsg = TransformStampedMsg()
        transform_message.header.stamp = stamp
        transform_message.header.frame_id = parent_frame_id
        transform_message.child_frame_id = child_frame_id
        transform_message.transform.translation.x = 0.0
        transform_message.transform.translation.y = 0.0
        transform_message.transform.translation.z = 0.0
        transform_message.transform.rotation.x = 0.0
        transform_message.transform.rotation.y = 0.0
        transform_message.transform.rotation.z = 0.0
        transform_message.transform.rotation.w = 1.0
        return transform_message

    def _current_stamp(self) -> TimeMsg:
        return self.get_clock().now().to_msg()


def _time_msg_to_ns(stamp: TimeMsg) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _ns_to_time_msg(timestamp_ns: int) -> TimeMsg:
    time_message: TimeMsg = TimeMsg()
    time_message.sec = int(timestamp_ns // 1_000_000_000)
    time_message.nanosec = int(timestamp_ns % 1_000_000_000)
    return time_message


def _yaw_from_quaternion_xyzw(quaternion_xyzw: Quaternion) -> float:
    x_value, y_value, z_value, w_value = quaternion_xyzw
    return math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value),
    )


def _quaternion_from_yaw_rad(yaw_rad: float) -> Quaternion:
    half_yaw_rad: float = 0.5 * yaw_rad
    return (
        0.0,
        0.0,
        math.sin(half_yaw_rad),
        math.cos(half_yaw_rad),
    )
