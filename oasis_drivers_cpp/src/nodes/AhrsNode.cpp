/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "AhrsNode.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

using namespace OASIS::ROS;
using namespace OASIS::AHRS;

namespace
{
// Default node name
constexpr const char* kNodeName = "ahrs_node";

// ROS topics
constexpr const char* kGravityTopic = "gravity";
constexpr const char* kImuGravityTopic = "imu_gravity";
constexpr const char* kImuTopic = "imu";
constexpr const char* kOutputDiagTopic = "ahrs/diag";
constexpr const char* kOutputGravityTopic = "ahrs/gravity";
constexpr const char* kOutputImuGravityTopic = "ahrs/imu_gravity";
constexpr const char* kOutputImuTopic = "ahrs/imu";
constexpr const char* kOutputOdomTopic = "ahrs/odom";

// ROS frame IDs
constexpr const char* kBaseFrameIdDefault = "base_link";
constexpr const char* kImuFrameIdDefault = "imu_link";
constexpr const char* kOdomFrameIdDefault = "odom";
constexpr const char* kWorldFrameIdDefault = "world";

// QoS parameters
constexpr std::size_t kRawImuGravityQosDepth = 256;
constexpr std::size_t kRawGravityQosDepth = 256;
constexpr std::size_t kRawImuQosDepth = 512;
constexpr std::size_t kAhrsImuGravityQosDepth = 512;
constexpr std::size_t kAhrsImuQosDepth = 512;
constexpr std::size_t kAhrsGravityQosDepth = 256;
constexpr std::size_t kAhrsDiagQosDepth = 10;
constexpr std::size_t kAhrsOdomQosDepth = 10;

// AHRS constants
constexpr double kStatusTimerPeriodSec = 0.1;
constexpr double kMaxGravityCovarianceAgeSec = 0.2;
constexpr double kGravityResidualRejectThresholdDefault = 0.35;
constexpr double kGravityMahalanobisRejectThresholdDefault = 5.0;
constexpr double kMountingCalibrationDurationSecDefault = 2.0;
constexpr double kMountingStationaryAngularSpeedThresholdRadsDefault = 0.35;
constexpr int kMountingMinSampleCountDefault = 10;

int64_t StampToNs(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
}

builtin_interfaces::msg::Time NsToStamp(int64_t timestamp_ns)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(timestamp_ns / 1'000'000'000LL);
  stamp.nanosec = static_cast<std::uint32_t>(timestamp_ns % 1'000'000'000LL);
  return stamp;
}

Eigen::Quaterniond QuaternionFromMsg(const geometry_msgs::msg::Quaternion& quaternion)
{
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
}

Eigen::Vector3d VectorFromMsg(const geometry_msgs::msg::Vector3& vector)
{
  return Eigen::Vector3d(vector.x, vector.y, vector.z);
}

std::array<double, 9> Array9FromRos(const std::array<double, 9>& values)
{
  return values;
}

std::array<double, 36> Array36FromRos(const std::array<double, 36>& values)
{
  return values;
}

bool FrameMatches(const std::string& frame_id, const std::string& expected_frame_id)
{
  return frame_id == expected_frame_id;
}

void CopyMatrix3ToArray(const Eigen::Matrix3d& matrix, std::array<double, 9>& output)
{
  const std::array<double, 9> values = FlattenMatrix3(matrix);
  std::copy(values.begin(), values.end(), output.begin());
}

void CopyCovariance36ToArray(const std::array<double, 36>& values, std::array<double, 36>& output)
{
  std::copy(values.begin(), values.end(), output.begin());
}

bool IsStale(std::optional<int64_t> last_timestamp_ns, int64_t timestamp_ns)
{
  return last_timestamp_ns.has_value() && timestamp_ns < *last_timestamp_ns;
}
} // namespace

AhrsNode::AhrsNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node(kNodeName, options),
    m_baseFrameId(kBaseFrameIdDefault),
    m_imuFrameId(kImuFrameIdDefault),
    m_odomFrameId(kOdomFrameIdDefault),
    m_worldFrameId(kWorldFrameIdDefault),
    m_gravityResidualRejectThreshold(kGravityResidualRejectThresholdDefault),
    m_gravityMahalanobisRejectThreshold(kGravityMahalanobisRejectThresholdDefault),
    m_mountingCalibrator(AhrsMountingConfig{})
{
  declare_parameter("base_frame_id", m_baseFrameId);
  declare_parameter("imu_frame_id", m_imuFrameId);
  declare_parameter("odom_frame_id", m_odomFrameId);
  declare_parameter("world_frame_id", m_worldFrameId);
  declare_parameter("gravity_residual_reject_threshold", kGravityResidualRejectThresholdDefault);
  declare_parameter("gravity_mahalanobis_reject_threshold",
                    kGravityMahalanobisRejectThresholdDefault);
  declare_parameter("mounting_calibration_duration_sec", kMountingCalibrationDurationSecDefault);
  declare_parameter("mounting_stationary_angular_speed_threshold_rads",
                    kMountingStationaryAngularSpeedThresholdRadsDefault);
  declare_parameter("mounting_min_sample_count", kMountingMinSampleCountDefault);

  m_baseFrameId = get_parameter("base_frame_id").as_string();
  m_imuFrameId = get_parameter("imu_frame_id").as_string();
  m_odomFrameId = get_parameter("odom_frame_id").as_string();
  m_worldFrameId = get_parameter("world_frame_id").as_string();
  m_gravityResidualRejectThreshold = get_parameter("gravity_residual_reject_threshold").as_double();
  m_gravityMahalanobisRejectThreshold =
      get_parameter("gravity_mahalanobis_reject_threshold").as_double();

  AhrsMountingConfig mounting_config;
  mounting_config.parent_frame_id = m_baseFrameId;
  mounting_config.child_frame_id = m_imuFrameId;
  mounting_config.calibration_duration_sec =
      get_parameter("mounting_calibration_duration_sec").as_double();
  mounting_config.stationary_angular_speed_threshold_rads =
      get_parameter("mounting_stationary_angular_speed_threshold_rads").as_double();
  mounting_config.min_sample_count =
      static_cast<int>(get_parameter("mounting_min_sample_count").as_int());
  m_mountingCalibrator = BootMountingCalibrator(mounting_config);

  m_diagPublisher = create_publisher<oasis_msgs::msg::AhrsStatus>(
      kOutputDiagTopic, ReliableSensorQos(kAhrsDiagQosDepth));
  m_gravityPublisher = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      kOutputGravityTopic, ReliableSensorQos(kAhrsGravityQosDepth));
  m_imuGravityPublisher = create_publisher<sensor_msgs::msg::Imu>(
      kOutputImuGravityTopic, ReliableSensorQos(kAhrsImuGravityQosDepth));
  m_imuPublisher =
      create_publisher<sensor_msgs::msg::Imu>(kOutputImuTopic, ReliableSensorQos(kAhrsImuQosDepth));
  m_odomPublisher = create_publisher<nav_msgs::msg::Odometry>(
      kOutputOdomTopic, BestEffortSensorQos(kAhrsOdomQosDepth));

  m_gravitySubscription = create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
      kGravityTopic, ReliableSensorQos(kRawGravityQosDepth),
      [this](const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr message)
      { HandleGravity(*message); });
  m_imuGravitySubscription = create_subscription<sensor_msgs::msg::Imu>(
      kImuGravityTopic, ReliableSensorQos(kRawImuGravityQosDepth),
      [this](const sensor_msgs::msg::Imu::SharedPtr message) { HandleImuGravity(*message); });
  m_imuSubscription = create_subscription<sensor_msgs::msg::Imu>(
      kImuTopic, ReliableSensorQos(kRawImuQosDepth),
      [this](const sensor_msgs::msg::Imu::SharedPtr message) { HandleImu(*message); });

  m_tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  m_statusTimer = create_wall_timer(std::chrono::duration<double>(kStatusTimerPeriodSec),
                                    [this]() { PublishRuntimeOutputs(); });

  PublishRuntimeOutputs();
  RCLCPP_INFO(get_logger(), "AHRS component initialized");
}

rclcpp::QoS AhrsNode::ReliableSensorQos(std::size_t depth)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();
}

rclcpp::QoS AhrsNode::BestEffortSensorQos(std::size_t depth)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort().durability_volatile();
}

void AhrsNode::HandleGravity(const geometry_msgs::msg::AccelWithCovarianceStamped& message)
{
  std::string rejection_reason;
  const std::optional<GravitySample> sample = ValidateGravityMessage(message, rejection_reason);
  if (!sample.has_value())
  {
    ++m_diag.rejected_gravity_count;
    if (rejection_reason == "bad_frame")
      m_diag.last_bad_gravity_frame_id = message.header.frame_id;
    PublishRuntimeOutputs();
    return;
  }

  if (IsStale(m_diag.last_accepted_gravity_timestamp_ns, sample->timestamp_ns))
  {
    ++m_diag.dropped_stale_gravity_count;
    m_diag.last_bad_gravity_frame_id.clear();
    PublishRuntimeOutputs();
    return;
  }

  m_latestGravitySample = sample;
  ++m_diag.accepted_gravity_count;
  m_diag.has_gravity = true;
  m_diag.last_bad_gravity_frame_id.clear();
  m_diag.last_accepted_gravity_timestamp_ns = sample->timestamp_ns;
  UpdateMountingCalibration(*sample);

  if (ResolveMounting().has_value())
    m_gravityPublisher->publish(BuildGravityMessage(MountGravitySample(*sample)));

  PublishRuntimeOutputs();
}

void AhrsNode::HandleImuGravity(const sensor_msgs::msg::Imu& message)
{
  std::string rejection_reason;
  const std::optional<ImuSample> sample = ValidateImuMessage(message, rejection_reason);
  if (!sample.has_value())
  {
    PublishRuntimeOutputs();
    return;
  }

  if (IsStale(m_lastAcceptedImuGravityTimestampNs, sample->timestamp_ns))
  {
    PublishRuntimeOutputs();
    return;
  }

  m_lastAcceptedImuGravityTimestampNs = sample->timestamp_ns;

  if (!ResolveMounting().has_value())
  {
    PublishRuntimeOutputs();
    return;
  }

  m_imuGravityPublisher->publish(BuildImuMessage(MountImuSample(*sample)));
  PublishRuntimeOutputs();
}

void AhrsNode::HandleImu(const sensor_msgs::msg::Imu& message)
{
  std::string rejection_reason;
  const std::optional<ImuSample> sample = ValidateImuMessage(message, rejection_reason);
  if (!sample.has_value())
  {
    ++m_diag.rejected_imu_count;
    if (rejection_reason == "bad_frame")
      m_diag.last_bad_imu_frame_id = message.header.frame_id;
    PublishRuntimeOutputs();
    return;
  }

  if (IsStale(m_diag.last_accepted_imu_timestamp_ns, sample->timestamp_ns))
  {
    ++m_diag.dropped_stale_imu_count;
    m_diag.last_bad_imu_frame_id.clear();
    PublishRuntimeOutputs();
    return;
  }

  ++m_diag.accepted_imu_count;
  m_diag.last_bad_imu_frame_id.clear();
  m_diag.last_accepted_imu_timestamp_ns = sample->timestamp_ns;
  m_latestImuAngularVelocityRads = sample->angular_velocity_rads;

  if (!ResolveMounting().has_value())
  {
    m_latestOutput.reset();
    PublishRuntimeOutputs();
    return;
  }

  MountedImuSample mounted_sample = MountImuSample(*sample);
  const std::optional<MountedGravitySample> gravity_sample =
      FreshMountedGravitySample(mounted_sample.timestamp_ns);

  if (m_latestGravitySample.has_value())
  {
    const MountedGravitySample mounted_gravity = MountGravitySample(*m_latestGravitySample);
    const std::optional<AhrsGravityResidual> residual =
        ComputeGravityResidual(mounted_gravity.gravity_mps2,
                               mounted_gravity.gravity_covariance_mps2_2, mounted_sample.q_WB);
    m_diag.last_gravity_residual = residual;
    m_diag.gravity_gated_in =
        residual.has_value() && residual->residual_norm <= m_gravityResidualRejectThreshold;
    m_diag.gravity_rejected = residual.has_value() && !m_diag.gravity_gated_in;
    m_diag.last_gravity_rejection_reason =
        m_diag.gravity_rejected ? std::string("residual_norm") : std::string();
    if (m_diag.gravity_rejected)
      ++m_diag.gravity_rejection_count;
  }
  else
  {
    m_diag.gravity_gated_in = false;
    m_diag.gravity_rejected = false;
    m_diag.last_gravity_rejection_reason.clear();
    m_diag.last_gravity_residual.reset();
  }

  m_latestOutput = ApplySessionYawZero(mounted_sample, gravity_sample);
  m_imuPublisher->publish(BuildImuMessage(*m_latestOutput));
  m_odomPublisher->publish(BuildOdomMessage(*m_latestOutput));
  PublishRuntimeOutputs();
}

void AhrsNode::PublishRuntimeOutputs()
{
  PublishTf();
  PublishStatus();
}

void AhrsNode::PublishTf()
{
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.push_back(BuildIdentityTransform());

  if (m_mounting.has_value())
    transforms.push_back(BuildMountingTransform());

  if (m_latestOutput.has_value())
    transforms.push_back(BuildOdomToBaseTransform());

  m_tfBroadcaster->sendTransform(transforms);
}

void AhrsNode::PublishStatus()
{
  oasis_msgs::msg::AhrsStatus status;
  status.header.stamp = CurrentStamp();
  status.header.frame_id = m_worldFrameId;
  status.status = ComputeStatusCode();
  status.accepted_imu_count = m_diag.accepted_imu_count;
  status.accepted_gravity_count = m_diag.accepted_gravity_count;
  status.rejected_imu_count = m_diag.rejected_imu_count;
  status.rejected_gravity_count = m_diag.rejected_gravity_count;
  status.dropped_stale_imu_count = m_diag.dropped_stale_imu_count;
  status.dropped_stale_gravity_count = m_diag.dropped_stale_gravity_count;
  status.gravity_rejection_count = m_diag.gravity_rejection_count;
  status.gravity_residual_norm = std::numeric_limits<double>::quiet_NaN();
  status.gravity_mahalanobis_distance = std::numeric_limits<double>::quiet_NaN();
  if (m_diag.last_gravity_residual.has_value())
  {
    status.gravity_residual_norm = m_diag.last_gravity_residual->residual_norm;
    status.gravity_mahalanobis_distance = m_diag.last_gravity_residual->mahalanobis_distance;
  }
  status.has_gravity = m_diag.has_gravity;
  status.has_mounting = m_diag.has_mounting;
  status.gravity_gated_in = m_diag.gravity_gated_in;
  status.gravity_rejected = m_diag.gravity_rejected;
  status.last_mounting_lookup_error = m_diag.last_mounting_lookup_error;
  status.last_gravity_rejection_reason = m_diag.last_gravity_rejection_reason;
  status.status_text = ComputeStatusText();
  m_diagPublisher->publish(status);
}

void AhrsNode::UpdateMountingCalibration(const GravitySample& gravity_sample)
{
  if (m_mounting.has_value())
    return;

  m_mounting = m_mountingCalibrator.AddGravitySample(
      gravity_sample.timestamp_ns, gravity_sample.gravity_mps2, m_latestImuAngularVelocityRads);
  if (!m_mounting.has_value())
  {
    m_diag.has_mounting = false;
    m_diag.last_mounting_lookup_error = "boot mounting calibration in progress";
    return;
  }

  m_diag.has_mounting = true;
  m_diag.last_mounting_lookup_error.clear();
  RCLCPP_INFO(get_logger(), "AHRS mounting solved: roll=%.4f pitch=%.4f yaw=0 samples=%d span=%.3f",
              m_mounting->roll_rad, m_mounting->pitch_rad, m_mounting->sample_count,
              m_mounting->span_sec);
}

std::optional<AhrsMountingSolution> AhrsNode::ResolveMounting()
{
  if (!m_mounting.has_value())
  {
    m_diag.has_mounting = false;
    if (m_diag.accepted_gravity_count == 0)
      m_diag.last_mounting_lookup_error = "waiting for boot mounting calibration";
    else
      m_diag.last_mounting_lookup_error = "boot mounting calibration in progress";
    return std::nullopt;
  }

  m_diag.has_mounting = true;
  m_diag.last_mounting_lookup_error.clear();
  return m_mounting;
}

std::optional<AhrsNode::MountedGravitySample> AhrsNode::FreshMountedGravitySample(
    int64_t imu_timestamp_ns) const
{
  if (!m_latestGravitySample.has_value() || !m_mounting.has_value())
    return std::nullopt;

  const int64_t age_ns = imu_timestamp_ns - m_latestGravitySample->timestamp_ns;
  const int64_t max_age_ns = static_cast<int64_t>(kMaxGravityCovarianceAgeSec * 1.0e9);
  if (age_ns < 0 || age_ns > max_age_ns)
    return std::nullopt;

  return MountGravitySample(*m_latestGravitySample);
}

AhrsNode::MountedImuSample AhrsNode::MountImuSample(const ImuSample& sample) const
{
  MountedImuSample mounted;
  mounted.timestamp_ns = sample.timestamp_ns;
  mounted.q_WB = m_mounting->q_BI * sample.q_WI;
  mounted.orientation_covariance_unknown = sample.orientation_covariance_unknown;
  if (sample.orientation_covariance_rad2.has_value() && !sample.orientation_covariance_unknown)
    mounted.orientation_covariance_rad2 =
        RotateCovariance(m_mounting->R_BI, *sample.orientation_covariance_rad2);
  mounted.angular_velocity_rads = m_mounting->R_BI * sample.angular_velocity_rads;
  mounted.angular_velocity_covariance_rads2 =
      RotateCovariance(m_mounting->R_BI, sample.angular_velocity_covariance_rads2);
  mounted.linear_acceleration_mps2 = m_mounting->R_BI * sample.linear_acceleration_mps2;
  mounted.linear_acceleration_covariance_mps2_2 =
      RotateCovariance(m_mounting->R_BI, sample.linear_acceleration_covariance_mps2_2);
  return mounted;
}

AhrsNode::MountedGravitySample AhrsNode::MountGravitySample(const GravitySample& sample) const
{
  MountedGravitySample mounted;
  mounted.timestamp_ns = sample.timestamp_ns;
  mounted.gravity_mps2 = m_mounting->R_BI * sample.gravity_mps2;
  if (sample.gravity_covariance_mps2_2.has_value())
    mounted.gravity_covariance_mps2_2 =
        RotateCovariance(m_mounting->R_BI, *sample.gravity_covariance_mps2_2);
  return mounted;
}

AhrsNode::MountedImuSample AhrsNode::ApplySessionYawZero(
    const MountedImuSample& sample, const std::optional<MountedGravitySample>& gravity_sample)
{
  MountedImuSample output = sample;
  const double mounted_yaw_rad = YawFromQuaternion(sample.q_WB);
  if (!m_sessionYawZeroInitialized)
  {
    m_sessionYawOffset = QuaternionFromYaw(-mounted_yaw_rad);
    m_sessionYawZeroInitialized = true;
    RCLCPP_INFO(get_logger(), "AHRS session yaw initialized: mounted_yaw_rad=%.4f",
                mounted_yaw_rad);
  }

  const std::optional<Eigen::Quaterniond> normalized =
      NormalizeQuaternion(m_sessionYawOffset * sample.q_WB);
  if (normalized.has_value())
    output.q_WB = *normalized;

  std::optional<Eigen::Matrix3d> gravity_covariance;
  Eigen::Vector3d gravity_base(0.0, 0.0, 0.0);
  if (gravity_sample.has_value())
  {
    gravity_base = gravity_sample->gravity_mps2;
    gravity_covariance = gravity_sample->gravity_covariance_mps2_2;
  }

  output.orientation_covariance_rad2 =
      GravityCovarianceToRollPitchCovariance(gravity_base, gravity_covariance, 0.0);
  output.orientation_covariance_unknown = !output.orientation_covariance_rad2.has_value();
  return output;
}

std::optional<AhrsNode::ImuSample> AhrsNode::ValidateImuMessage(
    const sensor_msgs::msg::Imu& message, std::string& rejection_reason) const
{
  rejection_reason.clear();
  if (!FrameMatches(message.header.frame_id, m_imuFrameId))
  {
    rejection_reason = "bad_frame";
    return std::nullopt;
  }

  const std::optional<Eigen::Quaterniond> normalized_driver_quaternion =
      NormalizeQuaternion(QuaternionFromMsg(message.orientation));
  if (!normalized_driver_quaternion.has_value())
  {
    rejection_reason = "bad_orientation";
    return std::nullopt;
  }

  ImuSample sample;
  sample.timestamp_ns = StampToNs(message.header.stamp);
  sample.q_WI = ConjugateQuaternion(*normalized_driver_quaternion);

  const std::array<double, 9> orientation_covariance =
      Array9FromRos(message.orientation_covariance);
  sample.orientation_covariance_unknown = orientation_covariance[0] == -1.0;
  if (!sample.orientation_covariance_unknown)
  {
    sample.orientation_covariance_rad2 = ParseMatrix3(orientation_covariance);
    if (!sample.orientation_covariance_rad2.has_value())
    {
      rejection_reason = "bad_orientation_covariance";
      return std::nullopt;
    }
  }

  sample.angular_velocity_rads = VectorFromMsg(message.angular_velocity);
  if (!IsFiniteVector3(sample.angular_velocity_rads))
  {
    rejection_reason = "bad_angular_velocity";
    return std::nullopt;
  }

  sample.linear_acceleration_mps2 = VectorFromMsg(message.linear_acceleration);
  if (!IsFiniteVector3(sample.linear_acceleration_mps2))
  {
    rejection_reason = "bad_linear_acceleration";
    return std::nullopt;
  }

  const std::optional<Eigen::Matrix3d> angular_covariance =
      ParseMatrix3(Array9FromRos(message.angular_velocity_covariance));
  if (!angular_covariance.has_value())
  {
    rejection_reason = "bad_angular_covariance";
    return std::nullopt;
  }
  sample.angular_velocity_covariance_rads2 = *angular_covariance;

  const std::optional<Eigen::Matrix3d> linear_covariance =
      ParseMatrix3(Array9FromRos(message.linear_acceleration_covariance));
  if (!linear_covariance.has_value())
  {
    rejection_reason = "bad_linear_covariance";
    return std::nullopt;
  }
  sample.linear_acceleration_covariance_mps2_2 = *linear_covariance;

  return sample;
}

std::optional<AhrsNode::GravitySample> AhrsNode::ValidateGravityMessage(
    const geometry_msgs::msg::AccelWithCovarianceStamped& message,
    std::string& rejection_reason) const
{
  rejection_reason.clear();
  if (!FrameMatches(message.header.frame_id, m_imuFrameId))
  {
    rejection_reason = "bad_frame";
    return std::nullopt;
  }

  GravitySample sample;
  sample.timestamp_ns = StampToNs(message.header.stamp);
  sample.gravity_mps2 = VectorFromMsg(message.accel.accel.linear);
  if (!IsFiniteVector3(sample.gravity_mps2) || sample.gravity_mps2.norm() <= 1.0e-9)
  {
    rejection_reason = "bad_vector";
    return std::nullopt;
  }

  sample.gravity_covariance_mps2_2 =
      ParseLinearCovariance3(Array36FromRos(message.accel.covariance));
  return sample;
}

sensor_msgs::msg::Imu AhrsNode::BuildImuMessage(const MountedImuSample& sample) const
{
  sensor_msgs::msg::Imu message;
  message.header.stamp = NsToStamp(sample.timestamp_ns);
  message.header.frame_id = m_baseFrameId;
  message.orientation.x = sample.q_WB.x();
  message.orientation.y = sample.q_WB.y();
  message.orientation.z = sample.q_WB.z();
  message.orientation.w = sample.q_WB.w();
  if (sample.orientation_covariance_unknown)
  {
    message.orientation_covariance[0] = -1.0;
  }
  else if (sample.orientation_covariance_rad2.has_value())
  {
    CopyMatrix3ToArray(*sample.orientation_covariance_rad2, message.orientation_covariance);
  }
  message.angular_velocity.x = sample.angular_velocity_rads.x();
  message.angular_velocity.y = sample.angular_velocity_rads.y();
  message.angular_velocity.z = sample.angular_velocity_rads.z();
  CopyMatrix3ToArray(sample.angular_velocity_covariance_rads2, message.angular_velocity_covariance);
  message.linear_acceleration.x = sample.linear_acceleration_mps2.x();
  message.linear_acceleration.y = sample.linear_acceleration_mps2.y();
  message.linear_acceleration.z = sample.linear_acceleration_mps2.z();
  CopyMatrix3ToArray(sample.linear_acceleration_covariance_mps2_2,
                     message.linear_acceleration_covariance);
  return message;
}

geometry_msgs::msg::AccelWithCovarianceStamped AhrsNode::BuildGravityMessage(
    const MountedGravitySample& sample) const
{
  geometry_msgs::msg::AccelWithCovarianceStamped message;
  message.header.stamp = NsToStamp(sample.timestamp_ns);
  message.header.frame_id = m_baseFrameId;
  message.accel.accel.linear.x = sample.gravity_mps2.x();
  message.accel.accel.linear.y = sample.gravity_mps2.y();
  message.accel.accel.linear.z = sample.gravity_mps2.z();
  if (sample.gravity_covariance_mps2_2.has_value())
  {
    const std::array<double, 36> covariance =
        EmbedLinearCovariance3(*sample.gravity_covariance_mps2_2);
    CopyCovariance36ToArray(covariance, message.accel.covariance);
  }
  message.accel.covariance[21] = -1.0;
  return message;
}

nav_msgs::msg::Odometry AhrsNode::BuildOdomMessage(const MountedImuSample& sample) const
{
  nav_msgs::msg::Odometry message;
  message.header.stamp = NsToStamp(sample.timestamp_ns);
  message.header.frame_id = m_odomFrameId;
  message.child_frame_id = m_baseFrameId;
  message.pose.pose.orientation.x = sample.q_WB.x();
  message.pose.pose.orientation.y = sample.q_WB.y();
  message.pose.pose.orientation.z = sample.q_WB.z();
  message.pose.pose.orientation.w = sample.q_WB.w();
  if (sample.orientation_covariance_rad2.has_value())
  {
    message.pose.covariance[21] = (*sample.orientation_covariance_rad2)(0, 0);
    message.pose.covariance[22] = (*sample.orientation_covariance_rad2)(0, 1);
    message.pose.covariance[23] = (*sample.orientation_covariance_rad2)(0, 2);
    message.pose.covariance[27] = (*sample.orientation_covariance_rad2)(1, 0);
    message.pose.covariance[28] = (*sample.orientation_covariance_rad2)(1, 1);
    message.pose.covariance[29] = (*sample.orientation_covariance_rad2)(1, 2);
    message.pose.covariance[33] = (*sample.orientation_covariance_rad2)(2, 0);
    message.pose.covariance[34] = (*sample.orientation_covariance_rad2)(2, 1);
    message.pose.covariance[35] = (*sample.orientation_covariance_rad2)(2, 2);
  }
  message.twist.twist.angular.x = sample.angular_velocity_rads.x();
  message.twist.twist.angular.y = sample.angular_velocity_rads.y();
  message.twist.twist.angular.z = sample.angular_velocity_rads.z();
  const std::array<double, 36> angular_covariance =
      EmbedLinearCovariance3(sample.angular_velocity_covariance_rads2);
  message.twist.covariance[21] = angular_covariance[0];
  message.twist.covariance[22] = angular_covariance[1];
  message.twist.covariance[23] = angular_covariance[2];
  message.twist.covariance[27] = angular_covariance[6];
  message.twist.covariance[28] = angular_covariance[7];
  message.twist.covariance[29] = angular_covariance[8];
  message.twist.covariance[33] = angular_covariance[12];
  message.twist.covariance[34] = angular_covariance[13];
  message.twist.covariance[35] = angular_covariance[14];
  return message;
}

geometry_msgs::msg::TransformStamped AhrsNode::BuildIdentityTransform() const
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = CurrentStamp();
  transform.header.frame_id = m_worldFrameId;
  transform.child_frame_id = m_odomFrameId;
  transform.transform.rotation.w = 1.0;
  return transform;
}

geometry_msgs::msg::TransformStamped AhrsNode::BuildMountingTransform() const
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = CurrentStamp();
  transform.header.frame_id = m_baseFrameId;
  transform.child_frame_id = m_imuFrameId;
  transform.transform.rotation.x = m_mounting->q_BI.x();
  transform.transform.rotation.y = m_mounting->q_BI.y();
  transform.transform.rotation.z = m_mounting->q_BI.z();
  transform.transform.rotation.w = m_mounting->q_BI.w();
  return transform;
}

geometry_msgs::msg::TransformStamped AhrsNode::BuildOdomToBaseTransform() const
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = NsToStamp(m_latestOutput->timestamp_ns);
  transform.header.frame_id = m_odomFrameId;
  transform.child_frame_id = m_baseFrameId;
  transform.transform.rotation.x = m_latestOutput->q_WB.x();
  transform.transform.rotation.y = m_latestOutput->q_WB.y();
  transform.transform.rotation.z = m_latestOutput->q_WB.z();
  transform.transform.rotation.w = m_latestOutput->q_WB.w();
  return transform;
}

std::uint8_t AhrsNode::ComputeStatusCode() const
{
  if (!m_diag.last_bad_imu_frame_id.empty())
    return oasis_msgs::msg::AhrsStatus::STATUS_BAD_IMU_FRAME;
  if (!m_diag.last_bad_gravity_frame_id.empty())
    return oasis_msgs::msg::AhrsStatus::STATUS_BAD_GRAVITY_FRAME;
  if (m_diag.accepted_imu_count == 0)
    return oasis_msgs::msg::AhrsStatus::STATUS_WAITING_FOR_IMU;
  if (m_diag.accepted_gravity_count == 0)
    return oasis_msgs::msg::AhrsStatus::STATUS_WAITING_FOR_GRAVITY;
  if (!m_diag.has_mounting)
    return oasis_msgs::msg::AhrsStatus::STATUS_MOUNTING_UNAVAILABLE;
  return oasis_msgs::msg::AhrsStatus::STATUS_OK;
}

std::string AhrsNode::ComputeStatusText() const
{
  if (!m_diag.last_bad_imu_frame_id.empty())
    return "Bad IMU frame";
  if (!m_diag.last_bad_gravity_frame_id.empty())
    return "Bad gravity frame";
  if (m_diag.accepted_imu_count == 0)
    return "Waiting for IMU samples";
  if (m_diag.accepted_gravity_count == 0)
    return "Waiting for gravity samples";
  if (!m_diag.has_mounting)
    return "Mounting calibration not solved";
  if (m_diag.gravity_rejected)
    return "Gravity consistency rejected";
  return "Mounted attitude output available";
}

builtin_interfaces::msg::Time AhrsNode::CurrentStamp() const
{
  const int64_t now_ns = get_clock()->now().nanoseconds();
  return NsToStamp(now_ns);
}

RCLCPP_COMPONENTS_REGISTER_NODE(OASIS::ROS::AhrsNode)
