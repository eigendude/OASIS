/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "ahrs/AhrsMath.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <oasis_msgs/msg/ahrs_status.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace OASIS::ROS
{
/*!
 * \brief Mounted AHRS component for BNO086-derived attitude streams
 */
class AhrsNode : public rclcpp::Node
{
public:
  explicit AhrsNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~AhrsNode() override = default;

private:
  struct ImuSample
  {
    int64_t timestamp_ns{0};
    Eigen::Quaterniond q_WI{Eigen::Quaterniond::Identity()};
    std::optional<Eigen::Matrix3d> orientation_covariance_rad2;
    bool orientation_covariance_unknown{false};
    Eigen::Vector3d angular_velocity_rads{0.0, 0.0, 0.0};
    Eigen::Matrix3d angular_velocity_covariance_rads2{Eigen::Matrix3d::Zero()};
    Eigen::Vector3d linear_acceleration_mps2{0.0, 0.0, 0.0};
    Eigen::Matrix3d linear_acceleration_covariance_mps2_2{Eigen::Matrix3d::Zero()};
  };

  struct GravitySample
  {
    int64_t timestamp_ns{0};
    Eigen::Vector3d gravity_mps2{0.0, 0.0, -9.81};
    std::optional<Eigen::Matrix3d> gravity_covariance_mps2_2;
  };

  struct MountedImuSample
  {
    int64_t timestamp_ns{0};
    Eigen::Quaterniond q_WB{Eigen::Quaterniond::Identity()};
    std::optional<Eigen::Matrix3d> orientation_covariance_rad2;
    bool orientation_covariance_unknown{false};
    Eigen::Vector3d angular_velocity_rads{0.0, 0.0, 0.0};
    Eigen::Matrix3d angular_velocity_covariance_rads2{Eigen::Matrix3d::Zero()};
    Eigen::Vector3d linear_acceleration_mps2{0.0, 0.0, 0.0};
    Eigen::Matrix3d linear_acceleration_covariance_mps2_2{Eigen::Matrix3d::Zero()};
  };

  struct MountedGravitySample
  {
    int64_t timestamp_ns{0};
    Eigen::Vector3d gravity_mps2{0.0, 0.0, -9.81};
    std::optional<Eigen::Matrix3d> gravity_covariance_mps2_2;
  };

  struct Diagnostics
  {
    std::uint32_t accepted_imu_count{0};
    std::uint32_t accepted_gravity_count{0};
    std::uint32_t rejected_imu_count{0};
    std::uint32_t rejected_gravity_count{0};
    std::uint32_t dropped_stale_imu_count{0};
    std::uint32_t dropped_stale_gravity_count{0};
    std::uint32_t gravity_rejection_count{0};
    bool has_gravity{false};
    bool has_mounting{false};
    bool gravity_gated_in{false};
    bool gravity_rejected{false};
    std::string last_bad_imu_frame_id;
    std::string last_bad_gravity_frame_id;
    std::optional<int64_t> last_accepted_imu_timestamp_ns;
    std::optional<int64_t> last_accepted_gravity_timestamp_ns;
    std::string last_mounting_lookup_error;
    std::string last_gravity_rejection_reason;
    std::optional<OASIS::AHRS::AhrsGravityResidual> last_gravity_residual;
  };

  static rclcpp::QoS ReliableSensorQos(std::size_t depth);
  static rclcpp::QoS BestEffortSensorQos(std::size_t depth);

  void HandleGravity(const geometry_msgs::msg::AccelWithCovarianceStamped& message);
  void HandleImuGravity(const sensor_msgs::msg::Imu& message);
  void HandleImu(const sensor_msgs::msg::Imu& message);
  void PublishRuntimeOutputs();
  void PublishTf();
  void PublishStatus();
  void UpdateMountingCalibration(const GravitySample& gravity_sample);
  std::optional<OASIS::AHRS::AhrsMountingSolution> ResolveMounting();
  std::optional<MountedGravitySample> FreshMountedGravitySample(int64_t imu_timestamp_ns) const;
  MountedImuSample MountImuSample(const ImuSample& sample) const;
  MountedGravitySample MountGravitySample(const GravitySample& sample) const;
  MountedImuSample ApplySessionYawZero(const MountedImuSample& sample,
                                       const std::optional<MountedGravitySample>& gravity_sample);
  std::optional<ImuSample> ValidateImuMessage(const sensor_msgs::msg::Imu& message,
                                              std::string& rejection_reason) const;
  std::optional<GravitySample> ValidateGravityMessage(
      const geometry_msgs::msg::AccelWithCovarianceStamped& message,
      std::string& rejection_reason) const;
  sensor_msgs::msg::Imu BuildImuMessage(const MountedImuSample& sample) const;
  geometry_msgs::msg::AccelWithCovarianceStamped BuildGravityMessage(
      const MountedGravitySample& sample) const;
  nav_msgs::msg::Odometry BuildOdomMessage(const MountedImuSample& sample) const;
  geometry_msgs::msg::TransformStamped BuildIdentityTransform() const;
  geometry_msgs::msg::TransformStamped BuildMountingTransform() const;
  geometry_msgs::msg::TransformStamped BuildOdomToBaseTransform() const;
  std::uint8_t ComputeStatusCode() const;
  std::string ComputeStatusText() const;
  builtin_interfaces::msg::Time CurrentStamp() const;

  std::string m_baseFrameId;
  std::string m_imuFrameId;
  std::string m_odomFrameId;
  std::string m_worldFrameId;
  double m_gravityResidualRejectThreshold;
  double m_gravityMahalanobisRejectThreshold;
  OASIS::AHRS::BootMountingCalibrator m_mountingCalibrator;
  std::optional<OASIS::AHRS::AhrsMountingSolution> m_mounting;
  Diagnostics m_diag;
  std::optional<int64_t> m_lastAcceptedImuGravityTimestampNs;
  std::optional<GravitySample> m_latestGravitySample;
  std::optional<MountedImuSample> m_latestOutput;
  std::optional<Eigen::Vector3d> m_latestImuAngularVelocityRads;
  bool m_sessionYawZeroInitialized{false};
  Eigen::Quaterniond m_sessionYawOffset{Eigen::Quaterniond::Identity()};

  // ROS publishers
  rclcpp::Publisher<oasis_msgs::msg::AhrsStatus>::SharedPtr m_diagPublisher;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr m_gravityPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuGravityPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuPublisher;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odomPublisher;

  // ROS subscribers
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr
      m_gravitySubscription;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuGravitySubscription;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSubscription;

  // TF2 broadcasters
  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster;

  // Timing parameters
  rclcpp::TimerBase::SharedPtr m_statusTimer;
};
} // namespace OASIS::ROS
