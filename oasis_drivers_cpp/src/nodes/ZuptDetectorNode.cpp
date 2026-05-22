/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "ZuptDetectorNode.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using OASIS::Localization::Vector3;
using OASIS::Localization::ZuptDecision;
using OASIS::Localization::ZuptDetectorConfig;
using OASIS::ROS::ZuptDetectorNode;

namespace
{
constexpr const char* kNodeName = "zupt_detector";
constexpr const char* kImuTopic = "imu";
constexpr const char* kZuptFlagTopic = "zupt_flag";
constexpr const char* kZuptTopic = "zupt";

constexpr const char* kParamGyroEnterThresholdRads = "gyro_enter_threshold_rads";
constexpr const char* kParamGyroExitThresholdRads = "gyro_exit_threshold_rads";
constexpr const char* kParamAccelEnterThresholdMps2 = "accel_enter_threshold_mps2";
constexpr const char* kParamAccelExitThresholdMps2 = "accel_exit_threshold_mps2";
constexpr const char* kParamMinStationarySec = "min_stationary_sec";
constexpr const char* kParamMinMovingSec = "min_moving_sec";
constexpr const char* kParamStationaryLinearVelocitySigmaMps =
    "stationary_linear_velocity_sigma_mps";
constexpr const char* kParamStationaryAngularVelocitySigmaRads =
    "stationary_angular_velocity_sigma_rads";
constexpr const char* kParamMovingLinearVarianceMps2 = "moving_linear_variance_mps2";
constexpr const char* kParamMovingAngularVarianceRads2 = "moving_angular_variance_rads2";

constexpr std::array<std::size_t, 3> kLinearCovarianceDiagonalIndices{{0, 7, 14}};
constexpr std::array<std::size_t, 3> kAngularCovarianceDiagonalIndices{{21, 28, 35}};

double StampToSec(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

double SanitizePositiveVariance(double variance, double default_variance)
{
  if (!std::isfinite(variance) || variance <= 0.0)
  {
    return default_variance;
  }

  return variance;
}
} // namespace

ZuptDetectorNode::ZuptDetectorNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node(kNodeName, options), m_config(ReadDetectorConfig()), m_detector(m_config)
{
  const rclcpp::SensorDataQoS sensor_qos;

  m_zuptFlagPublisher = create_publisher<std_msgs::msg::Bool>(kZuptFlagTopic, sensor_qos);
  m_zuptPublisher =
      create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(kZuptTopic, sensor_qos);
  m_imuSubscription = create_subscription<sensor_msgs::msg::Imu>(
      kImuTopic, sensor_qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr message) { HandleImu(*message); });

  RCLCPP_INFO(get_logger(),
              "ZUPT detector component initialized "
              "(gyro enter=%.3f rad/s, accel enter=%.3f m/s^2, "
              "stationary linear sigma=%.3f m/s)",
              m_config.gyro_enter_threshold_rads, m_config.accel_enter_threshold_mps2,
              m_config.stationary_linear_velocity_sigma_mps);
}

void ZuptDetectorNode::HandleImu(const sensor_msgs::msg::Imu& message)
{
  const Vector3 angular_velocity_rads{
      message.angular_velocity.x,
      message.angular_velocity.y,
      message.angular_velocity.z,
  };
  const Vector3 linear_accel_mps2{
      message.linear_acceleration.x,
      message.linear_acceleration.y,
      message.linear_acceleration.z,
  };

  const std::optional<ZuptDecision> decision =
      m_detector.Update(StampToSec(message.header.stamp), angular_velocity_rads, linear_accel_mps2);
  if (!decision.has_value())
  {
    return;
  }

  RCLCPP_DEBUG(get_logger(),
               "ZUPT stationary=%s reason=%s gyro_norm=%.4f accel_norm=%.4f "
               "linear_var=%.6f angular_var=%.6f enter_dwell=%.3f "
               "exit_dwell=%.3f",
               decision->stationary ? "true" : "false", decision->reason.c_str(),
               decision->gyro_norm_rads, decision->accel_norm_mps2,
               decision->linear_zupt_variance_mps2, decision->angular_zupt_variance_rads2,
               decision->enter_dwell_sec, decision->exit_dwell_sec);

  std_msgs::msg::Bool flag_message;
  flag_message.data = decision->stationary;
  m_zuptFlagPublisher->publish(flag_message);
  m_zuptPublisher->publish(BuildZuptMessage(message, *decision));
}

ZuptDetectorConfig ZuptDetectorNode::ReadDetectorConfig()
{
  ZuptDetectorConfig defaults;
  declare_parameter(kParamGyroEnterThresholdRads, defaults.gyro_enter_threshold_rads);
  declare_parameter(kParamGyroExitThresholdRads, defaults.gyro_exit_threshold_rads);
  declare_parameter(kParamAccelEnterThresholdMps2, defaults.accel_enter_threshold_mps2);
  declare_parameter(kParamAccelExitThresholdMps2, defaults.accel_exit_threshold_mps2);
  declare_parameter(kParamMinStationarySec, defaults.min_stationary_sec);
  declare_parameter(kParamMinMovingSec, defaults.min_moving_sec);
  declare_parameter(kParamStationaryLinearVelocitySigmaMps,
                    defaults.stationary_linear_velocity_sigma_mps);
  declare_parameter(kParamStationaryAngularVelocitySigmaRads,
                    defaults.stationary_angular_velocity_sigma_rads);
  declare_parameter(kParamMovingLinearVarianceMps2, defaults.moving_linear_variance_mps2);
  declare_parameter(kParamMovingAngularVarianceRads2, defaults.moving_angular_variance_rads2);

  ZuptDetectorConfig config;
  config.gyro_enter_threshold_rads = get_parameter(kParamGyroEnterThresholdRads).as_double();
  config.gyro_exit_threshold_rads = get_parameter(kParamGyroExitThresholdRads).as_double();
  config.accel_enter_threshold_mps2 = get_parameter(kParamAccelEnterThresholdMps2).as_double();
  config.accel_exit_threshold_mps2 = get_parameter(kParamAccelExitThresholdMps2).as_double();
  config.min_stationary_sec = get_parameter(kParamMinStationarySec).as_double();
  config.min_moving_sec = get_parameter(kParamMinMovingSec).as_double();
  config.stationary_linear_velocity_sigma_mps =
      get_parameter(kParamStationaryLinearVelocitySigmaMps).as_double();
  config.stationary_angular_velocity_sigma_rads =
      get_parameter(kParamStationaryAngularVelocitySigmaRads).as_double();
  config.moving_linear_variance_mps2 = get_parameter(kParamMovingLinearVarianceMps2).as_double();
  config.moving_angular_variance_rads2 =
      get_parameter(kParamMovingAngularVarianceRads2).as_double();
  return config;
}

geometry_msgs::msg::TwistWithCovarianceStamped ZuptDetectorNode::BuildZuptMessage(
    const sensor_msgs::msg::Imu& message, const ZuptDecision& decision) const
{
  geometry_msgs::msg::TwistWithCovarianceStamped zupt_message;
  zupt_message.header = message.header;
  zupt_message.twist.twist.linear.x = 0.0;
  zupt_message.twist.twist.linear.y = 0.0;
  zupt_message.twist.twist.linear.z = 0.0;
  zupt_message.twist.twist.angular.x = 0.0;
  zupt_message.twist.twist.angular.y = 0.0;
  zupt_message.twist.twist.angular.z = 0.0;
  zupt_message.twist.covariance =
      BuildZuptCovariance(decision.linear_zupt_variance_mps2, decision.angular_zupt_variance_rads2);
  return zupt_message;
}

std::array<double, 36> ZuptDetectorNode::BuildZuptCovariance(double linear_variance_mps2,
                                                             double angular_variance_rads2) const
{
  const ZuptDetectorConfig defaults;
  const double published_linear_variance_mps2 =
      SanitizePositiveVariance(linear_variance_mps2, defaults.moving_linear_variance_mps2);
  const double published_angular_variance_rads2 =
      SanitizePositiveVariance(angular_variance_rads2, defaults.moving_angular_variance_rads2);

  std::array<double, 36> covariance{};
  std::fill(covariance.begin(), covariance.end(), 0.0);

  for (std::size_t index : kLinearCovarianceDiagonalIndices)
  {
    covariance[index] = published_linear_variance_mps2;
  }

  for (std::size_t index : kAngularCovarianceDiagonalIndices)
  {
    covariance[index] = published_angular_variance_rads2;
  }

  return covariance;
}

RCLCPP_COMPONENTS_REGISTER_NODE(OASIS::ROS::ZuptDetectorNode)
