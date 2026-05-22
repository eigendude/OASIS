/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "localization/ZuptDetector.hpp"

#include <array>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

namespace OASIS::ROS
{
/*!
 * \brief Composable ROS adapter for the IMU-only ZUPT detector
 */
class ZuptDetectorNode : public rclcpp::Node
{
public:
  explicit ZuptDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ZuptDetectorNode() override = default;

private:
  void HandleImu(const sensor_msgs::msg::Imu& message);
  OASIS::Localization::ZuptDetectorConfig ReadDetectorConfig();
  geometry_msgs::msg::TwistWithCovarianceStamped BuildZuptMessage(
      const sensor_msgs::msg::Imu& message,
      const OASIS::Localization::ZuptDecision& decision) const;
  std::array<double, 36> BuildZuptCovariance(double linear_variance_mps2,
                                             double angular_variance_rads2) const;

  OASIS::Localization::ZuptDetectorConfig m_config;
  OASIS::Localization::ZuptDetector m_detector;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_zuptFlagPublisher;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr m_zuptPublisher;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSubscription;
};
} // namespace OASIS::ROS
