/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/ZuptDetectorNode.hpp"

#include <chrono>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

using OASIS::ROS::ZuptDetectorNode;

namespace
{
template<typename Predicate>
bool SpinUntil(rclcpp::executors::SingleThreadedExecutor& executor, Predicate predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!predicate() && std::chrono::steady_clock::now() < deadline)
    executor.spin_some();
  return predicate();
}

rclcpp::NodeOptions DetectorOptions(bool configure_frame = true)
{
  std::vector<rclcpp::Parameter> parameters{
      rclcpp::Parameter("gyro_enter_threshold_rads", 0.06),
      rclcpp::Parameter("gyro_exit_threshold_rads", 0.09),
      rclcpp::Parameter("accel_enter_threshold_mps2", 0.18),
      rclcpp::Parameter("accel_exit_threshold_mps2", 0.28),
      rclcpp::Parameter("min_stationary_sec", 0.18),
      rclcpp::Parameter("min_moving_sec", 0.01),
      rclcpp::Parameter("stationary_linear_velocity_sigma_mps", 0.06),
      rclcpp::Parameter("stationary_angular_velocity_sigma_rads", 0.06),
      rclcpp::Parameter("moving_linear_variance_mps2", 1.0e6),
      rclcpp::Parameter("moving_angular_variance_rads2", 1.0e6),
  };
  if (configure_frame)
  {
    parameters.emplace_back("imu_frame_id", "sensor_imu_link");
  }
  return rclcpp::NodeOptions().parameter_overrides(parameters);
}

class ZuptDetectorNodeTest : public testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};
} // namespace

TEST_F(ZuptDetectorNodeTest, PublishesReliableConfiguredImuFrameZuptAndLatchedFlag)
{
  const auto detector = std::make_shared<ZuptDetectorNode>(DetectorOptions());
  const auto observer = std::make_shared<rclcpp::Node>("zupt_qos_test");
  const rclcpp::QoS measurement_qos =
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();
  const rclcpp::QoS state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

  std::vector<geometry_msgs::msg::TwistWithCovarianceStamped> zupt_messages;
  const auto zupt_subscription =
      observer->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
          "zupt", measurement_qos,
          [&](const geometry_msgs::msg::TwistWithCovarianceStamped& message)
          { zupt_messages.push_back(message); });
  const auto imu_publisher =
      observer->create_publisher<sensor_msgs::msg::Imu>("imu", measurement_qos);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(detector);
  executor.add_node(observer);
  ASSERT_TRUE(SpinUntil(executor, [&]() { return imu_publisher->get_subscription_count() == 1; }));

  sensor_msgs::msg::Imu first;
  first.header.frame_id = "imu_link";
  imu_publisher->publish(first);
  sensor_msgs::msg::Imu second = first;
  second.header.stamp.nanosec = 180'000'000U;
  imu_publisher->publish(second);
  ASSERT_TRUE(SpinUntil(executor, [&]() { return zupt_messages.size() == 2; }));
  EXPECT_EQ(zupt_messages.back().header.frame_id, "sensor_imu_link");
  EXPECT_EQ(zupt_messages.back().header.stamp.nanosec, 180'000'000U);
  EXPECT_EQ(zupt_messages.back().twist.twist.linear.x, 0.0);
  EXPECT_EQ(zupt_messages.back().twist.twist.angular.z, 0.0);
  EXPECT_GT(zupt_messages.back().twist.covariance[0], 0.0);
  EXPECT_EQ(zupt_messages.back().twist.covariance[0], zupt_messages.back().twist.covariance[7]);
  EXPECT_EQ(zupt_messages.back().twist.covariance[21], zupt_messages.back().twist.covariance[35]);

  std::vector<std_msgs::msg::Bool> late_flags;
  const auto late_flag_subscription = observer->create_subscription<std_msgs::msg::Bool>(
      "zupt_flag", state_qos,
      [&](const std_msgs::msg::Bool& message) { late_flags.push_back(message); });
  ASSERT_TRUE(SpinUntil(executor, [&]() { return !late_flags.empty(); }));
  EXPECT_TRUE(late_flags.back().data);
}

TEST_F(ZuptDetectorNodeTest, DefaultsStandaloneZuptFrameToImuLink)
{
  const auto detector = std::make_shared<ZuptDetectorNode>(DetectorOptions(false));
  EXPECT_EQ(detector->get_parameter("imu_frame_id").as_string(), "imu_link");
}
