/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "ahrs/AhrsMath.hpp"
#include "nodes/AhrsNode.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using namespace OASIS::AHRS;
using OASIS::ROS::AhrsNode;

namespace
{
constexpr double kTolerance = 1.0e-9;

Eigen::Quaterniond QuaternionFromRos(const geometry_msgs::msg::Quaternion& quaternion)
{
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
}

void CopyQuaternion(const Eigen::Quaterniond& quaternion, geometry_msgs::msg::Quaternion& output)
{
  output.x = quaternion.x();
  output.y = quaternion.y();
  output.z = quaternion.z();
  output.w = quaternion.w();
}

sensor_msgs::msg::Imu MakeImu(const Eigen::Quaterniond& q_WI, std::int32_t stamp_sec)
{
  sensor_msgs::msg::Imu message;
  message.header.stamp.sec = stamp_sec;
  message.header.frame_id = "imu_link";
  CopyQuaternion(q_WI, message.orientation);
  message.orientation_covariance = {0.01, 0.002, 0.003, 0.002, 0.02, 0.004, 0.003, 0.004, 0.03};
  message.angular_velocity.x = 0.1;
  message.angular_velocity.y = -0.2;
  message.angular_velocity.z = 0.3;
  message.angular_velocity_covariance = {0.04,  0.005, 0.006, 0.005, 0.05,
                                         0.007, 0.006, 0.007, 0.06};
  message.linear_acceleration.x = 0.7;
  message.linear_acceleration.y = -0.4;
  message.linear_acceleration.z = 0.2;
  message.linear_acceleration_covariance = {0.07, 0.008, 0.009, 0.008, 0.08,
                                            0.01, 0.009, 0.01,  0.09};
  return message;
}

geometry_msgs::msg::AccelWithCovarianceStamped MakeGravity(const Eigen::Vector3d& gravity_imu)
{
  geometry_msgs::msg::AccelWithCovarianceStamped message;
  message.header.stamp.sec = 1;
  message.header.frame_id = "imu_link";
  message.accel.accel.linear.x = gravity_imu.x();
  message.accel.accel.linear.y = gravity_imu.y();
  message.accel.accel.linear.z = gravity_imu.z();
  message.accel.covariance[0] = 0.02;
  message.accel.covariance[7] = 0.03;
  message.accel.covariance[14] = 0.04;
  return message;
}

std::optional<geometry_msgs::msg::TransformStamped> FindTransform(
    const std::vector<tf2_msgs::msg::TFMessage>& messages,
    const std::string& parent,
    const std::string& child)
{
  for (auto message = messages.rbegin(); message != messages.rend(); ++message)
  {
    for (const geometry_msgs::msg::TransformStamped& transform : message->transforms)
    {
      if (transform.header.frame_id == parent && transform.child_frame_id == child)
        return transform;
    }
  }
  return std::nullopt;
}

template<typename Predicate>
bool SpinUntil(rclcpp::executors::SingleThreadedExecutor& executor, Predicate predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!predicate() && std::chrono::steady_clock::now() < deadline)
    executor.spin_some();
  return predicate();
}

void ExpectSameRotation(const Eigen::Quaterniond& actual, const Eigen::Quaterniond& expected)
{
  const Eigen::Vector3d vectors[] = {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
                                     Eigen::Vector3d(0.2, -0.3, 0.9)};
  for (const Eigen::Vector3d& vector : vectors)
    EXPECT_TRUE((actual * vector).isApprox(expected * vector, kTolerance));
}

class AhrsNodeTest : public testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};
} // namespace

TEST_F(AhrsNodeTest, PublishesDirectionalOrientationsAndTransforms)
{
  const rclcpp::NodeOptions options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("mounting_calibration_duration_sec", 0.0),
      rclcpp::Parameter("mounting_min_sample_count", static_cast<std::int64_t>(1)),
  });
  const auto ahrs = std::make_shared<AhrsNode>(options);
  const auto observer = std::make_shared<rclcpp::Node>("ahrs_direction_test");

  std::vector<sensor_msgs::msg::Imu> mounted_imu_messages;
  std::vector<sensor_msgs::msg::Imu> session_imu_messages;
  std::vector<nav_msgs::msg::Odometry> odom_messages;
  std::vector<tf2_msgs::msg::TFMessage> tf_messages;

  const auto mounted_subscription = observer->create_subscription<sensor_msgs::msg::Imu>(
      "ahrs/imu_gravity", rclcpp::QoS(10).reliable(),
      [&](const sensor_msgs::msg::Imu& message) { mounted_imu_messages.push_back(message); });
  const auto session_subscription = observer->create_subscription<sensor_msgs::msg::Imu>(
      "ahrs/imu", rclcpp::QoS(10).reliable(),
      [&](const sensor_msgs::msg::Imu& message) { session_imu_messages.push_back(message); });
  const auto odom_subscription = observer->create_subscription<nav_msgs::msg::Odometry>(
      "ahrs/odom", rclcpp::QoS(10).best_effort(),
      [&](const nav_msgs::msg::Odometry& message) { odom_messages.push_back(message); });
  const auto tf_subscription = observer->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100).best_effort(),
      [&](const tf2_msgs::msg::TFMessage& message) { tf_messages.push_back(message); });

  const auto gravity_publisher =
      observer->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
          "gravity", rclcpp::QoS(10).reliable());
  const auto imu_gravity_publisher =
      observer->create_publisher<sensor_msgs::msg::Imu>("imu_gravity", rclcpp::QoS(10).reliable());
  const auto imu_publisher =
      observer->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(10).reliable());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ahrs);
  executor.add_node(observer);
  ASSERT_TRUE(SpinUntil(executor,
                        [&]()
                        {
                          return gravity_publisher->get_subscription_count() == 1 &&
                                 imu_gravity_publisher->get_subscription_count() == 1 &&
                                 imu_publisher->get_subscription_count() == 1;
                        }));

  const Eigen::Quaterniond expected_q_BI = QuaternionFromRollPitchYaw(0.24, -0.31, 0.0);
  const Eigen::Vector3d gravity_imu = expected_q_BI.conjugate() * Eigen::Vector3d(0.0, 0.0, -9.81);
  gravity_publisher->publish(MakeGravity(gravity_imu));
  ASSERT_TRUE(SpinUntil(
      executor, [&]() { return FindTransform(tf_messages, "base_link", "imu_link").has_value(); }));

  const Eigen::Quaterniond expected_q_WB = QuaternionFromRollPitchYaw(0.27, -0.36, M_PI / 2.0);
  const Eigen::Quaterniond q_WI = expected_q_WB * expected_q_BI;
  sensor_msgs::msg::Imu zero_orientation = MakeImu(q_WI, 2);
  zero_orientation.orientation.x = 0.0;
  zero_orientation.orientation.y = 0.0;
  zero_orientation.orientation.z = 0.0;
  zero_orientation.orientation.w = 0.0;
  imu_gravity_publisher->publish(zero_orientation);

  sensor_msgs::msg::Imu nonfinite_orientation = MakeImu(q_WI, 3);
  nonfinite_orientation.orientation.x = std::numeric_limits<double>::infinity();
  imu_gravity_publisher->publish(nonfinite_orientation);

  sensor_msgs::msg::Imu scaled_orientation = MakeImu(q_WI, 4);
  scaled_orientation.orientation.x *= 2.5;
  scaled_orientation.orientation.y *= 2.5;
  scaled_orientation.orientation.z *= 2.5;
  scaled_orientation.orientation.w *= 2.5;
  imu_gravity_publisher->publish(scaled_orientation);
  ASSERT_TRUE(SpinUntil(executor,
                        [&]()
                        {
                          return std::any_of(mounted_imu_messages.begin(),
                                             mounted_imu_messages.end(),
                                             [](const sensor_msgs::msg::Imu& message)
                                             { return message.header.stamp.sec == 4; });
                        }));
  ASSERT_EQ(mounted_imu_messages.size(), 1U);
  EXPECT_EQ(mounted_imu_messages.front().header.stamp.sec, 4);

  const Eigen::Quaterniond mounted_output =
      QuaternionFromRos(mounted_imu_messages.front().orientation);
  ExpectSameRotation(mounted_output, expected_q_WB);
  EXPECT_EQ(mounted_imu_messages.front().header.frame_id, "base_link");
  EXPECT_NEAR(mounted_output.norm(), 1.0, kTolerance);
  const Eigen::Vector3d direction_probe(0.31, -0.47, 0.82);
  EXPECT_GT((mounted_output * direction_probe - expected_q_WB.conjugate() * direction_probe).norm(),
            0.5);

  imu_publisher->publish(MakeImu(q_WI, 3));
  ASSERT_TRUE(SpinUntil(executor,
                        [&]() { return !session_imu_messages.empty() && !odom_messages.empty(); }));

  const Eigen::Quaterniond expected_q_OW = QuaternionFromYaw(-YawFromQuaternion(expected_q_WB));
  const Eigen::Quaterniond expected_q_OB = ComposeOdomFromBase(expected_q_OW, expected_q_WB);
  const Eigen::Quaterniond session_output =
      QuaternionFromRos(session_imu_messages.back().orientation);
  const Eigen::Quaterniond odom_output =
      QuaternionFromRos(odom_messages.back().pose.pose.orientation);
  ExpectSameRotation(session_output, expected_q_OB);
  ExpectSameRotation(odom_output, expected_q_OB);
  EXPECT_NEAR(YawFromQuaternion(session_output), 0.0, kTolerance);
  EXPECT_EQ(odom_messages.back().header.frame_id, "odom");
  EXPECT_EQ(odom_messages.back().child_frame_id, "base_link");
  EXPECT_EQ(session_imu_messages.back().header.frame_id, "base_link");

  ASSERT_TRUE(SpinUntil(executor,
                        [&]()
                        {
                          return FindTransform(tf_messages, "base_link", "imu_link").has_value() &&
                                 FindTransform(tf_messages, "odom", "base_link").has_value();
                        }));
  const auto mounting_tf = FindTransform(tf_messages, "base_link", "imu_link");
  const auto odom_tf = FindTransform(tf_messages, "odom", "base_link");
  ASSERT_TRUE(mounting_tf.has_value());
  ASSERT_TRUE(odom_tf.has_value());
  ExpectSameRotation(QuaternionFromRos(mounting_tf->transform.rotation), expected_q_BI);
  ExpectSameRotation(QuaternionFromRos(odom_tf->transform.rotation), expected_q_OB);

  const Eigen::Vector3d imu_vector(0.5, -0.2, 0.7);
  EXPECT_TRUE((QuaternionFromRos(mounting_tf->transform.rotation) * imu_vector)
                  .isApprox(expected_q_BI * imu_vector, kTolerance));
  const Eigen::Vector3d base_vector(-0.4, 0.6, 0.3);
  EXPECT_TRUE((QuaternionFromRos(odom_tf->transform.rotation) * base_vector)
                  .isApprox(expected_q_OB * base_vector, kTolerance));

  const Eigen::Quaterniond later_q_WB = QuaternionFromRollPitchYaw(0.27, -0.36, M_PI / 4.0);
  const Eigen::Quaterniond later_q_WI = later_q_WB * expected_q_BI;
  imu_publisher->publish(MakeImu(later_q_WI, 4));
  const std::size_t first_session_count = session_imu_messages.size();
  ASSERT_TRUE(
      SpinUntil(executor, [&]() { return session_imu_messages.size() > first_session_count; }));
  const Eigen::Quaterniond later_q_OB = QuaternionFromRos(session_imu_messages.back().orientation);
  EXPECT_LT(YawFromQuaternion(later_q_OB), 0.0);
  EXPECT_NEAR(later_q_OB.norm(), 1.0, kTolerance);

  const Eigen::Quaterniond positive_q_WB =
      QuaternionFromRollPitchYaw(0.27, -0.36, 3.0 * M_PI / 4.0);
  imu_publisher->publish(MakeImu(positive_q_WB * expected_q_BI, 5));
  const std::size_t negative_session_count = session_imu_messages.size();
  ASSERT_TRUE(
      SpinUntil(executor, [&]() { return session_imu_messages.size() > negative_session_count; }));
  const Eigen::Quaterniond positive_q_OB =
      QuaternionFromRos(session_imu_messages.back().orientation);
  EXPECT_GT(YawFromQuaternion(positive_q_OB), 0.0);
  EXPECT_NEAR(positive_q_OB.norm(), 1.0, kTolerance);

  (void)mounted_subscription;
  (void)session_subscription;
  (void)odom_subscription;
  (void)tf_subscription;
}

TEST_F(AhrsNodeTest, ValidatesCovariancesAtInputBoundaries)
{
  const rclcpp::NodeOptions options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("mounting_calibration_duration_sec", 0.0),
      rclcpp::Parameter("mounting_min_sample_count", static_cast<std::int64_t>(1)),
  });
  const auto ahrs = std::make_shared<AhrsNode>(options);
  const auto observer = std::make_shared<rclcpp::Node>("ahrs_covariance_test");

  std::vector<geometry_msgs::msg::AccelWithCovarianceStamped> gravity_messages;
  std::vector<sensor_msgs::msg::Imu> imu_messages;
  const auto gravity_subscription =
      observer->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
          "ahrs/gravity", rclcpp::QoS(10).reliable(),
          [&](const geometry_msgs::msg::AccelWithCovarianceStamped& message)
          { gravity_messages.push_back(message); });
  const auto imu_subscription = observer->create_subscription<sensor_msgs::msg::Imu>(
      "ahrs/imu_gravity", rclcpp::QoS(10).reliable(),
      [&](const sensor_msgs::msg::Imu& message) { imu_messages.push_back(message); });
  const auto gravity_publisher =
      observer->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
          "gravity", rclcpp::QoS(10).reliable());
  const auto imu_publisher =
      observer->create_publisher<sensor_msgs::msg::Imu>("imu_gravity", rclcpp::QoS(10).reliable());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ahrs);
  executor.add_node(observer);
  ASSERT_TRUE(SpinUntil(executor,
                        [&]()
                        {
                          return gravity_publisher->get_subscription_count() == 1 &&
                                 imu_publisher->get_subscription_count() == 1;
                        }));

  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  gravity_publisher->publish(MakeGravity(gravity));
  ASSERT_TRUE(SpinUntil(executor, [&]() { return gravity_messages.size() == 1; }));

  geometry_msgs::msg::AccelWithCovarianceStamped invalid_gravity = MakeGravity(gravity);
  invalid_gravity.accel.covariance[7] = -1.0;
  gravity_publisher->publish(invalid_gravity);
  ASSERT_TRUE(SpinUntil(executor, [&]() { return gravity_messages.size() == 2; }));
  EXPECT_DOUBLE_EQ(gravity_messages.back().accel.accel.linear.z, -9.81);
  EXPECT_DOUBLE_EQ(gravity_messages.back().accel.covariance[0], -1.0);

  geometry_msgs::msg::AccelWithCovarianceStamped cross_axis_gravity = MakeGravity(gravity);
  cross_axis_gravity.accel.covariance[1] = 0.004;
  cross_axis_gravity.accel.covariance[6] = -0.006;
  gravity_publisher->publish(cross_axis_gravity);
  ASSERT_TRUE(SpinUntil(executor, [&]() { return gravity_messages.size() == 3; }));
  EXPECT_DOUBLE_EQ(gravity_messages.back().accel.covariance[1], 0.004);
  EXPECT_DOUBLE_EQ(gravity_messages.back().accel.covariance[6], -0.006);

  const sensor_msgs::msg::Imu valid_imu = MakeImu(Eigen::Quaterniond::Identity(), 1);
  for (const std::size_t diagonal_index : {0U, 4U, 8U})
  {
    sensor_msgs::msg::Imu invalid_orientation = valid_imu;
    invalid_orientation.orientation_covariance[diagonal_index] = -0.1;
    imu_publisher->publish(invalid_orientation);

    sensor_msgs::msg::Imu invalid_angular = valid_imu;
    invalid_angular.angular_velocity_covariance[diagonal_index] = -0.1;
    imu_publisher->publish(invalid_angular);

    sensor_msgs::msg::Imu invalid_linear = valid_imu;
    invalid_linear.linear_acceleration_covariance[diagonal_index] = -0.1;
    imu_publisher->publish(invalid_linear);
  }

  sensor_msgs::msg::Imu nonfinite_orientation = valid_imu;
  nonfinite_orientation.orientation_covariance[1] = std::numeric_limits<double>::quiet_NaN();
  imu_publisher->publish(nonfinite_orientation);
  sensor_msgs::msg::Imu nonfinite_angular = valid_imu;
  nonfinite_angular.angular_velocity_covariance[1] = std::numeric_limits<double>::infinity();
  imu_publisher->publish(nonfinite_angular);
  sensor_msgs::msg::Imu nonfinite_linear = valid_imu;
  nonfinite_linear.linear_acceleration_covariance[1] = -std::numeric_limits<double>::infinity();
  imu_publisher->publish(nonfinite_linear);

  executor.spin_some();
  EXPECT_TRUE(imu_messages.empty());

  sensor_msgs::msg::Imu unknown_orientation = valid_imu;
  unknown_orientation.orientation_covariance.fill(std::numeric_limits<double>::quiet_NaN());
  unknown_orientation.orientation_covariance[0] = -1.0;
  imu_publisher->publish(unknown_orientation);
  ASSERT_TRUE(SpinUntil(executor, [&]() { return imu_messages.size() == 1; }));
  EXPECT_DOUBLE_EQ(imu_messages.back().orientation_covariance[0], -1.0);

  imu_publisher->publish(valid_imu);
  ASSERT_TRUE(SpinUntil(executor, [&]() { return imu_messages.size() == 2; }));

  (void)gravity_subscription;
  (void)imu_subscription;
}
