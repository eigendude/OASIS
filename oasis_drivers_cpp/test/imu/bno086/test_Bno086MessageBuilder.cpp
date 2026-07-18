/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/ros/Bno086MessageBuilder.hpp"
#include "imu/bno086/ros/Bno086Qos.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <rclcpp/time.hpp>
#include <rmw/types.h>

namespace
{
OASIS::ROS::Bno086RosMessageConfig MakeConfig(double prediction_horizon_sec = 0.0)
{
  OASIS::ROS::Bno086RosMessageConfig config;
  config.frame_id = "imu_link";
  config.prediction_horizon_sec = prediction_horizon_sec;
  config.prediction_source = "rotation_vector_plus_host_prediction";
  return config;
}

OASIS::IMU::BNO086::ImuSampleFrame MakeFrame()
{
  OASIS::IMU::BNO086::ImuSampleFrame frame;
  frame.has_orientation = true;
  frame.has_gyro = true;
  frame.has_linear_accel = true;
  frame.has_accel = true;
  frame.has_gravity = true;
  frame.orientation_world_from_imu_xyzw = {0.0, 0.0, 0.0, 1.0};
  frame.gyro_rads = {0.1, 0.2, 0.3};
  frame.linear_accel_mps2 = {1.0, 2.0, 3.0};
  frame.accel_mps2 = {4.0, 5.0, 6.0};
  frame.gravity_mps2 = {0.0, 0.0, -9.81};
  frame.orientation_cov_rad2 = {{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}}};
  frame.gyro_cov_rads2_2 = {{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, {0.7, 0.8, 0.9}}};
  frame.linear_accel_cov_mps2_2 = {{{1.1, 1.2, 1.3}, {1.4, 1.5, 1.6}, {1.7, 1.8, 1.9}}};
  frame.gravity_cov_mps2_2 = {{{2.1, 2.2, 2.3}, {2.4, 2.5, 2.6}, {2.7, 2.8, 2.9}}};
  frame.has_orientation_covariance = true;
  frame.has_gyro_covariance = true;
  frame.has_linear_accel_covariance = true;
  frame.has_gravity_covariance = true;
  return frame;
}
} // namespace

TEST(Bno086MessageBuilder, PresentImuPreservesStampAndFrameId)
{
  const OASIS::ROS::Bno086RosMessageConfig config = MakeConfig();
  const OASIS::IMU::BNO086::ImuSampleFrame frame = MakeFrame();
  const rclcpp::Time stamp{123, 456, RCL_ROS_TIME};

  const sensor_msgs::msg::Imu msg = OASIS::ROS::BuildBno086PresentImuMessage(config, frame, stamp);

  EXPECT_EQ(stamp, msg.header.stamp);
  EXPECT_EQ("imu_link", msg.header.frame_id);
  EXPECT_DOUBLE_EQ(1.0, msg.linear_acceleration.x);
  EXPECT_DOUBLE_EQ(5.0, msg.orientation_covariance[4]);
}

TEST(Bno086MessageBuilder, RotationVectorPublishesWorldFromImuOrientation)
{
  const OASIS::ROS::Bno086RosMessageConfig config = MakeConfig();
  OASIS::IMU::BNO086::ImuSampleFrame frame = MakeFrame();
  frame.orientation_world_from_imu_xyzw = {0.0, 0.0, std::sin(M_PI / 4.0), std::cos(M_PI / 4.0)};

  const sensor_msgs::msg::Imu msg =
      OASIS::ROS::BuildBno086PresentImuMessage(config, frame, rclcpp::Time{});
  const Eigen::Quaterniond orientation_world_from_imu(msg.orientation.w, msg.orientation.x,
                                                      msg.orientation.y, msg.orientation.z);
  const Eigen::Vector3d imu_x_in_world = orientation_world_from_imu * Eigen::Vector3d::UnitX();

  EXPECT_NEAR(imu_x_in_world.x(), 0.0, 1.0e-12);
  EXPECT_NEAR(imu_x_in_world.y(), 1.0, 1.0e-12);
  EXPECT_NEAR(imu_x_in_world.z(), 0.0, 1.0e-12);
}

TEST(Bno086MessageBuilder, ImuGravityUsesSelectedAccelSample)
{
  const OASIS::ROS::Bno086RosMessageConfig config = MakeConfig();
  const OASIS::IMU::BNO086::ImuSampleFrame frame = MakeFrame();
  OASIS::IMU::BNO086::Bno086ImuGravityAccelSample sample;
  sample.accel_mps2 = {7.0, 8.0, 9.0};
  sample.covariance_mps2_2 = {{{3.1, 3.2, 3.3}, {3.4, 3.5, 3.6}, {3.7, 3.8, 3.9}}};
  sample.has_covariance = true;

  const sensor_msgs::msg::Imu msg = OASIS::ROS::BuildBno086ImuGravityMessage(
      config, frame, sample, rclcpp::Time{123, 456, RCL_ROS_TIME});

  EXPECT_DOUBLE_EQ(7.0, msg.linear_acceleration.x);
  EXPECT_DOUBLE_EQ(8.0, msg.linear_acceleration.y);
  EXPECT_DOUBLE_EQ(9.0, msg.linear_acceleration.z);
  EXPECT_DOUBLE_EQ(3.5, msg.linear_acceleration_covariance[4]);
}

TEST(Bno086MessageBuilder, GravityPreservesVectorAndCovarianceFields)
{
  const OASIS::ROS::Bno086RosMessageConfig config = MakeConfig();
  const OASIS::IMU::BNO086::ImuSampleFrame frame = MakeFrame();

  const geometry_msgs::msg::AccelWithCovarianceStamped msg =
      OASIS::ROS::BuildBno086GravityMessage(config, frame, rclcpp::Time{123, 456, RCL_ROS_TIME});

  EXPECT_DOUBLE_EQ(0.0, msg.accel.accel.linear.x);
  EXPECT_DOUBLE_EQ(0.0, msg.accel.accel.linear.y);
  EXPECT_DOUBLE_EQ(-9.81, msg.accel.accel.linear.z);
  EXPECT_DOUBLE_EQ(2.1, msg.accel.covariance[0]);
  EXPECT_DOUBLE_EQ(2.5, msg.accel.covariance[7]);
  EXPECT_DOUBLE_EQ(2.9, msg.accel.covariance[14]);
  EXPECT_DOUBLE_EQ(-1.0, msg.accel.covariance[21]);
}

TEST(Bno086MessageBuilder, PredictedImuWithZeroHorizonPreservesOrientation)
{
  const OASIS::ROS::Bno086RosMessageConfig config = MakeConfig(0.0);
  const OASIS::IMU::BNO086::ImuSampleFrame frame = MakeFrame();
  const sensor_msgs::msg::Imu present =
      OASIS::ROS::BuildBno086PresentImuMessage(config, frame, rclcpp::Time{123, 456, RCL_ROS_TIME});

  const sensor_msgs::msg::Imu predicted =
      OASIS::ROS::BuildBno086PredictedImuMessage(config, frame, present);

  EXPECT_DOUBLE_EQ(present.orientation.x, predicted.orientation.x);
  EXPECT_DOUBLE_EQ(present.orientation.y, predicted.orientation.y);
  EXPECT_DOUBLE_EQ(present.orientation.z, predicted.orientation.z);
  EXPECT_DOUBLE_EQ(present.orientation.w, predicted.orientation.w);
}

TEST(Bno086MessageBuilder, PredictedImuWithZeroGyroPreservesOrientation)
{
  const OASIS::ROS::Bno086RosMessageConfig config = MakeConfig(0.05);
  OASIS::IMU::BNO086::ImuSampleFrame frame = MakeFrame();
  frame.gyro_rads = {0.0, 0.0, 0.0};
  const sensor_msgs::msg::Imu present =
      OASIS::ROS::BuildBno086PresentImuMessage(config, frame, rclcpp::Time{123, 456, RCL_ROS_TIME});

  const sensor_msgs::msg::Imu predicted =
      OASIS::ROS::BuildBno086PredictedImuMessage(config, frame, present);

  EXPECT_DOUBLE_EQ(present.orientation.x, predicted.orientation.x);
  EXPECT_DOUBLE_EQ(present.orientation.y, predicted.orientation.y);
  EXPECT_DOUBLE_EQ(present.orientation.z, predicted.orientation.z);
  EXPECT_DOUBLE_EQ(present.orientation.w, predicted.orientation.w);
}

TEST(Bno086MessageBuilder, PredictionAppliesImuFrameDeltaAfterWorldFromImuAttitude)
{
  const OASIS::ROS::Bno086RosMessageConfig config = MakeConfig(0.1);
  OASIS::IMU::BNO086::ImuSampleFrame frame = MakeFrame();
  frame.orientation_world_from_imu_xyzw = {0.0, 0.0, std::sin(M_PI / 4.0), std::cos(M_PI / 4.0)};
  frame.gyro_rads = {M_PI, 0.0, 0.0};
  const sensor_msgs::msg::Imu present =
      OASIS::ROS::BuildBno086PresentImuMessage(config, frame, rclcpp::Time{});

  const sensor_msgs::msg::Imu predicted =
      OASIS::ROS::BuildBno086PredictedImuMessage(config, frame, present);
  const Eigen::Quaterniond actual(predicted.orientation.w, predicted.orientation.x,
                                  predicted.orientation.y, predicted.orientation.z);
  const Eigen::Quaterniond q_WI(std::cos(M_PI / 4.0), 0.0, 0.0, std::sin(M_PI / 4.0));
  const Eigen::Quaterniond delta_I(std::cos(M_PI / 20.0), std::sin(M_PI / 20.0), 0.0, 0.0);
  const Eigen::Quaterniond expected = q_WI * delta_I;
  const Eigen::Quaterniond wrong_order = delta_I * q_WI;

  const Eigen::Vector3d actual_imu_y_in_world = actual * Eigen::Vector3d::UnitY();
  const Eigen::Vector3d expected_imu_y_in_world = expected * Eigen::Vector3d::UnitY();
  EXPECT_TRUE(actual_imu_y_in_world.isApprox(expected_imu_y_in_world, 1.0e-12));
  EXPECT_GT((actual_imu_y_in_world - wrong_order * Eigen::Vector3d::UnitY()).norm(), 0.1);
}

TEST(Bno086MessageBuilder, ImuVrPreservesPredictionMetadata)
{
  const OASIS::ROS::Bno086RosMessageConfig config = MakeConfig(0.05);
  const OASIS::IMU::BNO086::ImuSampleFrame frame = MakeFrame();
  const sensor_msgs::msg::Imu present =
      OASIS::ROS::BuildBno086PresentImuMessage(config, frame, rclcpp::Time{123, 456, RCL_ROS_TIME});
  const sensor_msgs::msg::Imu predicted =
      OASIS::ROS::BuildBno086PredictedImuMessage(config, frame, present);
  const OASIS::ROS::Bno086RosPredictionState predictionState{true, true, 3, 2};

  const oasis_msgs::msg::ImuVr vr =
      OASIS::ROS::BuildBno086PredictedVrMessage(config, predictionState, present, predicted);

  EXPECT_TRUE(vr.valid);
  EXPECT_EQ("rotation_vector_plus_host_prediction", vr.source);
  EXPECT_DOUBLE_EQ(0.05, vr.prediction_horizon_sec);
  EXPECT_EQ(2, vr.accuracy_status);
  EXPECT_TRUE(vr.covariance_is_prediction_model_based);
  EXPECT_DOUBLE_EQ(predicted.orientation.w, vr.orientation.w);
}

TEST(Bno086Qos, HelpersPreserveReliabilityAndDepth)
{
  const rclcpp::QoS reliable = OASIS::ROS::ReliableSensorQos(512);
  const rclcpp::QoS bestEffort = OASIS::ROS::BestEffortSensorQos(10);

  EXPECT_EQ(512U, reliable.get_rmw_qos_profile().depth);
  EXPECT_EQ(RMW_QOS_POLICY_RELIABILITY_RELIABLE, reliable.get_rmw_qos_profile().reliability);
  EXPECT_EQ(RMW_QOS_POLICY_DURABILITY_VOLATILE, reliable.get_rmw_qos_profile().durability);

  EXPECT_EQ(10U, bestEffort.get_rmw_qos_profile().depth);
  EXPECT_EQ(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, bestEffort.get_rmw_qos_profile().reliability);
  EXPECT_EQ(RMW_QOS_POLICY_DURABILITY_VOLATILE, bestEffort.get_rmw_qos_profile().durability);
}
