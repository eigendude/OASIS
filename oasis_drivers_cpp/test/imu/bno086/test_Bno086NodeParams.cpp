/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/ros/Bno086NodeParams.hpp"

#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

namespace
{
class RclcppContext
{
public:
  RclcppContext()
  {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  ~RclcppContext() { rclcpp::shutdown(); }
};
} // namespace

TEST(Bno086NodeParams, loadsStableReportDefaultsFromDeclaredRosParameters)
{
  const RclcppContext context;
  const std::shared_ptr<rclcpp::Node> node =
      std::make_shared<rclcpp::Node>("test_bno086_node_params_defaults");

  OASIS::ROS::DeclareBno086NodeParameters(*node);

  const OASIS::ROS::Bno086NodeParams params = OASIS::ROS::LoadBno086NodeParameters(*node);

  EXPECT_EQ(params.transport.i2c_device, "/dev/i2c-1");
  EXPECT_EQ(params.transport.i2c_address, 0x4B);
  EXPECT_EQ(params.transport.gpio_chip_device, "/dev/gpiochip0");
  EXPECT_EQ(params.transport.int_gpio, 23);

  EXPECT_DOUBLE_EQ(params.reports.rotation_vector_rate_hz, 50.0);
  EXPECT_DOUBLE_EQ(params.reports.gyro_rate_hz, 50.0);
  EXPECT_DOUBLE_EQ(params.reports.accelerometer_rate_hz, 100.0);
  EXPECT_DOUBLE_EQ(params.reports.linear_acceleration_rate_hz, 50.0);
  EXPECT_DOUBLE_EQ(params.reports.gravity_rate_hz, 25.0);

  EXPECT_EQ(params.reports.rotation_vector_batch_interval_us, 50'000U);
  EXPECT_EQ(params.reports.gyro_batch_interval_us, 50'000U);
  EXPECT_EQ(params.reports.accelerometer_batch_interval_us, 50'000U);
  EXPECT_EQ(params.reports.linear_acceleration_batch_interval_us, 50'000U);
  EXPECT_EQ(params.reports.gravity_batch_interval_us, 100'000U);
}

TEST(Bno086NodeParams, loadsTransportOverridesFromDeclaredRosParameters)
{
  const RclcppContext context;
  const std::shared_ptr<rclcpp::Node> node =
      std::make_shared<rclcpp::Node>("test_bno086_node_params_transport_overrides");

  OASIS::ROS::DeclareBno086NodeParameters(*node);

  node->set_parameter(rclcpp::Parameter("i2c_device", "/dev/i2c-test"));
  node->set_parameter(rclcpp::Parameter("i2c_address", 0x4A));
  node->set_parameter(rclcpp::Parameter("gpio_chip_device", "/dev/gpiochip-test"));
  node->set_parameter(rclcpp::Parameter("int_gpio", 24));

  const OASIS::ROS::Bno086NodeParams params = OASIS::ROS::LoadBno086NodeParameters(*node);

  EXPECT_EQ(params.transport.i2c_device, "/dev/i2c-test");
  EXPECT_EQ(params.transport.i2c_address, 0x4A);
  EXPECT_EQ(params.transport.gpio_chip_device, "/dev/gpiochip-test");
  EXPECT_EQ(params.transport.int_gpio, 24);
}
