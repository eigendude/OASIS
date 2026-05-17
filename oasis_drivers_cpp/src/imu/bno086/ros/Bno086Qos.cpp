/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/ros/Bno086Qos.hpp"

namespace OASIS::ROS
{
rclcpp::QoS BestEffortSensorQos(std::size_t depth)
{
  // BEST_EFFORT is reserved for disposable live/debug sensor streams
  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

rclcpp::QoS ReliableSensorQos(std::size_t depth)
{
  // RELIABLE is used for required pipeline and replay-facing streams
  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.reliable();
  qos.durability_volatile();
  return qos;
}
} // namespace OASIS::ROS
