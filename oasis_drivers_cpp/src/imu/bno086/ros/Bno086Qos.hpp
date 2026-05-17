/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstddef>

#include <rclcpp/qos.hpp>

namespace OASIS::ROS
{
rclcpp::QoS BestEffortSensorQos(std::size_t depth);
rclcpp::QoS ReliableSensorQos(std::size_t depth);
} // namespace OASIS::ROS
