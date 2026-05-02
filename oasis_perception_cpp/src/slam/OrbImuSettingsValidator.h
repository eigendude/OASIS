/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <string>

namespace rclcpp
{
class Logger;
} // namespace rclcpp

namespace OASIS
{
namespace SLAM
{

bool ValidateOrbImuSettings(const std::string& settingsFile, rclcpp::Logger& logger);

} // namespace SLAM
} // namespace OASIS
