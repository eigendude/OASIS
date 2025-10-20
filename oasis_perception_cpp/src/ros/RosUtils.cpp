/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "RosUtils.h"

using namespace OASIS::ROS;

double RosUtils::HeaderStampToSeconds(const std_msgs::msg::Header& header)
{
  return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1E-9;
}
