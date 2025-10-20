/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <std_msgs/msg/header.hpp>

namespace oasis_perception
{
class RosUtils
{
public:
  /*!\brief Convert a ROS message header timestamp to seconds.
   *
   * \param header The ROS message header containing a timestamp
   *
   * \return The timestamp expressed in seconds
   */
  static double HeaderStampToSeconds(const std_msgs::msg::Header& header);
};
} // namespace oasis_perception
